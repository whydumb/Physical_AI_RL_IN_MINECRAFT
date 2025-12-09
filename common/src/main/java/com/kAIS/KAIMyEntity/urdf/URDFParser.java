package com.kAIS.KAIMyEntity.urdf;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.joml.Vector3f;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import java.io.File;
import java.util.HashSet;  // ✅ 추가
import java.util.Set;       // ✅ 추가

public class URDFParser {
    private static final Logger logger = LogManager.getLogger();
    /** URDF 파일이 있는 디렉토리 */
    private static File baseDir;

    // =====================================================================
    //  메인 진입점
    // =====================================================================
    public static URDFModel parse(File urdfFile) {
        try {
            baseDir = urdfFile.getParentFile();
            logger.info("=== URDF Parsing Start ===");
            logger.info("File: " + urdfFile.getAbsolutePath());
            logger.info("Base directory: " + baseDir.getAbsolutePath());

            DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
            factory.setNamespaceAware(true);
            DocumentBuilder builder = factory.newDocumentBuilder();
            Document doc = builder.parse(urdfFile);
            doc.getDocumentElement().normalize();

            Element robotElement = doc.getDocumentElement();
            String robotName = robotElement.getAttribute("name");

            URDFModel robot = new URDFModel(robotName);

            // ----- links -----
            NodeList all = robotElement.getElementsByTagName("*");
            int linkCount = 0;
            for (int i = 0; i < all.getLength(); i++) {
                Node n = all.item(i);
                if (n.getNodeType() != Node.ELEMENT_NODE) continue;
                String ln = n.getLocalName(), qn = n.getNodeName();
                if (!"link".equals(ln) && !"link".equals(qn)) continue;

                URDFLink link = parseLink((Element) n);
                if (link != null) {
                    robot.addLink(link);
                    linkCount++;
                    logger.debug("  + Link: " + link.name);
                }
            }
            logger.info("Found " + linkCount + " links");

            // ----- joints ----- ✅ 수정 시작
            NodeList jointNodes = robotElement.getElementsByTagName("joint");
            Set<String> processedJoints = new HashSet<>();
            int jointCount = 0;

            for (int i = 0; i < jointNodes.getLength(); i++) {
                Node n = jointNodes.item(i);
                if (n.getNodeType() != Node.ELEMENT_NODE) continue;

                Element jointElement = (Element) n;
                String jointName = safeGetAttr(jointElement, "name");
                if (isEmpty(jointName)) {
                    jointName = findAttrByLocalName(jointElement, "name");
                }

                // 중복 체크
                if (!isEmpty(jointName) && processedJoints.contains(jointName)) {
                    logger.debug("Skipping duplicate joint: " + jointName);
                    continue;
                }

                URDFJoint joint = parseJoint(jointElement);
                if (joint != null) {
                    robot.addJoint(joint);
                    processedJoints.add(joint.name);
                    jointCount++;
                    logger.debug("  + Joint: " + joint.name + " (" + joint.type + ")");
                }
            }
            logger.info("Found " + jointCount + " joints");
            // ✅ 수정 끝

            // 계층 구성
            robot.buildHierarchy();

            if (robot.rootLinkName == null || robot.getLink(robot.rootLinkName) == null) {
                logger.error("✗ No valid root link found!");
                logger.error("  Check parent/child relationships in joints");
                return null;
            }

            logger.info("=== URDF Parsing Complete ===");
            logger.info("  Robot: " + robot.name);
            logger.info("  Links: " + robot.getLinkCount());
            logger.info("  Joints: " + robot.getJointCount());
            logger.info("  Movable Joints: " + robot.getMovableJointCount());
            logger.info("  Root Link: " + robot.rootLinkName);

            // 상세 로깅 추가
            logger.info("[URDFParser] Links={}, Joints={}, Movable={}",
                    robot.getLinkCount(), robot.getJointCount(), robot.getMovableJointCount());
            for (URDFJoint j : robot.joints) {
                logger.info(" joint '{}': type={}, parent={}, child={}",
                        j.name, j.type, j.parentLinkName, j.childLinkName);
            }

            return robot;

        } catch (Exception e) {
            logger.error("✗ Failed to parse URDF file: " + urdfFile.getAbsolutePath(), e);
            return null;
        }
    }

    // =====================================================================
    //  메시 경로 해석
    // =====================================================================

    private static String resolveMeshPath(String uri) {
        if (uri == null || uri.isEmpty()) return null;

        logger.debug("Resolving mesh URI: " + uri);

        // 1) file://
        if (uri.startsWith("file://")) {
            try {
                String path = new java.net.URI(uri).getPath();
                File f = new File(path);
                if (f.exists()) return f.getAbsolutePath();
                logger.warn("  -> file:// path not found: " + path);
                return null;
            } catch (Exception e) {
                logger.warn("Invalid file:// URI: " + uri);
                return null;
            }
        }

        // 2) package://
        if (uri.startsWith("package://")) {
            String withoutScheme = uri.substring("package://".length());
            int slash = withoutScheme.indexOf('/');
            String relativePath = (slash >= 0) ? withoutScheme.substring(slash + 1) : withoutScheme;

            File resolved = new File(baseDir, relativePath);
            if (resolved.exists()) return resolved.getAbsolutePath();

            logger.warn("  -> package:// not found: " + resolved.getAbsolutePath());

            // meshes 디렉토리 fallback
            File meshDir = new File(baseDir, "meshes");
            File fallback = new File(meshDir, new File(relativePath).getName());
            if (fallback.exists()) return fallback.getAbsolutePath();
            return null;
        }

        // 3) 절대 경로
        File f = new File(uri);
        if (f.isAbsolute()) return f.exists() ? f.getAbsolutePath() : null;

        // 4) baseDir 기준 상대 경로
        File resolved = new File(baseDir, uri);
        if (resolved.exists()) return resolved.getAbsolutePath();

        // 5) baseDir/meshes/ 내에서 파일명으로 검색
        String filename = new File(uri).getName();
        File meshDir = new File(baseDir, "meshes");
        if (meshDir.exists() && meshDir.isDirectory()) {
            File meshFile = new File(meshDir, filename);
            if (meshFile.exists()) return meshFile.getAbsolutePath();

            File[] list = meshDir.listFiles();
            if (list != null) {
                for (File c : list) {
                    if (c.getName().equalsIgnoreCase(filename)) return c.getAbsolutePath();
                }
            }
        }

        // 6) 얕은 재귀 검색
        File found = searchFileRecursive(baseDir, filename, 2);
        if (found != null) return found.getAbsolutePath();

        logger.warn("  -> Could not resolve mesh: " + uri);
        return null;
    }

    private static File searchFileRecursive(File directory, String filename, int maxDepth) {
        if (maxDepth <= 0 || !directory.isDirectory()) return null;
        File[] files = directory.listFiles();
        if (files == null) return null;

        for (File file : files) {
            if (file.isFile() && file.getName().equalsIgnoreCase(filename)) return file;
        }
        for (File file : files) {
            if (file.isDirectory() && !file.getName().startsWith(".")) {
                File found = searchFileRecursive(file, filename, maxDepth - 1);
                if (found != null) return found;
            }
        }
        return null;
    }

    // =====================================================================
    //  Link / Visual / Collision / Inertial 파싱
    // =====================================================================

    private static URDFLink parseLink(Element linkElement) {
        String name = safeGetAttr(linkElement, "name");
        if (isEmpty(name)) name = findAttrByLocalName(linkElement, "name");
        if (isEmpty(name)) {
            logger.warn("Link element without name attribute. Skipping.");
            return null;
        }

        URDFLink link = new URDFLink(name);

        // visual
        Element visualEl = getDirectChildByLocalName(linkElement, "visual");
        if (visualEl == null) {
            NodeList visualNodes = linkElement.getElementsByTagName("visual");
            if (visualNodes.getLength() > 0) visualEl = (Element) visualNodes.item(0);
        }
        if (visualEl != null) link.visual = parseVisual(visualEl);

        // collision
        Element collisionEl = getDirectChildByLocalName(linkElement, "collision");
        if (collisionEl == null) {
            NodeList nodes = linkElement.getElementsByTagName("collision");
            if (nodes.getLength() > 0) collisionEl = (Element) nodes.item(0);
        }
        if (collisionEl != null) link.collision = parseCollision(collisionEl);

        // inertial
        Element inertialEl = getDirectChildByLocalName(linkElement, "inertial");
        if (inertialEl == null) {
            NodeList nodes = linkElement.getElementsByTagName("inertial");
            if (nodes.getLength() > 0) inertialEl = (Element) nodes.item(0);
        }
        if (inertialEl != null) link.inertial = parseInertial(inertialEl);

        return link;
    }

    private static URDFLink.Visual parseVisual(Element visualElement) {
        URDFLink.Visual visual = new URDFLink.Visual();

        Element originEl = getDirectChildByLocalName(visualElement, "origin");
        if (originEl == null) {
            NodeList list = visualElement.getElementsByTagName("origin");
            if (list.getLength() > 0) originEl = (Element) list.item(0);
        }
        if (originEl != null) visual.origin = parseOrigin(originEl);

        Element geomEl = getDirectChildByLocalName(visualElement, "geometry");
        if (geomEl == null) {
            NodeList list = visualElement.getElementsByTagName("geometry");
            if (list.getLength() > 0) geomEl = (Element) list.item(0);
        }
        if (geomEl != null) visual.geometry = parseGeometry(geomEl);

        Element matEl = getDirectChildByLocalName(visualElement, "material");
        if (matEl == null) {
            NodeList list = visualElement.getElementsByTagName("material");
            if (list.getLength() > 0) matEl = (Element) list.item(0);
        }
        if (matEl != null) visual.material = parseMaterial(matEl);

        return visual;
    }

    private static URDFLink.Collision parseCollision(Element collisionElement) {
        URDFLink.Collision collision = new URDFLink.Collision();

        Element originEl = getDirectChildByLocalName(collisionElement, "origin");
        if (originEl == null) {
            NodeList list = collisionElement.getElementsByTagName("origin");
            if (list.getLength() > 0) originEl = (Element) list.item(0);
        }
        if (originEl != null) collision.origin = parseOrigin(originEl);

        Element geomEl = getDirectChildByLocalName(collisionElement, "geometry");
        if (geomEl == null) {
            NodeList list = collisionElement.getElementsByTagName("geometry");
            if (list.getLength() > 0) geomEl = (Element) list.item(0);
        }
        if (geomEl != null) collision.geometry = parseGeometry(geomEl);

        return collision;
    }

    private static URDFLink.Inertial parseInertial(Element inertialElement) {
        URDFLink.Inertial inertial = new URDFLink.Inertial();

        Element originEl = getDirectChildByLocalName(inertialElement, "origin");
        if (originEl == null) {
            NodeList list = inertialElement.getElementsByTagName("origin");
            if (list.getLength() > 0) originEl = (Element) list.item(0);
        }
        if (originEl != null) inertial.origin = parseOrigin(originEl);

        Element massEl = getDirectChildByLocalName(inertialElement, "mass");
        if (massEl == null) {
            NodeList list = inertialElement.getElementsByTagName("mass");
            if (list.getLength() > 0) massEl = (Element) list.item(0);
        }
        if (massEl != null) {
            inertial.mass = new URDFLink.Inertial.Mass();
            String v = safeOrLocal(massEl, "value");
            if (!isEmpty(v)) inertial.mass.value = parseFloatSafe(v, 0f);
        }

        Element inertiaEl = getDirectChildByLocalName(inertialElement, "inertia");
        if (inertiaEl == null) {
            NodeList list = inertialElement.getElementsByTagName("inertia");
            if (list.getLength() > 0) inertiaEl = (Element) list.item(0);
        }
        if (inertiaEl != null) {
            inertial.inertia = new URDFLink.Inertial.Inertia();
            inertial.inertia.ixx = parseFloatSafe(safeOrLocal(inertiaEl, "ixx"), 0f);
            inertial.inertia.ixy = parseFloatSafe(safeOrLocal(inertiaEl, "ixy"), 0f);
            inertial.inertia.ixz = parseFloatSafe(safeOrLocal(inertiaEl, "ixz"), 0f);
            inertial.inertia.iyy = parseFloatSafe(safeOrLocal(inertiaEl, "iyy"), 0f);
            inertial.inertia.iyz = parseFloatSafe(safeOrLocal(inertiaEl, "iyz"), 0f);
            inertial.inertia.izz = parseFloatSafe(safeOrLocal(inertiaEl, "izz"), 0f);
        }

        return inertial;
    }

    private static URDFLink.Geometry parseGeometry(Element geometryElement) {
        URDFLink.Geometry geometry = new URDFLink.Geometry();

        // mesh
        Element meshEl = getDirectChildByLocalName(geometryElement, "mesh");
        if (meshEl == null) {
            NodeList list = geometryElement.getElementsByTagName("mesh");
            if (list.getLength() > 0) meshEl = (Element) list.item(0);
        }
        if (meshEl != null) {
            geometry.type = URDFLink.Geometry.GeometryType.MESH;

            String rawUri = safeOrLocal(meshEl, "filename");
            String resolved = resolveMeshPath(rawUri);
            geometry.meshFilename = (resolved != null) ? resolved : rawUri;

            String scl = safeOrLocal(meshEl, "scale");
            geometry.scale = !isEmpty(scl) ? parseVector3(scl) : new Vector3f(1f, 1f, 1f);
            return geometry;
        }

        // box
        Element boxEl = getDirectChildByLocalName(geometryElement, "box");
        if (boxEl == null) {
            NodeList list = geometryElement.getElementsByTagName("box");
            if (list.getLength() > 0) boxEl = (Element) list.item(0);
        }
        if (boxEl != null) {
            geometry.type = URDFLink.Geometry.GeometryType.BOX;
            geometry.boxSize = parseVector3(safeOrLocal(boxEl, "size"));
            return geometry;
        }

        // cylinder
        Element cylEl = getDirectChildByLocalName(geometryElement, "cylinder");
        if (cylEl == null) {
            NodeList list = geometryElement.getElementsByTagName("cylinder");
            if (list.getLength() > 0) cylEl = (Element) list.item(0);
        }
        if (cylEl != null) {
            geometry.type = URDFLink.Geometry.GeometryType.CYLINDER;
            geometry.cylinderRadius = parseFloatSafe(safeOrLocal(cylEl, "radius"), 0f);
            geometry.cylinderLength = parseFloatSafe(safeOrLocal(cylEl, "length"), 0f);
            return geometry;
        }

        // sphere
        Element sphEl = getDirectChildByLocalName(geometryElement, "sphere");
        if (sphEl == null) {
            NodeList list = geometryElement.getElementsByTagName("sphere");
            if (list.getLength() > 0) sphEl = (Element) list.item(0);
        }
        if (sphEl != null) {
            geometry.type = URDFLink.Geometry.GeometryType.SPHERE;
            geometry.sphereRadius = parseFloatSafe(safeOrLocal(sphEl, "radius"), 0f);
            return geometry;
        }

        logger.warn("No geometry found in element");
        return geometry;
    }

    private static URDFLink.Material parseMaterial(Element materialElement) {
        URDFLink.Material material = new URDFLink.Material();
        String name = safeGetAttr(materialElement, "name");
        if (isEmpty(name)) name = findAttrByLocalName(materialElement, "name");
        material.name = name;

        Element colorEl = getDirectChildByLocalName(materialElement, "color");
        if (colorEl == null) {
            NodeList list = materialElement.getElementsByTagName("color");
            if (list.getLength() > 0) colorEl = (Element) list.item(0);
        }
        if (colorEl != null) {
            String rgbaStr = safeOrLocal(colorEl, "rgba");
            if (!isEmpty(rgbaStr)) {
                String[] rgba = rgbaStr.trim().split("\\s+");
                if (rgba.length == 4) {
                    material.color = new URDFLink.Material.Vector4f(
                            parseFloatSafe(rgba[0], 0f),
                            parseFloatSafe(rgba[1], 0f),
                            parseFloatSafe(rgba[2], 0f),
                            parseFloatSafe(rgba[3], 1f)
                    );
                }
            }
        }

        Element texEl = getDirectChildByLocalName(materialElement, "texture");
        if (texEl == null) {
            NodeList list = materialElement.getElementsByTagName("texture");
            if (list.getLength() > 0) texEl = (Element) list.item(0);
        }
        if (texEl != null) material.textureFilename = safeOrLocal(texEl, "filename");

        return material;
    }

    /** link용 origin 파서 - 기본값 초기화 개선 */
    private static URDFLink.Origin parseOrigin(Element originElement) {
        URDFLink.Origin origin = new URDFLink.Origin();
        String xyz = safeOrLocal(originElement, "xyz");
        String rpy = safeOrLocal(originElement, "rpy");
        origin.xyz = !isEmpty(xyz) ? parseVector3(xyz) : new Vector3f(0, 0, 0);
        origin.rpy = !isEmpty(rpy) ? parseVector3(rpy) : new Vector3f(0, 0, 0);
        return origin;
    }

    /** Joint용 origin 파서 (URDFJoint.Origin 타입에 맞게 분리) */
    private static URDFJoint.Origin parseJointOrigin(Element originElement) {
        URDFJoint.Origin origin = new URDFJoint.Origin();
        String xyz = safeOrLocal(originElement, "xyz");
        String rpy = safeOrLocal(originElement, "rpy");
        origin.xyz = !isEmpty(xyz) ? parseVector3(xyz) : new Vector3f(0, 0, 0);
        origin.rpy = !isEmpty(rpy) ? parseVector3(rpy) : new Vector3f(0, 0, 0);
        return origin;
    }

    // =====================================================================
    //  Joint 파싱 (패치 + 유효성 검증)
    // =====================================================================

    private static URDFJoint parseJoint(Element jointElement) {
        if (jointElement == null) return null;

        // --- 1) name ---
        String name = safeGetAttr(jointElement, "name");
        if (isEmpty(name)) name = findAttrByLocalName(jointElement, "name");
        if (isEmpty(name)) {
            logger.warn("Joint element without name attribute. Skipping.");
            return null;
        }

        // --- 2) type (attribute → <type> 태그 텍스트 → FIXED) ---
        String typeRaw = safeGetAttr(jointElement, "type");
        if (isEmpty(typeRaw)) typeRaw = findAttrByLocalName(jointElement, "type");

        // fallback: <type> 자식/후손 태그의 텍스트
        if (isEmpty(typeRaw)) {
            NodeList typeNodes = jointElement.getElementsByTagName("type");
            if (typeNodes.getLength() > 0) {
                String txt = typeNodes.item(0).getTextContent();
                if (!isEmpty(txt)) typeRaw = txt.trim();
            }
        }

        URDFJoint.JointType type;
        if (isEmpty(typeRaw)) {
            logger.warn("Joint '{}' has empty type. Defaulting to FIXED.", name);
            type = URDFJoint.JointType.FIXED;
        } else {
            try {
                type = URDFJoint.JointType.valueOf(typeRaw.trim().toUpperCase());
            } catch (IllegalArgumentException ex) {
                logger.warn("Unknown joint type '{}' for '{}'. Defaulting to FIXED.", typeRaw, name);
                type = URDFJoint.JointType.FIXED;
            }
        }

        URDFJoint joint = new URDFJoint(name, type);

        // --- 3) parent / child ---
        // parent: 먼저 direct child, 없으면 getElementsByTagName 사용
        Element parentEl = getDirectChildByLocalName(jointElement, "parent");
        if (parentEl == null) {
            NodeList parentList = jointElement.getElementsByTagName("parent");
            if (parentList.getLength() > 0) parentEl = (Element) parentList.item(0);
        }
        if (parentEl != null) {
            String p = safeOrLocal(parentEl, "link");
            if (isEmpty(p)) {
                String txt = parentEl.getTextContent();
                if (!isEmpty(txt)) p = txt.trim();
            }
            joint.parentLinkName = p;
        }

        // child
        Element childEl = getDirectChildByLocalName(jointElement, "child");
        if (childEl == null) {
            NodeList childList = jointElement.getElementsByTagName("child");
            if (childList.getLength() > 0) childEl = (Element) childList.item(0);
        }
        if (childEl != null) {
            String c = safeOrLocal(childEl, "link");
            if (isEmpty(c)) {
                String txt = childEl.getTextContent();
                if (!isEmpty(txt)) c = txt.trim();
            }
            joint.childLinkName = c;
        }

        // --- 4) origin (xyz / rpy) ---
        Element originEl = getDirectChildByLocalName(jointElement, "origin");
        if (originEl == null) {
            NodeList originList = jointElement.getElementsByTagName("origin");
            if (originList.getLength() > 0) originEl = (Element) originList.item(0);
        }
        if (originEl != null) {
            joint.origin = parseJointOrigin(originEl);
        }

        // --- 5) axis (xyz attr → 텍스트 → 기본값 (1 0 0)) ---
        Element axisEl = getDirectChildByLocalName(jointElement, "axis");
        if (axisEl == null) {
            NodeList axisList = jointElement.getElementsByTagName("axis");
            if (axisList.getLength() > 0) axisEl = (Element) axisList.item(0);
        }

        Vector3f axis = null;
        if (axisEl != null) {
            String axisStr = safeOrLocal(axisEl, "xyz");
            if (!isEmpty(axisStr)) axis = parseVector3(axisStr);

            // attribute가 비어있으면 텍스트에서 파싱
            if (axis == null || axis.lengthSquared() == 0f) {
                String txt = axisEl.getTextContent();
                if (!isEmpty(txt)) axis = parseVector3(txt.trim());
            }
        }

        // 기본축: revolute/continuous/prismatic 은 (1,0,0)
        if (axis == null || axis.lengthSquared() == 0f) {
            switch (type) {
                case REVOLUTE:
                case CONTINUOUS:
                case PRISMATIC:
                    if (axis == null) axis = new Vector3f(1, 0, 0);
                    else axis.set(1, 0, 0);
                    logger.debug("Joint '{}' : using default axis (1,0,0)", name);
                    break;
                default:
                    if (axis == null) axis = new Vector3f(0, 0, 0);
                    else axis.set(0, 0, 0);
                    break;
            }
        } else {
            axis.normalize();
        }

        if (joint.axis == null) joint.axis = new URDFJoint.Axis();
        joint.axis.xyz = axis;

        // --- 6) limit ---
        Element limitEl = getDirectChildByLocalName(jointElement, "limit");
        if (limitEl == null) {
            NodeList list = jointElement.getElementsByTagName("limit");
            if (list.getLength() > 0) limitEl = (Element) list.item(0);
        }
        if (limitEl != null) {
            joint.limit = new URDFJoint.Limit();
            String lower    = safeOrLocal(limitEl, "lower");
            String upper    = safeOrLocal(limitEl, "upper");
            String effort   = safeOrLocal(limitEl, "effort");
            String velocity = safeOrLocal(limitEl, "velocity");

            if (!isEmpty(lower))    joint.limit.lower    = parseFloatSafe(lower, 0f);
            if (!isEmpty(upper))    joint.limit.upper    = parseFloatSafe(upper, 0f);
            if (!isEmpty(effort))   joint.limit.effort   = parseFloatSafe(effort, 0f);
            if (!isEmpty(velocity)) joint.limit.velocity = parseFloatSafe(velocity, 0f);
        }

        // --- 7) dynamics ---
        Element dynEl = getDirectChildByLocalName(jointElement, "dynamics");
        if (dynEl == null) {
            NodeList list = jointElement.getElementsByTagName("dynamics");
            if (list.getLength() > 0) dynEl = (Element) list.item(0);
        }
        if (dynEl != null) {
            joint.dynamics = new URDFJoint.Dynamics();
            String damping  = safeOrLocal(dynEl, "damping");
            String friction = safeOrLocal(dynEl, "friction");

            if (!isEmpty(damping))  joint.dynamics.damping  = parseFloatSafe(damping, 0f);
            if (!isEmpty(friction)) joint.dynamics.friction = parseFloatSafe(friction, 0f);
        }

        // --- 8) parent/child 최종 검증 ---
        if (isEmpty(joint.parentLinkName)) {
            logger.warn("Joint '{}' missing parent link!", name);
        }
        if (isEmpty(joint.childLinkName)) {
            logger.warn("Joint '{}' missing child link!", name);
        }

        if (isEmpty(joint.parentLinkName) || isEmpty(joint.childLinkName)) {
            logger.warn("Joint '{}' has invalid parent/child. Ignoring this joint element.", name);
            return null;
        }

        return joint;
    }

    // =====================================================================
    //  공통 헬퍼
    // =====================================================================

    private static boolean isEmpty(String s) {
        return s == null || s.trim().isEmpty();
    }

    private static float parseFloatSafe(String s, float def) {
        try {
            return Float.parseFloat(s.trim());
        } catch (Exception e) {
            return def;
        }
    }

    private static Vector3f parseVector3(String str) {
        if (str == null || str.trim().isEmpty()) return new Vector3f(0f, 0f, 0f);
        String[] parts = str.trim().split("\\s+");
        if (parts.length == 3) {
            try {
                return new Vector3f(
                        Float.parseFloat(parts[0]),
                        Float.parseFloat(parts[1]),
                        Float.parseFloat(parts[2])
                );
            } catch (Exception ignored) {
            }
        }
        logger.warn("Invalid Vector3 string: " + str);
        return new Vector3f(0f, 0f, 0f);
    }

    /** 바로 아래 자식만, 대소문자/네임스페이스 무시 */
    private static Element getDirectChildByLocalName(Element parent, String wanted) {
        if (parent == null || wanted == null) return null;
        for (Node n = parent.getFirstChild(); n != null; n = n.getNextSibling()) {
            if (n.getNodeType() != Node.ELEMENT_NODE) continue;
            String local = n.getLocalName(); // namespace-aware일 때 로컬명
            String qname = n.getNodeName();  // 접두사 포함
            if ((local != null && local.equalsIgnoreCase(wanted)) ||
                    (qname != null && qname.equalsIgnoreCase(wanted))) {
                return (Element) n;
            }
        }
        return null;
    }

    /** 속성 찾기: 대소문자/네임스페이스 무시 */
    private static String findAttrByLocalName(Element el, String wanted) {
        if (el == null || el.getAttributes() == null || wanted == null) return null;
        for (int i = 0; i < el.getAttributes().getLength(); i++) {
            Node a = el.getAttributes().item(i);
            String ln = a.getLocalName(); // 로컬명
            String nn = a.getNodeName();  // 접두사 포함 이름
            if ((ln != null && ln.equalsIgnoreCase(wanted)) ||
                    (nn != null && nn.equalsIgnoreCase(wanted))) {
                String v = a.getNodeValue();
                return (v == null || v.trim().isEmpty()) ? null : v.trim();
            }
        }
        return null;
    }

    /**
     * 안전 속성 읽기: 정확 이름 → 실패 시 케이스/네임스페이스 무시 검색
     */
    private static String safeGetAttr(Element el, String nameCI) {
        if (el == null || nameCI == null) return null;
        // 1) 정확 이름
        String v = el.getAttribute(nameCI);
        if (v != null && !v.trim().isEmpty()) return v.trim();
        // 2) fallback: 대소문자/네임스페이스 무시
        return findAttrByLocalName(el, nameCI);
    }

    private static String safeOrLocal(Element el, String nameCI) {
        return safeGetAttr(el, nameCI);
    }
}
