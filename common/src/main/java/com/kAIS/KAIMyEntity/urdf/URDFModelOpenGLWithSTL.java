package com.kAIS.KAIMyEntity.urdf;

import com.kAIS.KAIMyEntity.renderer.IMMDModel;
import com.kAIS.KAIMyEntity.urdf.control.URDFSimpleController;
import com.mojang.blaze3d.systems.RenderSystem;
import com.mojang.blaze3d.vertex.PoseStack;
import com.mojang.blaze3d.vertex.VertexConsumer;
import net.minecraft.client.Minecraft;
import net.minecraft.client.renderer.MultiBufferSource;
import net.minecraft.client.renderer.RenderType;
import net.minecraft.world.entity.Entity;
import net.minecraft.world.level.Level;
import net.minecraft.world.phys.Vec3;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.joml.Matrix4f;
import org.joml.Quaternionf;
import org.joml.Vector3f;

import java.io.File;
import java.util.*;

/**
 * URDF 모델 렌더링 + ODE4J 물리 통합 + 블록 충돌 연동 버전
 */
public class URDFModelOpenGLWithSTL implements IMMDModel {
    private static final Logger logger = LogManager.getLogger();
    private static int renderCount = 0;

    private URDFModel robotModel;
    private String modelDir;

    private final URDFSimpleController controller;
    private final Map<String, STLLoader.STLMesh> meshCache = new HashMap<>();

    // 렌더 전용 스케일 (물리는 1블록 = 1m 기준으로 동작)
    private static final float GLOBAL_SCALE = 5.0f;

    // 기본 루트 높이 + 지면 정렬용 오프셋
    private static final float BASE_HEIGHT = 1.5f;
    private float groundOffset = 0.0f;

    private static final boolean FLIP_NORMALS = true;
    private static final boolean DEBUG_MODE = false;

    // ROS → Minecraft 좌표계 보정
    private static final Vector3f SRC_UP  = new Vector3f(0, 0, 1);
    private static final Vector3f SRC_FWD = new Vector3f(1, 0, 0);
    private static final Vector3f DST_UP  = new Vector3f(0, 1, 0);
    private static final Vector3f DST_FWD = new Vector3f(0, 0, -1);
    private static final Quaternionf Q_ROS2MC = makeUprightQuat(SRC_UP, SRC_FWD, DST_UP, DST_FWD);

    // VMD ↔ URDF 조인트 이름 매핑
    private final Map<String, String> jointNameMapping = new HashMap<>();
    private boolean jointMappingInitialized = false;

    // ========================================================================
    // 생성자 / 초기화
    // ========================================================================

    public URDFModelOpenGLWithSTL(URDFModel robotModel, String modelDir) {
        this.robotModel = robotModel;
        this.modelDir = modelDir;

        initJointNameMapping();

        // 물리 모드 켜서 컨트롤러 생성 (ODE4J + BlockCollisionManager 사용)
        this.controller = new URDFSimpleController(
                robotModel,
                robotModel.joints,
                true,
                jointNameMapping
        );

        logger.info("=== URDFSimpleController created (physics mode: {}) ===",
                controller.isUsingPhysics());

        logger.info("=== URDF renderer Created (Scale: {}) ===", GLOBAL_SCALE);

        loadAllMeshes();
        calculateGroundOffset();
    }

    /**
     * STL 메쉬를 전부 미리 로드
     */
    private void loadAllMeshes() {
        logger.info("=== Loading STL meshes ===");
        int loadedCount = 0;

        for (URDFLink link : robotModel.links) {
            if (link.visual != null && link.visual.geometry != null) {
                URDFLink.Geometry g = link.visual.geometry;
                if (g.type == URDFLink.Geometry.GeometryType.MESH && g.meshFilename != null) {
                    File f = new File(g.meshFilename);
                    if (f.exists()) {
                        STLLoader.STLMesh mesh = STLLoader.load(g.meshFilename);
                        if (mesh != null) {
                            // URDF 내 scale 적용
                            if (g.scale != null &&
                                (g.scale.x != 1f || g.scale.y != 1f || g.scale.z != 1f)) {
                                STLLoader.scaleMesh(mesh, g.scale);
                            }
                            meshCache.put(link.name, mesh);
                            loadedCount++;
                        }
                    }
                }
            }
        }

        logger.info("=== STL Loading Complete: {}/{} meshes ===",
                loadedCount, robotModel.getLinkCount());
    }

    /**
     * 로봇의 발/최저점 기준으로 groundOffset 계산
     */
    private void calculateGroundOffset() {
        float minZ = 0.0f;
        boolean foundFootLink = false;

        for (URDFLink link : robotModel.links) {
            String lowerName = link.name.toLowerCase();
            if (lowerName.contains("foot") || lowerName.contains("ankle") ||
                lowerName.contains("toe")  || lowerName.contains("sole")) {

                foundFootLink = true;

                if (link.visual != null && link.visual.origin != null) {
                    minZ = Math.min(minZ, link.visual.origin.xyz.z);
                }

                STLLoader.STLMesh mesh = meshCache.get(link.name);
                if (mesh != null) {
                    for (STLLoader.Triangle tri : mesh.triangles) {
                        for (Vector3f v : tri.vertices) {
                            minZ = Math.min(minZ, v.z);
                        }
                    }
                }
            }
        }

        // 발 링크 못 찾으면 전체 메쉬에서 최소 Z
        if (!foundFootLink) {
            for (STLLoader.STLMesh mesh : meshCache.values()) {
                for (STLLoader.Triangle tri : mesh.triangles) {
                    for (Vector3f v : tri.vertices) {
                        minZ = Math.min(minZ, v.z);
                    }
                }
            }
        }

        // 렌더 스케일 적용해서 블록 단위 오프셋으로 변환
        groundOffset = -minZ * GLOBAL_SCALE;

        logger.info("✓ Ground offset: {} blocks (raw Z: {}m, feet: {})",
                groundOffset, minZ, foundFootLink);
    }

    /**
     * VMD 스타일 이름을 URDF 조인트 이름과 매핑
     */
    private void initJointNameMapping() {
        logger.info("=== Initializing Joint Name Mapping ===");

        Map<String, String[]> vmdToUrdfCandidates = new HashMap<>();
        vmdToUrdfCandidates.put("head_pan",     new String[]{"head_pan", "HeadYaw", "head_yaw", "Neck", "neck"});
        vmdToUrdfCandidates.put("head_tilt",    new String[]{"head_tilt", "HeadPitch", "head_pitch", "Head", "head"});
        vmdToUrdfCandidates.put("l_sho_pitch",  new String[]{"l_sho_pitch", "LShoulderPitch", "l_shoulder_pitch"});
        vmdToUrdfCandidates.put("l_sho_roll",   new String[]{"l_sho_roll", "LShoulderRoll",  "l_shoulder_roll"});
        vmdToUrdfCandidates.put("l_el",         new String[]{"l_el", "LElbowYaw", "l_elbow", "LElbowRoll"});
        vmdToUrdfCandidates.put("r_sho_pitch",  new String[]{"r_sho_pitch", "RShoulderPitch", "r_shoulder_pitch"});
        vmdToUrdfCandidates.put("r_sho_roll",   new String[]{"r_sho_roll", "RShoulderRoll",  "r_shoulder_roll"});
        vmdToUrdfCandidates.put("r_el",         new String[]{"r_el", "RElbowYaw", "r_elbow", "RElbowRoll"});
        vmdToUrdfCandidates.put("l_hip_pitch",  new String[]{"l_hip_pitch", "LHipPitch", "LeftHipPitch"});
        vmdToUrdfCandidates.put("r_hip_pitch",  new String[]{"r_hip_pitch", "RHipPitch", "RightHipPitch"});

        for (Map.Entry<String, String[]> entry : vmdToUrdfCandidates.entrySet()) {
            String vmdName = entry.getKey();
            for (String candidate : entry.getValue()) {
                for (URDFJoint j : robotModel.joints) {
                    if (j.name.equals(candidate) || j.name.equalsIgnoreCase(candidate)) {
                        jointNameMapping.put(vmdName, j.name);
                        logger.info("  Mapped: '{}' -> '{}'", vmdName, j.name);
                        break;
                    }
                }
                if (jointNameMapping.containsKey(vmdName)) break;
            }
        }

        jointMappingInitialized = true;
        logger.info("=== Joint Mapping Complete: {} mappings ===", jointNameMapping.size());
    }

    // ========================================================================
    // 업데이트 / 월드 컨텍스트
    // ========================================================================

    /**
     * 구버전 호환용 단순 업데이트 (월드 정보 없음)
     */
    public void tickUpdate(float dt) {
        tickUpdate(dt, null);
    }

    /**
     * 월드/엔티티 정보 함께 전달하는 버전
     */
    public void tickUpdate(float dt, Entity entity) {
        if (controller != null) {
            if (entity != null) {
                controller.setWorldContext(entity.level(), entity.position());
            }
            controller.update(dt);
        }
    }

    // ========================================================================
    // 조인트 제어 유틸
    // ========================================================================

    private String resolveJointName(String name) {
        if (name == null) return null;

        // 1차: VMD → URDF 매핑
        String mappedName = jointNameMapping.getOrDefault(name, name);

        // 컨트롤러에 실제 존재하는지 확인
        if (controller.hasJoint(mappedName)) {
            return mappedName;
        }

        // 매핑 이름이 없으면 원래 이름도 한 번 더 체크
        if (!mappedName.equals(name) && controller.hasJoint(name)) {
            return name;
        }

        // 대소문자 무시 검색
        String found = controller.findJointIgnoreCase(name);
        if (found != null) return found;

        return null;
    }

    public void setJointTarget(String name, float value) {
        if (controller == null) return;

        String resolvedName = resolveJointName(name);
        if (resolvedName != null) {
            controller.setTarget(resolvedName, value);
        } else if (DEBUG_MODE && renderCount < 5) {
            logger.warn("✗ Joint NOT FOUND: '{}'", name);
        }
    }

    public void setJointTargets(Map<String, Float> values) {
        for (Map.Entry<String, Float> entry : values.entrySet()) {
            setJointTarget(entry.getKey(), entry.getValue());
        }
    }

    public void setJointVelocity(String name, float velocity) {
        if (controller == null) return;
        String resolvedName = resolveJointName(name);
        if (resolvedName != null) {
            controller.setTargetVelocity(resolvedName, velocity);
        }
    }

    public void setJointPreview(String name, float value) {
        if (controller == null) return;
        String resolvedName = resolveJointName(name);
        if (resolvedName != null) {
            controller.setPreviewPosition(resolvedName, value);
        }
    }

    public float getJointVelocity(String jointName) {
        if (controller == null) return 0f;
        String resolvedName = resolveJointName(jointName);
        return resolvedName != null ? controller.getJointVelocity(resolvedName) : 0f;
    }

    public List<String> getMovableJointNames() {
        return controller != null ? controller.getMovableJointNames() : Collections.emptyList();
    }

    public float[] getJointLimits(String jointName) {
        if (controller == null) return new float[]{(float) -Math.PI, (float) Math.PI};
        String resolvedName = resolveJointName(jointName);
        return resolvedName != null
                ? controller.getJointLimits(resolvedName)
                : new float[]{(float) -Math.PI, (float) Math.PI};
    }

    public float getJointPosition(String jointName) {
        if (controller == null) return 0f;
        String resolvedName = resolveJointName(jointName);
        return resolvedName != null ? controller.getJointPosition(resolvedName) : 0f;
    }

    public Map<String, Float> getAllJointPositions() {
        return controller != null ? controller.getAllJointPositions() : Collections.emptyMap();
    }

    public Set<String> getJointNames() {
        return controller != null ? controller.getJointNameSet() : Collections.emptySet();
    }

    public void printAllJoints() {
        logger.info("=== Joint Name Mappings ({}) ===", jointNameMapping.size());
        for (Map.Entry<String, String> entry : jointNameMapping.entrySet()) {
            logger.info("  - '{}' -> '{}'", entry.getKey(), entry.getValue());
        }
    }

    // ========================================================================
    // 루트 위치 / 물리 사용 여부
    // ========================================================================

    /**
     * 물리 루트 위치(엔티티 로컬 좌표) → 없으면 기본 높이 반환
     */
    public float[] getRootWorldPosition() {
        if (controller != null && controller.isUsingPhysics() && robotModel.rootLinkName != null) {
            // 컨트롤러 쪽에서 루트 로컬 좌표를 관리한다고 가정
            return controller.getRootLinkLocalPosition();
        }
        // 물리 안 쓰면 그냥 기본값
        return new float[]{0.0f, BASE_HEIGHT + groundOffset, 0.0f};
    }

    public boolean isUsingPhysics() {
        return controller != null && controller.isUsingPhysics();
    }

    public URDFSimpleController getController() {
        return controller;
    }

    public URDFModel getRobotModel() {
        return robotModel;
    }

    // ========================================================================
    // 렌더링
    // ========================================================================

    @Override
    public void Render(Entity entityIn, float entityYaw, float entityPitch,
                       Vector3f entityTrans, float tickDelta, PoseStack poseStack, int packedLight) {

        renderCount++;

        // 컨트롤러에 월드 컨텍스트 전달 (블록 충돌/물리에서 사용)
        if (controller != null && entityIn != null) {
            Level level = entityIn.level();
            Vec3 worldPos = entityIn.position();
            controller.setWorldContext(level, worldPos);
        }

        if (renderCount % 120 == 1) {
            logger.info("=== URDF RENDER #{} (Scale: {}, Physics: {}) ===",
                    renderCount, GLOBAL_SCALE, isUsingPhysics());
        }

        RenderSystem.enableBlend();
        RenderSystem.defaultBlendFunc();
        RenderSystem.enableDepthTest();
        RenderSystem.disableCull();

        MultiBufferSource.BufferSource bufferSource =
                Minecraft.getInstance().renderBuffers().bufferSource();
        VertexConsumer vc = bufferSource.getBuffer(RenderType.solid());

        if (robotModel.rootLinkName != null) {
            poseStack.pushPose();

            if (controller != null && controller.isUsingPhysics()) {
                // ★ 물리 모드: 루트 링크 로컬 위치(ODE 좌표)를 렌더 위치에 반영
                float[] rootLocal = controller.getRootLinkLocalPosition();
                poseStack.translate(
                        rootLocal[0],
                        rootLocal[1] + BASE_HEIGHT + groundOffset,
                        rootLocal[2]
                );
            } else {
                // 키네마틱 모드: 기존 고정 오프셋
                float totalYOffset = BASE_HEIGHT + groundOffset;
                poseStack.translate(0.0f, totalYOffset, 0.0f);
            }

            // ROS → Minecraft 좌표계 회전
            poseStack.mulPose(new Quaternionf(Q_ROS2MC));

            // 메쉬 스케일 (위치에는 스케일 다시 안 곱함)
            poseStack.scale(GLOBAL_SCALE, GLOBAL_SCALE, GLOBAL_SCALE);

            renderLinkRecursive(robotModel.rootLinkName, poseStack, vc, packedLight);
            poseStack.popPose();
        }

        bufferSource.endBatch(RenderType.solid());
        RenderSystem.enableCull();

    }

    private void renderLinkRecursive(String linkName, PoseStack poseStack,
                                     VertexConsumer vc, int packedLight) {
        URDFLink link = robotModel.getLink(linkName);
        if (link == null) return;

        poseStack.pushPose();

        if (link.visual != null) {
            renderVisual(link, poseStack, vc, packedLight);
        }

        for (URDFJoint childJoint : robotModel.getChildJoints(linkName)) {
            poseStack.pushPose();
            applyJointTransform(childJoint, poseStack);
            renderLinkRecursive(childJoint.childLinkName, poseStack, vc, packedLight);
            poseStack.popPose();
        }

        poseStack.popPose();
    }

    private void renderVisual(URDFLink link, PoseStack poseStack,
                              VertexConsumer vc, int packedLight) {
        if (link.visual == null || link.visual.geometry == null) return;

        poseStack.pushPose();

        if (link.visual.origin != null) {
            applyLinkOriginTransform(link.visual.origin, poseStack);
        }

        STLLoader.STLMesh mesh = meshCache.get(link.name);
        if (mesh != null) {
            renderMesh(mesh, link, poseStack, vc, packedLight);
        }

        poseStack.popPose();
    }

    private void renderMesh(STLLoader.STLMesh mesh, URDFLink link,
                            PoseStack poseStack, VertexConsumer vc, int packedLight) {
        Matrix4f matrix = poseStack.last().pose();

        int r = 220, g = 220, b = 220, a = 255;
        if (link.visual.material != null && link.visual.material.color != null) {
            URDFLink.Material.Vector4f color = link.visual.material.color;
            r = (int) (color.x * 255);
            g = (int) (color.y * 255);
            b = (int) (color.z * 255);
            a = (int) (color.w * 255);
        }

        int blockLight = Math.max((packedLight & 0xFFFF), 0xA0);
        int skyLight   = Math.max((packedLight >> 16) & 0xFFFF, 0xA0);

        for (STLLoader.Triangle tri : mesh.triangles) {
            for (int i = 2; i >= 0; i--) {
                Vector3f v = tri.vertices[i];
                Vector3f n = tri.normal;

                float nx = FLIP_NORMALS ? -n.x : n.x;
                float ny = FLIP_NORMALS ? -n.y : n.y;
                float nz = FLIP_NORMALS ? -n.z : n.z;

                vc.addVertex(matrix, v.x, v.y, v.z)
                        .setColor(r, g, b, a)
                        .setUv(0.5f, 0.5f)
                        .setUv2(blockLight, skyLight)
                        .setNormal(nx, ny, nz);
            }
        }
    }

    // ========================================================================
    // 변환 유틸
    // ========================================================================

    private void applyLinkOriginTransform(URDFLink.Origin origin, PoseStack poseStack) {
        poseStack.translate(origin.xyz.x, origin.xyz.y, origin.xyz.z);
        if (origin.rpy.x != 0f || origin.rpy.y != 0f || origin.rpy.z != 0f) {
            poseStack.mulPose(origin.getQuaternion());
        }
    }

    private void applyJointOriginTransform(URDFJoint.Origin origin, PoseStack poseStack) {
        poseStack.translate(origin.xyz.x, origin.xyz.y, origin.xyz.z);
        if (origin.rpy.x != 0f || origin.rpy.y != 0f || origin.rpy.z != 0f) {
            Quaternionf qx = new Quaternionf().rotateX(origin.rpy.x);
            Quaternionf qy = new Quaternionf().rotateY(origin.rpy.y);
            Quaternionf qz = new Quaternionf().rotateZ(origin.rpy.z);
            poseStack.mulPose(qz.mul(qy).mul(qx));
        }
    }

    private void applyJointTransform(URDFJoint joint, PoseStack poseStack) {
        if (joint.origin != null) {
            applyJointOriginTransform(joint.origin, poseStack);
        }
        if (joint.isMovable()) {
            applyJointMotion(joint, poseStack);
        }
    }

    private void applyJointMotion(URDFJoint joint, PoseStack poseStack) {
        if (joint == null) return;

        switch (joint.type) {
            case REVOLUTE:
            case CONTINUOUS: {
                Vector3f axis;
                if (joint.axis == null || joint.axis.xyz == null ||
                    joint.axis.xyz.lengthSquared() < 1e-12f) {
                    axis = new Vector3f(1, 0, 0);
                } else {
                    axis = new Vector3f(joint.axis.xyz);
                    if (axis.lengthSquared() < 1e-12f) axis.set(1, 0, 0);
                    else axis.normalize();
                }
                Quaternionf quat = new Quaternionf()
                        .rotateAxis(joint.currentPosition, axis.x, axis.y, axis.z);
                poseStack.mulPose(quat);
                break;
            }
            case PRISMATIC: {
                Vector3f axis;
                if (joint.axis == null || joint.axis.xyz == null ||
                    joint.axis.xyz.lengthSquared() < 1e-12f) {
                    axis = new Vector3f(1, 0, 0);
                } else {
                    axis = new Vector3f(joint.axis.xyz);
                    if (axis.lengthSquared() < 1e-12f) axis.set(1, 0, 0);
                    else axis.normalize();
                }
                Vector3f t = axis.mul(joint.currentPosition);
                poseStack.translate(t.x, t.y, t.z);
                break;
            }
            default:
                break;
        }
    }

    // ========================================================================
    // IMMDModel 구현
    // ========================================================================

    @Override
    public void ChangeAnim(long anim, long layer) { }

    @Override
    public void ResetPhysics() {
        logger.info("ResetPhysics called");
        if (controller != null) {
            controller.resetPhysics();
        }
    }

    @Override
    public long GetModelLong() {
        return 0;
    }

    @Override
    public String GetModelDir() {
        return modelDir;
    }

    public static URDFModelOpenGLWithSTL Create(String urdfPath, String modelDir) {
        File urdfFile = new File(urdfPath);
        if (!urdfFile.exists()) return null;
        URDFModel robot = URDFParser.parse(urdfFile);
        if (robot == null || robot.rootLinkName == null) return null;
        return new URDFModelOpenGLWithSTL(robot, modelDir);
    }

    // ========================================================================
    // 업라이트 보정 유틸
    // ========================================================================

    private static Quaternionf makeUprightQuat(Vector3f srcUp, Vector3f srcFwd,
                                               Vector3f dstUp, Vector3f dstFwd) {
        Vector3f su = new Vector3f(srcUp).normalize();
        Vector3f sf = new Vector3f(srcFwd).normalize();
        Vector3f du = new Vector3f(dstUp).normalize();
        Vector3f df = new Vector3f(dstFwd).normalize();

        // 1) Up 벡터 정렬
        Quaternionf qUp = fromToQuat(su, du);
        Vector3f sf1 = sf.rotate(new Quaternionf(qUp));

        // 2) Up 축에 수직인 평면에서 Forward 정렬
        Vector3f sf1p = new Vector3f(sf1).sub(new Vector3f(du).mul(sf1.dot(du)));
        Vector3f dfp = new Vector3f(df).sub(new Vector3f(du).mul(df.dot(du)));
        if (sf1p.lengthSquared() < 1e-10f || dfp.lengthSquared() < 1e-10f) {
            return qUp.normalize();
        }
        sf1p.normalize();
        dfp.normalize();

        float cos = clamp(sf1p.dot(dfp), -1f, 1f);
        float angle = (float) Math.acos(cos);
        Vector3f cross = sf1p.cross(dfp, new Vector3f());
        if (cross.dot(du) < 0) angle = -angle;

        Quaternionf qFwd = new Quaternionf().fromAxisAngleRad(du, angle);
        return qFwd.mul(qUp).normalize();
    }

    private static Quaternionf fromToQuat(Vector3f a, Vector3f b) {
        Vector3f v1 = new Vector3f(a).normalize();
        Vector3f v2 = new Vector3f(b).normalize();
        float dot = clamp(v1.dot(v2), -1f, 1f);

        if (dot > 1.0f - 1e-6f) return new Quaternionf(); // 동일
        if (dot < -1.0f + 1e-6f) {
            Vector3f axis = pickAnyPerp(v1).normalize();
            return new Quaternionf().fromAxisAngleRad(axis, (float) Math.PI);
        }
        Vector3f axis = v1.cross(v2, new Vector3f()).normalize();
        float angle = (float) Math.acos(dot);
        return new Quaternionf().fromAxisAngleRad(axis, angle);
    }

    private static Vector3f pickAnyPerp(Vector3f v) {
        Vector3f x = new Vector3f(1, 0, 0);
        Vector3f y = new Vector3f(0, 1, 0);
        Vector3f z = new Vector3f(0, 0, 1);
        float dx = Math.abs(v.dot(x));
        float dy = Math.abs(v.dot(y));
        float dz = Math.abs(v.dot(z));
        return (dx < dy && dx < dz) ? x : ((dy < dz) ? y : z);
    }

    private static float clamp(float v, float lo, float hi) {
        return v < lo ? lo : (Math.min(v, hi));
    }
}
