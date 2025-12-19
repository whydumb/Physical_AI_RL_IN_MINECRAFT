package com.kAIS.KAIMyEntity.urdf.control;

import com.kAIS.KAIMyEntity.PhysicsManager;
import com.kAIS.KAIMyEntity.urdf.URDFJoint;
import com.kAIS.KAIMyEntity.urdf.URDFLink;
import com.kAIS.KAIMyEntity.urdf.URDFModel;
import net.minecraft.util.Mth;
import net.minecraft.world.level.Level;
import net.minecraft.world.phys.Vec3;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.lang.reflect.Method;
import java.util.*;

/**
 * URDF 컨트롤러 - 물리 + 블록 충돌 통합 (Geom 추가)
 *
 * 좌표계:
 * - ODE World = 마인크래프트 월드 좌표계
 * - 각 링크 Body = 월드 좌표에서의 위치/회전
 * - 위치 계산은 실시간으로 전달받은 엔티티 위치 기준
 */
public final class URDFSimpleController {
    private static final Logger logger = LogManager.getLogger();

    // ========== 공통 필드 ==========
    private final Map<String, URDFJoint> joints;
    private final Map<String, Float> target = new HashMap<>();
    private final Map<String, String> jointNameMapping;

    // ========== 키네마틱 모드 ==========
    private float kp = 30f;
    private float kd = 6f;
    private float defaultMaxVel = 4.0f;
    private float defaultMaxAcc = 12.0f;

    // ========== 물리 모드 ==========
    private final URDFModel urdfModel;
    private final PhysicsManager physics;
    private boolean usePhysics = false;
    private boolean physicsInitialized = false;

    private final Map<String, Object> bodies = new HashMap<>();
    private final Map<String, Object> geoms = new HashMap<>();
    private final Map<String, Object> odeJoints = new HashMap<>();
    private final Map<String, Float> linkRadii = new HashMap<>();

    private final Map<String, Float> targetVelocities = new HashMap<>();

    // 물리용 PD 게인 / 토크 제한
    private float physicsKp = 20f;
    private float physicsKd = 2f;
    private float maxTorque = 10f;
    private float maxForce = 100f;

    // ========== 블록 충돌 ==========
    private BlockCollisionManager blockCollisionManager;
    private Level currentLevel;

    // ✅ 초기 스폰 위치 (앵커링용, 한 번만 저장)
    private Vec3 initialAnchorPosition = null;

    // ========== 스케일 (URDF 단위 → 물리 단위) ==========
    private float physicsScale = 1.0f;

    // 대표 루트 바디 (전신 위치/엔티티 이동 기준으로 사용할 링크 이름)
    private String rootBodyLinkName;

    // 한 번 월드 위치에 정렬(앵커)되었는지 여부
    private boolean worldAnchored = false;

    // 조인트 PD 모터 사용 여부
    private boolean motorsEnabled = true;

    // 물리 서브스텝 수
    private int physicsSubSteps = 4;

    // ODE4J 버전
    private enum ODE4JVersion { V03X, V04X, V05X, UNKNOWN }
    private ODE4JVersion odeVersion = ODE4JVersion.UNKNOWN;

    // 리플렉션 캐시
    private Class<?> dHingeJointClass;
    private Class<?> dSliderJointClass;
    private Class<?> dFixedJointClass;
    private Class<?> dMassClass;
    private Class<?> dBodyClass;
    private Class<?> dWorldClass;
    private Class<?> odeHelperClass;
    private Class<?> dJointGroupClass;
    private Class<?> dGeomClass;

    private Method massSetBoxMethod;
    private Method massSetSphereMethod;
    private Method bodySetMassMethod;
    private Method createMassMethod;

    // ========================================================================
    // 생성자
    // ========================================================================

    public URDFSimpleController(Collection<URDFJoint> allJoints) {
        this(null, allJoints, false, Collections.emptyMap());
    }

    public URDFSimpleController(URDFModel model, Collection<URDFJoint> allJoints, boolean enablePhysics) {
        this(model, allJoints, enablePhysics, Collections.emptyMap());
    }

    public URDFSimpleController(URDFModel model, Collection<URDFJoint> allJoints,
                                boolean enablePhysics, Map<String, String> nameMapping) {
        this.urdfModel = model;
        this.jointNameMapping = nameMapping != null ? new HashMap<>(nameMapping) : new HashMap<>();

        // 루트 바디 후보: URDF rootLinkName
        this.rootBodyLinkName = (model != null && model.rootLinkName != null)
                ? model.rootLinkName
                : null;

        Map<String, URDFJoint> m = new HashMap<>();
        for (URDFJoint j : allJoints) {
            m.put(j.name, j);
            target.put(j.name, j.currentPosition);
            targetVelocities.put(j.name, 0f);
        }
        this.joints = m;

        if (enablePhysics) {
            PhysicsManager pm = null;
            try {
                pm = PhysicsManager.GetInst();
            } catch (Exception e) {
                logger.warn("PhysicsManager not available", e);
            }
            this.physics = pm;

            if (physics != null && physics.isInitialized()) {
                try {
                    initializeODE4JClasses();
                    detectODE4JVersion();
                    buildPhysicsModel();

                    this.blockCollisionManager = new BlockCollisionManager();

                    usePhysics = true;
                    physicsInitialized = true;

                    logger.info("URDFSimpleController: PHYSICS mode");
                    logger.info("  ODE4J version: {}", odeVersion);
                    logger.info("  Bodies: {}, Geoms: {}, Joints: {}",
                            bodies.size(), geoms.size(), odeJoints.size());
                    logger.info("  BlockCollisionManager: active");

                } catch (Exception e) {
                    logger.error("Failed to initialize physics", e);
                    usePhysics = false;
                    physicsInitialized = false;
                }
            } else {
                this.usePhysics = false;
                this.physicsInitialized = false;
                logger.info("PhysicsManager not initialized, using KINEMATIC mode");
            }
        } else {
            this.physics = null;
            this.usePhysics = false;
            logger.info("URDFSimpleController: KINEMATIC mode");
        }

        // 공통 후처리: 물리 모드일 때 기본 설정
        if (usePhysics && physicsInitialized && physics != null) {
            physics.setGravity(0, -9.81f, 0);
            this.motorsEnabled = true;

            try {
                physics.setWorldTuning(0.8, 1e-5, 50);
                physics.setContactTuning(true, 0.8, 1e-5, 1.0);
                physics.setDebugContacts(false);
                logger.info("Applied recommended physics tuning (firm ground contact)");
            } catch (Exception e) {
                logger.warn("Failed to apply physics tuning: {}", e.getMessage());
            }
        }
    }

    // ========================================================================
    // 월드 컨텍스트 (렌더러/엔티티에서 호출)
    // ========================================================================

    /**
     * ✅ 수정: Level만 업데이트, 위치는 앵커링 시에만 저장
     * 현재 월드와 엔티티의 월드 위치 전달.
     * 물리 모드에서는 최초 한 번, 루트 바디를 해당 위치로 정렬(앵커)한다.
     */
    public void setWorldContext(Level level, Vec3 worldPos) {
        this.currentLevel = level;

        // ✅ 앵커링 전에만 초기 위치 저장
        if (usePhysics && physicsInitialized && !worldAnchored &&
                physics != null && !bodies.isEmpty() && worldPos != null) {

            Vec3 safePos = new Vec3(worldPos.x, worldPos.y + 1.0, worldPos.z);
            this.initialAnchorPosition = safePos;

            anchorPhysicsToWorld(safePos);
            worldAnchored = true;

            if (blockCollisionManager != null && currentLevel != null) {
                blockCollisionManager.forceUpdate(currentLevel, safePos.x, safePos.y, safePos.z);
                logger.info("BlockCollisionManager force-updated at spawn position. activeBlocks={}",
                        blockCollisionManager.getActiveBlockCount());
            }
        }
    }

    /**
     * ✅ 더 이상 사용하지 않음 (호환성 유지용)
     */
    @Deprecated
    public Vec3 getWorldPosition() {
        return initialAnchorPosition != null ? initialAnchorPosition : Vec3.ZERO;
    }

    /**
     * 루트 바디 기준으로 모든 바디를 주어진 월드 위치로 평행 이동시킨다.
     *
     * ✅ 중요: 바디만 이동하면 힌지 앵커는 "월드 고정"이라서 뒤에 남는다.
     * 따라서 이동 후 힌지 앵커를 다시 세팅(refresh)해야 관통/폭주가 줄어든다.
     */
    private void anchorPhysicsToWorld(Vec3 anchorWorldPos) {
        try {
            Object root = getRootBody();
            if (root == null || physics == null) return;

            double[] rootPos = physics.getBodyPosition(root);
            if (rootPos == null || rootPos.length < 3) return;

            double dx = anchorWorldPos.x - rootPos[0];
            double dy = anchorWorldPos.y - rootPos[1];
            double dz = anchorWorldPos.z - rootPos[2];

            for (Object body : bodies.values()) {
                if (body == null) continue;
                double[] p = physics.getBodyPosition(body);
                if (p == null || p.length < 3) continue;
                physics.setBodyPosition(body, p[0] + dx, p[1] + dy, p[2] + dz);
            }

            // ✅ PATCH: 힌지 앵커 월드좌표 재설정
            refreshHingeAnchors();

            logger.info("Anchored physics to world at ({}, {}, {})",
                    anchorWorldPos.x, anchorWorldPos.y, anchorWorldPos.z);

        } catch (Exception e) {
            logger.warn("anchorPhysicsToWorld failed: {}", e.getMessage());
        }
    }

    /**
     * ✅ PATCH: 힌지 조인트 앵커를 "현재 바디 위치" 기반으로 다시 월드좌표에 세팅.
     * 바디를 평행이동(스폰 앵커)한 뒤 호출해야 함.
     */
    private void refreshHingeAnchors() {
        if (physics == null) return;

        try {
            Method setAnchor = dHingeJointClass.getMethod(
                    "setAnchor", double.class, double.class, double.class);

            for (URDFJoint j : joints.values()) {
                if (j == null) continue;
                if (!(j.type == URDFJoint.JointType.REVOLUTE || j.type == URDFJoint.JointType.CONTINUOUS)) continue;

                Object odeJoint = odeJoints.get(j.name);
                if (odeJoint == null) continue;

                Object childBody  = bodies.get(j.childLinkName);
                Object parentBody = bodies.get(j.parentLinkName);

                double ax = 0.0, ay = 0.0, az = 0.0;
                boolean baseFromBody = false;

                if (childBody != null) {
                    double[] p = physics.getBodyPosition(childBody);
                    if (p != null && p.length >= 3) {
                        ax = p[0]; ay = p[1]; az = p[2];
                        baseFromBody = true;
                    }
                }
                if (!baseFromBody && parentBody != null) {
                    double[] p = physics.getBodyPosition(parentBody);
                    if (p != null && p.length >= 3) {
                        ax = p[0]; ay = p[1]; az = p[2];
                    }
                }

                if (j.origin != null && j.origin.xyz != null) {
                    ax += j.origin.xyz.x * physicsScale;
                    ay += j.origin.xyz.y * physicsScale;
                    az += j.origin.xyz.z * physicsScale;
                }

                setAnchor.invoke(odeJoint, ax, ay, az);
            }

            logger.info("Refreshed hinge anchors for {} joints", odeJoints.size());
        } catch (Exception e) {
            // 메서드가 없거나 버전 차이인 경우 무시
            logger.debug("refreshHingeAnchors skipped: {}", e.getMessage());
        }
    }

    // ========================================================================
    // 메인 업데이트
    // ========================================================================

    /**
     * ✅ 수정: 현재 엔티티 위치를 인자로 받음
     */
    public void update(float dt, Vec3 currentEntityPos) {
        if (usePhysics && physicsInitialized) {
            updatePhysicsWithCollision(dt, currentEntityPos);
        } else {
            updateKinematic(dt);
        }
    }

    /**
     * ✅ 하위 호환용 (위치 없이 호출)
     */
    public void update(float dt) {
        update(dt, initialAnchorPosition != null ? initialAnchorPosition : Vec3.ZERO);
    }

    /**
     * ✅ 수정: 블록 충돌 업데이트 시 currentEntityPos 사용
     */
    private void updatePhysicsWithCollision(float dt, Vec3 currentEntityPos) {
        if (blockCollisionManager != null && currentLevel != null && currentEntityPos != null) {
            blockCollisionManager.updateCollisionArea(
                    currentLevel,
                    currentEntityPos.x,
                    currentEntityPos.y,
                    currentEntityPos.z
            );
        }

        int subSteps = Math.max(1, physicsSubSteps);
        float subDt = dt / subSteps;

        for (int i = 0; i < subSteps; i++) {
            if (motorsEnabled) {
                applyJointControls();
            }
            physics.step(subDt);
        }

        syncJointStates();
    }

    private void applyJointControls() {
        for (Map.Entry<String, Object> entry : odeJoints.entrySet()) {
            String jointName = entry.getKey();
            Object odeJoint = entry.getValue();
            URDFJoint urdfJoint = joints.get(jointName);

            if (urdfJoint == null || !urdfJoint.isMovable()) continue;

            float targetPos = target.getOrDefault(jointName, 0f);
            float targetVel = targetVelocities.getOrDefault(jointName, 0f);

            applyJointControl(odeJoint, urdfJoint, targetPos, targetVel);
        }
    }

    // ========================================================================
    // ODE4J 초기화
    // ========================================================================

    private void initializeODE4JClasses() throws Exception {
        ClassLoader cl = physics.getClassLoader();
        String base = "com.kAIS.ode4j.ode.";

        odeHelperClass    = cl.loadClass(base + "OdeHelper");
        dWorldClass       = cl.loadClass(base + "DWorld");
        dBodyClass        = cl.loadClass(base + "DBody");
        dMassClass        = cl.loadClass(base + "DMass");
        dGeomClass        = cl.loadClass(base + "DGeom");
        dHingeJointClass  = cl.loadClass(base + "DHingeJoint");
        dSliderJointClass = cl.loadClass(base + "DSliderJoint");
        dFixedJointClass  = cl.loadClass(base + "DFixedJoint");
        dJointGroupClass  = cl.loadClass(base + "DJointGroup");

        createMassMethod = odeHelperClass.getMethod("createMass");
    }

    private void detectODE4JVersion() {
        try {
            bodySetMassMethod = dBodyClass.getMethod("setMass", dMassClass);
            odeVersion = ODE4JVersion.V03X;
            return;
        } catch (NoSuchMethodException e) {
            // ignore
        }

        try {
            physics.getClassLoader().loadClass("com.kAIS.ode4j.ode.internal.DxMass");
            odeVersion = ODE4JVersion.V05X;
            return;
        } catch (ClassNotFoundException e) {
            // ignore
        }

        odeVersion = ODE4JVersion.V05X;
        logger.info("Assuming ODE4J 0.5.x");
    }

    private void findMassSetMethods() {
        try {
            massSetBoxMethod = dMassClass.getMethod("setBox",
                    double.class, double.class, double.class, double.class);
        } catch (NoSuchMethodException e) {
            try {
                massSetBoxMethod = dMassClass.getMethod("setBoxTotal",
                        double.class, double.class, double.class, double.class);
            } catch (NoSuchMethodException e2) { }
        }

        try {
            massSetSphereMethod = dMassClass.getMethod("setSphere", double.class, double.class);
        } catch (NoSuchMethodException e) { }
    }

    // ========================================================================
    // 물리 모델 빌드
    // ========================================================================

    private void buildPhysicsModel() throws Exception {
        if (urdfModel == null) {
            throw new IllegalStateException("URDFModel is null");
        }

        Object world = physics.getWorld();
        findMassSetMethods();

        // 링크 → body + geom
        for (URDFLink link : urdfModel.links) {
            if (link == null || isFixedLink(link)) continue;

            Object body = createBodyForLink(link, world);
            if (body != null) {
                bodies.put(link.name, body);

                float radius = estimateLinkRadius(link);
                linkRadii.put(link.name, radius);
            }
        }

        // 조인트 생성
        for (URDFJoint joint : joints.values()) {
            if (!joint.isMovable()) continue;

            Object odeJoint = createODEJoint(joint, world);
            if (odeJoint != null) {
                odeJoints.put(joint.name, odeJoint);
            }
        }

        // 루트 바디 이름 보정
        if (rootBodyLinkName != null && !bodies.containsKey(rootBodyLinkName)) {
            rootBodyLinkName = bodies.keySet().stream().findFirst().orElse(null);
        }

        logger.info("Physics model: {} bodies, {} geoms, {} joints (rootBody = {})",
                bodies.size(), geoms.size(), odeJoints.size(), rootBodyLinkName);

        if (bodies.isEmpty()) {
            throw new IllegalStateException("No ODE bodies created");
        }
    }

    private float estimateLinkRadius(URDFLink link) {
        float defaultRadius = 0.1f;

        if (link.visual != null && link.visual.geometry != null) {
            URDFLink.Geometry g = link.visual.geometry;
            switch (g.type) {
                case BOX:
                    if (g.boxSize != null) {
                        return (float) Math.sqrt(
                                g.boxSize.x * g.boxSize.x +
                                        g.boxSize.y * g.boxSize.y +
                                        g.boxSize.z * g.boxSize.z
                        ) * 0.5f;
                    }
                    break;
                case SPHERE:
                    return g.sphereRadius;
                case CYLINDER:
                    return Math.max(g.cylinderRadius, g.cylinderLength * 0.5f);
                case MESH:
                    if (g.scale != null) {
                        return Math.max(g.scale.x, Math.max(g.scale.y, g.scale.z)) * 0.1f;
                    }
                    break;
            }
        }

        String name = link.name.toLowerCase();
        if (name.contains("torso") || name.contains("body") || name.contains("chest")) {
            return 0.25f;
        } else if (name.contains("head")) {
            return 0.12f;
        } else if (name.contains("arm") || name.contains("leg")) {
            return 0.08f;
        } else if (name.contains("hand") || name.contains("foot")) {
            return 0.06f;
        }

        return defaultRadius;
    }

    // ========================================================================
    // Body + Geom 생성
    // ========================================================================

    private Object createBodyForLink(URDFLink link, Object world) {
        try {
            Object body = physics.createBody();
            if (body == null) return null;

            // 질량 + geom 정보
            GeometryInfo geomInfo = setDefaultMass(body, link);

            // 초기 위치 (URDF 기반, 이후 anchorPhysicsToWorld 에서 월드 위치에 정렬)
            if (link.visual != null && link.visual.origin != null) {
                physics.setBodyPosition(body,
                        link.visual.origin.xyz.x * physicsScale,
                        link.visual.origin.xyz.y * physicsScale,
                        link.visual.origin.xyz.z * physicsScale);
            }

            Object geom = createGeomForLink(link, geomInfo);
            if (geom != null) {
                double[] bp = physics.getBodyPosition(body);
                physics.setGeomPosition(geom, bp[0], bp[1], bp[2]);

                physics.setGeomBody(geom, body);

                try {
                    physics.setGeomOffsetPosition(geom, 0, 0, 0);
                } catch (Exception e) {
                    logger.debug("setGeomOffsetPosition not available for link: {}", link.name);
                }

                geoms.put(link.name, geom);
                physics.registerDynamicGeom(geom);

                logger.debug("Created geom for link: {} (type: {}) with zero offset",
                        link.name, geomInfo.type);
            } else {
                logger.warn("Failed to create geom for link: {}", link.name);
            }

            return body;
        } catch (Exception e) {
            logger.error("Failed to create body for {}", link.name, e);
            return null;
        }
    }

    private static class GeometryInfo {
        String type;
        double lx, ly, lz;  // box dimensions
        double radius;      // sphere/cylinder radius
        double height;      // cylinder height

        GeometryInfo(String type) {
            this.type = type;
        }
    }

    private Object createGeomForLink(URDFLink link, GeometryInfo geomInfo) {
        try {
            Object geom;

            switch (geomInfo.type) {
                case "box":
                    geom = physics.createBoxGeom(
                            geomInfo.lx * physicsScale,
                            geomInfo.ly * physicsScale,
                            geomInfo.lz * physicsScale
                    );
                    break;

                case "sphere":
                    geom = physics.createSphereGeom(geomInfo.radius * physicsScale);
                    break;

                case "cylinder":
                    geom = physics.createCylinderGeom(
                            geomInfo.radius * physicsScale,
                            geomInfo.height * physicsScale
                    );
                    break;

                default:
                    float radius = estimateLinkRadius(link);
                    geom = physics.createSphereGeom(radius * physicsScale);
                    break;
            }

            return geom;

        } catch (Exception e) {
            logger.error("Failed to create geom for {}", link.name, e);
            return null;
        }
    }

    // ========================================================================
    // 질량 설정
    // ========================================================================

    private GeometryInfo setDefaultMass(Object body, URDFLink link) {
        GeometryInfo geomInfo = new GeometryInfo("sphere");
        geomInfo.radius = 0.1;

        try {
            Object mass = createMassMethod.invoke(null);
            double density = 500.0;

            double lx = 0.1, ly = 0.1, lz = 0.1;

            if (link != null && link.visual != null && link.visual.geometry != null) {
                URDFLink.Geometry geom = link.visual.geometry;
                if (massSetBoxMethod == null && massSetSphereMethod == null) {
                    findMassSetMethods();
                }

                switch (geom.type) {
                    case BOX:
                        if (geom.boxSize != null) {
                            lx = Math.max(0.05, Math.abs(geom.boxSize.x));
                            ly = Math.max(0.05, Math.abs(geom.boxSize.y));
                            lz = Math.max(0.05, Math.abs(geom.boxSize.z));

                            geomInfo.type = "box";
                            geomInfo.lx = lx;
                            geomInfo.ly = ly;
                            geomInfo.lz = lz;
                        }
                        break;

                    case SPHERE:
                        if (geom.sphereRadius > 0) {
                            double r = Math.max(0.03, Math.abs(geom.sphereRadius));

                            geomInfo.type = "sphere";
                            geomInfo.radius = r;

                            if (massSetSphereMethod != null) {
                                massSetSphereMethod.invoke(mass, density, r * physicsScale);
                                applyMassToBody(body, mass);
                                return geomInfo;
                            } else {
                                lx = ly = lz = r * 2.0;
                            }
                        }
                        break;

                    case CYLINDER:
                        if (geom.cylinderRadius > 0 && geom.cylinderLength > 0) {
                            double r = Math.max(0.03, Math.abs(geom.cylinderRadius));
                            double h = Math.max(0.05, Math.abs(geom.cylinderLength));

                            geomInfo.type = "cylinder";
                            geomInfo.radius = r;
                            geomInfo.height = h;

                            lx = ly = r * 2.0;
                            lz = h;
                        }
                        break;

                    case MESH:
                        if (geom.scale != null) {
                            double s = Math.max(
                                    Math.max(Math.abs(geom.scale.x), Math.abs(geom.scale.y)),
                                    Math.abs(geom.scale.z)
                            );
                            s = Math.max(0.05, s);
                            lx = ly = lz = s * 0.5;
                        }
                        break;
                }
            }

            if (massSetBoxMethod != null) {
                massSetBoxMethod.invoke(mass,
                        density,
                        lx * physicsScale,
                        ly * physicsScale,
                        lz * physicsScale);
            } else if (massSetSphereMethod != null) {
                double r = Math.max(0.05, Math.max(lx, Math.max(ly, lz)) * 0.5);
                massSetSphereMethod.invoke(mass, density, r * physicsScale);
            } else {
                double fallbackMass = 1.0;
                setMassValue(mass, fallbackMass);
            }

            if (link != null && link.inertial != null && link.inertial.mass != null) {
                float mv = link.inertial.mass.value;
                if (mv > 0 && Float.isFinite(mv)) {
                    setMassValue(mass, mv);
                }
            }

            applyMassToBody(body, mass);

        } catch (Exception e) {
            logger.warn("Failed to set mass for {}", link != null ? link.name : "unknown", e);
        }

        return geomInfo;
    }

    private void setMassValue(Object mass, double value) {
        try {
            Method setMass = dMassClass.getMethod("setMass", double.class);
            setMass.invoke(mass, value);
            return;
        } catch (Exception e) { }

        try {
            Method adjust = dMassClass.getMethod("adjust", double.class);
            adjust.invoke(mass, value);
        } catch (Exception e) { }
    }

    private void applyMassToBody(Object body, Object mass) throws Exception {
        if (bodySetMassMethod != null) {
            bodySetMassMethod.invoke(body, mass);
            return;
        }

        try {
            for (Method m : body.getClass().getMethods()) {
                if (m.getName().equals("setMass") && m.getParameterCount() == 1) {
                    if (m.getParameterTypes()[0].isAssignableFrom(mass.getClass())) {
                        m.invoke(body, mass);
                        return;
                    }
                }
            }
        } catch (Exception e) { }

        logger.warn("Could not apply mass to body");
    }

    // ========================================================================
    // 조인트 생성
    // ========================================================================

    private Object createODEJoint(URDFJoint joint, Object world) {
        try {
            Object parentBody = bodies.get(joint.parentLinkName);
            Object childBody  = bodies.get(joint.childLinkName);

            if (parentBody == null && childBody == null) {
                return null;
            }

            switch (joint.type) {
                case REVOLUTE:
                case CONTINUOUS:
                    return createHingeJoint(joint, world, parentBody, childBody);
                case PRISMATIC:
                    return createSliderJoint(joint, world, parentBody, childBody);
                case FIXED:
                    return createFixedJoint(joint, world, parentBody, childBody);
                default:
                    return null;
            }
        } catch (Exception e) {
            logger.error("Failed to create joint {}", joint.name, e);
            return null;
        }
    }

    private Object createHingeJoint(URDFJoint joint, Object world,
                                    Object parentBody, Object childBody) throws Exception {
        Method createHinge = odeHelperClass.getMethod(
                "createHingeJoint", dWorldClass, dJointGroupClass);
        Object odeJoint = createHinge.invoke(null, world, null);

        Method attach = dHingeJointClass.getMethod("attach", dBodyClass, dBodyClass);
        attach.invoke(odeJoint, childBody, parentBody);

        double[] axis = getJointAxis(joint);
        double len = Math.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
        if (len < 1e-6) {
            axis[0] = 0.0;
            axis[1] = 0.0;
            axis[2] = 1.0;
        } else {
            axis[0] /= len;
            axis[1] /= len;
            axis[2] /= len;
        }

        Method setAxis = dHingeJointClass.getMethod(
                "setAxis", double.class, double.class, double.class);
        setAxis.invoke(odeJoint, axis[0], axis[1], axis[2]);

        // 앵커 설정 (생성 시점에도 한 번은 맞춰둠)
        try {
            Method setAnchor = dHingeJointClass.getMethod(
                    "setAnchor", double.class, double.class, double.class);

            double anchorX = 0.0;
            double anchorY = 0.0;
            double anchorZ = 0.0;
            boolean baseFromBody = false;

            if (physics != null) {
                if (childBody != null) {
                    double[] p = physics.getBodyPosition(childBody);
                    if (p != null && p.length >= 3) {
                        anchorX = p[0];
                        anchorY = p[1];
                        anchorZ = p[2];
                        baseFromBody = true;
                    }
                }
                if (!baseFromBody && parentBody != null) {
                    double[] p = physics.getBodyPosition(parentBody);
                    if (p != null && p.length >= 3) {
                        anchorX = p[0];
                        anchorY = p[1];
                        anchorZ = p[2];
                        baseFromBody = true;
                    }
                }
            }

            if (joint.origin != null && joint.origin.xyz != null) {
                anchorX += joint.origin.xyz.x * physicsScale;
                anchorY += joint.origin.xyz.y * physicsScale;
                anchorZ += joint.origin.xyz.z * physicsScale;
            }

            setAnchor.invoke(odeJoint, anchorX, anchorY, anchorZ);
        } catch (Exception e) {
            // ignore
        }

        if (joint.type == URDFJoint.JointType.REVOLUTE &&
                joint.limit != null && joint.limit.hasLimits()) {
            setHingeJointLimits(odeJoint, joint.limit.lower, joint.limit.upper);
        }

        return odeJoint;
    }

    private void setHingeJointLimits(Object odeJoint, float lower, float upper) {
        try {
            Method setLo = dHingeJointClass.getMethod("setParamLoStop", double.class);
            Method setHi = dHingeJointClass.getMethod("setParamHiStop", double.class);
            setLo.invoke(odeJoint, (double) lower);
            setHi.invoke(odeJoint, (double) upper);
        } catch (Exception e) { }
    }

    private Object createSliderJoint(URDFJoint joint, Object world,
                                     Object parentBody, Object childBody) throws Exception {
        Method createSlider = odeHelperClass.getMethod(
                "createSliderJoint", dWorldClass, dJointGroupClass);
        Object odeJoint = createSlider.invoke(null, world, null);

        Method attach = dSliderJointClass.getMethod("attach", dBodyClass, dBodyClass);
        attach.invoke(odeJoint, childBody, parentBody);

        double[] axis = getJointAxis(joint);
        Method setAxis = dSliderJointClass.getMethod(
                "setAxis", double.class, double.class, double.class);
        setAxis.invoke(odeJoint, axis[0], axis[1], axis[2]);

        return odeJoint;
    }

    private Object createFixedJoint(URDFJoint joint, Object world,
                                    Object parentBody, Object childBody) throws Exception {
        Method createFixed = odeHelperClass.getMethod(
                "createFixedJoint", dWorldClass, dJointGroupClass);
        Object odeJoint = createFixed.invoke(null, world, null);

        Method attach = dFixedJointClass.getMethod("attach", dBodyClass, dBodyClass);
        attach.invoke(odeJoint, childBody, parentBody);

        Method setFixed = dFixedJointClass.getMethod("setFixed");
        setFixed.invoke(odeJoint);

        return odeJoint;
    }

    private double[] getJointAxis(URDFJoint joint) {
        if (joint.axis != null && joint.axis.xyz != null) {
            return new double[]{joint.axis.xyz.x, joint.axis.xyz.y, joint.axis.xyz.z};
        }
        return new double[]{0, 0, 1};
    }

    // ========================================================================
    // 관절 제어 및 동기화
    // ========================================================================

    private void applyJointControl(Object odeJoint, URDFJoint urdfJoint,
                                   float targetPos, float targetVel) {
        try {
            if (urdfJoint.type == URDFJoint.JointType.REVOLUTE ||
                    urdfJoint.type == URDFJoint.JointType.CONTINUOUS) {

                float currentPos = getHingeAngle(odeJoint);
                float currentVel = getHingeAngleRate(odeJoint);

                float posError = targetPos - currentPos;
                if (urdfJoint.type == URDFJoint.JointType.CONTINUOUS) {
                    posError = wrapToPi(posError);
                }

                float torque = physicsKp * posError + physicsKd * (targetVel - currentVel);
                float limit = (urdfJoint.limit != null && urdfJoint.limit.effort > 0)
                        ? urdfJoint.limit.effort : maxTorque;
                torque = Mth.clamp(torque, -limit, limit);

                addHingeTorque(odeJoint, torque);

            } else if (urdfJoint.type == URDFJoint.JointType.PRISMATIC) {

                float currentPos = getSliderPosition(odeJoint);
                float currentVel = getSliderPositionRate(odeJoint);

                float posError = targetPos - currentPos;
                float force = physicsKp * posError + physicsKd * (targetVel - currentVel);
                float limit = (urdfJoint.limit != null && urdfJoint.limit.effort > 0)
                        ? urdfJoint.limit.effort : maxForce;
                force = Mth.clamp(force, -limit, limit);

                addSliderForce(odeJoint, force);
            }
        } catch (Exception e) { }
    }

    private float getHingeAngle(Object joint) {
        try {
            Method m = dHingeJointClass.getMethod("getAngle");
            return ((Number) m.invoke(joint)).floatValue();
        } catch (Exception e) {
            return 0f;
        }
    }

    private float getHingeAngleRate(Object joint) {
        try {
            Method m = dHingeJointClass.getMethod("getAngleRate");
            return ((Number) m.invoke(joint)).floatValue();
        } catch (Exception e) {
            return 0f;
        }
    }

    private void addHingeTorque(Object joint, float torque) {
        try {
            Method m = dHingeJointClass.getMethod("addTorque", double.class);
            m.invoke(joint, (double) torque);
        } catch (Exception e) { }
    }

    private float getSliderPosition(Object joint) {
        try {
            Method m = dSliderJointClass.getMethod("getPosition");
            return ((Number) m.invoke(joint)).floatValue();
        } catch (Exception e) {
            return 0f;
        }
    }

    private float getSliderPositionRate(Object joint) {
        try {
            Method m = dSliderJointClass.getMethod("getPositionRate");
            return ((Number) m.invoke(joint)).floatValue();
        } catch (Exception e) {
            return 0f;
        }
    }

    private void addSliderForce(Object joint, float force) {
        try {
            Method m = dSliderJointClass.getMethod("addForce", double.class);
            m.invoke(joint, (double) force);
        } catch (Exception e) { }
    }

    private void syncJointStates() {
        for (Map.Entry<String, Object> entry : odeJoints.entrySet()) {
            String jointName = entry.getKey();
            Object odeJoint = entry.getValue();
            URDFJoint urdfJoint = joints.get(jointName);

            if (urdfJoint == null) continue;

            if (urdfJoint.type == URDFJoint.JointType.REVOLUTE ||
                    urdfJoint.type == URDFJoint.JointType.CONTINUOUS) {
                urdfJoint.currentPosition = getHingeAngle(odeJoint);
                urdfJoint.currentVelocity = getHingeAngleRate(odeJoint);
            } else if (urdfJoint.type == URDFJoint.JointType.PRISMATIC) {
                urdfJoint.currentPosition = getSliderPosition(odeJoint);
                urdfJoint.currentVelocity = getSliderPositionRate(odeJoint);
            }
        }
    }

    // ========================================================================
    // 키네마틱 모드
    // ========================================================================

    private void updateKinematic(float dt) {
        for (URDFJoint j : joints.values()) {
            if (!j.isMovable()) continue;

            float tgt = target.getOrDefault(j.name, j.currentPosition);
            float pos = j.currentPosition;
            float vel = j.currentVelocity;

            if (j.type == URDFJoint.JointType.CONTINUOUS) {
                float d = (float) Math.atan2(Math.sin(tgt - pos), Math.cos(tgt - pos));
                tgt = pos + d;
            }

            float err = tgt - pos;
            float acc = kp * err - kd * vel;

            float maxVel = (j.limit != null && j.limit.velocity > 0f)
                    ? j.limit.velocity : defaultMaxVel;

            acc = Mth.clamp(acc, -defaultMaxAcc, defaultMaxAcc);
            vel += acc * dt;
            vel = Mth.clamp(vel, -maxVel, maxVel);
            pos += vel * dt;

            if (j.limit != null && j.limit.hasLimits()) {
                pos = Mth.clamp(pos, j.limit.lower, j.limit.upper);
                if (pos == j.limit.lower || pos == j.limit.upper) vel = 0f;
            }

            if (j.type == URDFJoint.JointType.CONTINUOUS) {
                pos = wrapToPi(pos);
            }

            j.currentVelocity = vel;
            j.currentPosition = pos;
        }
    }

    // ========================================================================
    // 루트 링크 / 루트 바디 위치 (렌더러/엔티티용)
    // ========================================================================

    private Object getRootBody() {
        if (bodies.isEmpty()) return null;

        if (rootBodyLinkName != null) {
            Object root = bodies.get(rootBodyLinkName);
            if (root != null) return root;
        }
        return bodies.values().iterator().next();
    }

    /**
     * ✅ 수정: baseWorldPos를 인자로 받음
     * 로봇 루트 바디의 "현재 엔티티 기준 로컬 오프셋" 반환
     */
    public float[] getRootLinkLocalPosition(Vec3 baseWorldPos) {
        if (!usePhysics || !physicsInitialized || physics == null || bodies.isEmpty()) {
            return new float[]{0, 0, 0};
        }

        Object root = getRootBody();
        if (root == null) return new float[]{0, 0, 0};

        double[] pos = physics.getBodyPosition(root);
        if (pos == null || pos.length < 3) {
            return new float[]{0, 0, 0};
        }

        Vec3 base = (baseWorldPos != null) ? baseWorldPos : Vec3.ZERO;

        return new float[]{
                (float) (pos[0] - base.x),
                (float) (pos[1] - base.y),
                (float) (pos[2] - base.z)
        };
    }

    /**
     * ✅ 하위 호환용 (deprecated)
     */
    @Deprecated
    public float[] getRootLinkLocalPosition() {
        return getRootLinkLocalPosition(initialAnchorPosition != null ? initialAnchorPosition : Vec3.ZERO);
    }

    /**
     * ✅ 수정: 링크의 월드 위치 반환 (baseWorldPos 불필요, 바로 월드 좌표 반환)
     */
    public float[] getLinkWorldPosition(String linkName) {
        if (!usePhysics || !physicsInitialized || physics == null) {
            Vec3 fallback = initialAnchorPosition != null ? initialAnchorPosition : Vec3.ZERO;
            return new float[]{
                    (float) fallback.x,
                    (float) fallback.y,
                    (float) fallback.z
            };
        }

        Object body = bodies.get(linkName);
        if (body != null) {
            double[] pos = physics.getBodyPosition(body);
            if (pos != null && pos.length >= 3) {
                return new float[]{(float) pos[0], (float) pos[1], (float) pos[2]};
            }
        }

        Vec3 fallback = initialAnchorPosition != null ? initialAnchorPosition : Vec3.ZERO;
        return new float[]{
                (float) fallback.x,
                (float) fallback.y,
                (float) fallback.z
        };
    }

    /**
     * ✅ 수정: 모든 바디 중 가장 낮은 Y 좌표 (월드 좌표계)
     */
    public float getApproxBaseHeightWorldY() {
        if (!usePhysics || !physicsInitialized || bodies.isEmpty() || physics == null) {
            Vec3 fallback = initialAnchorPosition != null ? initialAnchorPosition : Vec3.ZERO;
            return (float) fallback.y;
        }

        double minBottomWorldY = Double.POSITIVE_INFINITY;

        for (Map.Entry<String, Object> entry : bodies.entrySet()) {
            Object body = entry.getValue();
            if (body == null) continue;

            double[] pos = physics.getBodyPosition(body);
            if (pos == null || pos.length < 2) continue;

            float radius = getLinkRadius(entry.getKey());
            double bottomY = pos[1] - radius;

            if (bottomY < minBottomWorldY) {
                minBottomWorldY = bottomY;
            }
        }

        if (!Double.isFinite(minBottomWorldY)) {
            Object firstBody = bodies.values().iterator().next();
            double[] pos = physics.getBodyPosition(firstBody);
            if (pos == null || pos.length < 2) {
                Vec3 fallback = initialAnchorPosition != null ? initialAnchorPosition : Vec3.ZERO;
                return (float) fallback.y;
            }
            minBottomWorldY = pos[1];
        }

        return (float) minBottomWorldY;
    }

    /**
     * ✅ 루트 바디의 월드 위치 반환
     */
    public double[] getRootBodyWorldPosition() {
        if (!usePhysics || !physicsInitialized || bodies.isEmpty() || physics == null) {
            Vec3 fallback = initialAnchorPosition != null ? initialAnchorPosition : Vec3.ZERO;
            return new double[]{fallback.x, fallback.y, fallback.z};
        }

        Object root = getRootBody();
        if (root == null) {
            Vec3 fallback = initialAnchorPosition != null ? initialAnchorPosition : Vec3.ZERO;
            return new double[]{fallback.x, fallback.y, fallback.z};
        }

        double[] pos = physics.getBodyPosition(root);
        if (pos == null || pos.length < 3) {
            Vec3 fallback = initialAnchorPosition != null ? initialAnchorPosition : Vec3.ZERO;
            return new double[]{fallback.x, fallback.y, fallback.z};
        }

        return new double[]{pos[0], pos[1], pos[2]};
    }

    // ========================================================================
    // 공개 API
    // ========================================================================

    public void setTarget(String name, float value) {
        URDFJoint j = joints.get(name);
        if (j == null) return;

        if (j.type == URDFJoint.JointType.CONTINUOUS) {
            value = wrapToPi(value);
        }
        if (j.limit != null && j.limit.hasLimits()) {
            value = Mth.clamp(value, j.limit.lower, j.limit.upper);
        }
        target.put(name, value);
    }

    public void setTargets(Map<String, Float> targets) {
        for (var e : targets.entrySet()) {
            setTarget(e.getKey(), e.getValue());
        }
    }

    public float getTarget(String name) {
        return target.getOrDefault(name, 0f);
    }

    public void setTargetVelocity(String name, float velocity) {
        targetVelocities.put(name, velocity);
    }

    public boolean hasJoint(String name) {
        return joints.containsKey(name);
    }

    public String findJointIgnoreCase(String name) {
        if (name == null) return null;
        for (String key : joints.keySet()) {
            if (key.equalsIgnoreCase(name)) return key;
        }
        return null;
    }

    public Set<String> getJointNameSet() {
        return new HashSet<>(joints.keySet());
    }

    public List<String> getMovableJointNames() {
        List<String> names = new ArrayList<>();
        for (URDFJoint j : joints.values()) {
            if (j.isMovable()) names.add(j.name);
        }
        return names;
    }

    public float[] getJointLimits(String name) {
        URDFJoint j = joints.get(name);
        if (j != null && j.limit != null) {
            return new float[]{j.limit.lower, j.limit.upper};
        }
        return new float[]{(float) -Math.PI, (float) Math.PI};
    }

    public float getJointPosition(String name) {
        URDFJoint j = joints.get(name);
        return j != null ? j.currentPosition : 0f;
    }

    public float getJointVelocity(String name) {
        URDFJoint j = joints.get(name);
        return j != null ? j.currentVelocity : 0f;
    }

    public Map<String, Float> getAllJointPositions() {
        Map<String, Float> positions = new HashMap<>();
        for (URDFJoint j : joints.values()) {
            if (j.isMovable()) {
                positions.put(j.name, j.currentPosition);
            }
        }
        return positions;
    }

    public void setPreviewPosition(String name, float value) {
        URDFJoint j = joints.get(name);
        if (j == null) return;
        if (j.limit != null && j.limit.hasLimits()) {
            value = Mth.clamp(value, j.limit.lower, j.limit.upper);
        }
        j.currentPosition = value;
        target.put(j.name, value);
    }

    public boolean isUsingPhysics() {
        return usePhysics && physicsInitialized;
    }

    public void setGains(float kp, float kd) {
        this.kp = kp;
        this.kd = kd;
        this.physicsKp = kp * 5f;
        this.physicsKd = kd * 3f;
    }

    public void setLimits(float maxVel, float maxAcc) {
        this.defaultMaxVel = maxVel;
        this.defaultMaxAcc = maxAcc;
    }

    public void setPhysicsGains(float kp, float kd) {
        this.physicsKp = kp;
        this.physicsKd = kd;
    }

    public void setEffortLimits(float maxTorque, float maxForce) {
        this.maxTorque = maxTorque;
        this.maxForce = maxForce;
    }

    public void setPhysicsScale(float scale) {
        this.physicsScale = scale;
    }

    public void setMotorsEnabled(boolean enabled) {
        this.motorsEnabled = enabled;
    }

    public boolean isMotorsEnabled() {
        return motorsEnabled;
    }

    public void setPhysicsSubSteps(int subSteps) {
        this.physicsSubSteps = Math.max(1, Math.min(subSteps, 10));
    }

    public void applyExternalForce(String linkName, float fx, float fy, float fz) {
        if (!usePhysics) return;
        Object body = bodies.get(linkName);
        if (body != null && physics != null) {
            physics.addForce(body, fx, fy, fz);
        }
    }

    public void applyExternalTorque(String linkName, float tx, float ty, float tz) {
        if (!usePhysics) return;
        Object body = bodies.get(linkName);
        if (body != null && physics != null) {
            physics.addTorque(body, tx, ty, tz);
        }
    }

    public void setGravity(float x, float y, float z) {
        if (physics != null) {
            physics.setGravity(x, y, z);
        }
    }

    // ========================================================================
    // 정리
    // ========================================================================

    public void cleanup() {
        if (blockCollisionManager != null) {
            blockCollisionManager.cleanup();
            blockCollisionManager = null;
        }

        bodies.clear();
        geoms.clear();
        odeJoints.clear();
        linkRadii.clear();
        physicsInitialized = false;
        worldAnchored = false;
        initialAnchorPosition = null;

        logger.info("URDFSimpleController cleaned up");
    }

    public void resetPhysics() {
        for (URDFJoint j : joints.values()) {
            j.currentPosition = 0f;
            j.currentVelocity = 0f;
            target.put(j.name, 0f);
            targetVelocities.put(j.name, 0f);
        }

        for (Object body : bodies.values()) {
            if (body != null && physics != null) {
                physics.setBodyLinearVel(body, 0, 0, 0);
                physics.setBodyAngularVel(body, 0, 0, 0);
            }
        }

        worldAnchored = false;
        initialAnchorPosition = null;

        logger.info("Physics state reset");
    }

    // ========================================================================
    // 유틸리티
    // ========================================================================

    private float getLinkRadius(String linkName) {
        Float r = linkRadii.get(linkName);
        if (r == null || r <= 0f || !Float.isFinite(r)) {
            r = 0.1f;
        }
        return r * physicsScale;
    }

    private boolean isFixedLink(URDFLink link) {
        return "world".equals(link.name);
    }

    private static float wrapToPi(float a) {
        float twoPi = (float) (Math.PI * 2.0);
        a = a % twoPi;
        if (a > Math.PI) a -= twoPi;
        if (a < -Math.PI) a += twoPi;
        return a;
    }
}
