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
 * - ODE World = 엔티티 로컬 좌표계
 * - 루트 링크 = ODE (0, 0, 0) 근처 (고정 또는 자유)
 * - worldPosition = 마인크래프트 엔티티 절대 위치
 * - 블록 충돌 시: worldPosition + bodyLocalPos = 절대 위치
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

    // 너무 과격하지 않게 초기값 낮춤
    private float physicsKp = 50f;
    private float physicsKd = 5f;
    private float maxTorque = 20f;
    private float maxForce = 100f;

    // ========== 블록 충돌 ==========
    private BlockCollisionManager blockCollisionManager;
    private Level currentLevel;
    private Vec3 worldPosition = Vec3.ZERO;

    // ========== 스케일 (URDF 단위 → 물리 단위) ==========
    private float physicsScale = 1.0f;

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
    }

    // ========================================================================
    // 월드 컨텍스트 (렌더러에서 호출)
    // ========================================================================

    public void setWorldContext(Level level, Vec3 worldPos) {
        this.currentLevel = level;
        this.worldPosition = worldPos != null ? worldPos : Vec3.ZERO;
    }

    public Vec3 getWorldPosition() {
        return worldPosition;
    }

    // ========================================================================
    // 메인 업데이트
    // ========================================================================

    public void update(float dt) {
        if (usePhysics && physicsInitialized) {
            updatePhysicsWithCollision(dt);
        } else {
            updateKinematic(dt);
        }
    }

    /**
     * 물리 + 블록 충돌 업데이트
     * BlockCollisionManager가 static geometry를 관리하고,
     * PhysicsManager.step 내부에서 space.collide() + contact 처리가 수행됨.
     */
    private void updatePhysicsWithCollision(float dt) {
        // 1. 블록 static geom 업데이트
        if (blockCollisionManager != null && currentLevel != null) {
            blockCollisionManager.updateCollisionArea(
                    currentLevel,
                    worldPosition.x,
                    worldPosition.y,
                    worldPosition.z
            );
        }

        // 2. 조인트 제어 (토크/힘 계산)
        applyJointControls();

        // 3. ODE 스텝 (중력 + 조인트 + 블록 충돌 전부)
        physics.step(dt);

        // 4. URDF 조인트 상태 동기화
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

        logger.info("Physics model: {} bodies, {} geoms, {} joints",
                bodies.size(), geoms.size(), odeJoints.size());

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

            // 초기 위치 (엔티티 로컬)
            if (link.visual != null && link.visual.origin != null) {
                physics.setBodyPosition(body,
                        link.visual.origin.xyz.x * physicsScale,
                        link.visual.origin.xyz.y * physicsScale,
                        link.visual.origin.xyz.z * physicsScale);
            }

            // Geom 생성
            Object geom = createGeomForLink(link, geomInfo);
            if (geom != null) {
                physics.setGeomBody(geom, body);
                geoms.put(link.name, geom);

                // 동적 geom 으로 등록 (self-collision 제어용)
                physics.registerDynamicGeom(geom);

                logger.debug("Created geom for link: {} (type: {})",
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

    // ========================================================================
    // Geom 생성 헬퍼
    // ========================================================================

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
            Object geom = null;

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
        Method setAxis = dHingeJointClass.getMethod(
                "setAxis", double.class, double.class, double.class);
        setAxis.invoke(odeJoint, axis[0], axis[1], axis[2]);

        if (joint.origin != null) {
            try {
                Method setAnchor = dHingeJointClass.getMethod(
                        "setAnchor", double.class, double.class, double.class);
                setAnchor.invoke(odeJoint,
                        joint.origin.xyz.x * physicsScale,
                        joint.origin.xyz.y * physicsScale,
                        joint.origin.xyz.z * physicsScale);
            } catch (Exception e) { }
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
    // 루트 링크 위치 (렌더러용)
    // ========================================================================

    public float[] getRootLinkLocalPosition() {
        if (!usePhysics) return new float[]{0, 0, 0};
        if (bodies.isEmpty()) return new float[]{0, 0, 0};

        Object firstBody = bodies.values().iterator().next();
        if (firstBody != null && physics != null) {
            double[] pos = physics.getBodyPosition(firstBody);
            return new float[]{(float) pos[0], (float) pos[1], (float) pos[2]};
        }

        return new float[]{0, 0, 0};
    }

    public float[] getLinkWorldPosition(String linkName) {
        if (!usePhysics) {
            return new float[]{
                    (float) worldPosition.x,
                    (float) worldPosition.y,
                    (float) worldPosition.z
            };
        }

        Object body = bodies.get(linkName);
        if (body != null && physics != null) {
            double[] localPos = physics.getBodyPosition(body);
            return new float[]{
                    (float) (worldPosition.x + localPos[0]),
                    (float) (worldPosition.y + localPos[1]),
                    (float) (worldPosition.z + localPos[2])
            };
        }

        return new float[]{
                (float) worldPosition.x,
                (float) worldPosition.y,
                (float) worldPosition.z
        };
    }

    // ========================================================================
    // 베이스(발바닥) 높이 & 루트 바디 월드 위치 (RL/렌더러용)
    // ========================================================================

    /**
     * 로봇에서 가장 낮은 바디의 "바닥면" 높이를 월드 좌표계 Y로 반환.
     *
     * - 발이 지면에 닿아 있으면 ≈ 0 근처
     * - 로봇이 떠 있으면 > 0
     * - 지면 아래로 파고들면 < 0
     */
    public float getApproxBaseHeightWorldY() {
        if (!usePhysics || !physicsInitialized || bodies.isEmpty() || physics == null) {
            // 물리 안 쓰는 경우에는 엔티티 기준 위치를 그대로 사용
            return (float) worldPosition.y;
        }

        double minBottomLocalY = Double.POSITIVE_INFINITY;

        for (Map.Entry<String, Object> entry : bodies.entrySet()) {
            Object body = entry.getValue();
            if (body == null) continue;

            double[] pos = physics.getBodyPosition(body);
            if (pos == null || pos.length < 2) continue;

            // linkRadii는 URDF 단위로 저장되어 있으므로 physicsScale까지 곱해 물리 단위로 맞춘다
            float radius = getLinkRadius(entry.getKey());
            double bottomY = pos[1] - radius; // 바디 중심 Y - 반경

            if (bottomY < minBottomLocalY) {
                minBottomLocalY = bottomY;
            }
        }

        if (!Double.isFinite(minBottomLocalY)) {
            // 실패 시: 첫 번째 바디 중심으로 fallback
            Object firstBody = bodies.values().iterator().next();
            double[] pos = physics.getBodyPosition(firstBody);
            if (pos == null || pos.length < 2) {
                return (float) worldPosition.y;
            }
            minBottomLocalY = pos[1];
        }

        // ODE 로컬(Y) + 엔티티 월드 Y → 절대 월드 Y
        double worldY = worldPosition.y + minBottomLocalY;
        return (float) worldY;
    }

    /**
     * "루트"로 사용할 대표 바디의 월드 위치를 반환.
     * 현재 구조상 별도의 루트 바디가 없어서, 첫 번째 동적 바디를 대표로 사용.
     */
    public double[] getRootBodyWorldPosition() {
        if (!usePhysics || !physicsInitialized || bodies.isEmpty() || physics == null) {
            return new double[]{worldPosition.x, worldPosition.y, worldPosition.z};
        }

        Object firstBody = bodies.values().iterator().next();
        double[] localPos = physics.getBodyPosition(firstBody);
        if (localPos == null || localPos.length < 3) {
            return new double[]{worldPosition.x, worldPosition.y, worldPosition.z};
        }

        return new double[]{
                worldPosition.x + localPos[0],
                worldPosition.y + localPos[1],
                worldPosition.z + localPos[2]
        };
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
        this.physicsKp = kp * 5f;  // 예전보다 스케일 낮게
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

        logger.info("Physics state reset");
    }

    // ========================================================================
    // 유틸리티
    // ========================================================================

    /**
     * linkRadii에 저장된 반경(URDF 단위)을 physicsScale까지 곱해
     * 물리 월드(ODE) 단위의 반경으로 반환.
     */
    private float getLinkRadius(String linkName) {
        Float r = linkRadii.get(linkName);
        if (r == null || r <= 0f || !Float.isFinite(r)) {
            r = 0.1f;
        }
        return r * physicsScale;
    }

    private boolean isFixedLink(URDFLink link) {
        if ("world".equals(link.name)) return true;
        if (urdfModel != null && link.name.equals(urdfModel.rootLinkName)) {
            return true;
        }
        return false;
    }

    private static float wrapToPi(float a) {
        float twoPi = (float) (Math.PI * 2.0);
        a = a % twoPi;
        if (a > Math.PI) a -= twoPi;
        if (a < -Math.PI) a += twoPi;
        return a;
    }
}
