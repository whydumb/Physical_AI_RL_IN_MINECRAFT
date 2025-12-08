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
 * URDF 컨트롤러 - 물리 + 블록 충돌 통합
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
    private final Map<String, Object> odeJoints = new HashMap<>();
    private final Map<String, Float> linkRadii = new HashMap<>();  // 링크별 충돌 반경

    private final Map<String, Float> targetVelocities = new HashMap<>();
    private float physicsKp = 500f;
    private float physicsKd = 50f;
    private float maxTorque = 100f;
    private float maxForce = 500f;

    // ========== 블록 충돌 ==========
    private BlockCollisionManager blockCollisionManager;
    private Level currentLevel;
    private Vec3 worldPosition = Vec3.ZERO;

    // ========== 스케일 (렌더러와 일치) ==========
    private float physicsScale = 1.0f;  // URDF 단위 → 물리 단위

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
                    
                    // 블록 충돌 매니저 생성
                    this.blockCollisionManager = new BlockCollisionManager();
                    
                    usePhysics = true;
                    physicsInitialized = true;
                    
                    logger.info("URDFSimpleController: PHYSICS mode");
                    logger.info("  ODE4J version: {}", odeVersion);
                    logger.info("  Bodies: {}, Joints: {}", bodies.size(), odeJoints.size());
                    logger.info("  BlockCollisionManager: active");
                    
                } catch (Exception e) {
                    logger.error("Failed to initialize physics", e);
                    usePhysics = false;
                    physicsInitialized = false;
                }
            } else {
                this.usePhysics = false;
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

    /**
     * 마인크래프트 월드 참조 및 엔티티 위치 설정
     * 매 렌더/틱마다 호출해야 함
     */
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

    private void updatePhysicsWithCollision(float dt) {
        // 1. 블록 충돌 영역 업데이트
        if (blockCollisionManager != null && currentLevel != null) {
            blockCollisionManager.updateCollisionArea(
                currentLevel,
                worldPosition.x,
                worldPosition.y,
                worldPosition.z
            );
        }

        // 2. 각 바디에 대해 블록 충돌 검사 및 응답
        if (blockCollisionManager != null && currentLevel != null) {
            for (Map.Entry<String, Object> entry : bodies.entrySet()) {
                String linkName = entry.getKey();
                Object body = entry.getValue();
                
                // 링크별 충돌 반경 가져오기
                float radius = linkRadii.getOrDefault(linkName, 0.15f);
                
                blockCollisionManager.handleBodyBlockCollision(
                    body, worldPosition, currentLevel, radius
                );
            }
        }

        // 3. 관절 제어 적용
        applyJointControls();

        // 4. 물리 스텝
        physics.step(dt);

        // 5. ODE → URDF 동기화
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

        odeHelperClass = cl.loadClass("org.ode4j.ode.OdeHelper");
        dWorldClass = cl.loadClass("org.ode4j.ode.DWorld");
        dBodyClass = cl.loadClass("org.ode4j.ode.DBody");
        dMassClass = cl.loadClass("org.ode4j.ode.DMass");
        dHingeJointClass = cl.loadClass("org.ode4j.ode.DHingeJoint");
        dSliderJointClass = cl.loadClass("org.ode4j.ode.DSliderJoint");
        dFixedJointClass = cl.loadClass("org.ode4j.ode.DFixedJoint");

        createMassMethod = odeHelperClass.getMethod("createMass");
    }

    private void detectODE4JVersion() {
        try {
            bodySetMassMethod = dBodyClass.getMethod("setMass", dMassClass);
            odeVersion = ODE4JVersion.V03X;
            return;
        } catch (NoSuchMethodException e) { }

        try {
            physics.getClassLoader().loadClass("org.ode4j.ode.internal.DxMass");
            odeVersion = ODE4JVersion.V05X;
            return;
        } catch (ClassNotFoundException e) { }

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

        // 링크 → 강체 (루트 제외)
        for (URDFLink link : urdfModel.links) {
            if (link == null || isFixedLink(link)) continue;

            Object body = createBodyForLink(link, world);
            if (body != null) {
                bodies.put(link.name, body);
                
                // 링크 크기로 충돌 반경 계산
                float radius = estimateLinkRadius(link);
                linkRadii.put(link.name, radius);
            }
        }

        // 조인트 → ODE 조인트
        for (URDFJoint joint : joints.values()) {
            if (!joint.isMovable()) continue;

            Object odeJoint = createODEJoint(joint, world);
            if (odeJoint != null) {
                odeJoints.put(joint.name, odeJoint);
            }
        }

        logger.info("Physics model: {} bodies, {} joints", bodies.size(), odeJoints.size());

        if (bodies.isEmpty()) {
            throw new IllegalStateException("No ODE bodies created");
        }
    }

    /**
     * 링크 크기로 충돌 반경 추정
     */
    private float estimateLinkRadius(URDFLink link) {
        float defaultRadius = 0.1f;
        
        if (link.visual != null && link.visual.geometry != null) {
            URDFLink.Geometry g = link.visual.geometry;
            switch (g.type) {
                case BOX:
                    if (g.boxSize != null) {
                        // 박스 대각선의 절반
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
                    // 메시는 스케일 기반 추정
                    if (g.scale != null) {
                        return Math.max(g.scale.x, Math.max(g.scale.y, g.scale.z)) * 0.1f;
                    }
                    break;
            }
        }
        
        // 이름 기반 추정
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

    private Object createBodyForLink(URDFLink link, Object world) {
        try {
            Object body = physics.createBody();
            if (body == null) return null;

            setDefaultMass(body, link);

            // 초기 위치 (URDF origin 기반, 엔티티 로컬)
            if (link.visual != null && link.visual.origin != null) {
                physics.setBodyPosition(body,
                    link.visual.origin.xyz.x * physicsScale,
                    link.visual.origin.xyz.y * physicsScale,
                    link.visual.origin.xyz.z * physicsScale);
            }

            return body;
        } catch (Exception e) {
            logger.error("Failed to create body for {}", link.name, e);
            return null;
        }
    }

    private void setDefaultMass(Object body, URDFLink link) {
        try {
            Object mass = createMassMethod.invoke(null);

            double density = 1000.0;
            double lx = 0.1, ly = 0.1, lz = 0.1;

            if (link != null && link.inertial != null && link.inertial.mass != null) {
                float massValue = link.inertial.mass.value;
                if (massValue > 0 && Float.isFinite(massValue)) {
                    setMassValue(mass, (double) massValue);
                    applyMassToBody(body, mass);
                    return;
                }
            }

            if (link != null && link.visual != null && link.visual.geometry != null) {
                URDFLink.Geometry geom = link.visual.geometry;
                if (massSetBoxMethod == null) findMassSetMethods();

                switch (geom.type) {
                    case BOX:
                        if (geom.boxSize != null) {
                            lx = geom.boxSize.x;
                            ly = geom.boxSize.y;
                            lz = geom.boxSize.z;
                        }
                        break;
                    case SPHERE:
                        lx = ly = lz = geom.sphereRadius * 2;
                        break;
                    case CYLINDER:
                        lx = lz = geom.cylinderRadius * 2;
                        ly = geom.cylinderLength;
                        break;
                    default:
                        break;
                }
            }

            if (massSetBoxMethod != null) {
                massSetBoxMethod.invoke(mass, density, lx, ly, lz);
            }

            applyMassToBody(body, mass);

        } catch (Exception e) {
            logger.warn("Failed to set mass for {}", link != null ? link.name : "unknown");
        }
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

        // 0.5.x fallback
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
            Object childBody = bodies.get(joint.childLinkName);

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
        Method createHinge = odeHelperClass.getMethod("createHingeJoint", dWorldClass);
        Object odeJoint = createHinge.invoke(null, world);

        Method attach = dHingeJointClass.getMethod("attach", dBodyClass, dBodyClass);
        attach.invoke(odeJoint, childBody, parentBody);

        double[] axis = getJointAxis(joint);
        Method setAxis = dHingeJointClass.getMethod("setAxis",
            double.class, double.class, double.class);
        setAxis.invoke(odeJoint, axis[0], axis[1], axis[2]);

        if (joint.origin != null) {
            try {
                Method setAnchor = dHingeJointClass.getMethod("setAnchor",
                    double.class, double.class, double.class);
                setAnchor.invoke(odeJoint,
                    (double) joint.origin.xyz.x * physicsScale,
                    (double) joint.origin.xyz.y * physicsScale,
                    (double) joint.origin.xyz.z * physicsScale);
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
        Method createSlider = odeHelperClass.getMethod("createSliderJoint", dWorldClass);
        Object odeJoint = createSlider.invoke(null, world);

        Method attach = dSliderJointClass.getMethod("attach", dBodyClass, dBodyClass);
        attach.invoke(odeJoint, childBody, parentBody);

        double[] axis = getJointAxis(joint);
        Method setAxis = dSliderJointClass.getMethod("setAxis",
            double.class, double.class, double.class);
        setAxis.invoke(odeJoint, axis[0], axis[1], axis[2]);

        return odeJoint;
    }

    private Object createFixedJoint(URDFJoint joint, Object world,
                                    Object parentBody, Object childBody) throws Exception {
        Method createFixed = odeHelperClass.getMethod("createFixedJoint", dWorldClass);
        Object odeJoint = createFixed.invoke(null, world);

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

    /**
     * 루트 링크의 물리 위치 반환 (엔티티 로컬 좌표)
     * 렌더러에서 로봇 전체 위치/자세 결정에 사용
     */
    public float[] getRootLinkLocalPosition() {
        if (!usePhysics) return new float[]{0, 0, 0};
        
        // 루트 링크는 body가 없으므로, 첫 번째 자식 링크의 위치 사용
        // 또는 모든 바디의 무게중심 계산
        if (bodies.isEmpty()) return new float[]{0, 0, 0};
        
        // 간단히 첫 번째 바디 위치 반환
        Object firstBody = bodies.values().iterator().next();
        if (firstBody != null && physics != null) {
            double[] pos = physics.getBodyPosition(firstBody);
            return new float[]{(float) pos[0], (float) pos[1], (float) pos[2]};
        }
        
        return new float[]{0, 0, 0};
    }

    /**
     * 특정 링크의 월드 위치 반환 (마인크래프트 절대 좌표)
     */
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
        target.put(name, value);
    }

    public boolean isUsingPhysics() {
        return usePhysics && physicsInitialized;
    }

    public void setGains(float kp, float kd) {
        this.kp = kp;
        this.kd = kd;
        this.physicsKp = kp * 15f;
        this.physicsKd = kd * 8f;
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
        // physics.enableBody(body); // 현재 PhysicsManager에 없으므로 제거
    }
}

public void applyExternalTorque(String linkName, float tx, float ty, float tz) {
    if (!usePhysics) return;
    Object body = bodies.get(linkName);
    if (body != null && physics != null) {
        physics.addTorque(body, tx, ty, tz);
        // physics.enableBody(body); // 현재 PhysicsManager에 없으므로 제거
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
        
        // 바디 위치/속도 리셋
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
