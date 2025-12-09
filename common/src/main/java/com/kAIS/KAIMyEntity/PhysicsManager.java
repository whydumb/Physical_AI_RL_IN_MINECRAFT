package com.kAIS.KAIMyEntity;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.Set;

/**
 * ODE4J 물리 엔진 래퍼 - 완전한 ODE 충돌 처리 포함
 */
public class PhysicsManager {
    private static final Logger logger = LogManager.getLogger();
    private static PhysicsManager instance;

    private boolean initialized = false;
    private ClassLoader odeClassLoader;

    // ODE4J 핵심 객체
    private Object world;
    private Object space;
    private Object contactGroup;
    private Object groundPlane;
    private Object nearCallback;

    // ODE4J 클래스 캐시
    private Class<?> odeHelperClass;
    private Class<?> dWorldClass;
    private Class<?> dSpaceClass;
    private Class<?> dBodyClass;
    private Class<?> dMassClass;
    private Class<?> dGeomClass;
    private Class<?> dBoxClass;
    private Class<?> dPlaneClass;
    private Class<?> dJointGroupClass;
    private Class<?> dContactClass;
    private Class<?> dContactBufferClass;
    private Class<?> dJointClass;
    private Class<?> odeConstantsClass;
    private Class<?> nearCallbackInterface;

    // 물리 설정
    private double gravity = -9.81;
    private double stepSize = 1.0 / 60.0;
    private int maxContacts = 32;

    // OdeConstants 값들 (reflection으로 로드)
    private int dContactBounce;
    private int dContactApprox1;
    private int dContactSoftERP;
    private int dContactSoftCFM;

    // geom 타입 구분용 (self-collision 제어)
    // IdentityHashMap 을 써서 동일 객체 기준으로 비교
    private final Set<Object> staticGeoms  =
            Collections.newSetFromMap(new IdentityHashMap<>());
    private final Set<Object> dynamicGeoms =
            Collections.newSetFromMap(new IdentityHashMap<>());

    private PhysicsManager() {
        try {
            initialize();
        } catch (Exception e) {
            logger.error("Failed to initialize PhysicsManager", e);
            initialized = false;
        }
    }

    public static synchronized PhysicsManager GetInst() {
        if (instance == null) {
            instance = new PhysicsManager();
        }
        return instance;
    }

    private void initialize() throws Exception {
        logger.info("Initializing ODE4J Physics...");

        // ODE4J 클래스 로드
        odeClassLoader = Thread.currentThread().getContextClassLoader();

        try {
            final String base = "com.kAIS.ode4j.ode.";

            odeHelperClass      = odeClassLoader.loadClass(base + "OdeHelper");
            dWorldClass         = odeClassLoader.loadClass(base + "DWorld");
            dSpaceClass         = odeClassLoader.loadClass(base + "DSpace");
            dBodyClass          = odeClassLoader.loadClass(base + "DBody");
            dMassClass          = odeClassLoader.loadClass(base + "DMass");
            dGeomClass          = odeClassLoader.loadClass(base + "DGeom");
            dJointGroupClass    = odeClassLoader.loadClass(base + "DJointGroup");
            dContactClass       = odeClassLoader.loadClass(base + "DContact");
            dContactBufferClass = odeClassLoader.loadClass(base + "DContactBuffer");
            dJointClass         = odeClassLoader.loadClass(base + "DJoint");
            odeConstantsClass   = odeClassLoader.loadClass(base + "OdeConstants");
            nearCallbackInterface = odeClassLoader.loadClass(base + "DGeom$DNearCallback");

            // Box와 Plane 클래스
            try {
                dBoxClass = odeClassLoader.loadClass(base + "DBox");
            } catch (ClassNotFoundException e) {
                dBoxClass = dGeomClass;
            }
            try {
                dPlaneClass = odeClassLoader.loadClass(base + "DPlane");
            } catch (ClassNotFoundException e) {
                dPlaneClass = dGeomClass;
            }

            // OdeConstants 값 로드
            dContactBounce   = odeConstantsClass.getField("dContactBounce").getInt(null);
            dContactApprox1  = odeConstantsClass.getField("dContactApprox1").getInt(null);
            dContactSoftERP  = odeConstantsClass.getField("dContactSoftERP").getInt(null);
            dContactSoftCFM  = odeConstantsClass.getField("dContactSoftCFM").getInt(null);

        } catch (ClassNotFoundException e) {
            logger.error("ODE4J classes not found - physics disabled", e);
            initialized = false;
            return;
        }

        // ODE 초기화
        try {
            Method initODE = odeHelperClass.getMethod("initODE2", int.class);
            initODE.invoke(null, 0);
        } catch (NoSuchMethodException e) {
            try {
                Method initODE = odeHelperClass.getMethod("initODE");
                initODE.invoke(null);
            } catch (Exception e2) {
                logger.warn("ODE init method not found, continuing anyway");
            }
        }

        // World 생성
        Method createWorld = odeHelperClass.getMethod("createWorld");
        world = createWorld.invoke(null);

        // Space 생성
        try {
            Method createHashSpace = odeHelperClass.getMethod("createHashSpace");
            space = createHashSpace.invoke(null);
        } catch (NoSuchMethodException e) {
            Method createSimpleSpace = odeHelperClass.getMethod("createSimpleSpace");
            space = createSimpleSpace.invoke(null);
        }

        // ContactGroup 생성
        try {
            Method createJointGroup = odeHelperClass.getMethod("createJointGroup");
            contactGroup = createJointGroup.invoke(null);
        } catch (Exception e) {
            logger.warn("Could not create contact group: {}", e.getMessage());
        }

        // NearCallback 생성 (Proxy 사용)
        createNearCallback();

        // 중력 설정 (Y-down)
        setGravity(0, gravity, 0);

        // World 파라미터 설정
        setupWorldParameters();

        initialized = true;
        logger.info("ODE4J Physics initialized successfully (gravityY = {})", gravity);
    }

    private void createNearCallback() {
        try {
            nearCallback = Proxy.newProxyInstance(
                    odeClassLoader,
                    new Class<?>[]{nearCallbackInterface},
                    new InvocationHandler() {
                        @Override
                        public Object invoke(Object proxy, Method method, Object[] args) throws Throwable {
                            if ("call".equals(method.getName()) && args != null && args.length >= 3) {
                                Object g1 = args[1];
                                Object g2 = args[2];
                                handleNearCallback(g1, g2);
                            }
                            return null;
                        }
                    }
            );
        } catch (Exception e) {
            logger.error("Failed to create near callback", e);
        }
    }

    /**
     * space.collide 에서 호출되는 실제 충돌 처리
     */
    private void handleNearCallback(Object g1, Object g2) {
        try {
            // 0) 동적/정적 geom 타입 판별
            boolean g1Static  = staticGeoms.contains(g1);
            boolean g2Static  = staticGeoms.contains(g2);
            boolean g1Dynamic = dynamicGeoms.contains(g1);
            boolean g2Dynamic = dynamicGeoms.contains(g2);

            // self-collision(둘 다 dynamic) 은 일단 무시해서 조인트랑 안 싸우게 함
            if (g1Dynamic && g2Dynamic) {
                return;
            }

            // Body 가져오기
            Method getBody = dGeomClass.getMethod("getBody");
            Object b1 = getBody.invoke(g1);
            Object b2 = getBody.invoke(g2);

            // 둘 다 static / body 없음이면 무시
            if (b1 == null && b2 == null) {
                return;
            }

            // DContactBuffer 생성
            Object contacts = dContactBufferClass
                    .getConstructor(int.class)
                    .newInstance(maxContacts);

            // getGeomBuffer() 호출
            Method getGeomBuffer = dContactBufferClass.getMethod("getGeomBuffer");
            Object contactGeomBuffer = getGeomBuffer.invoke(contacts);

            // OdeHelper.collide(g1, g2, maxContacts, contactGeomBuffer)
            Method collide = odeHelperClass.getMethod(
                    "collide",
                    dGeomClass, dGeomClass, int.class, contactGeomBuffer.getClass()
            );
            int numc = (Integer) collide.invoke(null, g1, g2, maxContacts, contactGeomBuffer);

            Method getContact = dContactBufferClass.getMethod("get", int.class);

            for (int i = 0; i < numc; i++) {
                Object contact = getContact.invoke(contacts, i);

                // contact.surface 설정
                Object surface = dContactClass.getField("surface").get(contact);
                Class<?> surfaceClass = surface.getClass();

                // 너무 딱딱하게 하지 말고, 약간 말랑하게 설정
                surfaceClass.getField("mode").setInt(
                        surface,
                        // Bounce 는 일단 끄고 SoftERP/CFM + Approx1 만 사용
                        dContactApprox1 | dContactSoftERP | dContactSoftCFM
                );

                surfaceClass.getField("mu").setDouble(surface, 0.8);   // 마찰
                surfaceClass.getField("bounce").setDouble(surface, 0.0);
                surfaceClass.getField("bounce_vel").setDouble(surface, 0.0);

                // 부드러운 침투 보정
                surfaceClass.getField("soft_erp").setDouble(surface, 0.2);   // 작은 ERP
                surfaceClass.getField("soft_cfm").setDouble(surface, 1e-3);  // 조금 말랑하게

                // Contact joint 생성
                Method createContactJoint = odeHelperClass.getMethod(
                        "createContactJoint",
                        dWorldClass, dJointGroupClass, dContactClass
                );
                Object joint = createContactJoint.invoke(null, world, contactGroup, contact);

                // Joint attach
                Method attach = dJointClass.getMethod("attach", dBodyClass, dBodyClass);
                attach.invoke(joint, b1, b2);
            }

        } catch (Exception e) {
            logger.debug("Near callback error: {}", e.getMessage());
        }
    }

    private void setupWorldParameters() {
        try {
            // ERP (Error Reduction Parameter)
            Method setERP = dWorldClass.getMethod("setERP", double.class);
            setERP.invoke(world, 0.2);  // 전체적으로 조금 부드럽게

            // CFM (Constraint Force Mixing)
            Method setCFM = dWorldClass.getMethod("setCFM", double.class);
            setCFM.invoke(world, 1e-3);

            // Auto-disable (성능 최적화)
            try {
                Method setAutoDisable = dWorldClass.getMethod("setAutoDisableFlag", boolean.class);
                setAutoDisable.invoke(world, true);

                Method setAutoDisableLinearThreshold =
                        dWorldClass.getMethod("setAutoDisableLinearThreshold", double.class);
                setAutoDisableLinearThreshold.invoke(world, 0.01);

                Method setAutoDisableAngularThreshold =
                        dWorldClass.getMethod("setAutoDisableAngularThreshold", double.class);
                setAutoDisableAngularThreshold.invoke(world, 0.01);

                Method setAutoDisableSteps = dWorldClass.getMethod("setAutoDisableSteps", int.class);
                setAutoDisableSteps.invoke(world, 10);
            } catch (Exception e) {
                logger.debug("Auto-disable not supported: {}", e.getMessage());
            }

        } catch (Exception e) {
            logger.warn("Could not set world parameters: {}", e.getMessage());
        }
    }

    // ========================================================================
    // Body 관련
    // ========================================================================

    public Object createBody() {
        if (!initialized || world == null) return null;

        try {
            Method createBody = odeHelperClass.getMethod("createBody", dWorldClass);
            return createBody.invoke(null, world);
        } catch (Exception e) {
            logger.error("Failed to create body", e);
            return null;
        }
    }

    public double[] getBodyPosition(Object body) {
        if (body == null) return new double[]{0, 0, 0};

        try {
            Method getPosition = body.getClass().getMethod("getPosition");
            Object pos = getPosition.invoke(body);

            Method get0 = pos.getClass().getMethod("get0");
            Method get1 = pos.getClass().getMethod("get1");
            Method get2 = pos.getClass().getMethod("get2");

            return new double[]{
                    ((Number) get0.invoke(pos)).doubleValue(),
                    ((Number) get1.invoke(pos)).doubleValue(),
                    ((Number) get2.invoke(pos)).doubleValue()
            };
        } catch (Exception e) {
            logger.debug("Failed to get body position: {}", e.getMessage());
            return new double[]{0, 0, 0};
        }
    }

    public void setBodyPosition(Object body, double x, double y, double z) {
        if (body == null) return;

        try {
            Method setPosition = body.getClass().getMethod(
                    "setPosition", double.class, double.class, double.class);
            setPosition.invoke(body, x, y, z);
        } catch (Exception e) {
            logger.debug("Failed to set body position: {}", e.getMessage());
        }
    }

    public double[] getBodyLinearVel(Object body) {
        if (body == null) return new double[]{0, 0, 0};

        try {
            Method getLinearVel = body.getClass().getMethod("getLinearVel");
            Object vel = getLinearVel.invoke(body);

            Method get0 = vel.getClass().getMethod("get0");
            Method get1 = vel.getClass().getMethod("get1");
            Method get2 = vel.getClass().getMethod("get2");

            return new double[]{
                    ((Number) get0.invoke(vel)).doubleValue(),
                    ((Number) get1.invoke(vel)).doubleValue(),
                    ((Number) get2.invoke(vel)).doubleValue()
            };
        } catch (Exception e) {
            logger.debug("Failed to get body linear vel: {}", e.getMessage());
            return new double[]{0, 0, 0};
        }
    }

    public void setBodyLinearVel(Object body, double x, double y, double z) {
        if (body == null) return;

        try {
            Method setLinearVel = body.getClass().getMethod(
                    "setLinearVel", double.class, double.class, double.class);
            setLinearVel.invoke(body, x, y, z);
        } catch (Exception e) {
            logger.debug("Failed to set body linear vel: {}", e.getMessage());
        }
    }

    public double[] getBodyAngularVel(Object body) {
        if (body == null) return new double[]{0, 0, 0};

        try {
            Method getAngularVel = body.getClass().getMethod("getAngularVel");
            Object vel = getAngularVel.invoke(body);

            Method get0 = vel.getClass().getMethod("get0");
            Method get1 = vel.getClass().getMethod("get1");
            Method get2 = vel.getClass().getMethod("get2");

            return new double[]{
                    ((Number) get0.invoke(vel)).doubleValue(),
                    ((Number) get1.invoke(vel)).doubleValue(),
                    ((Number) get2.invoke(vel)).doubleValue()
            };
        } catch (Exception e) {
            return new double[]{0, 0, 0};
        }
    }

    public void setBodyAngularVel(Object body, double x, double y, double z) {
        if (body == null) return;

        try {
            Method setAngularVel = body.getClass().getMethod(
                    "setAngularVel", double.class, double.class, double.class);
            setAngularVel.invoke(body, x, y, z);
        } catch (Exception e) {
            logger.debug("Failed to set body angular vel: {}", e.getMessage());
        }
    }

    public void addForce(Object body, float fx, float fy, float fz) {
        if (body == null) return;

        try {
            Method addForce = body.getClass().getMethod(
                    "addForce", double.class, double.class, double.class);
            addForce.invoke(body, (double) fx, (double) fy, (double) fz);
        } catch (Exception e) {
            logger.debug("Failed to add force: {}", e.getMessage());
        }
    }

    public void addTorque(Object body, float tx, float ty, float tz) {
        if (body == null) return;

        try {
            Method addTorque = body.getClass().getMethod(
                    "addTorque", double.class, double.class, double.class);
            addTorque.invoke(body, (double) tx, (double) ty, (double) tz);
        } catch (Exception e) {
            logger.debug("Failed to add torque: {}", e.getMessage());
        }
    }

    // ========================================================================
    // Geometry 관련 (블록 충돌 + 링크 콜리전용)
    // ========================================================================

    /** Box geometry 생성 */
    public Object createBoxGeom(double lx, double ly, double lz) {
        if (!initialized || space == null) return null;

        try {
            Method createBox = odeHelperClass.getMethod(
                    "createBox", dSpaceClass, double.class, double.class, double.class);
            return createBox.invoke(null, space, lx, ly, lz);
        } catch (Exception e) {
            logger.debug("Failed to create box geom: {}", e.getMessage());
            return null;
        }
    }

    /** Sphere geometry 생성 */
    public Object createSphereGeom(double radius) {
        if (!initialized || space == null) return null;

        try {
            Method createSphere = odeHelperClass.getMethod(
                    "createSphere", dSpaceClass, double.class);
            return createSphere.invoke(null, space, radius);
        } catch (Exception e) {
            logger.debug("Failed to create sphere geom: {}", e.getMessage());
            return null;
        }
    }

    /** Cylinder / Capsule geometry 생성 (URDFSimpleController에서 사용) */
    public Object createCylinderGeom(double radius, double height) {
        if (!initialized || space == null) return null;

        try {
            Method createCylinder = odeHelperClass.getMethod(
                    "createCylinder", dSpaceClass, double.class, double.class);
            return createCylinder.invoke(null, space, radius, height);
        } catch (NoSuchMethodException e) {
            try {
                Method createCapsule = odeHelperClass.getMethod(
                        "createCapsule", dSpaceClass, double.class, double.class);
                return createCapsule.invoke(null, space, radius, height);
            } catch (Exception e2) {
                logger.debug("Failed to create cylinder/capsule geom: {}", e2.getMessage());
                return null;
            }
        } catch (Exception e) {
            logger.debug("Failed to create cylinder geom: {}", e.getMessage());
            return null;
        }
    }

    /** Geometry 위치 설정 */
    public void setGeomPosition(Object geom, double x, double y, double z) {
        if (geom == null) return;

        try {
            Method setPosition = geom.getClass().getMethod(
                    "setPosition", double.class, double.class, double.class);
            setPosition.invoke(geom, x, y, z);
        } catch (Exception e) {
            logger.debug("Failed to set geom position: {}", e.getMessage());
        }
    }

    /** Geometry에 Body 연결 */
    public void setGeomBody(Object geom, Object body) {
        if (geom == null) return;

        try {
            Method setBody = geom.getClass().getMethod("setBody", dBodyClass);
            setBody.invoke(geom, body);
        } catch (Exception e) {
            logger.debug("Failed to set geom body: {}", e.getMessage());
        }
    }

    /** 동적 geom 등록 (로봇 링크 등) */
    public void registerDynamicGeom(Object geom) {
        if (geom != null) {
            dynamicGeoms.add(geom);
        }
    }

    /** 정적 geom 등록 (블록, ground plane 등) */
    public void registerStaticGeom(Object geom) {
        if (geom != null) {
            staticGeoms.add(geom);
        }
    }

    /** Geometry 제거 */
    public void destroyGeom(Object geom) {
        if (geom == null) return;

        try {
            // set에서 제거
            staticGeoms.remove(geom);
            dynamicGeoms.remove(geom);

            Method destroy = geom.getClass().getMethod("destroy");
            destroy.invoke(geom);
        } catch (Exception e) {
            logger.debug("Failed to destroy geom: {}", e.getMessage());
        }
    }

    /** 지면 평면 생성 (원하면 사용) */
    public void createGroundPlane(double height) {
        if (!initialized || space == null) return;

        try {
            Method createPlane = odeHelperClass.getMethod(
                    "createPlane",
                    dSpaceClass, double.class, double.class, double.class, double.class);
            groundPlane = createPlane.invoke(null, space, 0.0, 1.0, 0.0, height);

            // 정적 geom으로 등록
            registerStaticGeom(groundPlane);

            logger.info("Created ground plane at Y={}", height);
        } catch (Exception e) {
            logger.warn("Failed to create ground plane: {}", e.getMessage());
        }
    }

    // ========================================================================
    // 시뮬레이션
    // ========================================================================

    /** 물리 스텝 실행 - ODE 충돌 처리 포함 */
    public void step(float dt) {
        if (!initialized || world == null) return;

        try {
            // 1. 모든 geom 쌍 충돌 검사
            if (space != null && nearCallback != null) {
                Method spaceCollide = dSpaceClass.getMethod(
                        "collide", Object.class, nearCallbackInterface);
                spaceCollide.invoke(space, null, nearCallback);
            }

            // 2. World step
            Method quickStep = dWorldClass.getMethod("quickStep", double.class);
            quickStep.invoke(world, (double) dt);

            // 3. Contact group 비우기
            if (contactGroup != null) {
                try {
                    Method empty = contactGroup.getClass().getMethod("empty");
                    empty.invoke(contactGroup);
                } catch (Exception e) {
                    // ignore
                }
            }

        } catch (Exception e) {
            logger.error("Physics step failed: {}", e.getMessage());
        }
    }

    // ========================================================================
    // 설정
    // ========================================================================

    public void setGravity(float x, float y, float z) {
        setGravity((double) x, (double) y, (double) z);
    }

    public void setGravity(double x, double y, double z) {
        if (!initialized || world == null) return;

        try {
            Method setGravity = dWorldClass.getMethod(
                    "setGravity", double.class, double.class, double.class);
            setGravity.invoke(world, x, y, z);
            this.gravity = y;
            logger.info("Set ODE gravity to ({}, {}, {})", x, y, z);
        } catch (Exception e) {
            logger.warn("Failed to set gravity: {}", e.getMessage());
        }
    }

    // ========================================================================
    // 접근자
    // ========================================================================

    public boolean isInitialized() {
        return initialized;
    }

    public Object getWorld() {
        return world;
    }

    public Object getSpace() {
        return space;
    }

    public ClassLoader getClassLoader() {
        return odeClassLoader;
    }

    public double getGravity() {
        return gravity;
    }

    // ========================================================================
    // 정리
    // ========================================================================

    public void cleanup() {
        logger.info("Cleaning up PhysicsManager...");

        try {
            if (groundPlane != null) {
                destroyGeom(groundPlane);
                groundPlane = null;
            }

            if (contactGroup != null) {
                Method destroy = contactGroup.getClass().getMethod("destroy");
                destroy.invoke(contactGroup);
                contactGroup = null;
            }

            if (space != null) {
                Method destroy = space.getClass().getMethod("destroy");
                destroy.invoke(space);
                space = null;
            }

            if (world != null) {
                Method destroy = world.getClass().getMethod("destroy");
                destroy.invoke(world);
                world = null;
            }

            try {
                Method closeODE = odeHelperClass.getMethod("closeODE");
                closeODE.invoke(null);
            } catch (Exception e) {
                // ignore
            }

        } catch (Exception e) {
            logger.error("Error during cleanup", e);
        }

        initialized = false;
        nearCallback = null;
        staticGeoms.clear();
        dynamicGeoms.clear();

        logger.info("PhysicsManager cleaned up");
    }
}
