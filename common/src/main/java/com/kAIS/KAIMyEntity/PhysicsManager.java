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
 * ODE4J 물리 엔진 래퍼 - 로봇 안정성 개선 패치 적용
 *
 * 주요 변경사항:
 * 1. 중력 방향 수정 (Y-down)
 * 2. 단단한 접촉 파라미터 (ERP=0.8, CFM=1e-5)
 * 3. QuickStep iterations 증가 (50)
 * 4. collide 메서드 시그니처 캐싱
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
    private Class<?> dContactGeomBufferClass;  // ✅ 추가
    private Class<?> dJointClass;
    private Class<?> odeConstantsClass;
    private Class<?> nearCallbackInterface;

    // ✅ 메서드 캐시 추가
    private Method collideMethod;
    private Method setQuickStepNumIterationsMethod;

    // 물리 설정
    private double gravity = 9.81;
    private double stepSize = 1.0 / 60.0;
    private int maxContacts = 32;

    // OdeConstants 값들
    private int dContactBounce;
    private int dContactApprox1;
    private int dContactSoftERP;
    private int dContactSoftCFM;

    // ✅ 튜닝 파라미터 (단단한 바닥 프리셋)
    private double worldERP = 0.8;
    private double worldCFM = 1e-5;
    private boolean useSoftContacts = true;
    private double contactSoftERP = 0.8;
    private double contactSoftCFM = 1e-5;
    private double contactMu = 1.0;
    private boolean debugContacts = false;

    // geom 타입 구분용
    private final Set<Object> staticGeoms = Collections.newSetFromMap(new IdentityHashMap<>());
    private final Set<Object> dynamicGeoms = Collections.newSetFromMap(new IdentityHashMap<>());

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

        odeClassLoader = Thread.currentThread().getContextClassLoader();

        try {
            final String base = "com.kAIS.ode4j.ode.";

            odeHelperClass = odeClassLoader.loadClass(base + "OdeHelper");
            dWorldClass = odeClassLoader.loadClass(base + "DWorld");
            dSpaceClass = odeClassLoader.loadClass(base + "DSpace");
            dBodyClass = odeClassLoader.loadClass(base + "DBody");
            dMassClass = odeClassLoader.loadClass(base + "DMass");
            dGeomClass = odeClassLoader.loadClass(base + "DGeom");
            dJointGroupClass = odeClassLoader.loadClass(base + "DJointGroup");
            dContactClass = odeClassLoader.loadClass(base + "DContact");
            dContactBufferClass = odeClassLoader.loadClass(base + "DContactBuffer");
            dContactGeomBufferClass = odeClassLoader.loadClass(base + "DContactGeomBuffer");  // ✅ 추가
            dJointClass = odeClassLoader.loadClass(base + "DJoint");
            odeConstantsClass = odeClassLoader.loadClass(base + "OdeConstants");
            nearCallbackInterface = odeClassLoader.loadClass(base + "DGeom$DNearCallback");

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

            dContactBounce = odeConstantsClass.getField("dContactBounce").getInt(null);
            dContactApprox1 = odeConstantsClass.getField("dContactApprox1").getInt(null);
            dContactSoftERP = odeConstantsClass.getField("dContactSoftERP").getInt(null);
            dContactSoftCFM = odeConstantsClass.getField("dContactSoftCFM").getInt(null);

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

        // NearCallback 생성
        createNearCallback();

        // ✅ 중력 설정: Y-down으로 수정
        setGravity(0, -gravity, 0);

        // World 파라미터 설정
        setupWorldParameters();

        // ✅ collide 메서드 캐시
        collideMethod = odeHelperClass.getMethod(
                "collide",
                dGeomClass, dGeomClass, int.class, dContactGeomBufferClass
        );

        // ✅ quickStep iteration 캐시
        try {
            setQuickStepNumIterationsMethod = dWorldClass.getMethod("setQuickStepNumIterations", int.class);
        } catch (Exception ignored) {
            setQuickStepNumIterationsMethod = null;
        }

        initialized = true;
        logger.info("ODE4J Physics initialized successfully (gravityY = {})", this.gravity);
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
     * ✅ 개선된 충돌 처리: 단단한 접촉 파라미터 사용
     */
    private void handleNearCallback(Object g1, Object g2) {
        try {
            boolean g1Dynamic = dynamicGeoms.contains(g1);
            boolean g2Dynamic = dynamicGeoms.contains(g2);

            // self-collision 방지
            if (g1Dynamic && g2Dynamic) {
                return;
            }

            Method getBody = dGeomClass.getMethod("getBody");
            Object b1 = getBody.invoke(g1);
            Object b2 = getBody.invoke(g2);

            if (b1 == null && b2 == null) {
                return;
            }

            Object contacts = dContactBufferClass
                    .getConstructor(int.class)
                    .newInstance(maxContacts);

            Method getGeomBuffer = dContactBufferClass.getMethod("getGeomBuffer");
            Object contactGeomBuffer = getGeomBuffer.invoke(contacts);

            // ✅ 캐시된 collideMethod 사용
            int numc = (Integer) collideMethod.invoke(null, g1, g2, maxContacts, contactGeomBuffer);

            if (debugContacts && numc > 0) {
                logger.info("contacts: {}", numc);
            }

            Method getContact = dContactBufferClass.getMethod("get", int.class);

            for (int i = 0; i < numc; i++) {
                Object contact = getContact.invoke(contacts, i);

                Object surface = dContactClass.getField("surface").get(contact);
                Class<?> surfaceClass = surface.getClass();

                // ✅ 단단한 접촉 모드
                int mode = dContactApprox1;
                if (useSoftContacts) {
                    mode |= (dContactSoftERP | dContactSoftCFM);
                }

                surfaceClass.getField("mode").setInt(surface, mode);
                surfaceClass.getField("mu").setDouble(surface, contactMu);
                surfaceClass.getField("bounce").setDouble(surface, 0.0);
                surfaceClass.getField("bounce_vel").setDouble(surface, 0.0);

                if (useSoftContacts) {
                    // ✅ 단단하게: ERP=0.8, CFM=1e-5
                    surfaceClass.getField("soft_erp").setDouble(surface, contactSoftERP);
                    surfaceClass.getField("soft_cfm").setDouble(surface, contactSoftCFM);
                }

                Method createContactJoint = odeHelperClass.getMethod(
                        "createContactJoint",
                        dWorldClass, dJointGroupClass, dContactClass
                );
                Object joint = createContactJoint.invoke(null, world, contactGroup, contact);

                Method attach = dJointClass.getMethod("attach", dBodyClass, dBodyClass);
                attach.invoke(joint, b1, b2);
            }

        } catch (Exception e) {
            logger.debug("Near callback error: {}", e.getMessage());
        }
    }

    /**
     * ✅ 단단한 물리 파라미터 설정
     */
    private void setupWorldParameters() {
        try {
            // ✅ ERP/CFM: 단단하게
            Method setERP = dWorldClass.getMethod("setERP", double.class);
            setERP.invoke(world, worldERP);

            Method setCFM = dWorldClass.getMethod("setCFM", double.class);
            setCFM.invoke(world, worldCFM);

            // ✅ QuickStep iterations: 로봇 안정성을 위해 50으로 증가
            if (setQuickStepNumIterationsMethod != null) {
                setQuickStepNumIterationsMethod.invoke(world, 50);
            }

            // ✅ Auto-disable는 로봇에게 문제를 일으킬 수 있어 끔
            try {
                Method setAutoDisable = dWorldClass.getMethod("setAutoDisableFlag", boolean.class);
                setAutoDisable.invoke(world, false);
            } catch (Exception ignored) {}

        } catch (Exception e) {
            logger.warn("Could not set world parameters: {}", e.getMessage());
        }
    }

    // ========================================================================
    // ✅ 새로운 유틸리티 메서드
    // ========================================================================

    public double[] getGeomPosition(Object geom) {
        if (geom == null) return new double[]{0, 0, 0};
        try {
            Method getPosition = geom.getClass().getMethod("getPosition");
            Object pos = getPosition.invoke(geom);

            Method get0 = pos.getClass().getMethod("get0");
            Method get1 = pos.getClass().getMethod("get1");
            Method get2 = pos.getClass().getMethod("get2");

            return new double[]{
                    ((Number) get0.invoke(pos)).doubleValue(),
                    ((Number) get1.invoke(pos)).doubleValue(),
                    ((Number) get2.invoke(pos)).doubleValue()
            };
        } catch (Exception e) {
            return new double[]{0, 0, 0};
        }
    }

    public void setGeomOffsetPosition(Object geom, double x, double y, double z) {
        if (geom == null) return;

        String[] candidates = new String[]{"setOffsetPosition", "setOffsetPos"};

        for (String name : candidates) {
            try {
                Method m = geom.getClass().getMethod(name, double.class, double.class, double.class);
                m.invoke(geom, x, y, z);
                return;
            } catch (Exception ignored) {}
        }

        // fallback
        setGeomPosition(geom, x, y, z);
    }

    /**
     * 런타임 접촉 파라미터 조정
     */
    public void setContactTuning(boolean soft, double softErp, double softCfm, double mu) {
        this.useSoftContacts = soft;
        this.contactSoftERP = softErp;
        this.contactSoftCFM = softCfm;
        this.contactMu = mu;
    }

    /**
     * 런타임 월드 파라미터 조정
     */
    public void setWorldTuning(double erp, double cfm, int quickStepIterations) {
        this.worldERP = erp;
        this.worldCFM = cfm;
        try {
            Method setERP = dWorldClass.getMethod("setERP", double.class);
            setERP.invoke(world, erp);
            Method setCFM = dWorldClass.getMethod("setCFM", double.class);
            setCFM.invoke(world, cfm);
            if (setQuickStepNumIterationsMethod != null) {
                setQuickStepNumIterationsMethod.invoke(world, quickStepIterations);
            }
        } catch (Exception ignored) {}
    }

    public void setDebugContacts(boolean enabled) {
        this.debugContacts = enabled;
    }

    // ========================================================================
    // Body 관련 (기존 메서드 유지)
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
    // Geometry 관련
    // ========================================================================

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

    public void setGeomBody(Object geom, Object body) {
        if (geom == null) return;

        try {
            Method setBody = geom.getClass().getMethod("setBody", dBodyClass);
            setBody.invoke(geom, body);
        } catch (Exception e) {
            logger.debug("Failed to set geom body: {}", e.getMessage());
        }
    }

    public void registerDynamicGeom(Object geom) {
        if (geom != null) {
            dynamicGeoms.add(geom);
        }
    }

    public void registerStaticGeom(Object geom) {
        if (geom != null) {
            staticGeoms.add(geom);
        }
    }

    public void destroyGeom(Object geom) {
        if (geom == null) return;

        try {
            staticGeoms.remove(geom);
            dynamicGeoms.remove(geom);

            Method destroy = geom.getClass().getMethod("destroy");
            destroy.invoke(geom);
        } catch (Exception e) {
            logger.debug("Failed to destroy geom: {}", e.getMessage());
        }
    }

    public void createGroundPlane(double height) {
        if (!initialized || space == null) return;

        try {
            Method createPlane = odeHelperClass.getMethod(
                    "createPlane",
                    dSpaceClass, double.class, double.class, double.class, double.class);
            groundPlane = createPlane.invoke(null, space, 0.0, 1.0, 0.0, height);

            registerStaticGeom(groundPlane);

            logger.info("Created ground plane at Y={}", height);
        } catch (Exception e) {
            logger.warn("Failed to create ground plane: {}", e.getMessage());
        }
    }

    // ========================================================================
    // 시뮬레이션
    // ========================================================================

    public void step(float dt) {
        if (!initialized || world == null) return;

        try {
            // 1. 충돌 검사
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
