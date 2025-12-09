package com.kAIS.KAIMyEntity;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.lang.reflect.Method;

/**
 * ODE4J 물리 엔진 래퍼 - 블록 충돌 지원 확장
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

    // 물리 설정
    private double gravity = -9.81;
    private double stepSize = 1.0 / 60.0;
    private int maxContacts = 32;

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
            // 패키지명을 com.kAIS.ode4j.ode -> com.kAIS.ode4j.ode 로 변경
            final String base = "com.kAIS.ode4j.ode.";

            odeHelperClass   = odeClassLoader.loadClass(base + "OdeHelper");
            dWorldClass      = odeClassLoader.loadClass(base + "DWorld");
            dSpaceClass      = odeClassLoader.loadClass(base + "DSpace");
            dBodyClass       = odeClassLoader.loadClass(base + "DBody");
            dMassClass       = odeClassLoader.loadClass(base + "DMass");
            dGeomClass       = odeClassLoader.loadClass(base + "DGeom");
            dJointGroupClass = odeClassLoader.loadClass(base + "DJointGroup");

            // Box와 Plane 클래스
            try {
                dBoxClass = odeClassLoader.loadClass(base + "DBox");
            } catch (ClassNotFoundException e) {
                dBoxClass = dGeomClass; // fallback
            }
            try {
                dPlaneClass = odeClassLoader.loadClass(base + "DPlane");
            } catch (ClassNotFoundException e) {
                dPlaneClass = dGeomClass;
            }
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

        // Space 생성 (HashSpace 또는 SimpleSpace)
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

        // 중력 설정
        setGravity(0, gravity, 0);

        // World 파라미터 설정
        setupWorldParameters();

        initialized = true;
        logger.info("ODE4J Physics initialized successfully");
    }

    private void setupWorldParameters() {
        try {
            // ERP (Error Reduction Parameter)
            Method setERP = dWorldClass.getMethod("setERP", double.class);
            setERP.invoke(world, 0.8);

            // CFM (Constraint Force Mixing)
            Method setCFM = dWorldClass.getMethod("setCFM", double.class);
            setCFM.invoke(world, 1e-5);

            // Auto-disable (성능 최적화)
            try {
                Method setAutoDisable = dWorldClass.getMethod("setAutoDisableFlag", boolean.class);
                setAutoDisable.invoke(world, true);

                Method setAutoDisableLinearThreshold = dWorldClass.getMethod(
                        "setAutoDisableLinearThreshold", double.class);
                setAutoDisableLinearThreshold.invoke(world, 0.01);

                Method setAutoDisableAngularThreshold = dWorldClass.getMethod(
                        "setAutoDisableAngularThreshold", double.class);
                setAutoDisableAngularThreshold.invoke(world, 0.01);

                Method setAutoDisableSteps = dWorldClass.getMethod(
                        "setAutoDisableSteps", int.class);
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

            // DVector3 또는 DVector3C
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
            Method setPosition = body.getClass().getMethod("setPosition",
                    double.class, double.class, double.class);
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
            Method setLinearVel = body.getClass().getMethod("setLinearVel",
                    double.class, double.class, double.class);
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
            Method setAngularVel = body.getClass().getMethod("setAngularVel",
                    double.class, double.class, double.class);
            setAngularVel.invoke(body, x, y, z);
        } catch (Exception e) {
            logger.debug("Failed to set body angular vel: {}", e.getMessage());
        }
    }

    public void addForce(Object body, float fx, float fy, float fz) {
        if (body == null) return;

        try {
            Method addForce = body.getClass().getMethod("addForce",
                    double.class, double.class, double.class);
            addForce.invoke(body, (double) fx, (double) fy, (double) fz);
        } catch (Exception e) {
            logger.debug("Failed to add force: {}", e.getMessage());
        }
    }

    public void addTorque(Object body, float tx, float ty, float tz) {
        if (body == null) return;

        try {
            Method addTorque = body.getClass().getMethod("addTorque",
                    double.class, double.class, double.class);
            addTorque.invoke(body, (double) tx, (double) ty, (double) tz);
        } catch (Exception e) {
            logger.debug("Failed to add torque: {}", e.getMessage());
        }
    }

    // ========================================================================
    // Geometry 관련 (블록 충돌용)
    // ========================================================================

    /** Box geometry 생성 */
    public Object createBoxGeom(double lx, double ly, double lz) {
        if (!initialized || space == null) return null;

        try {
            Method createBox = odeHelperClass.getMethod("createBox",
                    dSpaceClass, double.class, double.class, double.class);
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
            Method createSphere = odeHelperClass.getMethod("createSphere",
                    dSpaceClass, double.class);
            return createSphere.invoke(null, space, radius);
        } catch (Exception e) {
            logger.debug("Failed to create sphere geom: {}", e.getMessage());
            return null;
        }
    }

    /** Geometry 위치 설정 */
    public void setGeomPosition(Object geom, double x, double y, double z) {
        if (geom == null) return;

        try {
            Method setPosition = geom.getClass().getMethod("setPosition",
                    double.class, double.class, double.class);
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

    /** Geometry 제거 */
    public void destroyGeom(Object geom) {
        if (geom == null) return;

        try {
            Method destroy = geom.getClass().getMethod("destroy");
            destroy.invoke(geom);
        } catch (Exception e) {
            logger.debug("Failed to destroy geom: {}", e.getMessage());
        }
    }

    /** 지면 평면 생성 */
    public void createGroundPlane(double height) {
        if (!initialized || space == null) return;

        try {
            // dCreatePlane(space, a, b, c, d) where ax + by + cz = d
            // Y-up: (0, 1, 0, height)
            Method createPlane = odeHelperClass.getMethod("createPlane",
                    dSpaceClass, double.class, double.class, double.class, double.class);
            groundPlane = createPlane.invoke(null, space, 0.0, 1.0, 0.0, height);
            logger.info("Created ground plane at Y={}", height);
        } catch (Exception e) {
            logger.warn("Failed to create ground plane: {}", e.getMessage());
        }
    }

    // ========================================================================
    // 시뮬레이션
    // ========================================================================

    /** 물리 스텝 실행 */
    public void step(float dt) {
        if (!initialized || world == null) return;

        try {
            // 충돌 감지 (space collide) – 현재는 외부에서 처리

            // World step
            Method quickStep = dWorldClass.getMethod("quickStep", double.class);
            quickStep.invoke(world, (double) dt);

            // Contact group 비우기
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
            Method setGravity = dWorldClass.getMethod("setGravity",
                    double.class, double.class, double.class);
            setGravity.invoke(world, x, y, z);
            this.gravity = y;
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

            // ODE 종료
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
        logger.info("PhysicsManager cleaned up");
    }
}
