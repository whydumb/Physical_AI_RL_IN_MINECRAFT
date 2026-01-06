package com.kAIS.KAIMyEntity.urdf.control;

import java.util.Locale;

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
 *
 * ===== 이번 통째 패치 핵심 =====
 * 1) FIXED joint도 생성 (기존: isMovable()만 만들면 FIXED가 빠짐)
 * 2) world(null body) attach joint는 기본 스킵 (루트 자유 => 넘어질 수 있게)
 *    - 필요하면 setAllowWorldAttachment(true)
 * 3) MESH geometry collider type 꼬임 수정 (sphere로 떨어지던 문제)
 * 4) RL reset용: 스폰 포즈 스냅샷 + hardResetToSpawn(위치/회전/속도)
 * 5) RL용 Physics-only 루트 API: getRootBodyWorldPositionPhysicsOnly()
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

    private double spawnCollisionMargin = 0.05;

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

    // ✅ NEW: world(null body)에 attach되는 조인트 생성 허용 여부
    private boolean allowWorldAttachment = false;

    // ===== RL reset: spawn pose snapshot (하드 리셋용) =====
    private boolean spawnPoseCaptured = false;
    private final Map<String, double[]> spawnBodyOffsetsFromRoot = new HashMap<>();
    private final Map<String, float[]> spawnBodyQuatWxyz = new HashMap<>();
    private boolean warnedNoQuaternionSetter = false;

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

    // ✅ ODE4J math 클래스 캐시 (shaded package 대응)
    private ClassLoader odeCl;
    private Class<?> dQuaternionClass;
    private Class<?> dMatrix3Class;

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
                    logger.info("  allowWorldAttachment: {}", allowWorldAttachment);
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
    // 옵션
    // ========================================================================

    /** world(null body) attach joint를 만들지 여부 (기본 false 추천: 넘어질 수 있게) */
    public void setAllowWorldAttachment(boolean allow) { this.allowWorldAttachment = allow; }
    public boolean isAllowWorldAttachment() { return allowWorldAttachment; }

    public void setSpawnCollisionMargin(double margin) {
        if (!Double.isFinite(margin) || margin < 0.0) {
            logger.warn("Ignoring invalid spawn collision margin: {}", margin);
            return;
        }
        this.spawnCollisionMargin = margin;
    }

    // ========================================================================
    // 월드 컨텍스트 (렌더러/엔티티에서 호출)
    // ========================================================================

    /**
     * ✅ 수정: Level만 업데이트, 위치는 앵커링 시에만 저장
     * 물리 모드에서는 최초 한 번, 루트 바디를 해당 위치로 정렬(앵커)한다.
     */
    public void setWorldContext(Level level, Vec3 worldPos) {
        this.currentLevel = level;

        if (usePhysics && physicsInitialized && !worldAnchored &&
                physics != null && !bodies.isEmpty() && worldPos != null) {

            double clearance = computeRootClearance();
            double targetY = worldPos.y + clearance + spawnCollisionMargin;
            if (!Double.isFinite(targetY)) targetY = worldPos.y + 1.0;

            Vec3 safePos = new Vec3(worldPos.x, targetY, worldPos.z);
            this.initialAnchorPosition = safePos;

            anchorPhysicsToWorld(safePos);
            captureSpawnPoseIfNeeded(); // ✅ 스폰 포즈 스냅샷
            worldAnchored = true;

            if (blockCollisionManager != null && currentLevel != null) {
                blockCollisionManager.forceUpdate(currentLevel, safePos.x, safePos.y, safePos.z);
                logger.info("BlockCollisionManager force-updated at spawn position. activeBlocks={}",
                        blockCollisionManager.getActiveBlockCount());
            }
        }
    }

    private double computeRootClearance() {
        double fallback = estimateFallbackClearance();
        if (!usePhysics || !physicsInitialized || physics == null || bodies.isEmpty()) return fallback;

        try {
            Object root = getRootBody();
            if (root == null) return fallback;

            double[] rootPos = physics.getBodyPosition(root);
            if (rootPos == null || rootPos.length < 3) return fallback;

            float baseBottom = getApproxBaseHeightWorldY();
            double clearance = rootPos[1] - baseBottom;
            if (!Double.isFinite(clearance) || clearance <= 0.0) return fallback;
            return clearance;
        } catch (Exception e) {
            logger.debug("computeRootClearance fallback: {}", e.getMessage());
            return fallback;
        }
    }

    private double estimateFallbackClearance() {
        double maxRadius = 0.0;
        for (float r : linkRadii.values()) {
            if (Float.isFinite(r)) maxRadius = Math.max(maxRadius, r);
        }
        return (maxRadius > 0.0) ? maxRadius : 1.0;
    }

    /** 호환용 */
    @Deprecated
    public Vec3 getWorldPosition() {
        return initialAnchorPosition != null ? initialAnchorPosition : Vec3.ZERO;
    }

    /**
     * 루트 바디 기준으로 모든 바디를 주어진 월드 위치로 평행 이동시킨다.
     * 이동 후 힌지 앵커를 다시 세팅(refresh)해야 관통/폭주가 줄어든다.
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

            refreshHingeAnchors();

            logger.info("Anchored physics to world at ({}, {}, {})",
                    anchorWorldPos.x, anchorWorldPos.y, anchorWorldPos.z);

        } catch (Exception e) {
            logger.warn("anchorPhysicsToWorld failed: {}", e.getMessage());
        }
    }

    private void refreshHingeAnchors() {
        if (physics == null || dHingeJointClass == null) return;

        try {
            Method setAnchor = dHingeJointClass.getMethod("setAnchor", double.class, double.class, double.class);
            int refreshed = 0;

            for (URDFJoint j : joints.values()) {
                if (j == null) continue;
                if (!(j.type == URDFJoint.JointType.REVOLUTE || j.type == URDFJoint.JointType.CONTINUOUS)) continue;

                Object odeJoint = odeJoints.get(j.name);
                if (odeJoint == null) continue;

                Object childBody  = bodies.get(j.childLinkName);
                Object parentBody = bodies.get(j.parentLinkName);

                if (!allowWorldAttachment && (childBody == null || parentBody == null)) continue;

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
                refreshed++;
            }

            logger.info("Refreshed hinge anchors for {} joints", refreshed);
        } catch (Exception e) {
            logger.debug("refreshHingeAnchors skipped: {}", e.getMessage());
        }
    }

    // ========================================================================
    // 루트 바디 / 루트 바디 위치
    // ========================================================================

    private Object getRootBody() {
        if (bodies.isEmpty()) return null;

        if (rootBodyLinkName != null) {
            Object root = bodies.get(rootBodyLinkName);
            if (root != null) return root;
        }
        return bodies.values().iterator().next();
    }

    /** ✅ RL용: 물리 모드일 때만 "진짜" 루트 바디 월드 위치 (실패하면 null) */
    public double[] getRootBodyWorldPositionPhysicsOnly() {
        if (!usePhysics || !physicsInitialized || bodies.isEmpty() || physics == null) return null;
        Object root = getRootBody();
        if (root == null) return null;

        double[] pos = physics.getBodyPosition(root);
        if (pos == null || pos.length < 3) return null;
        if (!Double.isFinite(pos[0]) || !Double.isFinite(pos[1]) || !Double.isFinite(pos[2])) return null;

        return new double[]{pos[0], pos[1], pos[2]};
    }

    /** ✅ RL용: 물리 모드일 때만 루트 쿼터니언(wxyz). 실패하면 null */
    public float[] getRootBodyWorldQuaternionWXYZPhysicsOnly() {
        if (!usePhysics || !physicsInitialized || bodies.isEmpty() || physics == null) return null;
        Object root = getRootBody();
        if (root == null) return null;
        return tryReadBodyQuaternionWXYZ(root);
    }

    /** 호환: 물리 OFF면 identity */
    public float[] getRootBodyWorldQuaternionWXYZ() {
        if (!usePhysics || !physicsInitialized || bodies.isEmpty() || physics == null) {
            return new float[]{1f, 0f, 0f, 0f};
        }
        Object root = getRootBody();
        if (root == null) return new float[]{1f, 0f, 0f, 0f};
        float[] q = tryReadBodyQuaternionWXYZ(root);
        return (q != null) ? q : new float[]{1f, 0f, 0f, 0f};
    }

    public float[] getRootLinkLocalPosition(Vec3 baseWorldPos) {
        if (!usePhysics || !physicsInitialized || physics == null || bodies.isEmpty()) return new float[]{0, 0, 0};

        Object root = getRootBody();
        if (root == null) return new float[]{0, 0, 0};

        double[] pos = physics.getBodyPosition(root);
        if (pos == null || pos.length < 3) return new float[]{0, 0, 0};

        Vec3 base = (baseWorldPos != null) ? baseWorldPos : Vec3.ZERO;

        return new float[]{
                (float) (pos[0] - base.x),
                (float) (pos[1] - base.y),
                (float) (pos[2] - base.z)
        };
    }

    @Deprecated
    public float[] getRootLinkLocalPosition() {
        return getRootLinkLocalPosition(initialAnchorPosition != null ? initialAnchorPosition : Vec3.ZERO);
    }

    public float[] getLinkWorldPosition(String linkName) {
        if (!usePhysics || !physicsInitialized || physics == null) {
            Vec3 fallback = initialAnchorPosition != null ? initialAnchorPosition : Vec3.ZERO;
            return new float[]{(float) fallback.x, (float) fallback.y, (float) fallback.z};
        }

        Object body = bodies.get(linkName);
        if (body != null) {
            double[] pos = physics.getBodyPosition(body);
            if (pos != null && pos.length >= 3) return new float[]{(float) pos[0], (float) pos[1], (float) pos[2]};
        }

        Vec3 fallback = initialAnchorPosition != null ? initialAnchorPosition : Vec3.ZERO;
        return new float[]{(float) fallback.x, (float) fallback.y, (float) fallback.z};
    }

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

            if (bottomY < minBottomWorldY) minBottomWorldY = bottomY;
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

    /** 호환: 물리 OFF면 fallback */
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
    // RL reset helper (하드 리셋)
    // ========================================================================

    /** 바디 속도만 0 */
    public void resetBodyVelocitiesOnly() {
        if (!usePhysics || !physicsInitialized || physics == null) return;

        for (Object body : bodies.values()) {
            if (body == null) continue;
            try {
                physics.setBodyLinearVel(body, 0, 0, 0);
                physics.setBodyAngularVel(body, 0, 0, 0);
            } catch (Exception ignored) { }
        }
    }

    /**
     * ✅ 핵심: 스폰 포즈로 하드 리셋 (위치/회전/속도)
     * @param anchorWorldPos null이면 initialAnchorPosition 사용
     */
    public boolean hardResetToSpawn(Vec3 anchorWorldPos) {
        if (!usePhysics || !physicsInitialized || physics == null || bodies.isEmpty()) return false;

        Vec3 anchor = (anchorWorldPos != null) ? anchorWorldPos : initialAnchorPosition;
        if (anchor == null) return false;

        captureSpawnPoseIfNeeded();
        if (!spawnPoseCaptured) {
            resetBodyVelocitiesOnly();
            return false;
        }

        Object root = getRootBody();
        if (root == null) {
            resetBodyVelocitiesOnly();
            return false;
        }

        final double rootX = anchor.x;
        final double rootY = anchor.y;
        final double rootZ = anchor.z;

        for (Map.Entry<String, Object> e : bodies.entrySet()) {
            String linkName = e.getKey();
            Object body = e.getValue();
            if (body == null) continue;

            double[] off = spawnBodyOffsetsFromRoot.get(linkName);
            if (off != null && off.length >= 3) {
                try {
                    physics.setBodyPosition(body, rootX + off[0], rootY + off[1], rootZ + off[2]);
                } catch (Exception ignored) { }
            }

            float[] q = spawnBodyQuatWxyz.get(linkName);
            if (q != null && q.length >= 4) {
                boolean ok = trySetBodyQuaternionWXYZ(body, q[0], q[1], q[2], q[3]);
                if (!ok && !warnedNoQuaternionSetter) {
                    warnedNoQuaternionSetter = true;
                    logger.warn("No quaternion setter found. Hard reset may not fully restore upright pose.");
                }
            }

            try {
                physics.setBodyLinearVel(body, 0, 0, 0);
                physics.setBodyAngularVel(body, 0, 0, 0);
            } catch (Exception ignored) { }
        }

        refreshHingeAnchors();

        try {
            if (blockCollisionManager != null && currentLevel != null) {
                blockCollisionManager.forceUpdate(currentLevel, anchor.x, anchor.y, anchor.z);
            }
        } catch (Exception ignored) { }

        try { syncJointStates(); } catch (Exception ignored) { }

        return true;
    }

    private void captureSpawnPoseIfNeeded() {
        if (spawnPoseCaptured) return;
        if (!usePhysics || !physicsInitialized || physics == null || bodies.isEmpty()) return;

        Object root = getRootBody();
        if (root == null) return;

        double[] rootPos;
        try {
            rootPos = physics.getBodyPosition(root);
        } catch (Exception e) {
            return;
        }
        if (rootPos == null || rootPos.length < 3) return;

        spawnBodyOffsetsFromRoot.clear();
        spawnBodyQuatWxyz.clear();

        for (Map.Entry<String, Object> e : bodies.entrySet()) {
            String linkName = e.getKey();
            Object body = e.getValue();
            if (body == null) continue;

            try {
                double[] p = physics.getBodyPosition(body);
                if (p == null || p.length < 3) continue;

                spawnBodyOffsetsFromRoot.put(linkName, new double[]{
                        p[0] - rootPos[0],
                        p[1] - rootPos[1],
                        p[2] - rootPos[2]
                });

                float[] q = tryReadBodyQuaternionWXYZ(body);
                if (q != null && q.length >= 4) spawnBodyQuatWxyz.put(linkName, q);

            } catch (Exception ignored) { }
        }

        spawnPoseCaptured = !spawnBodyOffsetsFromRoot.isEmpty();
        warnedNoQuaternionSetter = false;

        if (spawnPoseCaptured) {
            logger.info("Captured spawn pose snapshot: bodies={}, hasQuat={}",
                    spawnBodyOffsetsFromRoot.size(), spawnBodyQuatWxyz.size());
        }
    }

    private boolean trySetBodyQuaternionWXYZ(Object body, float w, float x, float y, float z) {
        if (body == null) return false;

        // 1) PhysicsManager setter 탐색
        if (physics != null) {
            try {
                for (Method m : physics.getClass().getMethods()) {
                    String n = m.getName().toLowerCase(Locale.ROOT);
                    if (!(n.contains("quaternion") || n.contains("quat"))) continue;

                    // set...(body, w,x,y,z)
                    if (m.getParameterCount() == 5) {
                        Class<?>[] pt = m.getParameterTypes();
                        if (!pt[0].isAssignableFrom(body.getClass()) && pt[0] != Object.class) continue;

                        if (pt[1] == float.class) {
                            m.invoke(physics, body, w, x, y, z);
                            return true;
                        }
                        if (pt[1] == double.class) {
                            m.invoke(physics, body, (double) w, (double) x, (double) y, (double) z);
                            return true;
                        }
                    }

                    // set...(body, float[]/double[])
                    if (m.getParameterCount() == 2) {
                        Class<?>[] pt = m.getParameterTypes();
                        if (!pt[0].isAssignableFrom(body.getClass()) && pt[0] != Object.class) continue;

                        if (pt[1] == float[].class) {
                            m.invoke(physics, body, new float[]{w, x, y, z});
                            return true;
                        }
                        if (pt[1] == double[].class) {
                            m.invoke(physics, body, new double[]{w, x, y, z});
                            return true;
                        }
                    }
                }
            } catch (Exception ignored) { }
        }

        // 2) body.setQuaternion(...) 시도
        try {
            for (Method m : body.getClass().getMethods()) {
                if (!m.getName().equals("setQuaternion")) continue;

                if (m.getParameterCount() == 4) {
                    Class<?>[] pt = m.getParameterTypes();
                    if (pt[0] == float.class) {
                        m.invoke(body, w, x, y, z);
                        return true;
                    }
                    if (pt[0] == double.class) {
                        m.invoke(body, (double) w, (double) x, (double) y, (double) z);
                        return true;
                    }
                }

                if (m.getParameterCount() == 1) {
                    Class<?> pt = m.getParameterTypes()[0];

                    if (pt == float[].class) {
                        m.invoke(body, new float[]{w, x, y, z});
                        return true;
                    }
                    if (pt == double[].class) {
                        m.invoke(body, new double[]{w, x, y, z});
                        return true;
                    }

                    Object qObj = makeQuaternionObjectIfPossible(pt, w, x, y, z);
                    if (qObj != null) {
                        m.invoke(body, qObj);
                        return true;
                    }
                }
            }
        } catch (Exception ignored) { }

        return false;
    }

    private Object makeQuaternionObjectIfPossible(Class<?> expectedType, float w, float x, float y, float z) {
        if (dQuaternionClass != null && expectedType.isAssignableFrom(dQuaternionClass)) {
            try {
                Object q = dQuaternionClass.getDeclaredConstructor().newInstance();

                try {
                    Method s0 = q.getClass().getMethod("set0", double.class);
                    Method s1 = q.getClass().getMethod("set1", double.class);
                    Method s2 = q.getClass().getMethod("set2", double.class);
                    Method s3 = q.getClass().getMethod("set3", double.class);
                    s0.invoke(q, (double) w);
                    s1.invoke(q, (double) x);
                    s2.invoke(q, (double) y);
                    s3.invoke(q, (double) z);
                    return q;
                } catch (Exception ignored) { }

                try {
                    Method set = q.getClass().getMethod("set", double.class, double.class, double.class, double.class);
                    set.invoke(q, (double) w, (double) x, (double) y, (double) z);
                    return q;
                } catch (Exception ignored) { }

            } catch (Exception ignored) { }
        }
        return null;
    }

    // ========================================================================
    // 메인 업데이트
    // ========================================================================

    public void update(float dt, Vec3 currentEntityPos) {
        if (usePhysics && physicsInitialized) {
            updatePhysicsWithCollision(dt, currentEntityPos);
        } else {
            updateKinematic(dt);
        }
    }

    public void update(float dt) {
        update(dt, initialAnchorPosition != null ? initialAnchorPosition : Vec3.ZERO);
    }

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
            if (motorsEnabled) applyJointControls();
            physics.step(subDt);
        }

        syncJointStates();
    }

    private void applyJointControls() {
        for (Map.Entry<String, Object> entry : odeJoints.entrySet()) {
            String jointName = entry.getKey();
            Object odeJoint = entry.getValue();
            URDFJoint urdfJoint = joints.get(jointName);

            if (urdfJoint == null) continue;
            if (!urdfJoint.isMovable()) continue;

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
        odeCl = cl;

        try { dQuaternionClass = cl.loadClass("com.kAIS.ode4j.math.DQuaternion"); } catch (Exception ignored) { }
        try { dMatrix3Class = cl.loadClass("com.kAIS.ode4j.math.DMatrix3"); } catch (Exception ignored) { }
    }

    private void detectODE4JVersion() {
        try {
            bodySetMassMethod = dBodyClass.getMethod("setMass", dMassClass);
            odeVersion = ODE4JVersion.V03X;
            return;
        } catch (NoSuchMethodException e) { /* ignore */ }

        try {
            physics.getClassLoader().loadClass("com.kAIS.ode4j.ode.internal.DxMass");
            odeVersion = ODE4JVersion.V05X;
            return;
        } catch (ClassNotFoundException e) { /* ignore */ }

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
            } catch (NoSuchMethodException e2) { /* ignore */ }
        }

        try {
            massSetSphereMethod = dMassClass.getMethod("setSphere", double.class, double.class);
        } catch (NoSuchMethodException e) { /* ignore */ }
    }

    // ========================================================================
    // 물리 모델 빌드
    // ========================================================================

    private void buildPhysicsModel() throws Exception {
        if (urdfModel == null) throw new IllegalStateException("URDFModel is null");

        Object world = physics.getWorld();
        findMassSetMethods();

        // 링크 → body + geom
        for (URDFLink link : urdfModel.links) {
            if (link == null || isFixedLink(link)) continue;

            Object body = createBodyForLink(link, world);
            if (body != null) {
                bodies.put(link.name, body);
                linkRadii.put(link.name, estimateLinkRadius(link));
            }
        }

        // ✅ PATCH: 조인트 생성은 모든 타입(FIXED 포함)
        for (URDFJoint joint : joints.values()) {
            if (joint == null) continue;
            Object odeJoint = createODEJoint(joint, world);
            if (odeJoint != null) odeJoints.put(joint.name, odeJoint);
        }

        if (rootBodyLinkName != null && !bodies.containsKey(rootBodyLinkName)) {
            rootBodyLinkName = bodies.keySet().stream().findFirst().orElse(null);
        }

        logger.info("Physics model: {} bodies, {} geoms, {} joints (rootBody = {})",
                bodies.size(), geoms.size(), odeJoints.size(), rootBodyLinkName);

        if (bodies.isEmpty()) throw new IllegalStateException("No ODE bodies created");
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
                        float s = Math.max(g.scale.x, Math.max(g.scale.y, g.scale.z));
                        s = Math.max(0.05f, Math.abs(s));
                        return Math.max(0.08f, s * 0.25f);
                    }
                    break;
            }
        }

        String name = link.name.toLowerCase();
        if (name.contains("torso") || name.contains("body") || name.contains("chest")) return 0.25f;
        if (name.contains("head")) return 0.12f;
        if (name.contains("arm") || name.contains("leg")) return 0.08f;
        if (name.contains("hand") || name.contains("foot")) return 0.06f;

        return defaultRadius;
    }

    // ========================================================================
    // Body + Geom 생성
    // ========================================================================

    private Object createBodyForLink(URDFLink link, Object world) {
        try {
            Object body = physics.createBody();
            if (body == null) return null;

            GeometryInfo geomInfo = setDefaultMass(body, link);

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

                try { physics.setGeomOffsetPosition(geom, 0, 0, 0); } catch (Exception ignored) { }

                geoms.put(link.name, geom);
                physics.registerDynamicGeom(geom);
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
        double lx, ly, lz;
        double radius;
        double height;
        GeometryInfo(String type) { this.type = type; }
    }

    private Object createGeomForLink(URDFLink link, GeometryInfo geomInfo) {
        try {
            return switch (geomInfo.type) {
                case "box" -> physics.createBoxGeom(
                        geomInfo.lx * physicsScale,
                        geomInfo.ly * physicsScale,
                        geomInfo.lz * physicsScale
                );
                case "sphere" -> physics.createSphereGeom(geomInfo.radius * physicsScale);
                case "cylinder" -> physics.createCylinderGeom(
                        geomInfo.radius * physicsScale,
                        geomInfo.height * physicsScale
                );
                default -> physics.createSphereGeom(estimateLinkRadius(link) * physicsScale);
            };
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
                if (massSetBoxMethod == null && massSetSphereMethod == null) findMassSetMethods();

                switch (geom.type) {
                    case BOX:
                        if (geom.boxSize != null) {
                            lx = Math.max(0.05, Math.abs(geom.boxSize.x));
                            ly = Math.max(0.05, Math.abs(geom.boxSize.y));
                            lz = Math.max(0.05, Math.abs(geom.boxSize.z));

                            geomInfo.type = "box";
                            geomInfo.lx = lx; geomInfo.ly = ly; geomInfo.lz = lz;
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
                        // ✅ PATCH: collider도 box로 맞춰서 sphere로 떨어지는 꼬임 방지
                        if (geom.scale != null) {
                            double sx = Math.abs(geom.scale.x);
                            double sy = Math.abs(geom.scale.y);
                            double sz = Math.abs(geom.scale.z);
                            double s = Math.max(sx, Math.max(sy, sz));
                            s = Math.max(0.05, s);

                            double size = s * 0.5; // 휴리스틱
                            geomInfo.type = "box";
                            geomInfo.lx = size;
                            geomInfo.ly = size;
                            geomInfo.lz = size;

                            lx = geomInfo.lx;
                            ly = geomInfo.ly;
                            lz = geomInfo.lz;
                        }
                        break;
                }
            }

            if (massSetBoxMethod != null) {
                massSetBoxMethod.invoke(mass, density,
                        lx * physicsScale, ly * physicsScale, lz * physicsScale);
            } else if (massSetSphereMethod != null) {
                double r = Math.max(0.05, Math.max(lx, Math.max(ly, lz)) * 0.5);
                geomInfo.type = "sphere";
                geomInfo.radius = r;
                massSetSphereMethod.invoke(mass, density, r * physicsScale);
            } else {
                setMassValue(mass, 1.0);
            }

            // URDF inertial mass override (inertia도 같이 스케일되는 adjust 우선)
            if (link != null && link.inertial != null && link.inertial.mass != null) {
                float mv = link.inertial.mass.value;
                if (mv > 0 && Float.isFinite(mv)) setMassValue(mass, mv);
            }

            applyMassToBody(body, mass);

        } catch (Exception e) {
            logger.warn("Failed to set mass for {}", link != null ? link.name : "unknown", e);
        }

        return geomInfo;
    }

    private void setMassValue(Object mass, double value) {
        try {
            Method adjust = dMassClass.getMethod("adjust", double.class);
            adjust.invoke(mass, value);
            return;
        } catch (Exception ignored) { }

        try {
            Method setMass = dMassClass.getMethod("setMass", double.class);
            setMass.invoke(mass, value);
        } catch (Exception ignored) { }
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
        } catch (Exception ignored) { }

        logger.warn("Could not apply mass to body");
    }

    // ========================================================================
    // 조인트 생성
    // ========================================================================

    private Object createODEJoint(URDFJoint joint, Object world) {
        try {
            Object parentBody = bodies.get(joint.parentLinkName);
            Object childBody  = bodies.get(joint.childLinkName);

            // ✅ PATCH: world attach joint는 기본 스킵 (루트 자유)
            if (!allowWorldAttachment && (parentBody == null || childBody == null)) return null;

            if (parentBody == null && childBody == null) return null;

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

    private Object createHingeJoint(URDFJoint joint, Object world, Object parentBody, Object childBody) throws Exception {
        Method createHinge = odeHelperClass.getMethod("createHingeJoint", dWorldClass, dJointGroupClass);
        Object odeJoint = createHinge.invoke(null, world, null);

        Method attach = dHingeJointClass.getMethod("attach", dBodyClass, dBodyClass);
        attach.invoke(odeJoint, childBody, parentBody);

        double[] axis = getJointAxis(joint);
        double len = Math.sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
        if (len < 1e-6) { axis[0]=0; axis[1]=0; axis[2]=1; }
        else { axis[0]/=len; axis[1]/=len; axis[2]/=len; }

        Method setAxis = dHingeJointClass.getMethod("setAxis", double.class, double.class, double.class);
        setAxis.invoke(odeJoint, axis[0], axis[1], axis[2]);

        // 앵커 설정(생성 시점)
        try {
            Method setAnchor = dHingeJointClass.getMethod("setAnchor", double.class, double.class, double.class);

            double anchorX=0, anchorY=0, anchorZ=0;
            boolean baseFromBody = false;

            if (physics != null) {
                if (childBody != null) {
                    double[] p = physics.getBodyPosition(childBody);
                    if (p != null && p.length >= 3) {
                        anchorX=p[0]; anchorY=p[1]; anchorZ=p[2];
                        baseFromBody = true;
                    }
                }
                if (!baseFromBody && parentBody != null) {
                    double[] p = physics.getBodyPosition(parentBody);
                    if (p != null && p.length >= 3) {
                        anchorX=p[0]; anchorY=p[1]; anchorZ=p[2];
                    }
                }
            }

            if (joint.origin != null && joint.origin.xyz != null) {
                anchorX += joint.origin.xyz.x * physicsScale;
                anchorY += joint.origin.xyz.y * physicsScale;
                anchorZ += joint.origin.xyz.z * physicsScale;
            }

            setAnchor.invoke(odeJoint, anchorX, anchorY, anchorZ);
        } catch (Exception ignored) { }

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
        } catch (Exception ignored) { }
    }

    private Object createSliderJoint(URDFJoint joint, Object world, Object parentBody, Object childBody) throws Exception {
        Method createSlider = odeHelperClass.getMethod("createSliderJoint", dWorldClass, dJointGroupClass);
        Object odeJoint = createSlider.invoke(null, world, null);

        Method attach = dSliderJointClass.getMethod("attach", dBodyClass, dBodyClass);
        attach.invoke(odeJoint, childBody, parentBody);

        double[] axis = getJointAxis(joint);
        Method setAxis = dSliderJointClass.getMethod("setAxis", double.class, double.class, double.class);
        setAxis.invoke(odeJoint, axis[0], axis[1], axis[2]);

        return odeJoint;
    }

    private Object createFixedJoint(URDFJoint joint, Object world, Object parentBody, Object childBody) throws Exception {
        Method createFixed = odeHelperClass.getMethod("createFixedJoint", dWorldClass, dJointGroupClass);
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

    private void applyJointControl(Object odeJoint, URDFJoint urdfJoint, float targetPos, float targetVel) {
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
        } catch (Exception ignored) { }
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
        } catch (Exception ignored) { }
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
        } catch (Exception ignored) { }
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
            if (j == null) continue;
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
    // Quaternion 읽기 유틸 (네 코드 유지)
    // ========================================================================

    private float[] tryReadBodyQuaternionWXYZ(Object body) {
        if (body == null) return null;

        float[] q = tryReadQuaternionFromPhysicsManager(body);
        if (q != null) return q;

        q = tryReadQuaternionFromBodyObject(body);
        if (q != null) return q;

        q = tryReadQuaternionFromRotation(body);
        return q;
    }

    private float[] tryReadQuaternionFromPhysicsManager(Object body) {
        if (physics == null) return null;
        try {
            for (Method m : physics.getClass().getMethods()) {
                String n = m.getName();
                if (m.getParameterCount() != 1) continue;
                if (!(n.equals("getBodyQuaternion") || n.equals("getBodyQuat") || n.equals("getQuaternion"))) continue;

                Object ret = m.invoke(physics, body);
                float[] q = parseQuatWXYZ(ret);
                if (q != null) return q;
            }
        } catch (Exception ignored) { }
        return null;
    }

    private float[] tryReadQuaternionFromBodyObject(Object body) {
        try {
            for (Method m : body.getClass().getMethods()) {
                if (!m.getName().equals("getQuaternion")) continue;

                if (m.getParameterCount() == 0) {
                    Object ret = m.invoke(body);
                    float[] q = parseQuatWXYZ(ret);
                    if (q != null) return q;
                }

                if (m.getParameterCount() == 1) {
                    Class<?> pt = m.getParameterTypes()[0];
                    Object out = createQuaternionOut(pt);
                    if (out == null) continue;

                    m.invoke(body, out);
                    float[] q = parseQuatWXYZ(out);
                    if (q != null) return q;
                }
            }
        } catch (Exception ignored) { }
        return null;
    }

    private Object createQuaternionOut(Class<?> expectedParamType) {
        try {
            if (!expectedParamType.isInterface()) return expectedParamType.getDeclaredConstructor().newInstance();
        } catch (Exception ignored) { }

        try {
            if (dQuaternionClass != null && expectedParamType.isAssignableFrom(dQuaternionClass)) {
                return dQuaternionClass.getDeclaredConstructor().newInstance();
            }
        } catch (Exception ignored) { }

        return null;
    }

    private float[] tryReadQuaternionFromRotation(Object body) {
        try {
            for (Method m : body.getClass().getMethods()) {
                if (!m.getName().equals("getRotation")) continue;

                if (m.getParameterCount() == 0) {
                    Object rot = m.invoke(body);
                    float[] q = parseRotationToQuatWXYZ(rot);
                    if (q != null) return q;
                }

                if (m.getParameterCount() == 1) {
                    Class<?> pt = m.getParameterTypes()[0];
                    Object out = createMatrix3Out(pt);
                    if (out == null) continue;

                    m.invoke(body, out);
                    float[] q = parseRotationToQuatWXYZ(out);
                    if (q != null) return q;
                }
            }
        } catch (Exception ignored) { }

        float[] q2 = tryReadRotationFromPhysicsManagerAnySignature(body);
        if (q2 != null) return q2;

        return null;
    }

    private Object createMatrix3Out(Class<?> expectedParamType) {
        try {
            if (!expectedParamType.isInterface()) return expectedParamType.getDeclaredConstructor().newInstance();
        } catch (Exception ignored) { }

        try {
            if (dMatrix3Class != null && expectedParamType.isAssignableFrom(dMatrix3Class)) {
                return dMatrix3Class.getDeclaredConstructor().newInstance();
            }
        } catch (Exception ignored) { }

        return null;
    }

    private float[] tryReadRotationFromPhysicsManagerAnySignature(Object body) {
        if (physics == null) return null;

        try {
            for (Method m : physics.getClass().getMethods()) {
                String ln = m.getName().toLowerCase(Locale.ROOT);
                if (!ln.contains("rotation")) continue;

                if (m.getParameterCount() == 1) {
                    Object ret = m.invoke(physics, body);
                    float[] q = parseRotationToQuatWXYZ(ret);
                    if (q != null) return q;
                }

                if (m.getParameterCount() == 2) {
                    Class<?> p1 = m.getParameterTypes()[1];

                    if (p1 == double[].class) {
                        double[] out = new double[12];
                        m.invoke(physics, body, out);
                        float[] q = parseRotationToQuatWXYZ(out);
                        if (q != null) return q;
                    } else if (p1 == float[].class) {
                        float[] out = new float[12];
                        m.invoke(physics, body, out);
                        float[] q = parseRotationToQuatWXYZ(out);
                        if (q != null) return q;
                    }
                }
            }
        } catch (Exception ignored) { }

        return null;
    }

    private float[] parseQuatWXYZ(Object qObj) {
        if (qObj == null) return null;

        if (qObj instanceof double[]) {
            double[] a = (double[]) qObj;
            if (a.length >= 4) return normalizeQuatWXYZ((float)a[0], (float)a[1], (float)a[2], (float)a[3]);
        }
        if (qObj instanceof float[]) {
            float[] a = (float[]) qObj;
            if (a.length >= 4) return normalizeQuatWXYZ(a[0], a[1], a[2], a[3]);
        }

        try {
            Method g0 = qObj.getClass().getMethod("get0");
            Method g1 = qObj.getClass().getMethod("get1");
            Method g2 = qObj.getClass().getMethod("get2");
            Method g3 = qObj.getClass().getMethod("get3");
            float w = ((Number) g0.invoke(qObj)).floatValue();
            float x = ((Number) g1.invoke(qObj)).floatValue();
            float y = ((Number) g2.invoke(qObj)).floatValue();
            float z = ((Number) g3.invoke(qObj)).floatValue();
            return normalizeQuatWXYZ(w, x, y, z);
        } catch (Exception ignored) { }

        try {
            Method gw = qObj.getClass().getMethod("getW");
            Method gx = qObj.getClass().getMethod("getX");
            Method gy = qObj.getClass().getMethod("getY");
            Method gz = qObj.getClass().getMethod("getZ");
            float w = ((Number) gw.invoke(qObj)).floatValue();
            float x = ((Number) gx.invoke(qObj)).floatValue();
            float y = ((Number) gy.invoke(qObj)).floatValue();
            float z = ((Number) gz.invoke(qObj)).floatValue();
            return normalizeQuatWXYZ(w, x, y, z);
        } catch (Exception ignored) { }

        return null;
    }

    private float[] parseRotationToQuatWXYZ(Object rotObj) {
        if (rotObj == null) return null;

        if (rotObj instanceof double[]) {
            double[] r = (double[]) rotObj;
            if (r.length >= 12) {
                return quatFromMat3_WXYZ(
                        (float) r[0], (float) r[1], (float) r[2],
                        (float) r[4], (float) r[5], (float) r[6],
                        (float) r[8], (float) r[9], (float) r[10]
                );
            }
            if (r.length >= 9) {
                return quatFromMat3_WXYZ(
                        (float) r[0], (float) r[1], (float) r[2],
                        (float) r[3], (float) r[4], (float) r[5],
                        (float) r[6], (float) r[7], (float) r[8]
                );
            }
        }

        if (rotObj instanceof float[]) {
            float[] r = (float[]) rotObj;
            if (r.length >= 12) {
                return quatFromMat3_WXYZ(r[0],r[1],r[2],r[4],r[5],r[6],r[8],r[9],r[10]);
            }
            if (r.length >= 9) {
                return quatFromMat3_WXYZ(r[0],r[1],r[2],r[3],r[4],r[5],r[6],r[7],r[8]);
            }
        }

        try {
            Method get00 = rotObj.getClass().getMethod("get00");
            Method get01 = rotObj.getClass().getMethod("get01");
            Method get02 = rotObj.getClass().getMethod("get02");
            Method get10 = rotObj.getClass().getMethod("get10");
            Method get11 = rotObj.getClass().getMethod("get11");
            Method get12 = rotObj.getClass().getMethod("get12");
            Method get20 = rotObj.getClass().getMethod("get20");
            Method get21 = rotObj.getClass().getMethod("get21");
            Method get22 = rotObj.getClass().getMethod("get22");

            return quatFromMat3_WXYZ(
                    ((Number)get00.invoke(rotObj)).floatValue(),
                    ((Number)get01.invoke(rotObj)).floatValue(),
                    ((Number)get02.invoke(rotObj)).floatValue(),
                    ((Number)get10.invoke(rotObj)).floatValue(),
                    ((Number)get11.invoke(rotObj)).floatValue(),
                    ((Number)get12.invoke(rotObj)).floatValue(),
                    ((Number)get20.invoke(rotObj)).floatValue(),
                    ((Number)get21.invoke(rotObj)).floatValue(),
                    ((Number)get22.invoke(rotObj)).floatValue()
            );
        } catch (Exception ignored) { }

        return null;
    }

    private float[] quatFromMat3_WXYZ(float m00,float m01,float m02,
                                      float m10,float m11,float m12,
                                      float m20,float m21,float m22) {
        float tr = m00 + m11 + m22;

        float w, x, y, z;
        if (tr > 0f) {
            float s = (float) Math.sqrt(tr + 1f) * 2f;
            w = 0.25f * s;
            x = (m21 - m12) / s;
            y = (m02 - m20) / s;
            z = (m10 - m01) / s;
        } else if (m00 > m11 && m00 > m22) {
            float s = (float) Math.sqrt(1f + m00 - m11 - m22) * 2f;
            w = (m21 - m12) / s;
            x = 0.25f * s;
            y = (m01 + m10) / s;
            z = (m02 + m20) / s;
        } else if (m11 > m22) {
            float s = (float) Math.sqrt(1f + m11 - m00 - m22) * 2f;
            w = (m02 - m20) / s;
            x = (m01 + m10) / s;
            y = 0.25f * s;
            z = (m12 + m21) / s;
        } else {
            float s = (float) Math.sqrt(1f + m22 - m00 - m11) * 2f;
            w = (m10 - m01) / s;
            x = (m02 + m20) / s;
            y = (m12 + m21) / s;
            z = 0.25f * s;
        }

        return normalizeQuatWXYZ(w, x, y, z);
    }

    private float[] normalizeQuatWXYZ(float w, float x, float y, float z) {
        if (!Float.isFinite(w) || !Float.isFinite(x) || !Float.isFinite(y) || !Float.isFinite(z)) return null;
        float n = (float) Math.sqrt(w*w + x*x + y*y + z*z);
        if (!Float.isFinite(n) || n < 1e-8f) return new float[]{1f, 0f, 0f, 0f};
        return new float[]{ w/n, x/n, y/n, z/n };
    }

    // ========================================================================
    // 공개 API
    // ========================================================================

    public void setTarget(String name, float value) {
        URDFJoint j = joints.get(name);
        if (j == null) return;

        if (j.type == URDFJoint.JointType.CONTINUOUS) value = wrapToPi(value);
        if (j.limit != null && j.limit.hasLimits()) value = Mth.clamp(value, j.limit.lower, j.limit.upper);

        target.put(name, value);
    }

    public void setTargets(Map<String, Float> targets) {
        for (var e : targets.entrySet()) setTarget(e.getKey(), e.getValue());
    }

    public float getTarget(String name) { return target.getOrDefault(name, 0f); }
    public void setTargetVelocity(String name, float velocity) { targetVelocities.put(name, velocity); }

    public boolean hasJoint(String name) { return joints.containsKey(name); }

    public String findJointIgnoreCase(String name) {
        if (name == null) return null;
        for (String key : joints.keySet()) if (key.equalsIgnoreCase(name)) return key;
        return null;
    }

    public Set<String> getJointNameSet() { return new HashSet<>(joints.keySet()); }

    public List<String> getMovableJointNames() {
        List<String> names = new ArrayList<>();
        for (URDFJoint j : joints.values()) if (j != null && j.isMovable()) names.add(j.name);
        return names;
    }

    public float[] getJointLimits(String name) {
        URDFJoint j = joints.get(name);
        if (j != null && j.limit != null) return new float[]{j.limit.lower, j.limit.upper};
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
            if (j != null && j.isMovable()) positions.put(j.name, j.currentPosition);
        }
        return positions;
    }

    public void setPreviewPosition(String name, float value) {
        URDFJoint j = joints.get(name);
        if (j == null) return;
        if (j.limit != null && j.limit.hasLimits()) value = Mth.clamp(value, j.limit.lower, j.limit.upper);
        j.currentPosition = value;
        target.put(j.name, value);
    }

    public boolean isUsingPhysics() { return usePhysics && physicsInitialized; }

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

    public void setPhysicsGains(float kp, float kd) { this.physicsKp = kp; this.physicsKd = kd; }
    public void setEffortLimits(float maxTorque, float maxForce) { this.maxTorque = maxTorque; this.maxForce = maxForce; }
    public void setPhysicsScale(float scale) { this.physicsScale = scale; }
    public void setMotorsEnabled(boolean enabled) { this.motorsEnabled = enabled; }
    public boolean isMotorsEnabled() { return motorsEnabled; }

    public void setPhysicsSubSteps(int subSteps) { this.physicsSubSteps = Math.max(1, Math.min(subSteps, 10)); }

    public void applyExternalForce(String linkName, float fx, float fy, float fz) {
        if (!usePhysics) return;
        Object body = bodies.get(linkName);
        if (body != null && physics != null) physics.addForce(body, fx, fy, fz);
    }

    public void applyExternalTorque(String linkName, float tx, float ty, float tz) {
        if (!usePhysics) return;
        Object body = bodies.get(linkName);
        if (body != null && physics != null) physics.addTorque(body, tx, ty, tz);
    }

    public void setGravity(float x, float y, float z) {
        if (physics != null) physics.setGravity(x, y, z);
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

        spawnPoseCaptured = false;
        spawnBodyOffsetsFromRoot.clear();
        spawnBodyQuatWxyz.clear();
        warnedNoQuaternionSetter = false;

        logger.info("URDFSimpleController cleaned up");
    }

    public void resetPhysics() {
        for (URDFJoint j : joints.values()) {
            if (j == null) continue;
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

        spawnPoseCaptured = false;
        spawnBodyOffsetsFromRoot.clear();
        spawnBodyQuatWxyz.clear();
        warnedNoQuaternionSetter = false;

        logger.info("Physics state reset");
    }

    // ========================================================================
    // 유틸리티
    // ========================================================================

    private float getLinkRadius(String linkName) {
        Float r = linkRadii.get(linkName);
        if (r == null || r <= 0f || !Float.isFinite(r)) r = 0.1f;
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
