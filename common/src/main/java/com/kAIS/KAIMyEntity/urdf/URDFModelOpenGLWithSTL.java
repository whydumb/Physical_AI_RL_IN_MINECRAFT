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
import java.util.concurrent.ConcurrentHashMap;

/**
 * URDF 모델 렌더링 + ODE4J 물리 통합 + 블록 충돌 연동 버전
 *
 * ✅ PATCH 포함:
 * - Shift(스닉) 시 바닐라 렌더 오프셋 때문에 "살짝 눌리는" 현상 제거
 * - WASD(플레이어 이동) 시 호스트 엔티티가 움직여도 모델이 같이 끌려가지 않게 "월드 앵커" 고정
 * - 렌더 위치 2배 이동 버그 수정: PoseStack translation을 누적이 아닌 절대값으로 설정
 */
public class URDFModelOpenGLWithSTL implements IMMDModel {
    private static final Logger logger = LogManager.getLogger();
    private static int renderCount = 0;

    private URDFModel robotModel;
    private String modelDir;

    private final URDFSimpleController controller;
    private final Map<String, STLLoader.STLMesh> meshCache = new HashMap<>();

    // 렌더 전용 스케일 (물리는 1블록 = 1m 기준으로 동작)
    private static final float GLOBAL_SCALE = 1.0f;

    private static final boolean FLIP_NORMALS = true;
    private static final boolean DEBUG_MODE = false;

    /**
     * Manual joint locks are persistent by default. Set to a positive value to auto-release after the
     * specified duration in milliseconds, or to 0 to disable locking while still stamping ownership.
     */
    private static final long MANUAL_LOCK_DURATION_MS = -1L;

    // ------------------------------------------------------------------------
    // ✅ "Shift / WASD 따라 움직임" 방지 패치 설정
    // ------------------------------------------------------------------------

    /**
     * true면: 처음 본 월드 위치를 "렌더 앵커"로 고정함.
     * => 플레이어(호스트 엔티티)가 WASD로 움직여도 URDF 모델은 월드에 고정.
     *
     * (로봇 전용 엔티티로 렌더하는 경우에는 false로 두는 게 보통 맞음)
     */
    private static final boolean LOCK_RENDER_ANCHOR_TO_FIRST_POSITION = true;

    /**
     * true면: entityTrans(렌더러가 주는 오프셋)를 무시하고,
     * "월드좌표 - 카메라좌표"로 직접 렌더 위치를 계산함.
     *
     * => Shift(스닉) 눌릴 때 바닐라가 넣는 renderOffset 때문에 모델이 눌리는 현상도 자동 제거됨.
     */
    private static final boolean USE_CAMERA_RELATIVE_TRANSLATION = false;

    /**
     * 물리/블록충돌 컨텍스트도 "로봇의 실제 월드 위치"로 세팅할지 여부.
     * (LOCK_RENDER_ANCHOR_TO_FIRST_POSITION가 true일 때는 true 권장)
     */
    private static final boolean USE_ROOT_WORLD_POS_FOR_WORLD_CONTEXT = true;

    /**
     * 렌더 앵커(월드 고정 기준점)
     */
    private Vec3 renderAnchorWorldPos = null;

    /**
     * Shift(스닉) 눌렀을 때 renderOffset 상쇄용
     */
    private float lastNonSneakEntityTransY = 0f;
    private boolean hasLastNonSneakEntityTransY = false;

    // ROS → Minecraft 좌표계 보정
    private static final Vector3f SRC_UP  = new Vector3f(0, 0, 1);
    private static final Vector3f SRC_FWD = new Vector3f(1, 0, 0);
    private static final Vector3f DST_UP  = new Vector3f(0, 1, 0);
    private static final Vector3f DST_FWD = new Vector3f(0, 0, -1);
    private static final Quaternionf Q_ROS2MC = makeUprightQuat(SRC_UP, SRC_FWD, DST_UP, DST_FWD);

    // VMD ↔ URDF 조인트 이름 매핑
    private final Map<String, String> jointNameMapping = new HashMap<>();
    private boolean jointMappingInitialized = false;

    private final Map<String, JointControlState> jointControlStates = new ConcurrentHashMap<>();

    /**
     * Control source priorities. GUI manual control always wins until released.
     */
    public enum JointControlSource {
        RL,
        VMD,
        GUI,
        OTHER
    }

    private static final class JointControlState {
        private boolean manualLocked = false;
        private long manualLockExpiresAt = 0L;

        boolean isManualLocked() {
            if (!manualLocked) {
                return false;
            }
            if (manualLockExpiresAt > 0 && System.currentTimeMillis() > manualLockExpiresAt) {
                manualLocked = false;
            }
            return manualLocked;
        }

        void markManualLocked() {
            manualLocked = true;
            if (MANUAL_LOCK_DURATION_MS > 0) {
                manualLockExpiresAt = System.currentTimeMillis() + MANUAL_LOCK_DURATION_MS;
            } else if (MANUAL_LOCK_DURATION_MS == 0) {
                manualLocked = false;
                manualLockExpiresAt = 0L;
            } else {
                manualLockExpiresAt = -1L;
            }
        }

        void clearManualLock() {
            manualLocked = false;
            manualLockExpiresAt = 0L;
        }

        void clearManualLockIfExpired() {
            if (manualLocked && manualLockExpiresAt > 0 && System.currentTimeMillis() > manualLockExpiresAt) {
                clearManualLock();
            }
        }
    }

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

    // ========================================================================
    // ✅ Render Anchor / Root World Pos 유틸
    // ========================================================================

    /**
     * 렌더 기준점(앵커) 월드 좌표를 결정.
     * - LOCK_RENDER_ANCHOR_TO_FIRST_POSITION = true: 첫 위치로 고정
     * - false: 엔티티 위치 따라감(기존 동작)
     */
    private Vec3 resolveRenderAnchorWorldPos(Entity entityIn) {
        if (entityIn == null) {
            return (renderAnchorWorldPos != null) ? renderAnchorWorldPos : Vec3.ZERO;
        }

        if (!LOCK_RENDER_ANCHOR_TO_FIRST_POSITION) {
            return entityIn.position();
        }

        if (renderAnchorWorldPos == null) {
            renderAnchorWorldPos = entityIn.position();
            if (DEBUG_MODE) {
                logger.info("Render anchor locked at {}", renderAnchorWorldPos);
            }
        }
        return renderAnchorWorldPos;
    }

    /**
     * 현재 로봇 "루트 링크"의 월드 좌표를 계산.
     * baseWorldPos(앵커) + physics rootLocal(있으면)로 구성.
     */
    private Vec3 computeRootWorldPos(Vec3 baseWorldPos) {
        if (baseWorldPos == null) {
            return Vec3.ZERO;
        }
        if (controller != null && controller.isUsingPhysics()) {
            float[] rootLocal = controller.getRootLinkLocalPosition(baseWorldPos);
            if (rootLocal != null && rootLocal.length >= 3) {
                return baseWorldPos.add(rootLocal[0], rootLocal[1], rootLocal[2]);
            }
        }
        return baseWorldPos;
    }

    /**
     * 월드 좌표 -> 카메라 상대 좌표(렌더 좌표)로 변환
     */
    private static Vector3f worldToCameraRelative(Vec3 worldPos) {
        Minecraft mc = Minecraft.getInstance();
        if (mc == null || mc.gameRenderer == null || mc.gameRenderer.getMainCamera() == null) {
            return new Vector3f(); // fallback
        }
        Vec3 cam = mc.gameRenderer.getMainCamera().getPosition();
        return new Vector3f(
                (float) (worldPos.x - cam.x),
                (float) (worldPos.y - cam.y),
                (float) (worldPos.z - cam.z)
        );
    }

    // ========================================================================
    // 조인트 이름 매핑
    // ========================================================================

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
                Vec3 base = resolveRenderAnchorWorldPos(entity);
                Vec3 rootWorld = computeRootWorldPos(base);
                Vec3 ctxPos = USE_ROOT_WORLD_POS_FOR_WORLD_CONTEXT ? rootWorld : base;
                controller.setWorldContext(entity.level(), ctxPos);
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
        setJointTarget(name, value, JointControlSource.OTHER);
    }

    public void setJointTarget(String name, float value, JointControlSource source) {
        if (controller == null) return;

        String resolvedName = resolveJointName(name);
        if (resolvedName != null) {
            if (!canApplyJointTarget(resolvedName, source)) {
                if (DEBUG_MODE && renderCount < 5) {
                    logger.debug("✗ Joint '{}' target blocked by manual lock (source={})", resolvedName, source);
                }
                return;
            }
            controller.setTarget(resolvedName, value);
            updateJointControlState(resolvedName, source);
        } else if (DEBUG_MODE && renderCount < 5) {
            logger.warn("✗ Joint NOT FOUND: '{}'", name);
        }
    }

    public void setJointTargets(Map<String, Float> values) {
        setJointTargets(values, JointControlSource.OTHER);
    }

    public void setJointTargets(Map<String, Float> values, JointControlSource source) {
        if (values == null) return;
        for (Map.Entry<String, Float> entry : values.entrySet()) {
            setJointTarget(entry.getKey(), entry.getValue(), source);
        }
    }

    public void setManualJointTarget(String name, float value) {
        setJointPreview(name, value, JointControlSource.GUI);
        setJointTarget(name, value, JointControlSource.GUI);
    }

    public void releaseManualJointLock(String name) {
        String resolvedName = resolveJointName(name);
        if (resolvedName == null) {
            resolvedName = name;
        }
        JointControlState state = jointControlStates.get(resolvedName);
        if (state != null) {
            state.clearManualLock();
        }
    }

    public void clearManualJointLocks() {
        jointControlStates.values().forEach(JointControlState::clearManualLock);
    }

    public boolean isJointLockedByManual(String name) {
        String resolvedName = resolveJointName(name);
        if (resolvedName == null) {
            resolvedName = name;
        }
        JointControlState state = jointControlStates.get(resolvedName);
        return state != null && state.isManualLocked();
    }

    public boolean hasManualJointLocks() {
        for (JointControlState state : jointControlStates.values()) {
            if (state.isManualLocked()) {
                return true;
            }
        }
        return false;
    }

    private boolean canApplyJointTarget(String resolvedName, JointControlSource source) {
        if (source == JointControlSource.GUI) {
            return true;
        }
        JointControlState state = jointControlStates.get(resolvedName);
        if (state == null) {
            return true;
        }
        return !state.isManualLocked();
    }

    private void updateJointControlState(String resolvedName, JointControlSource source) {
        JointControlState state = jointControlStates.computeIfAbsent(resolvedName, key -> new JointControlState());
        if (source == JointControlSource.GUI) {
            state.markManualLocked();
            return;
        }
        state.clearManualLockIfExpired();
    }

    public void setJointVelocity(String name, float velocity) {
        if (controller == null) return;
        String resolvedName = resolveJointName(name);
        if (resolvedName != null) {
            controller.setTargetVelocity(resolvedName, velocity);
        }
    }

    public void setJointPreview(String name, float value) {
        setJointPreview(name, value, JointControlSource.OTHER);
    }

    public void setJointPreview(String name, float value, JointControlSource source) {
        if (controller == null) return;
        String resolvedName = resolveJointName(name);
        if (resolvedName == null) {
            return;
        }

        if (source != JointControlSource.GUI) {
            JointControlState state = jointControlStates.get(resolvedName);
            if (state != null && state.isManualLocked()) {
                if (DEBUG_MODE && renderCount < 5) {
                    logger.debug("✗ Joint '{}' preview blocked by manual lock (source={})", resolvedName, source);
                }
                return;
            }
        }

        controller.setPreviewPosition(resolvedName, value);

        if (source == JointControlSource.GUI) {
            JointControlState state = jointControlStates.computeIfAbsent(resolvedName, key -> new JointControlState());
            state.markManualLocked();
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
    // 물리 사용 여부 / 컨트롤러 접근
    // ========================================================================

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

        if (renderCount % 120 == 1) {
            logger.info("=== URDF RENDER #{} (Scale: {}, Physics: {}, AnchorLock: {}) ===",
                    renderCount, GLOBAL_SCALE, isUsingPhysics(), LOCK_RENDER_ANCHOR_TO_FIRST_POSITION);
        }

        // 컨트롤러에 월드 컨텍스트 전달 (블록 충돌/물리에서 사용)
        if (controller != null && entityIn != null) {
            Level level = entityIn.level();
            Vec3 base = resolveRenderAnchorWorldPos(entityIn);
            Vec3 rootWorld = computeRootWorldPos(base);
            Vec3 ctxPos = USE_ROOT_WORLD_POS_FOR_WORLD_CONTEXT ? rootWorld : base;
            controller.setWorldContext(level, ctxPos);
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

            // ----------------------------------------------------------------
            // ✅ 렌더 위치 계산 (원래 코드 방식 유지: entityTrans 기반)
            // + WASD(호스트 엔티티 이동) 상쇄: (anchor - currentPos) 델타를 더함
            // + physics rootLocal도 동일하게 더함
            // ----------------------------------------------------------------
            Vector3f rootOffset = (entityTrans != null) ? new Vector3f(entityTrans) : new Vector3f();

            // 1) Shift(스닉) 눌렀을 때 renderOffset 상쇄
            if (entityIn != null && entityTrans != null) {
                if (!entityIn.isShiftKeyDown()) {
                    lastNonSneakEntityTransY = entityTrans.y;
                    hasLastNonSneakEntityTransY = true;
                } else if (hasLastNonSneakEntityTransY) {
                    float sneakDelta = entityTrans.y - lastNonSneakEntityTransY;
                    rootOffset.y -= sneakDelta;
                }
            }

            // 2) WASD 따라 움직임 방지 (앵커 고정이면: 현재 엔티티 이동분을 상쇄)
            if (LOCK_RENDER_ANCHOR_TO_FIRST_POSITION && entityIn != null) {
                Vec3 anchor = resolveRenderAnchorWorldPos(entityIn);
                Vec3 cur = entityIn.position();
                rootOffset.add(
                        (float) (anchor.x - cur.x),
                        (float) (anchor.y - cur.y),
                        (float) (anchor.z - cur.z)
                );
            }

            // 3) 물리 루트 위치 오프셋(기존 코드랑 동일 개념)
            if (controller != null && controller.isUsingPhysics()) {
                Vec3 baseForRootLocal = (entityIn != null)
                        ? (LOCK_RENDER_ANCHOR_TO_FIRST_POSITION ? resolveRenderAnchorWorldPos(entityIn) : entityIn.position())
                        : null;

                float[] rootLocal = controller.getRootLinkLocalPosition(baseForRootLocal);
                if (rootLocal != null && rootLocal.length >= 3) {
                    rootOffset.add(rootLocal[0], rootLocal[1], rootLocal[2]);
                }
            }

            // 4) 최종 적용 (덮어쓰기 말고 translate로 "기존 파이프라인" 유지)
            poseStack.translate(rootOffset.x(), rootOffset.y(), rootOffset.z());

            // ✅ PATCH: 물리 루트 바디 회전(roll/pitch 포함)을 렌더에 반영
            if (controller != null && controller.isUsingPhysics()) {
                float[] qWxyz = controller.getRootBodyWorldQuaternionWXYZ();
                if (qWxyz != null && qWxyz.length >= 4) {
                    float w = qWxyz[0];
                    float x = qWxyz[1];
                    float y = qWxyz[2];
                    float z = qWxyz[3];

                    if (Float.isFinite(w) && Float.isFinite(x) && Float.isFinite(y) && Float.isFinite(z)) {
                        // JOML Quaternionf는 (x,y,z,w) 순서
                        Quaternionf qPhys = new Quaternionf(x, y, z, w).normalize();
                        poseStack.mulPose(qPhys);
                    }
                }
            }

            // ROS → Minecraft 좌표계 회전 (기존 유지)
            poseStack.mulPose(new Quaternionf(Q_ROS2MC));

            // 메쉬 스케일
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

    /**
     * PoseStack의 translation을 누적이 아닌 절대값으로 설정.
     * 마인크래프트 렌더러가 이미 엔티티-카메라 상대좌표로 세팅한 경우에 사용.
     */
    private static void setPoseStackTranslation(PoseStack poseStack, Vector3f t) {
        Matrix4f m = poseStack.last().pose();
        // JOML Matrix4f에서 translation 성분은 m30/m31/m32
        m.m30(t.x);
        m.m31(t.y);
        m.m32(t.z);
    }

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
