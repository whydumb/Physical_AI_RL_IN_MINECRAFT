package com.kAIS.KAIMyEntity.rl;

import com.kAIS.KAIMyEntity.urdf.URDFModelOpenGLWithSTL;
import com.kAIS.KAIMyEntity.urdf.URDFModelOpenGLWithSTL.JointControlSource;
import com.kAIS.KAIMyEntity.urdf.control.URDFMotion;
import com.kAIS.KAIMyEntity.urdf.control.URDFSimpleController;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.util.*;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * MuJoCo 스타일 RL 환경 (경량 버전)
 *
 * 핵심 아이디어:
 *  - 물리/기구학(FK/ODE)은 전부 URDF/ODE4J 쪽에서 처리
 *  - 이 클래스는 "상태 관찰 + action → URDF 명령" 만 담당
 *
 * 변경 사항:
 *  - 내부 JointState 물리 시뮬레이터 제거
 *  - 항상 renderer.getJointPosition / getJointVelocity 를 사용
 *  - applyAction() 에서 바로 renderer.setJointTarget / setJointVelocity 호출
 *  - 루트 위치/속도는 renderer가 아니라, 외부에서 주입하는 공급자(Supplier)를 통해 계산
 */
public class RLEnvironmentCore {
    private static final Logger logger = LogManager.getLogger();

    // ========== 싱글톤 ==========
    private static volatile RLEnvironmentCore instance;

    public static RLEnvironmentCore getInstance() {
        if (instance == null) {
            synchronized (RLEnvironmentCore.class) {
                if (instance == null) {
                    instance = new RLEnvironmentCore();
                }
            }
        }
        return instance;
    }

    // ========== 환경 상태 ==========
    /** URDF + ODE 기반 렌더러 (조인트 상태 조회/명령에만 사용) */
    private URDFModelOpenGLWithSTL renderer;

    /**
     * (신규) 루트 높이 / 위치 공급자
     *
     * - heightSupplier  : 월드 기준 루트 Y (예: 컨트롤러 getApproxBaseHeightWorldY, 또는 mcEntity.getY())
     * - rootXZSupplier  : 월드 기준 [x, z] (예: mcEntity.getX(), mcEntity.getZ())
     *
     * 이 두 공급자를 통해 getRootPosition()을 정의하므로,
     * renderer.getRootWorldPosition() 에 전혀 의존하지 않는다.
     */
    private Supplier<Float> heightSupplier;
    private Supplier<float[]> rootXZSupplier;

    /** 설정값 */
    private final Config config = new Config();

    /** 관절 메타 정보 (이름/리밋/초기값만) */
    private final List<JointMeta> jointMetas = new ArrayList<>();
    private final Map<String, Integer> jointIndexMap = new HashMap<>();

    // 에피소드 상태
    private int stepCount = 0;
    private int episodeCount = 0;
    private float episodeReward = 0f;
    private float lastReward = 0f;
    private boolean isDone = false;
    private boolean isInitialized = false;

    // 학습 상태
    private boolean trainingActive = false;
    private AgentMode agentMode = AgentMode.MANUAL;
    private SimpleAgent agent;

    // 참조 모션 (VMD 등) + 에피소드 시간
    private URDFMotion referenceMotion;
    private float timeInEpisode = 0f;

    // 이전 루트 상태 (속도 추정용)
    private final float[] prevRootPosition = new float[3];
    private final float[] spawnRootPosition = new float[3];
    private boolean spawnPositionInitialized = false;
    private boolean spawnPositionDirty = true;

    // 통계
    private final Statistics stats = new Statistics();

    // 콜백
    private Consumer<String> logCallback;

    // ========== 생성자 ==========
    private RLEnvironmentCore() {
        logger.info("RLEnvironmentCore created");
    }

    // ========== 초기화 ==========
    /**
     * URDF 렌더러 연결 및 환경 초기화
     */
    public void initialize(URDFModelOpenGLWithSTL renderer) {
        this.renderer = renderer;

        if (renderer == null) {
            log("WARN: Renderer is null");
            isInitialized = false;
            return;
        }

        jointMetas.clear();
        jointIndexMap.clear();

        // 1) 움직일 수 있는 관절 이름 수집
        List<String> jointNames = renderer.getMovableJointNames();
        if (jointNames == null || jointNames.isEmpty()) {
            log("WARN: No movable joints found");
            isInitialized = false;
            return;
        }

        int idx = 0;
        for (String jointName : jointNames) {
            float[] limits = renderer.getJointLimits(jointName);
            float lower = (limits != null && limits.length >= 2) ? limits[0] : (float) -Math.PI;
            float upper = (limits != null && limits.length >= 2) ? limits[1] : (float) Math.PI;
            float currentPos = renderer.getJointPosition(jointName);

            JointMeta jm = new JointMeta();
            jm.name = jointName;
            jm.minLimit = lower;
            jm.maxLimit = upper;
            jm.initialPosition = currentPos;

            jointMetas.add(jm);
            jointIndexMap.put(jointName, idx++);
        }

        // 2) 에이전트 초기화 (간단 선형 정책)
        agent = new SimpleAgent(jointMetas.size(), this::getObservationDim);

        // 3) 루트 상태 초기화 (스폰 캐시 기반)
        spawnPositionDirty = true;
        refreshSpawnRootPosition();
        float[] rootPos = getRootPosition();
        System.arraycopy(rootPos, 0, prevRootPosition, 0, 3);

        isInitialized = true;
        log("Initialized: joints=" + jointMetas.size()
                + ", obs=" + getObservationDim()
                + ", act=" + getActionDim());
    }

    // ========== 참조 모션 (VMD/URDFMotion) ==========
    public void setReferenceMotion(URDFMotion motion) {
        this.referenceMotion = motion;
        if (motion != null) {
            log("Reference motion set: " + motion.name + ", keys=" + motion.keys.size());
        } else {
            log("Reference motion cleared");
        }
    }

    public boolean hasReferenceMotion() {
        return referenceMotion != null;
    }

    public float getTimeInEpisode() {
        return timeInEpisode;
    }

    public URDFMotion getReferenceMotion() {
        return referenceMotion;
    }

    // ========== 루트 상태 공급자 설정 ==========
    /**
     * RL에서 사용할 루트 높이/위치 공급자를 등록한다.
     *
     * @param heightSupplier   월드 기준 루트 Y를 반환하는 Supplier (null 허용)
     * @param rootXZSupplier   월드 기준 [x, z] 를 반환하는 Supplier (null 허용, 길이 2 배열)
     *
     * 예)
     *   CopyURDFSimpleController controller = renderer.getController();
     *   env.setRootStateSuppliers(
     *       () -> controller.getApproxBaseHeightWorldY(),
     *       () -> new float[]{ (float)mcEntity.getX(), (float)mcEntity.getZ() }
     *   );
     *
     *   // 임시로 엔티티의 Y만 사용하고 X/Z도 엔티티 기준으로 사용
     *   env.setRootStateSuppliers(
     *       () -> (float)mcEntity.getY(),
     *       () -> new float[]{ (float)mcEntity.getX(), (float)mcEntity.getZ() }
     *   );
     */
    public void setRootStateSuppliers(Supplier<Float> heightSupplier,
                                      Supplier<float[]> rootXZSupplier) {
        this.heightSupplier = heightSupplier;
        this.rootXZSupplier = rootXZSupplier;
        this.spawnPositionDirty = true;
    }

    /** 높이만 바꾸고 싶을 때 */
    public void setHeightSupplier(Supplier<Float> heightSupplier) {
        this.heightSupplier = heightSupplier;
        this.spawnPositionDirty = true;
    }

    /** X/Z만 바꾸고 싶을 때 */
    public void setRootXZSupplier(Supplier<float[]> rootXZSupplier) {
        this.rootXZSupplier = rootXZSupplier;
        this.spawnPositionDirty = true;
    }

    // ========== 메인 틱 ==========
    /**
     * 매 틱마다 호출되는 메인 RL 루프
     *
     * 여기서는:
     *  1) renderer로부터 현재 관측(observation) 읽고
     *  2) agent로부터 action 얻고
     *  3) action을 renderer에 곧장 적용(setJointTarget/Velocity)
     *  4) reward 계산 및 에피소드 관리
     * 를 수행한다.
     *
     * 실제 물리 시뮬레이션은 URDF/ODE 쪽에서 별도로 진행된다.
     */
    public void tick(float deltaTime) {
        if (!isInitialized || !trainingActive) return;
        if (agentMode == AgentMode.MANUAL) return;
        if (renderer == null) return;

        // 1. 현재 관측 (URDF 기반 + 외부 공급 루트 상태)
        float[] observation = getObservation();

        // 2. 에이전트로부터 행동 얻기
        float[] action = agent.selectAction(observation, agentMode);

        // 3. 행동 적용 (URDF에 곧장 명령)
        applyAction(action);

        // 4. 보상 계산
        float reward = calculateReward(action);
        lastReward = reward;
        episodeReward += reward;

        // 5. 종료 조건
        stepCount++;
        boolean terminated = checkTermination();
        boolean truncated = stepCount >= config.maxEpisodeSteps;
        isDone = terminated || truncated;

        // 6. 새 관측 (학습용 nextObs)
        float[] newObservation = getObservation();

        // 7. 경험 저장 및 업데이트 (LEARNING 모드에서만)
        if (agentMode == AgentMode.LEARNING) {
            agent.storeExperience(observation, action, reward, newObservation, isDone);

            if (stepCount % config.updateInterval == 0) {
                agent.update();
            }
        }

        // 8. 에피소드 종료 처리
        if (isDone) {
            endEpisode(terminated ? "terminated" : "truncated");
        }

        // 9. 루트 상태, 에피소드 시간 업데이트
        float[] rootPos = getRootPosition();
        System.arraycopy(rootPos, 0, prevRootPosition, 0, 3);
        timeInEpisode += deltaTime;
    }

    // ========== 환경 리셋 ==========
    public float[] reset() {
        if (!isInitialized || renderer == null) return new float[0];

        stepCount = 0;
        episodeReward = 0f;
        lastReward = 0f;
        isDone = false;
        timeInEpisode = 0f;

        Random rand = config.randomizeInitial ? new Random() : null;

        // 초기 자세 → renderer.setJointTarget 로만 설정 (내부 물리 없음)
        for (JointMeta jm : jointMetas) {
            float initPos = jm.initialPosition;

            if (rand != null) {
                float range = (jm.maxLimit - jm.minLimit);
                float noise = (rand.nextFloat() - 0.5f) * range * config.initNoiseScale;
                initPos += noise;
                initPos = clamp(initPos, jm.minLimit, jm.maxLimit);
            }

            if (renderer != null) {
                renderer.setJointTarget(jm.name, initPos, JointControlSource.RL);
            }
        }

        spawnPositionDirty = true;
        refreshSpawnRootPosition();
        float[] rootPos = getRootPosition();
        System.arraycopy(rootPos, 0, prevRootPosition, 0, 3);

        return getObservation();
    }

    // ========== 행동 적용 ==========
    /**
     * 행동 적용: [-1,1] 범위 액션을 URDF 조인트 명령으로 변환
     *
     * TORQUE 모드는 현재 URDF 컨트롤러에 직접 토크 API가 없으므로
     * DELTA_POSITION과 유사하게 처리하는 것이 현실적이다.
     */
    private void applyAction(float[] action) {
        if (action == null || renderer == null) return;

        int numActions = Math.min(action.length, jointMetas.size());

        for (int i = 0; i < numActions; i++) {
            JointMeta jm = jointMetas.get(i);
            float a = clamp(action[i], -1f, 1f);

            // 현재 실제 관절 상태는 renderer에서 읽는다
            float currentPos = renderer.getJointPosition(jm.name);
            float currentVel = renderer.getJointVelocity(jm.name);

            switch (config.actionMode) {
                case TORQUE -> {
                    // 토크 직접 제어는 현재 미지원이므로,
                    // 간단히 DELTA_POSITION처럼 근처 각도로 이동시키는 식으로 근사.
                    float delta = a * config.maxDeltaPosition;
                    float target = clamp(currentPos + delta, jm.minLimit, jm.maxLimit);
                    renderer.setJointTarget(jm.name, target, JointControlSource.RL);
                }
                case POSITION -> {
                    // [-1,1] → [minLimit, maxLimit]
                    float target = jm.minLimit + (a + 1f) * 0.5f * (jm.maxLimit - jm.minLimit);
                    renderer.setJointTarget(jm.name, target, JointControlSource.RL);
                }
                case DELTA_POSITION -> {
                    float delta = a * config.maxDeltaPosition;
                    float target = clamp(currentPos + delta, jm.minLimit, jm.maxLimit);
                    renderer.setJointTarget(jm.name, target, JointControlSource.RL);
                }
                case VELOCITY -> {
                    float targetVel = a * config.maxVelocity;
                    // URDF 컨트롤러 쪽 setJointVelocity는 "목표 속도" 제어에 사용
                    renderer.setJointVelocity(jm.name, targetVel);
                }
            }
        }
    }

    // ========== 관측 ==========
    public float[] getObservation() {
        List<Float> obs = new ArrayList<>();

        if (renderer == null || jointMetas.isEmpty()) {
            return new float[0];
        }

        // 1. 관절 위치 (정규화 [-1,1])
        for (JointMeta jm : jointMetas) {
            float pos = renderer.getJointPosition(jm.name);
            float range = jm.maxLimit - jm.minLimit;
            float norm = (range > 0f)
                    ? 2f * (pos - jm.minLimit) / range - 1f
                    : 0f;
            obs.add(clamp(norm, -1f, 1f));
        }

        // 2. 관절 속도 (스케일링)
        if (config.includeVelocities) {
            for (JointMeta jm : jointMetas) {
                float vel = renderer.getJointVelocity(jm.name);
                obs.add(vel / config.maxVelocity);
            }
        }

        // 3. 루트 높이 (외부 공급자 기반, 정규화)
        float[] rootPos = getRootPosition();
        float heightRange = config.maxHeight - config.minHeight;
        float heightNorm = (heightRange > 0f)
                ? (rootPos[1] - config.minHeight) / heightRange
                : 0.5f;
        obs.add(heightNorm);

        // 4. 루트 속도 (수평)
        float[] rootVel = getRootVelocity();
        float speedScale = (config.targetSpeed > 0f) ? config.targetSpeed : 1f;
        obs.add(rootVel[0] / speedScale); // x
        obs.add(rootVel[2] / speedScale); // z

        // 5. 목표 속도와의 차이
        float currentSpeed = (float) Math.sqrt(rootVel[0] * rootVel[0] + rootVel[2] * rootVel[2]);
        float speedDiff = (config.targetSpeed > 0f)
                ? (config.targetSpeed - currentSpeed) / speedScale
                : 0f;
        obs.add(speedDiff);

        float[] result = new float[obs.size()];
        for (int i = 0; i < obs.size(); i++) {
            result[i] = obs.get(i);
        }
        return result;
    }

    // ========== 루트 상태 ==========
    /**
     * 스폰 시점에 외부 공급자에서 루트 좌표를 샘플링해 캐싱하고,
     * 이후에는 물리 엔진의 루트 바디 월드 좌표를 우선 사용한다.
     * 물리 좌표를 얻지 못한 경우에 한해 캐시된 스폰 위치를 반환한다.
     */
    private void refreshSpawnRootPosition() {
        if (!spawnPositionDirty && spawnPositionInitialized) {
            return;
        }

        float x = spawnPositionInitialized ? spawnRootPosition[0] : 0f;
        float y = spawnPositionInitialized ? spawnRootPosition[1] : config.targetHeight;
        float z = spawnPositionInitialized ? spawnRootPosition[2] : 0f;

        if (heightSupplier != null) {
            try {
                Float sampledY = heightSupplier.get();
                if (sampledY != null && Float.isFinite(sampledY)) {
                    y = sampledY;
                }
            } catch (Exception e) {
                logger.warn("heightSupplier threw exception while sampling spawn position", e);
            }
        }

        if (rootXZSupplier != null) {
            try {
                float[] xz = rootXZSupplier.get();
                if (xz != null && xz.length >= 2) {
                    if (Float.isFinite(xz[0])) {
                        x = xz[0];
                    }
                    if (Float.isFinite(xz[1])) {
                        z = xz[1];
                    }
                }
            } catch (Exception e) {
                logger.warn("rootXZSupplier threw exception while sampling spawn position", e);
            }
        }

        spawnRootPosition[0] = x;
        spawnRootPosition[1] = y;
        spawnRootPosition[2] = z;
        spawnPositionInitialized = true;
        spawnPositionDirty = false;
    }

    private float[] getRootPosition() {
        if (renderer != null) {
            try {
                URDFSimpleController controller = renderer.getController();
                if (controller != null) {
                    double[] rootWorld = controller.getRootBodyWorldPosition();
                    if (rootWorld != null && rootWorld.length >= 3) {
                        float x = (float) rootWorld[0];
                        float y = (float) rootWorld[1];
                        float z = (float) rootWorld[2];
                        if (Float.isFinite(x) && Float.isFinite(y) && Float.isFinite(z)) {
                            return new float[]{x, y, z};
                        }
                    }
                }
            } catch (Exception e) {
                logger.debug("Failed to read root position from physics", e);
            }
        }

        if (!spawnPositionInitialized) {
            refreshSpawnRootPosition();
        }

        return new float[]{spawnRootPosition[0], spawnRootPosition[1], spawnRootPosition[2]};
    }

    /**
     * 루트 속도 (이전 위치와의 차분)
     */
    private float[] getRootVelocity() {
        float[] current = getRootPosition();
        float dt = (config.timeStep > 0f) ? config.timeStep : 0.02f;
        return new float[]{
                (current[0] - prevRootPosition[0]) / dt,
                (current[1] - prevRootPosition[1]) / dt,
                (current[2] - prevRootPosition[2]) / dt
        };
    }

    // ========== 보상 ==========
    private float calculateReward(float[] action) {
        float reward = 0f;

        // 1. 살아있음 보상
        reward += config.aliveBonus;

        // 2. 높이 유지 보상
        float[] rootPos = getRootPosition();
        float heightDiff = Math.abs(rootPos[1] - config.targetHeight);
        float heightReward = (config.targetHeight > 0f)
                ? 1f - heightDiff / config.targetHeight
                : 0f;
        reward += heightReward * config.heightRewardWeight;

        // 3. 속도 매칭 (걷기/이동)
        float[] rootVel = getRootVelocity();
        float currentSpeed = (float) Math.sqrt(rootVel[0] * rootVel[0] + rootVel[2] * rootVel[2]);
        float speedError = Math.abs(currentSpeed - config.targetSpeed);
        reward -= speedError * config.speedMatchWeight;

        // 4. 제어 비용 (부드러운 동작)
        float controlCost = 0f;
        if (action != null) {
            for (float a : action) {
                controlCost += a * a;
            }
        }
        reward -= controlCost * config.controlCostWeight;

        // 5. 관절 속도 페널티
        float velocityPenalty = 0f;
        if (renderer != null) {
            for (JointMeta jm : jointMetas) {
                float vel = renderer.getJointVelocity(jm.name);
                velocityPenalty += vel * vel;
            }
        }
        reward -= velocityPenalty * config.velocityPenaltyWeight;

        // 6. 관절 제한 근접 페널티
        float limitPenalty = 0f;
        if (renderer != null) {
            for (JointMeta jm : jointMetas) {
                float pos = renderer.getJointPosition(jm.name);
                float range = jm.maxLimit - jm.minLimit;
                if (range <= 0f) continue;
                float margin = 0.1f * range;
                if (pos < jm.minLimit + margin || pos > jm.maxLimit - margin) {
                    limitPenalty += 0.1f;
                }
            }
        }
        reward -= limitPenalty;

        // 7. 좌우 대칭 보상
        reward += calculateSymmetryReward() * config.symmetryRewardWeight;

        // 8. VMD/참조 모션 포즈 매칭 보상 (있을 때만)
        if (referenceMotion != null && config.poseMatchWeight > 0f && renderer != null) {
            Map<String, Float> refPose = referenceMotion.samplePose(timeInEpisode);
            if (!refPose.isEmpty()) {
                float poseError = 0f;
                int count = 0;
                for (JointMeta jm : jointMetas) {
                    Float ref = refPose.get(jm.name);
                    if (ref == null) continue;
                    float cur = renderer.getJointPosition(jm.name);
                    float diff = cur - ref;
                    poseError += diff * diff;
                    count++;
                }
                if (count > 0) {
                    poseError /= count;
                    reward -= poseError * config.poseMatchWeight;
                }
            }
        }

        return reward;
    }

    private float calculateSymmetryReward() {
        if (renderer == null) return 0f;

        float symmetry = 0f;
        int pairs = 0;

        for (JointMeta jm : jointMetas) {
            String name = jm.name;
            // 이름 규칙 기반: 왼쪽 관절 이름 패턴
            if (name.contains("_L_") || name.contains("Left") || name.contains("_l_")) {
                String rightName = name
                        .replace("_L_", "_R_")
                        .replace("Left", "Right")
                        .replace("_l_", "_r_");
                Integer idx = jointIndexMap.get(rightName);
                if (idx != null) {
                    float leftPos = renderer.getJointPosition(name);
                    float rightPos = renderer.getJointPosition(rightName);
                    float diff = Math.abs(leftPos - rightPos);
                    symmetry += 1f - Math.min(diff / (float) Math.PI, 1f);
                    pairs++;
                }
            }
        }
        return (pairs > 0) ? symmetry / pairs : 0f;
    }

    // ========== 종료 조건 ==========
    private boolean checkTermination() {
        if (!config.terminateOnFall) return false;

        float[] rootPos = getRootPosition();

        if (rootPos[1] < config.minHeight) {
            log(String.format("Terminated: height too low (%.3f)", rootPos[1]));
            return true;
        }
        if (rootPos[1] > config.maxHeight) {
            log(String.format("Terminated: height too high (%.3f)", rootPos[1]));
            return true;
        }
        return false;
    }

    public boolean isHealthy() {
        float[] rootPos = getRootPosition();
        return rootPos[1] >= config.minHeight && rootPos[1] <= config.maxHeight;
    }

    // ========== 에피소드 관리 ==========
    private void endEpisode(String reason) {
        episodeCount++;
        stats.recordEpisode(episodeReward, stepCount);

        log(String.format("Episode %d ended (%s): reward=%.2f, steps=%d",
                episodeCount, reason, episodeReward, stepCount));

        if (trainingActive) {
            reset();
        }
    }

    // ========== 학습 제어 ==========
    public void startTraining(AgentMode mode) {
        if (!isInitialized) {
            log("ERROR: Cannot start training - not initialized");
            return;
        }
        agentMode = mode;
        boolean hadManualLocks = renderer != null && renderer.hasManualJointLocks();
        if (renderer != null) {
            renderer.clearManualJointLocks();
        }
        if (hadManualLocks) {
            log("Cleared manual joint overrides before training");
        }
        trainingActive = true;
        reset();
        log("Training started: mode=" + mode);
    }

    public void stopTraining() {
        trainingActive = false;
        log("Training stopped");
    }

    public void setAgentMode(AgentMode mode) {
        this.agentMode = mode;
        log("Agent mode: " + mode);
    }

    // ========== 수동 제어 ==========
    public void setJointPosition(String name, float position) {
        if (renderer == null) return;
        Integer idx = jointIndexMap.get(name);
        if (idx == null) return;
        JointMeta jm = jointMetas.get(idx);
        float p = clamp(position, jm.minLimit, jm.maxLimit);
        renderer.setJointTarget(name, p, JointControlSource.RL);
    }

    public void manualStep() {
        if (!isInitialized || renderer == null) return;

        float[] action = agent.selectAction(getObservation(), AgentMode.RANDOM);
        applyAction(action);

        float reward = calculateReward(action);
        lastReward = reward;
        episodeReward += reward;
        stepCount++;

        log(String.format("Manual step %d: reward=%.4f", stepCount, reward));
    }

    // ========== 정보 조회 ==========
    public int getObservationDim() {
        int dim = jointMetas.size();             // positions
        if (config.includeVelocities) dim += jointMetas.size(); // velocities
        dim += 4; // height + vel x/z + speed diff
        return dim;
    }

    public int getActionDim() {
        return jointMetas.size();
    }

    public int getJointCount() { return jointMetas.size(); }
    public boolean isInitialized() { return isInitialized; }
    public boolean isTraining() { return trainingActive; }
    public boolean isDone() { return isDone; }
    public int getStepCount() { return stepCount; }
    public int getEpisodeCount() { return episodeCount; }
    public float getEpisodeReward() { return episodeReward; }
    public float getLastReward() { return lastReward; }
    public AgentMode getAgentMode() { return agentMode; }
    public Config getConfig() { return config; }
    public Statistics getStats() { return stats; }
    public SimpleAgent getAgent() { return agent; }

    public List<String> getJointNames() {
        List<String> names = new ArrayList<>();
        for (JointMeta jm : jointMetas) {
            names.add(jm.name);
        }
        return names;
    }

    public float getJointPosition(String name) {
        if (renderer == null) return 0f;
        return renderer.getJointPosition(name);
    }

    public float getJointVelocity(String name) {
        if (renderer == null) return 0f;
        return renderer.getJointVelocity(name);
    }

    public Map<String, Object> getDebugInfo() {
        Map<String, Object> info = new LinkedHashMap<>();
        info.put("initialized", isInitialized);
        info.put("training", trainingActive);
        info.put("mode", agentMode.name());
        info.put("episode", episodeCount);
        info.put("step", stepCount);
        info.put("reward", String.format("%.3f", episodeReward));
        info.put("lastR", String.format("%.4f", lastReward));
        info.put("healthy", isHealthy());
        info.put("joints", jointMetas.size());
        info.put("avgReward", String.format("%.2f", stats.getAverageReward()));
        return info;
    }

    // ========== 유틸리티 ==========
    private float clamp(float v, float min, float max) {
        return Math.max(min, Math.min(max, v));
    }

    public void setLogCallback(Consumer<String> callback) {
        this.logCallback = callback;
    }

    private void log(String msg) {
        logger.info(msg);
        if (logCallback != null) logCallback.accept(msg);
    }

    // ========== 내부 클래스/enum ==========
    private static class JointMeta {
        String name;
        float minLimit;
        float maxLimit;
        float initialPosition;
    }

    public enum ActionMode {
        TORQUE,
        POSITION,
        DELTA_POSITION,
        VELOCITY
    }

    public enum AgentMode {
        MANUAL,
        RANDOM,
        LEARNING,
        INFERENCE,
        IMITATION
    }

    public static class Config {
        // 시뮬레이션
        public float timeStep = 0.02f;
        public int maxEpisodeSteps = 500;
        public int updateInterval = 64;

        // 관측
        public boolean includeVelocities = true;

        // 행동
        public ActionMode actionMode = ActionMode.POSITION;
        public float maxTorque = 50f;
        public float maxVelocity = 5f;
        public float maxDeltaPosition = 0.1f;

        // 보상 파라미터
        public float kp = 50f;  // (현재 내부 물리 없음, 남겨만 둠)
        public float kd = 5f;
        public float damping = 0.95f;

        // 보상 가중치
        public float aliveBonus = 0.1f;
        public float heightRewardWeight = 1.0f;
        public float speedMatchWeight = 0.5f;
        public float controlCostWeight = 0.01f;
        public float velocityPenaltyWeight = 0.001f;
        public float symmetryRewardWeight = 0.1f;
        public float poseMatchWeight = 0.0f; // VMD 모방 보상 (필요하면 >0)

        // 목표
        public float targetHeight = 1.0f;
        public float targetSpeed = 0f;

        // 종료 조건
        public boolean terminateOnFall = true;
        public float minHeight = 0.3f;
        public float maxHeight = 2.0f;

        // 초기화
        public boolean randomizeInitial = true;
        public float initNoiseScale = 0.05f;
    }

    public static class Statistics {
        private final List<Float> episodeRewards = new ArrayList<>();
        private final List<Integer> episodeLengths = new ArrayList<>();
        private float bestReward = Float.NEGATIVE_INFINITY;
        private int totalSteps = 0;

        public void recordEpisode(float reward, int length) {
            episodeRewards.add(reward);
            episodeLengths.add(length);
            totalSteps += length;
            if (reward > bestReward) bestReward = reward;
            while (episodeRewards.size() > 100) {
                episodeRewards.remove(0);
                episodeLengths.remove(0);
            }
        }

        public float getAverageReward() {
            if (episodeRewards.isEmpty()) return 0f;
            float sum = 0f;
            for (float r : episodeRewards) sum += r;
            return sum / episodeRewards.size();
        }

        public float getAverageLength() {
            if (episodeLengths.isEmpty()) return 0f;
            int sum = 0;
            for (int l : episodeLengths) sum += l;
            return (float) sum / episodeLengths.size();
        }

        public float getBestReward() {
            return (bestReward == Float.NEGATIVE_INFINITY) ? 0f : bestReward;
        }

        public int getTotalSteps() { return totalSteps; }

        public int getEpisodeCount() { return episodeRewards.size(); }
    }

    public static class SimpleAgent {
        private final int actionDim;
        private final java.util.function.IntSupplier obsDimSupplier;
        private final Random random = new Random();

        private final List<Experience> experiences = new ArrayList<>();
        private static final int BUFFER_SIZE = 2048;

        private float[][] weights;
        private float learningRate = 0.001f;

        private float[] imitationTargets;

        public SimpleAgent(int actionDim, java.util.function.IntSupplier obsDimSupplier) {
            this.actionDim = actionDim;
            this.obsDimSupplier = obsDimSupplier;
            initializeWeights();
        }

        private void initializeWeights() {
            int obsDim = obsDimSupplier.getAsInt();
            weights = new float[obsDim][actionDim];
            float scale = (float) Math.sqrt(2.0 / (obsDim + actionDim));
            for (int i = 0; i < obsDim; i++) {
                for (int j = 0; j < actionDim; j++) {
                    weights[i][j] = (random.nextFloat() - 0.5f) * 2f * scale;
                }
            }
        }

        public float[] selectAction(float[] observation, AgentMode mode) {
            return switch (mode) {
                case RANDOM -> randomAction();
                case LEARNING, INFERENCE -> policyAction(observation, mode == AgentMode.LEARNING);
                case IMITATION -> imitationAction();
                case MANUAL -> zeroAction();
            };
        }

        private float[] randomAction() {
            float[] a = new float[actionDim];
            for (int i = 0; i < actionDim; i++) {
                a[i] = random.nextFloat() * 2f - 1f;
            }
            return a;
        }

        private float[] zeroAction() {
            return new float[actionDim];
        }

        private float[] policyAction(float[] obs, boolean explore) {
            float[] action = new float[actionDim];
            for (int j = 0; j < actionDim; j++) {
                float sum = 0f;
                for (int i = 0; i < obs.length && i < weights.length; i++) {
                    sum += obs[i] * weights[i][j];
                }
                float a = (float) Math.tanh(sum);
                if (explore) {
                    a += (float) (random.nextGaussian() * 0.2);
                    a = Math.max(-1f, Math.min(1f, a));
                }
                action[j] = a;
            }
            return action;
        }

        private float[] imitationAction() {
            if (imitationTargets != null && imitationTargets.length == actionDim) {
                return imitationTargets.clone();
            }
            return zeroAction();
        }

        public void storeExperience(float[] obs, float[] action, float reward, float[] nextObs, boolean done) {
            experiences.add(new Experience(obs.clone(), action.clone(), reward, nextObs.clone(), done));
            if (experiences.size() > BUFFER_SIZE) experiences.remove(0);
        }

        public void update() {
            if (experiences.size() < 64) return;

            float mean = 0f;
            for (Experience e : experiences) mean += e.reward;
            mean /= experiences.size();

            float var = 0f;
            for (Experience e : experiences) {
                float d = e.reward - mean;
                var += d * d;
            }
            float std = (float) Math.sqrt(var / experiences.size() + 1e-8);

            for (Experience e : experiences) {
                float adv = (std > 0f) ? (e.reward - mean) / std : 0f;
                for (int j = 0; j < actionDim && j < e.action.length; j++) {
                    for (int i = 0; i < e.obs.length && i < weights.length; i++) {
                        float a = e.action[j];
                        float grad = a * (1f - a * a);   // tanh' approx
                        weights[i][j] += learningRate * adv * e.obs[i] * grad;
                    }
                }
            }
            experiences.clear();
        }

        public void setImitationTargets(float[] targets) {
            this.imitationTargets = targets;
        }

        public void setImitationTargets(Map<String, Float> targetMap, List<String> jointNames) {
            imitationTargets = new float[actionDim];
            for (int i = 0; i < jointNames.size() && i < actionDim; i++) {
                String name = jointNames.get(i);
                imitationTargets[i] = targetMap.getOrDefault(name, 0f);
            }
        }

        private static class Experience {
            float[] obs, action, nextObs;
            float reward;
            boolean done;

            Experience(float[] obs, float[] action, float reward, float[] nextObs, boolean done) {
                this.obs = obs;
                this.action = action;
                this.reward = reward;
                this.nextObs = nextObs;
                this.done = done;
            }
        }
    }
}
