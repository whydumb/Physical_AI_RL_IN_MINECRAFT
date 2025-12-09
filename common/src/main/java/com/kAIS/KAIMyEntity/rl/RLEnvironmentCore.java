package com.kAIS.KAIMyEntity.rl;

import com.kAIS.KAIMyEntity.urdf.URDFModelOpenGLWithSTL;
import com.kAIS.KAIMyEntity.urdf.control.URDFMotion;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.util.*;
import java.util.function.Consumer;

/**
 * MuJoCo 스타일 RL 환경 - 모드 내부 완결형 (FK + Physics 전제)
 *
 * - 관절 상태: 내부 JointState
 * - 액션: 조인트별 [-1,1] → 각도/델타각/속도
 * - 시뮬레이션: 간단한 PD + 적분 + 렌더러(ODE/URDF)에 동기화
 * - 선택: URDFMotion(VMD 등) 기반 참조 모션을 사용한 모방 보상
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
    /** URDF + ODE 기반 렌더러 */
    private URDFModelOpenGLWithSTL renderer;

    /** 설정값 */
    private final Config config = new Config();

    /** 관절 상태 리스트 (액션/관측의 기준 축) */
    private final List<JointState> jointStates = new ArrayList<>();
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

        jointStates.clear();
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

            JointState js = new JointState();
            js.name = jointName;
            js.position = currentPos;
            js.velocity = 0f;
            js.torque = 0f;
            js.minLimit = lower;
            js.maxLimit = upper;
            js.targetPosition = currentPos;
            js.initialPosition = currentPos;

            jointStates.add(js);
            jointIndexMap.put(jointName, idx++);
        }

        // 2) 에이전트 초기화 (간단 선형 정책)
        agent = new SimpleAgent(jointStates.size(), this::getObservationDim);

        // 3) 루트 상태 초기화
        float[] rootPos = getRootPosition();
        prevRootPosition[0] = rootPos[0];
        prevRootPosition[1] = rootPos[1];
        prevRootPosition[2] = rootPos[2];

        isInitialized = true;
        log("Initialized: joints=" + jointStates.size()
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

    // ========== 메인 틱 ==========
    /**
     * 매 틱마다 호출되는 메인 RL 루프
     */
    public void tick(float deltaTime) {
        if (!isInitialized || !trainingActive) return;
        if (agentMode == AgentMode.MANUAL) return;

        // 1. 현재 관측
        float[] observation = getObservation();

        // 2. 에이전트로부터 행동 얻기
        float[] action = agent.selectAction(observation, agentMode);

        // 3. 행동 적용 (FK: 조인트 target 설정)
        applyAction(action);

        // 4. 간단 물리 시뮬레이션 (내부 joint 상태 업데이트 + 렌더러 동기화)
        simulatePhysics(deltaTime);

        // 5. 새 관측
        float[] newObservation = getObservation();

        // 6. 보상 계산
        float reward = calculateReward(action);
        lastReward = reward;
        episodeReward += reward;

        // 7. 종료 조건
        stepCount++;
        boolean terminated = checkTermination();
        boolean truncated = stepCount >= config.maxEpisodeSteps;
        isDone = terminated || truncated;

        // 8. 경험 저장 및 업데이트 (학습 모드에서만)
        if (agentMode == AgentMode.LEARNING) {
            agent.storeExperience(observation, action, reward, newObservation, isDone);

            if (stepCount % config.updateInterval == 0) {
                agent.update();
            }
        }

        // 9. 에피소드 종료 처리
        if (isDone) {
            endEpisode(terminated ? "terminated" : "truncated");
        }

        // 10. 루트 상태, 에피소드 시간 업데이트
        float[] rootPos = getRootPosition();
        prevRootPosition[0] = rootPos[0];
        prevRootPosition[1] = rootPos[1];
        prevRootPosition[2] = rootPos[2];

        timeInEpisode += deltaTime;
    }

    // ========== 환경 인터페이스 ==========
    /**
     * 환경 리셋
     */
    public float[] reset() {
        if (!isInitialized) return new float[0];

        stepCount = 0;
        episodeReward = 0f;
        lastReward = 0f;
        isDone = false;
        timeInEpisode = 0f;

        Random rand = config.randomizeInitial ? new Random() : null;

        for (JointState js : jointStates) {
            float initPos = js.initialPosition;

            if (rand != null) {
                float range = (js.maxLimit - js.minLimit) * config.initNoiseScale;
                initPos += (rand.nextFloat() - 0.5f) * range;
                initPos = clamp(initPos, js.minLimit, js.maxLimit);
            }

            js.position = initPos;
            js.velocity = 0f;
            js.torque = 0f;
            js.targetPosition = initPos;
            js.targetVelocity = 0f;

            if (renderer != null) {
                renderer.setJointTarget(js.name, initPos);
            }
        }

        float[] rootPos = getRootPosition();
        prevRootPosition[0] = rootPos[0];
        prevRootPosition[1] = rootPos[1];
        prevRootPosition[2] = rootPos[2];

        return getObservation();
    }

    /**
     * 행동 적용: [-1,1] 범위 액션을 조인트 명령으로 변환
     */
    private void applyAction(float[] action) {
        if (action == null) return;

        int numActions = Math.min(action.length, jointStates.size());

        for (int i = 0; i < numActions; i++) {
            JointState js = jointStates.get(i);
            float a = clamp(action[i], -1f, 1f);

            switch (config.actionMode) {
                case TORQUE:
                    js.torque = a * config.maxTorque;
                    break;

                case POSITION:
                    // [-1,1] → [minLimit, maxLimit]
                    js.targetPosition = js.minLimit + (a + 1f) * 0.5f * (js.maxLimit - js.minLimit);
                    break;

                case VELOCITY:
                    js.targetVelocity = a * config.maxVelocity;
                    break;

                case DELTA_POSITION:
                    float delta = a * config.maxDeltaPosition;
                    js.targetPosition = clamp(js.position + delta, js.minLimit, js.maxLimit);
                    break;
            }

            // TORQUE 모드가 아닌 경우, targetPosition을 렌더러에 곧장 넘김
            if (renderer != null && config.actionMode != ActionMode.TORQUE) {
                renderer.setJointTarget(js.name, js.targetPosition);
            }
        }
    }

    /**
     * 간단 물리 시뮬레이션 (내부 joint 상태만 갱신, 실제 ODE는 렌더러가 담당)
     */
    private void simulatePhysics(float dt) {
        for (JointState js : jointStates) {
            // Position/Delta 모드: PD 제어
            if (config.actionMode == ActionMode.POSITION
                    || config.actionMode == ActionMode.DELTA_POSITION) {
                float error = js.targetPosition - js.position;
                float deriv = -js.velocity;
                js.torque = config.kp * error + config.kd * deriv;
                js.torque = clamp(js.torque, -config.maxTorque, config.maxTorque);
            }

            // Velocity 모드: 속도 추종
            if (config.actionMode == ActionMode.VELOCITY) {
                float velError = js.targetVelocity - js.velocity;
                js.torque = config.kp * velError;
                js.torque = clamp(js.torque, -config.maxTorque, config.maxTorque);
            }

            // 토크 → 가속도 (I=1 가정) → 속도 → 위치
            float acc = js.torque;
            js.velocity += acc * dt;
            js.velocity *= config.damping;
            js.position += js.velocity * dt;

            // 관절 제한
            if (js.position < js.minLimit) {
                js.position = js.minLimit;
                js.velocity = Math.max(0, js.velocity);
            } else if (js.position > js.maxLimit) {
                js.position = js.maxLimit;
                js.velocity = Math.min(0, js.velocity);
            }
        }

        // 렌더러와 상태 동기화
        syncWithRenderer();
    }

    /**
     * 렌더러와 관절 상태 동기화
     */
    private void syncWithRenderer() {
        if (renderer == null) return;

        for (JointState js : jointStates) {
            renderer.setJointTarget(js.name, js.position);
        }
    }

    // ========== 관측 ==========
    public float[] getObservation() {
        List<Float> obs = new ArrayList<>();

        // 1. 관절 위치 (정규화 [-1,1])
        for (JointState js : jointStates) {
            float range = js.maxLimit - js.minLimit;
            float norm = (range > 0f)
                    ? 2f * (js.position - js.minLimit) / range - 1f
                    : 0f;
            obs.add(clamp(norm, -1f, 1f));
        }

        // 2. 관절 속도 (스케일링)
        if (config.includeVelocities) {
            for (JointState js : jointStates) {
                obs.add(js.velocity / config.maxVelocity);
            }
        }

        // 3. 루트 높이 (정규화)
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

    /**
     * 루트 위치 (현재는 관절 평균 기반 추정; 나중에 실제 루트 링크 좌표로 교체 가능)
     */
    private float[] getRootPosition() {
        // 관절 위치 정규화 평균으로 대충 높이 추정
        float avgHeight = 1.0f;
        if (!jointStates.isEmpty()) {
            float sum = 0f;
            for (JointState js : jointStates) {
                float range = js.maxLimit - js.minLimit;
                sum += (range > 0f) ? (js.position - js.minLimit) / range : 0.5f;
            }
            avgHeight = 0.5f + sum / jointStates.size() * 0.5f;
        }
        return new float[]{0f, avgHeight, 0f};
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
        for (JointState js : jointStates) {
            velocityPenalty += js.velocity * js.velocity;
        }
        reward -= velocityPenalty * config.velocityPenaltyWeight;

        // 6. 관절 제한 근접 페널티 (단순)
        float limitPenalty = 0f;
        for (JointState js : jointStates) {
            float range = js.maxLimit - js.minLimit;
            if (range <= 0f) continue;
            float margin = 0.1f * range;
            if (js.position < js.minLimit + margin || js.position > js.maxLimit - margin) {
                limitPenalty += 0.1f;
            }
        }
        reward -= limitPenalty;

        // 7. 좌우 대칭 보상
        reward += calculateSymmetryReward() * config.symmetryRewardWeight;

        // 8. VMD/참조 모션 포즈 매칭 보상 (있을 때만)
        if (referenceMotion != null && config.poseMatchWeight > 0f) {
            Map<String, Float> refPose = referenceMotion.samplePose(timeInEpisode);
            if (!refPose.isEmpty()) {
                float poseError = 0f;
                int count = 0;
                for (JointState js : jointStates) {
                    Float ref = refPose.get(js.name);
                    if (ref == null) continue;
                    float diff = js.position - ref;
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

    /**
     * 좌/우 관절 쌍의 대칭성 보상 [-1,1] 정도 범위
     */
    private float calculateSymmetryReward() {
        float symmetry = 0f;
        int pairs = 0;

        for (JointState js : jointStates) {
            // 이름 규칙 기반: 왼쪽 관절 이름 패턴
            if (js.name.contains("_L_") || js.name.contains("Left") || js.name.contains("_l_")) {
                String rightName = js.name
                        .replace("_L_", "_R_")
                        .replace("Left", "Right")
                        .replace("_l_", "_r_");
                Integer idx = jointIndexMap.get(rightName);
                if (idx != null) {
                    JointState r = jointStates.get(idx);
                    float diff = Math.abs(js.position - r.position);
                    // diff=0 → 1, diff=pi → 0
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
        Integer idx = jointIndexMap.get(name);
        if (idx == null) return;
        JointState js = jointStates.get(idx);
        js.position = clamp(position, js.minLimit, js.maxLimit);
        js.targetPosition = js.position;
        js.velocity = 0f;
        if (renderer != null) {
            renderer.setJointTarget(name, js.position);
        }
    }

    public void manualStep() {
        if (!isInitialized) return;

        float[] action = agent.selectAction(getObservation(), AgentMode.RANDOM);
        applyAction(action);
        simulatePhysics(config.timeStep);

        float reward = calculateReward(action);
        lastReward = reward;
        episodeReward += reward;
        stepCount++;

        log(String.format("Manual step %d: reward=%.4f", stepCount, reward));
    }

    // ========== 정보 조회 ==========
    public int getObservationDim() {
        int dim = jointStates.size();             // positions
        if (config.includeVelocities) dim += jointStates.size(); // velocities
        dim += 4; // height + vel x/z + speed diff
        return dim;
    }

    public int getActionDim() {
        return jointStates.size();
    }

    public int getJointCount() { return jointStates.size(); }
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
        for (JointState js : jointStates) {
            names.add(js.name);
        }
        return names;
    }

    public float getJointPosition(String name) {
        Integer idx = jointIndexMap.get(name);
        return (idx != null) ? jointStates.get(idx).position : 0f;
    }

    public float getJointVelocity(String name) {
        Integer idx = jointIndexMap.get(name);
        return (idx != null) ? jointStates.get(idx).velocity : 0f;
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
        info.put("joints", jointStates.size());
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

    // ========== 내부 클래스들 ==========
    private static class JointState {
        String name;
        float position;
        float velocity;
        float torque;
        float minLimit;
        float maxLimit;
        float targetPosition;
        float targetVelocity;
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

        // PD 제어
        public float kp = 50f;
        public float kd = 5f;
        public float damping = 0.95f;

        // 보상 가중치
        public float aliveBonus = 0.1f;
        public float heightRewardWeight = 1.0f;
        public float speedMatchWeight = 0.5f;
        public float controlCostWeight = 0.01f;
        public float velocityPenaltyWeight = 0.001f;
        public float symmetryRewardWeight = 0.1f;
        public float poseMatchWeight = 0.0f; // VMD 모방 보상 (걷기 VMD 쓸 때 > 0)

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
            switch (mode) {
                case RANDOM:
                    return randomAction();
                case LEARNING:
                case INFERENCE:
                    return policyAction(observation, mode == AgentMode.LEARNING);
                case IMITATION:
                    return imitationAction();
                case MANUAL:
                default:
                    return zeroAction();
            }
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
