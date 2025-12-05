package com.kAIS.KAIMyEntity.rl;

import com.kAIS.KAIMyEntity.urdf.URDFModelOpenGLWithSTL;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.util.*;
import java.util.function.Consumer;

/**
 * MuJoCo 스타일 RL 환경 - 모드 내부 완결형
 * 
 * 기능:
 * - 환경 (Observation, Action, Reward)
 * - 내장 에이전트 (Random, Simple Policy, Imitation)
 * - 학습 루프 (틱 기반)
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
    private URDFModelOpenGLWithSTL renderer;
    
    private final Config config = new Config();
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
    
    // 이전 상태 (보상 계산용)
    private float[] prevRootPosition = new float[3];
    private float prevRootHeight = 1.0f;
    
    // 통계
    private final Statistics stats = new Statistics();
    
    // 콜백
    private Consumer<String> logCallback;
    
    // ========== 초기화 ==========
    
    private RLEnvironmentCore() {
        logger.info("RLEnvironmentCore created");
    }
    
    /**
     * 렌더러 연결 및 환경 초기화
     */
    public void initialize(URDFModelOpenGLWithSTL renderer) {
        this.renderer = renderer;
        
        if (renderer == null) {
            log("WARN: Renderer is null");
            isInitialized = false;
            return;
        }
        
        // 관절 정보 수집 - 렌더러의 API 사용
        jointStates.clear();
        jointIndexMap.clear();
        
        List<String> jointNames = renderer.getMovableJointNames();
        if (jointNames == null || jointNames.isEmpty()) {
            log("WARN: No movable joints found");
            isInitialized = false;
            return;
        }
        
        int idx = 0;
        for (String jointName : jointNames) {
            float[] limits = renderer.getJointLimits(jointName);
            float lower = (limits != null && limits.length >= 2) ? limits[0] : (float)-Math.PI;
            float upper = (limits != null && limits.length >= 2) ? limits[1] : (float)Math.PI;
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
        
        // 에이전트 초기화
        agent = new SimpleAgent(jointStates.size(), this::getObservationDim);
        
        isInitialized = true;
        log("Initialized: " + jointStates.size() + " joints, obs=" + getObservationDim() + ", act=" + getActionDim());
    }
    
    // ========== 메인 틱 (GUI에서 호출) ==========
    
    /**
     * 매 틱마다 호출 - 학습/추론 루프
     */
    public void tick(float deltaTime) {
        if (!isInitialized || !trainingActive) return;
        if (agentMode == AgentMode.MANUAL) return;
        
        // 1. 관측 수집
        float[] observation = getObservation();
        
        // 2. 에이전트에서 행동 얻기
        float[] action = agent.selectAction(observation, agentMode);
        
        // 3. 행동 적용
        applyAction(action);
        
        // 4. 물리 시뮬레이션 (간단 버전)
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
        
        // 8. 에이전트 학습 (경험 저장)
        if (agentMode == AgentMode.LEARNING) {
            agent.storeExperience(observation, action, reward, newObservation, isDone);
            
            // 배치 학습 (일정 스텝마다)
            if (stepCount % config.updateInterval == 0) {
                agent.update();
            }
        }
        
        // 9. 에피소드 종료 처리
        if (isDone) {
            endEpisode(terminated ? "terminated" : "truncated");
        }
        
        // 10. 이전 상태 업데이트
        prevRootPosition = getRootPosition();
        prevRootHeight = prevRootPosition[1];
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
        
        // 관절 초기화
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
            
            // 렌더러에 적용
            if (renderer != null) {
                renderer.setJointTarget(js.name, initPos);
            }
        }
        
        prevRootPosition = getRootPosition();
        prevRootHeight = prevRootPosition[1];
        
        return getObservation();
    }
    
    /**
     * 행동 적용
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
                    // [-1,1] -> [min, max]
                    js.targetPosition = js.minLimit + (a + 1f) / 2f * (js.maxLimit - js.minLimit);
                    break;
                    
                case VELOCITY:
                    js.targetVelocity = a * config.maxVelocity;
                    break;
                    
                case DELTA_POSITION:
                    // 현재 위치에서 델타 적용
                    float delta = a * config.maxDeltaPosition;
                    js.targetPosition = clamp(js.position + delta, js.minLimit, js.maxLimit);
                    break;
            }
            
            // 렌더러에 적용
            if (renderer != null && config.actionMode != ActionMode.TORQUE) {
                renderer.setJointTarget(js.name, js.targetPosition);
            }
        }
    }
    
    /**
     * 간단한 물리 시뮬레이션
     */
    private void simulatePhysics(float dt) {
        for (JointState js : jointStates) {
            // PD 제어 (Position 모드)
            if (config.actionMode == ActionMode.POSITION || 
                config.actionMode == ActionMode.DELTA_POSITION) {
                float error = js.targetPosition - js.position;
                float deriv = -js.velocity;
                js.torque = config.kp * error + config.kd * deriv;
                js.torque = clamp(js.torque, -config.maxTorque, config.maxTorque);
            }
            
            // 속도 제어 모드
            if (config.actionMode == ActionMode.VELOCITY) {
                float velError = js.targetVelocity - js.velocity;
                js.torque = config.kp * velError;
                js.torque = clamp(js.torque, -config.maxTorque, config.maxTorque);
            }
            
            // 적분
            float acceleration = js.torque; // 단순화: I=1
            js.velocity += acceleration * dt;
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
        
        // 실제 모델 상태 동기화
        syncWithRenderer();
    }
    
    /**
     * 렌더러와 상태 동기화
     */
    private void syncWithRenderer() {
        if (renderer == null) return;
        
        for (JointState js : jointStates) {
            renderer.setJointTarget(js.name, js.position);
        }
    }
    
    // ========== 관측 (Observation) ==========
    
    public float[] getObservation() {
        List<Float> obs = new ArrayList<>();
        
        // 1. 관절 위치 (정규화 [-1, 1])
        for (JointState js : jointStates) {
            float range = js.maxLimit - js.minLimit;
            float norm = range > 0 ? 2f * (js.position - js.minLimit) / range - 1f : 0f;
            obs.add(clamp(norm, -1f, 1f));
        }
        
        // 2. 관절 속도 (스케일링)
        if (config.includeVelocities) {
            for (JointState js : jointStates) {
                obs.add(js.velocity / config.maxVelocity);
            }
        }
        
        // 3. 루트 바디 높이 (정규화)
        float[] rootPos = getRootPosition();
        float heightRange = config.maxHeight - config.minHeight;
        obs.add(heightRange > 0 ? (rootPos[1] - config.minHeight) / heightRange : 0.5f);
        
        // 4. 루트 속도 (수평)
        float[] rootVel = getRootVelocity();
        float speedScale = config.targetSpeed > 0 ? config.targetSpeed : 1f;
        obs.add(rootVel[0] / speedScale); // x
        obs.add(rootVel[2] / speedScale); // z
        
        // 5. 목표 속도와의 차이
        float currentSpeed = (float)Math.sqrt(rootVel[0]*rootVel[0] + rootVel[2]*rootVel[2]);
        obs.add(speedScale > 0 ? (config.targetSpeed - currentSpeed) / speedScale : 0f);
        
        // float[] 변환
        float[] result = new float[obs.size()];
        for (int i = 0; i < obs.size(); i++) {
            result[i] = obs.get(i);
        }
        return result;
    }
    
    private float[] getRootPosition() {
        // 관절 평균 높이로 추정 (실제 구현에서는 루트 바디 위치 사용)
        float avgHeight = 1.0f;
        if (!jointStates.isEmpty()) {
            float sum = 0;
            for (JointState js : jointStates) {
                float range = js.maxLimit - js.minLimit;
                sum += range > 0 ? (js.position - js.minLimit) / range : 0.5f;
            }
            avgHeight = 0.5f + sum / jointStates.size() * 0.5f;
        }
        return new float[]{0, avgHeight, 0};
    }
    
    private float[] getRootVelocity() {
        float[] current = getRootPosition();
        float dt = config.timeStep > 0 ? config.timeStep : 0.02f;
        return new float[]{
            (current[0] - prevRootPosition[0]) / dt,
            (current[1] - prevRootPosition[1]) / dt,
            (current[2] - prevRootPosition[2]) / dt
        };
    }
    
    // ========== 보상 (Reward) ==========
    
    private float calculateReward(float[] action) {
        float reward = 0f;
        
        // 1. 살아있음 보상
        reward += config.aliveBonus;
        
        // 2. 높이 유지 보상
        float[] rootPos = getRootPosition();
        float heightDiff = Math.abs(rootPos[1] - config.targetHeight);
        float heightReward = config.targetHeight > 0 ? 1f - heightDiff / config.targetHeight : 0f;
        reward += heightReward * config.heightRewardWeight;
        
        // 3. 속도 매칭 보상 (목표 속도 추종)
        float[] rootVel = getRootVelocity();
        float currentSpeed = (float)Math.sqrt(rootVel[0]*rootVel[0] + rootVel[2]*rootVel[2]);
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
        
        // 5. 관절 속도 페널티 (급격한 움직임 방지)
        float velocityPenalty = 0f;
        for (JointState js : jointStates) {
            velocityPenalty += js.velocity * js.velocity;
        }
        reward -= velocityPenalty * config.velocityPenaltyWeight;
        
        // 6. 관절 제한 페널티
        float limitPenalty = 0f;
        for (JointState js : jointStates) {
            float range = js.maxLimit - js.minLimit;
            float margin = 0.1f * range;
            if (js.position < js.minLimit + margin || js.position > js.maxLimit - margin) {
                limitPenalty += 0.1f;
            }
        }
        reward -= limitPenalty;
        
        // 7. 대칭 보상 (양쪽 관절 유사하게)
        reward += calculateSymmetryReward() * config.symmetryRewardWeight;
        
        return reward;
    }
    
    private float calculateSymmetryReward() {
        float symmetry = 0f;
        int pairs = 0;
        
        for (JointState js : jointStates) {
            if (js.name.contains("_L_") || js.name.contains("Left") || js.name.contains("_l_")) {
                String rightName = js.name
                    .replace("_L_", "_R_")
                    .replace("Left", "Right")
                    .replace("_l_", "_r_");
                Integer rightIdx = jointIndexMap.get(rightName);
                if (rightIdx != null) {
                    JointState rightJs = jointStates.get(rightIdx);
                    float diff = Math.abs(js.position - rightJs.position);
                    symmetry += 1f - Math.min(diff / (float)Math.PI, 1f);
                    pairs++;
                }
            }
        }
        
        return pairs > 0 ? symmetry / pairs : 0f;
    }
    
    // ========== 종료 조건 ==========
    
    private boolean checkTermination() {
        if (!config.terminateOnFall) return false;
        
        float[] rootPos = getRootPosition();
        
        // 높이 체크
        if (rootPos[1] < config.minHeight) {
            log("Terminated: height too low (" + String.format("%.2f", rootPos[1]) + ")");
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
        
        // 자동 리셋
        if (trainingActive) {
            reset();
        }
    }
    
    // ========== 학습 제어 ==========
    
    public void startTraining(AgentMode mode) {
        if (!isInitialized) {
            log("ERROR: Cannot start - not initialized");
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
    
    /**
     * 외부에서 관절 직접 제어 (GUI 슬라이더 등)
     */
    public void setJointPosition(String name, float position) {
        Integer idx = jointIndexMap.get(name);
        if (idx == null) return;
        
        JointState js = jointStates.get(idx);
        js.position = clamp(position, js.minLimit, js.maxLimit);
        js.targetPosition = js.position;
        js.velocity = 0;
        
        if (renderer != null) {
            renderer.setJointTarget(name, js.position);
        }
    }
    
    /**
     * 수동 스텝 (GUI Step 버튼)
     */
    public void manualStep() {
        if (!isInitialized) return;
        
        // 랜덤 행동으로 한 스텝
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
        int dim = jointStates.size(); // 위치
        if (config.includeVelocities) dim += jointStates.size(); // 속도
        dim += 4; // 높이 + 속도xy + 속도차
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
    
    public List<String> getJointNames() {
        List<String> names = new ArrayList<>();
        for (JointState js : jointStates) {
            names.add(js.name);
        }
        return names;
    }
    
    public float getJointPosition(String name) {
        Integer idx = jointIndexMap.get(name);
        return idx != null ? jointStates.get(idx).position : 0;
    }
    
    public float getJointVelocity(String name) {
        Integer idx = jointIndexMap.get(name);
        return idx != null ? jointStates.get(idx).velocity : 0;
    }
    
    /**
     * 내장 에이전트 접근
     */
    public SimpleAgent getAgent() {
        return agent;
    }
    
    /**
     * 디버그 정보
     */
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
        if (logCallback != null) {
            logCallback.accept(msg);
        }
    }
    
    // ========== 내부 클래스 ==========
    
    /**
     * 관절 상태
     */
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
    
    /**
     * 행동 모드
     */
    public enum ActionMode {
        TORQUE,         // 직접 토크
        POSITION,       // 목표 위치 (절대)
        DELTA_POSITION, // 델타 위치 (상대)
        VELOCITY        // 목표 속도
    }
    
    /**
     * 에이전트 모드
     */
    public enum AgentMode {
        MANUAL,     // 수동 제어 (GUI)
        RANDOM,     // 랜덤 행동
        LEARNING,   // 학습 중
        INFERENCE,  // 학습된 정책 실행
        IMITATION   // 모방 학습 (VMD 추종)
    }
    
    /**
     * 환경 설정
     */
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
        
        // 보상
        public float aliveBonus = 0.1f;
        public float heightRewardWeight = 1.0f;
        public float speedMatchWeight = 0.5f;
        public float controlCostWeight = 0.01f;
        public float velocityPenaltyWeight = 0.001f;
        public float symmetryRewardWeight = 0.1f;
        
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
    
    /**
     * 통계
     */
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
            if (episodeRewards.isEmpty()) return 0;
            float sum = 0;
            for (float r : episodeRewards) sum += r;
            return sum / episodeRewards.size();
        }
        
        public float getAverageLength() {
            if (episodeLengths.isEmpty()) return 0;
            int sum = 0;
            for (int l : episodeLengths) sum += l;
            return (float) sum / episodeLengths.size();
        }
        
        public float getBestReward() { return bestReward == Float.NEGATIVE_INFINITY ? 0 : bestReward; }
        public int getTotalSteps() { return totalSteps; }
        public int getEpisodeCount() { return episodeRewards.size(); }
    }
    
    /**
     * 간단한 내장 에이전트
     */
    public static class SimpleAgent {
        private final int actionDim;
        private final java.util.function.IntSupplier obsDimSupplier;
        private final Random random = new Random();
        
        // 경험 버퍼
        private final List<Experience> experiences = new ArrayList<>();
        private static final int BUFFER_SIZE = 2048;
        
        // 간단한 선형 정책
        private float[][] weights;
        private float learningRate = 0.001f;
        
        // VMD 목표 (모방 학습용)
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
                    weights[i][j] = (random.nextFloat() - 0.5f) * 2 * scale;
                }
            }
        }
        
        /**
         * 행동 선택
         */
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
            float[] action = new float[actionDim];
            for (int i = 0; i < actionDim; i++) {
                action[i] = random.nextFloat() * 2 - 1;
            }
            return action;
        }
        
        private float[] zeroAction() {
            return new float[actionDim];
        }
        
        private float[] policyAction(float[] obs, boolean explore) {
            float[] action = new float[actionDim];
            
            for (int j = 0; j < actionDim; j++) {
                float sum = 0;
                for (int i = 0; i < obs.length && i < weights.length; i++) {
                    sum += obs[i] * weights[i][j];
                }
                action[j] = (float) Math.tanh(sum);
                
                if (explore) {
                    action[j] += (float)(random.nextGaussian() * 0.2);
                    action[j] = Math.max(-1f, Math.min(1f, action[j]));
                }
            }
            
            return action;
        }
        
        private float[] imitationAction() {
            if (imitationTargets != null && imitationTargets.length == actionDim) {
                return imitationTargets.clone();
            }
            return zeroAction();
        }
        
        /**
         * 경험 저장
         */
        public void storeExperience(float[] obs, float[] action, float reward, float[] nextObs, boolean done) {
            experiences.add(new Experience(obs.clone(), action.clone(), reward, nextObs.clone(), done));
            
            if (experiences.size() > BUFFER_SIZE) {
                experiences.remove(0);
            }
        }
        
        /**
         * 정책 업데이트 (간단한 REINFORCE)
         */
        public void update() {
            if (experiences.size() < 64) return;
            
            float meanReward = 0;
            for (Experience e : experiences) meanReward += e.reward;
            meanReward /= experiences.size();
            
            float stdReward = 0;
            for (Experience e : experiences) {
                stdReward += (e.reward - meanReward) * (e.reward - meanReward);
            }
            stdReward = (float) Math.sqrt(stdReward / experiences.size() + 1e-8);
            
            for (Experience e : experiences) {
                float advantage = stdReward > 0 ? (e.reward - meanReward) / stdReward : 0;
                
                for (int j = 0; j < actionDim && j < e.action.length; j++) {
                    for (int i = 0; i < e.obs.length && i < weights.length; i++) {
                        float actionGrad = e.action[j] * (1 - e.action[j] * e.action[j]);
                        weights[i][j] += learningRate * advantage * e.obs[i] * actionGrad;
                    }
                }
            }
            
            experiences.clear();
        }
        
        /**
         * VMD 모방 목표 설정
         */
        public void setImitationTargets(float[] targets) {
            this.imitationTargets = targets;
        }
        
        /**
         * 모방 목표 설정 (관절 이름 -> 값 맵)
         */
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
