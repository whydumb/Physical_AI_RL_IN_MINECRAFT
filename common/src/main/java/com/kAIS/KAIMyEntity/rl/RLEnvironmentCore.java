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
 * 핵심:
 * - 물리/기구학(ODE)은 외부(URDF/ODE4J)에서 처리
 * - 이 클래스는 "관측 + action -> 조인트 명령 + transition 구성" 담당
 *
 * 이번 통째 패치:
 * - Reset에서 컨트롤러 hardResetToSpawn() 호출(물리 바디 위치/회전/속도 리셋)
 * - 짧은 에피소드에서도 update 1회 가능(option)
 * - Observed Root / Physics Root 분리 유지
 * - one-step delay transition 유지
 */
public class RLEnvironmentCore {
    private static final Logger logger = LogManager.getLogger();

    // ========== 싱글톤 ==========
    private static volatile RLEnvironmentCore instance;

    public static RLEnvironmentCore getInstance() {
        if (instance == null) {
            synchronized (RLEnvironmentCore.class) {
                if (instance == null) instance = new RLEnvironmentCore();
            }
        }
        return instance;
    }

    // ========== 환경 상태 ==========
    private URDFModelOpenGLWithSTL renderer;

    // 루트 상태 공급자(관측용)
    private Supplier<Float> heightSupplier;
    private Supplier<float[]> rootXZSupplier;

    private final Config config = new Config();

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

    // 참조 모션
    private URDFMotion referenceMotion;
    private float timeInEpisode = 0f;

    // ===== 루트 상태 (Observed / Physics 분리) =====
    private final float[] prevObservedRootPosition = new float[3];
    private final float[] prevPhysicsRootPosition  = new float[3];

    private final float[] observedRootPosBuf = new float[3];
    private final float[] observedRootVelBuf = new float[3];
    private final float[] physicsRootPosBuf = new float[3];
    private final float[] physicsRootVelBuf = new float[3];
    private final float[] physRootWork = new float[3];

    // 스폰 fallback
    private final float[] spawnRootPosition = new float[3];
    private boolean spawnPositionInitialized = false;
    private boolean spawnPositionDirty = true;

    // 디버그
    private boolean lastPhysicsRootValid = false;
    private int debugTickCounter = 0;

    // 통계
    private final Statistics stats = new Statistics();

    // 콜백
    private Consumer<String> logCallback;

    // ===== 성능/버퍼 =====
    private float lastDeltaTime = 0.02f;
    private float[] obsBuffer = null;

    // ===== one-step delay transition =====
    private boolean transitionPrimed = false;
    private float[] lastObs = null;
    private float[] lastAction = null;

    private RLEnvironmentCore() {
        logger.info("RLEnvironmentCore created");
    }

    // ========== 초기화 ==========
    public void initialize(URDFModelOpenGLWithSTL renderer) {
        this.renderer = renderer;

        if (renderer != null) {
            URDFSimpleController controller = renderer.getController();
            if (controller != null) controller.setSpawnCollisionMargin(config.groundCollisionMargin);
        }

        if (renderer == null) {
            log("WARN: Renderer is null");
            isInitialized = false;
            return;
        }

        jointMetas.clear();
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

        agent = new SimpleAgent(jointMetas.size(), this::getObservationDim);

        ensureObservationBuffer();

        spawnPositionDirty = true;
        refreshSpawnRootPosition();
        updatePrevRootPositionsNow();

        transitionPrimed = false;
        lastObs = null;
        lastAction = null;

        isInitialized = true;
        log("Initialized: joints=" + jointMetas.size() + ", obs=" + getObservationDim() + ", act=" + getActionDim());
    }

    // ========== 참조 모션 ==========
    public void setReferenceMotion(URDFMotion motion) {
        this.referenceMotion = motion;
        if (motion != null) log("Reference motion set: " + motion.name + ", keys=" + motion.keys.size());
        else log("Reference motion cleared");
    }

    public boolean hasReferenceMotion() { return referenceMotion != null; }
    public float getTimeInEpisode() { return timeInEpisode; }
    public URDFMotion getReferenceMotion() { return referenceMotion; }

    // ========== 루트 supplier ==========
    public void setRootStateSuppliers(Supplier<Float> heightSupplier, Supplier<float[]> rootXZSupplier) {
        this.heightSupplier = heightSupplier;
        this.rootXZSupplier = rootXZSupplier;
        this.spawnPositionDirty = true;
    }

    public void setHeightSupplier(Supplier<Float> heightSupplier) {
        this.heightSupplier = heightSupplier;
        this.spawnPositionDirty = true;
    }

    public void setRootXZSupplier(Supplier<float[]> rootXZSupplier) {
        this.rootXZSupplier = rootXZSupplier;
        this.spawnPositionDirty = true;
    }

    // ========== 메인 tick ==========
    public void tick(float deltaTime) {
        if (!isInitialized || !trainingActive) return;
        if (agentMode == AgentMode.MANUAL) return;
        if (renderer == null) return;

        lastDeltaTime = sanitizeDeltaTime(deltaTime);

        float[] obsRef = getObservationRef();

        if (config.debugRootSource) maybeDebugRoot();

        // priming: 첫 tick은 action만 걸고 다음 tick부터 transition 저장
        if (!transitionPrimed) {
            ensureLastBuffers(obsRef.length);
            copyInto(lastObs, obsRef);

            ensureLastActionBuffer();
            float[] selected = agent.selectAction(lastObs, agentMode);
            copyInto(lastAction, selected);

            applyAction(lastAction);

            transitionPrimed = true;
            updatePrevRootPositionsNow();
            timeInEpisode += lastDeltaTime;
            return;
        }

        float reward = calculateReward(lastAction);
        lastReward = reward;
        episodeReward += reward;

        stepCount++;
        boolean terminated = checkTermination();
        boolean truncated = stepCount >= config.maxEpisodeSteps;
        isDone = terminated || truncated;

        if (agentMode == AgentMode.LEARNING) {
            agent.storeExperience(lastObs, lastAction, reward, obsRef, isDone);

            if (stepCount % config.updateInterval == 0) {
                agent.update(config.minUpdateBatch);
            }
        }

        if (isDone) {
            endEpisode(terminated ? "terminated" : "truncated");
            return;
        }

        float[] action = agent.selectAction(obsRef, agentMode);
        ensureLastActionBuffer();
        copyInto(lastAction, action);
        applyAction(lastAction);

        ensureLastBuffers(obsRef.length);
        copyInto(lastObs, obsRef);

        updatePrevRootPositionsNow();
        timeInEpisode += lastDeltaTime;
    }

    // ========== reset ==========
    public float[] reset() {
        if (!isInitialized || renderer == null) return new float[0];

        // ✅ 물리 바디까지 스폰 상태로 되돌리기 (넘어졌을 때 "진짜 처음 상태" 복구)
        if (config.resetPhysicsBodiesOnReset) {
            try {
                URDFSimpleController controller = renderer.getController();
                if (controller != null && controller.isUsingPhysics()) {
                    boolean ok = controller.hardResetToSpawn(null);
                    if (!ok) controller.resetBodyVelocitiesOnly();
                }
            } catch (Exception e) {
                logger.debug("resetPhysicsBodiesOnReset failed", e);
            }
        }

        stepCount = 0;
        episodeReward = 0f;
        lastReward = 0f;
        isDone = false;
        timeInEpisode = 0f;

        transitionPrimed = false;

        Random rand = config.randomizeInitial ? new Random() : null;

        for (JointMeta jm : jointMetas) {
            float initPos = jm.initialPosition;
            if (rand != null) {
                float range = (jm.maxLimit - jm.minLimit);
                float noise = (rand.nextFloat() - 0.5f) * range * config.initNoiseScale;
                initPos += noise;
                initPos = clamp(initPos, jm.minLimit, jm.maxLimit);
            }
            renderer.setJointTarget(jm.name, initPos, JointControlSource.RL);
        }

        spawnPositionDirty = true;
        refreshSpawnRootPosition();

        updatePrevRootPositionsNow();

        ensureObservationBuffer();
        float[] obs = getObservationRef();
        return obs.clone();
    }

    // ========== applyAction ==========
    private void applyAction(float[] action) {
        if (action == null || renderer == null) return;

        int numActions = Math.min(action.length, jointMetas.size());
        for (int i = 0; i < numActions; i++) {
            JointMeta jm = jointMetas.get(i);
            float a = clamp(action[i], -1f, 1f);

            float currentPos = renderer.getJointPosition(jm.name);

            switch (config.actionMode) {
                case TORQUE -> {
                    float delta = a * config.maxDeltaPosition;
                    float target = clamp(currentPos + delta, jm.minLimit, jm.maxLimit);
                    renderer.setJointTarget(jm.name, target, JointControlSource.RL);
                }
                case POSITION -> {
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
                    renderer.setJointVelocity(jm.name, targetVel);
                }
            }
        }
    }

    // ========== 관측 ==========
    public float[] getObservation() {
        ensureObservationBuffer();
        float[] ref = getObservationRef();
        return ref.clone();
    }

    private float[] getObservationRef() {
        ensureObservationBuffer();
        fillObservation(obsBuffer);
        return obsBuffer;
    }

    private void ensureObservationBuffer() {
        int dim = getObservationDim();
        if (obsBuffer == null || obsBuffer.length != dim) obsBuffer = new float[dim];
        if (agent != null) agent.ensureWeightsForObsDim(dim);
    }

    private void fillObservation(float[] out) {
        if (renderer == null || jointMetas.isEmpty()) {
            Arrays.fill(out, 0f);
            return;
        }

        int idx = 0;

        // joint positions [-1,1]
        for (JointMeta jm : jointMetas) {
            float pos = renderer.getJointPosition(jm.name);
            float range = jm.maxLimit - jm.minLimit;
            float norm = (range > 1e-8f) ? 2f * (pos - jm.minLimit) / range - 1f : 0f;
            out[idx++] = clamp(norm, -1f, 1f);
        }

        // joint velocities
        if (config.includeVelocities) {
            float denom = (config.maxVelocity > 1e-8f) ? config.maxVelocity : 1f;
            for (JointMeta jm : jointMetas) {
                float vel = renderer.getJointVelocity(jm.name);
                float v = vel / denom;
                if (config.jointVelocityObsClip > 0f) {
                    v = clamp(v, -config.jointVelocityObsClip, config.jointVelocityObsClip);
                }
                out[idx++] = v;
            }
        }

        // observed root
        getObservedRootPositionInto(observedRootPosBuf);
        computeVelocityFromCurrent(observedRootPosBuf, prevObservedRootPosition, observedRootVelBuf);

        float heightRange = config.maxHeight - config.minHeight;
        float heightNorm = (heightRange > 1e-8f) ? (observedRootPosBuf[1] - config.minHeight) / heightRange : 0.5f;
        if (config.heightObsClamp01) heightNorm = clamp(heightNorm, 0f, 1f);
        out[idx++] = heightNorm;

        float speedScale = (config.targetSpeed > 1e-8f) ? config.targetSpeed : 1f;
        float vx = observedRootVelBuf[0] / speedScale;
        float vz = observedRootVelBuf[2] / speedScale;
        if (config.rootVelocityObsClip > 0f) {
            vx = clamp(vx, -config.rootVelocityObsClip, config.rootVelocityObsClip);
            vz = clamp(vz, -config.rootVelocityObsClip, config.rootVelocityObsClip);
        }
        out[idx++] = vx;
        out[idx++] = vz;

        float currentSpeed = (float) Math.sqrt(observedRootVelBuf[0] * observedRootVelBuf[0] +
                                               observedRootVelBuf[2] * observedRootVelBuf[2]);
        float speedDiff = (config.targetSpeed > 1e-8f) ? (config.targetSpeed - currentSpeed) / speedScale : 0f;
        if (config.speedDiffObsClip > 0f) speedDiff = clamp(speedDiff, -config.speedDiffObsClip, config.speedDiffObsClip);
        out[idx++] = speedDiff;

        if (idx != out.length && idx < out.length) Arrays.fill(out, idx, out.length, 0f);
    }

    // ========== Root (Observed / Physics) ==========
    private void refreshSpawnRootPosition() {
        if (!spawnPositionInitialized || spawnPositionDirty) {
            float x = spawnPositionInitialized ? spawnRootPosition[0] : 0f;
            float y = spawnPositionInitialized ? spawnRootPosition[1] : config.targetHeight;
            float z = spawnPositionInitialized ? spawnRootPosition[2] : 0f;

            if (heightSupplier != null) {
                try {
                    Float sampledY = heightSupplier.get();
                    if (sampledY != null && Float.isFinite(sampledY)) y = sampledY;
                } catch (Exception e) {
                    logger.debug("heightSupplier sample failed", e);
                }
            }

            if (rootXZSupplier != null) {
                try {
                    float[] xz = rootXZSupplier.get();
                    if (xz != null && xz.length >= 2) {
                        if (Float.isFinite(xz[0])) x = xz[0];
                        if (Float.isFinite(xz[1])) z = xz[1];
                    }
                } catch (Exception e) {
                    logger.debug("rootXZSupplier sample failed", e);
                }
            }

            if (Float.isFinite(y) && config.groundCollisionMargin != 0f) y += config.groundCollisionMargin;

            spawnRootPosition[0] = x;
            spawnRootPosition[1] = y;
            spawnRootPosition[2] = z;
            spawnPositionInitialized = true;
            spawnPositionDirty = false;
        }
    }

    private void getPhysicsRootPositionInto(float[] out) {
        refreshSpawnRootPosition();

        float baseX = spawnRootPosition[0];
        float baseY = spawnRootPosition[1];
        float baseZ = spawnRootPosition[2];

        float x = baseX, y = baseY, z = baseZ;

        lastPhysicsRootValid = readPhysicsRootInto(physRootWork);
        if (lastPhysicsRootValid) {
            x = physRootWork[0];
            y = physRootWork[1];
            z = physRootWork[2];
        }

        if (!Float.isFinite(x)) x = baseX;
        if (!Float.isFinite(y)) y = baseY;
        if (!Float.isFinite(z)) z = baseZ;

        out[0] = x; out[1] = y; out[2] = z;
    }

    private void getObservedRootPositionInto(float[] out) {
        refreshSpawnRootPosition();

        float baseX = spawnRootPosition[0];
        float baseY = spawnRootPosition[1];
        float baseZ = spawnRootPosition[2];

        float x = baseX, y = baseY, z = baseZ;

        if (readPhysicsRootInto(physRootWork)) {
            x = physRootWork[0];
            y = physRootWork[1];
            z = physRootWork[2];
        }

        if (config.overrideRootWithSuppliers) {
            if (rootXZSupplier != null) {
                try {
                    float[] xz = rootXZSupplier.get();
                    if (xz != null && xz.length >= 2) {
                        if (Float.isFinite(xz[0])) x = xz[0];
                        if (Float.isFinite(xz[1])) z = xz[1];
                    }
                } catch (Exception e) {
                    logger.debug("rootXZSupplier read failed", e);
                }
            }
            if (heightSupplier != null) {
                try {
                    Float hy = heightSupplier.get();
                    if (hy != null && Float.isFinite(hy)) y = hy;
                } catch (Exception e) {
                    logger.debug("heightSupplier read failed", e);
                }
            }
        }

        if (!Float.isFinite(x)) x = baseX;
        if (!Float.isFinite(y)) y = baseY;
        if (!Float.isFinite(z)) z = baseZ;

        out[0] = x; out[1] = y; out[2] = z;
    }

    private boolean readPhysicsRootInto(float[] out) {
        if (renderer == null) return false;

        try {
            URDFSimpleController controller = renderer.getController();
            if (controller == null) return false;

            // ✅ 중요: 물리 모드가 아니면 physics root로 인정하지 않음
            if (!controller.isUsingPhysics()) return false;

            // ✅ 가능하면 physics-only API 우선 사용
            double[] rootWorld = controller.getRootBodyWorldPositionPhysicsOnly();
            if (rootWorld == null || rootWorld.length < 3) return false;

            float x = (float) rootWorld[0];
            float y = (float) rootWorld[1];
            float z = (float) rootWorld[2];

            if (!Float.isFinite(x) || !Float.isFinite(y) || !Float.isFinite(z)) return false;

            out[0] = x; out[1] = y; out[2] = z;
            return true;
        } catch (Exception e) {
            logger.debug("Failed to read physics root", e);
            return false;
        }
    }

    private void computeVelocityFromCurrent(float[] currentRootPos, float[] prevRootPos, float[] outVel) {
        float dt = safeDt();
        outVel[0] = (currentRootPos[0] - prevRootPos[0]) / dt;
        outVel[1] = (currentRootPos[1] - prevRootPos[1]) / dt;
        outVel[2] = (currentRootPos[2] - prevRootPos[2]) / dt;
    }

    private void updatePrevRootPositionsNow() {
        getObservedRootPositionInto(observedRootPosBuf);
        System.arraycopy(observedRootPosBuf, 0, prevObservedRootPosition, 0, 3);

        getPhysicsRootPositionInto(physicsRootPosBuf);
        System.arraycopy(physicsRootPosBuf, 0, prevPhysicsRootPosition, 0, 3);
    }

    private void maybeDebugRoot() {
        debugTickCounter++;
        int interval = Math.max(1, config.rootDebugPrintInterval);
        if ((debugTickCounter % interval) != 0) return;

        getObservedRootPositionInto(observedRootPosBuf);
        getPhysicsRootPositionInto(physicsRootPosBuf);

        log(String.format(
                "RootDebug dt=%.4f | obs(%.3f,%.3f,%.3f) | phys(%.3f,%.3f,%.3f) | physValid=%s",
                lastDeltaTime,
                observedRootPosBuf[0], observedRootPosBuf[1], observedRootPosBuf[2],
                physicsRootPosBuf[0], physicsRootPosBuf[1], physicsRootPosBuf[2],
                String.valueOf(lastPhysicsRootValid)
        ));
    }

    private float safeDt() {
        float dt = lastDeltaTime;
        if (dt > 1e-6f && Float.isFinite(dt)) return dt;
        if (config.timeStep > 1e-6f && Float.isFinite(config.timeStep)) return config.timeStep;
        return 0.02f;
    }

    private float sanitizeDeltaTime(float dt) {
        if (!Float.isFinite(dt) || dt <= 0f) return safeDt();
        if (config.maxAllowedDeltaTime > 0f && dt > config.maxAllowedDeltaTime) return config.maxAllowedDeltaTime;
        return dt;
    }

    // ========== 보상 ==========
    private float calculateReward(float[] action) {
        float reward = 0f;

        reward += config.aliveBonus;

        float[] rp;
        float[] rv;

        if (config.usePhysicsRootForReward) {
            getPhysicsRootPositionInto(physicsRootPosBuf);
            computeVelocityFromCurrent(physicsRootPosBuf, prevPhysicsRootPosition, physicsRootVelBuf);
            rp = physicsRootPosBuf;
            rv = physicsRootVelBuf;
        } else {
            getObservedRootPositionInto(observedRootPosBuf);
            computeVelocityFromCurrent(observedRootPosBuf, prevObservedRootPosition, observedRootVelBuf);
            rp = observedRootPosBuf;
            rv = observedRootVelBuf;
        }

        float heightDiff = Math.abs(rp[1] - config.targetHeight);
        float heightReward = (config.targetHeight > 1e-8f) ? (1f - heightDiff / config.targetHeight) : 0f;
        reward += heightReward * config.heightRewardWeight;

        float currentSpeed = (float) Math.sqrt(rv[0] * rv[0] + rv[2] * rv[2]);
        float speedError = Math.abs(currentSpeed - config.targetSpeed);
        reward -= speedError * config.speedMatchWeight;

        float controlCost = 0f;
        if (action != null) for (float a : action) controlCost += a * a;
        reward -= controlCost * config.controlCostWeight;

        float velocityPenalty = 0f;
        for (JointMeta jm : jointMetas) {
            float vel = renderer.getJointVelocity(jm.name);
            velocityPenalty += vel * vel;
        }
        reward -= velocityPenalty * config.velocityPenaltyWeight;

        float limitPenalty = 0f;
        for (JointMeta jm : jointMetas) {
            float pos = renderer.getJointPosition(jm.name);
            float range = jm.maxLimit - jm.minLimit;
            if (range <= 1e-8f) continue;
            float margin = 0.1f * range;
            if (pos < jm.minLimit + margin || pos > jm.maxLimit - margin) limitPenalty += 0.1f;
        }
        reward -= limitPenalty;

        reward += calculateSymmetryReward() * config.symmetryRewardWeight;

        if (referenceMotion != null && config.poseMatchWeight > 0f) {
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
        float symmetry = 0f;
        int pairs = 0;

        for (JointMeta jm : jointMetas) {
            String name = jm.name;
            if (name.contains("_L_") || name.contains("Left") || name.contains("_l_")) {
                String rightName = name.replace("_L_", "_R_").replace("Left", "Right").replace("_l_", "_r_");
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

        if (config.usePhysicsRootForTermination) {
            getPhysicsRootPositionInto(physicsRootPosBuf);

            if (physicsRootPosBuf[1] < config.minHeight) {
                log(String.format("Terminated: height too low (physics y=%.3f)", physicsRootPosBuf[1]));
                return true;
            }
            if (physicsRootPosBuf[1] > config.maxHeight) {
                log(String.format("Terminated: height too high (physics y=%.3f)", physicsRootPosBuf[1]));
                return true;
            }
            return false;
        }

        getObservedRootPositionInto(observedRootPosBuf);
        if (observedRootPosBuf[1] < config.minHeight) return true;
        if (observedRootPosBuf[1] > config.maxHeight) return true;
        return false;
    }

    public boolean isHealthy() {
        if (config.usePhysicsRootForTermination) {
            getPhysicsRootPositionInto(physicsRootPosBuf);
            return physicsRootPosBuf[1] >= config.minHeight && physicsRootPosBuf[1] <= config.maxHeight;
        } else {
            getObservedRootPositionInto(observedRootPosBuf);
            return observedRootPosBuf[1] >= config.minHeight && observedRootPosBuf[1] <= config.maxHeight;
        }
    }

    // ========== 에피소드 관리 ==========
    private void endEpisode(String reason) {
        episodeCount++;
        stats.recordEpisode(episodeReward, stepCount);
        log(String.format("Episode %d ended (%s): reward=%.2f, steps=%d",
                episodeCount, reason, episodeReward, stepCount));

        // ✅ 짧은 에피소드에서도 update 1회(옵션)
        if (trainingActive && agentMode == AgentMode.LEARNING && config.updateOnEpisodeEnd) {
            agent.update(config.minUpdateBatch);
        }

        if (trainingActive) reset();
    }

    // ========== 학습 제어 ==========
    public void startTraining(AgentMode mode) {
        if (!isInitialized) {
            log("ERROR: Cannot start training - not initialized");
            return;
        }

        agentMode = mode;

        boolean hadManualLocks = renderer != null && renderer.hasManualJointLocks();
        if (renderer != null) renderer.clearManualJointLocks();
        if (hadManualLocks) log("Cleared manual joint overrides before training");

        trainingActive = true;
        reset();
        log("Training started: mode=" + mode);
    }

    public void stopTraining() {
        trainingActive = false;
        transitionPrimed = false;
        lastObs = null;
        lastAction = null;
        log("Training stopped");
    }

    public void setAgentMode(AgentMode mode) {
        this.agentMode = mode;
        log("Agent mode: " + mode);
    }

    // ========== 디버그 수동 ==========
    public void manualStep() {
        if (!isInitialized || renderer == null) return;

        lastDeltaTime = sanitizeDeltaTime(config.timeStep);

        float[] obs = getObservationRef();
        float[] action = agent.selectAction(obs, AgentMode.RANDOM);
        applyAction(action);

        float reward = calculateReward(action);
        lastReward = reward;
        episodeReward += reward;

        stepCount++;
        updatePrevRootPositionsNow();
        timeInEpisode += lastDeltaTime;

        log(String.format("Manual step %d: reward=%.4f", stepCount, reward));
    }

    // ========== 정보 ==========
    public int getObservationDim() {
        int dim = jointMetas.size();
        if (config.includeVelocities) dim += jointMetas.size();
        dim += 4;
        return dim;
    }

    public int getActionDim() { return jointMetas.size(); }
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
        for (JointMeta jm : jointMetas) names.add(jm.name);
        return names;
    }

    public float getJointPosition(String name) { return (renderer == null) ? 0f : renderer.getJointPosition(name); }
    public float getJointVelocity(String name) { return (renderer == null) ? 0f : renderer.getJointVelocity(name); }

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
        info.put("primed", transitionPrimed);
        info.put("physicsRootValid", lastPhysicsRootValid);
        return info;
    }

    // ========== 유틸 ==========
    private float clamp(float v, float min, float max) { return Math.max(min, Math.min(max, v)); }

    private void ensureLastBuffers(int obsDim) {
        if (lastObs == null || lastObs.length != obsDim) lastObs = new float[obsDim];
    }

    private void ensureLastActionBuffer() {
        if (lastAction == null || lastAction.length != getActionDim()) lastAction = new float[getActionDim()];
    }

    private void copyInto(float[] dst, float[] src) {
        if (dst == null || src == null) return;
        int n = Math.min(dst.length, src.length);
        System.arraycopy(src, 0, dst, 0, n);
        if (n < dst.length) Arrays.fill(dst, n, dst.length, 0f);
    }

    public void setLogCallback(Consumer<String> callback) { this.logCallback = callback; }

    private void log(String msg) {
        logger.info(msg);
        if (logCallback != null) logCallback.accept(msg);
    }

    // ========== 내부 ==========
    private static class JointMeta {
        String name;
        float minLimit;
        float maxLimit;
        float initialPosition;
    }

    public enum ActionMode { TORQUE, POSITION, DELTA_POSITION, VELOCITY }
    public enum AgentMode { MANUAL, RANDOM, LEARNING, INFERENCE, IMITATION }

    public static class Config {
        // sim
        public float timeStep = 0.02f;
        public int maxEpisodeSteps = 500;
        public int updateInterval = 64;
        public float maxAllowedDeltaTime = 0.1f;

        // obs
        public boolean includeVelocities = true;
        public boolean heightObsClamp01 = true;
        public float jointVelocityObsClip = 5f;
        public float rootVelocityObsClip = 5f;
        public float speedDiffObsClip = 5f;

        public boolean overrideRootWithSuppliers = true;

        // ✅ reward/termination root source
        public boolean usePhysicsRootForReward = true;
        public boolean usePhysicsRootForTermination = true;

        // action
        public ActionMode actionMode = ActionMode.POSITION;
        public float maxTorque = 50f;
        public float maxVelocity = 5f;
        public float maxDeltaPosition = 0.1f;

        // reward weights
        public float aliveBonus = 0.1f;
        public float heightRewardWeight = 1.0f;
        public float speedMatchWeight = 0.5f;
        public float controlCostWeight = 0.01f;
        public float velocityPenaltyWeight = 0.001f;
        public float symmetryRewardWeight = 0.1f;
        public float poseMatchWeight = 0.0f;

        // targets
        public float targetHeight = 1.0f;
        public float targetSpeed = 0f;

        // termination
        public boolean terminateOnFall = true;
        public float minHeight = 0.3f;
        public float maxHeight = 2.0f;

        // init
        public boolean randomizeInitial = true;
        public float initNoiseScale = 0.05f;

        // collision margin
        public float groundCollisionMargin = 0.05f;

        // ✅ NEW: reset에서 물리 바디까지 되돌릴지
        public boolean resetPhysicsBodiesOnReset = true;

        // ✅ NEW: 짧은 에피소드에서도 종료 시 update 1회
        public boolean updateOnEpisodeEnd = true;
        public int minUpdateBatch = 32;

        // debug
        public boolean debugRootSource = false;
        public int rootDebugPrintInterval = 60;
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

        public float getBestReward() { return (bestReward == Float.NEGATIVE_INFINITY) ? 0f : bestReward; }
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
            initializeWeights(obsDimSupplier.getAsInt());
        }

        public void ensureWeightsForObsDim(int obsDim) {
            if (weights == null || weights.length != obsDim) initializeWeights(obsDim);
        }

        private void initializeWeights(int obsDim) {
            weights = new float[obsDim][actionDim];
            float scale = (float) Math.sqrt(2.0 / (obsDim + actionDim));
            for (int i = 0; i < obsDim; i++) {
                for (int j = 0; j < actionDim; j++) {
                    weights[i][j] = (random.nextFloat() - 0.5f) * 2f * scale;
                }
            }
        }

        public float[] selectAction(float[] observation, AgentMode mode) {
            if (observation == null) observation = new float[Math.max(1, obsDimSupplier.getAsInt())];
            ensureWeightsForObsDim(observation.length);

            return switch (mode) {
                case RANDOM -> randomAction();
                case LEARNING, INFERENCE -> policyAction(observation, mode == AgentMode.LEARNING);
                case IMITATION -> imitationAction();
                case MANUAL -> zeroAction();
            };
        }

        private float[] randomAction() {
            float[] a = new float[actionDim];
            for (int i = 0; i < actionDim; i++) a[i] = random.nextFloat() * 2f - 1f;
            return a;
        }

        private float[] zeroAction() { return new float[actionDim]; }

        private float[] policyAction(float[] obs, boolean explore) {
            float[] action = new float[actionDim];
            for (int j = 0; j < actionDim; j++) {
                float sum = 0f;
                int m = Math.min(obs.length, weights.length);
                for (int i = 0; i < m; i++) sum += obs[i] * weights[i][j];

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
            if (imitationTargets != null && imitationTargets.length == actionDim) return imitationTargets.clone();
            return zeroAction();
        }

        public void storeExperience(float[] obs, float[] action, float reward, float[] nextObs, boolean done) {
            experiences.add(new Experience(
                    (obs != null ? obs.clone() : new float[0]),
                    (action != null ? action.clone() : new float[0]),
                    reward,
                    (nextObs != null ? nextObs.clone() : new float[0]),
                    done
            ));
            if (experiences.size() > BUFFER_SIZE) experiences.remove(0);
        }

        public void update() { update(64); }

        public void update(int minBatch) {
            if (experiences.size() < Math.max(1, minBatch)) return;
            if (weights == null || weights.length == 0) return;

            float mean = 0f;
            for (Experience e : experiences) mean += e.reward;
            mean /= experiences.size();

            float var = 0f;
            for (Experience e : experiences) {
                float d = e.reward - mean;
                var += d * d;
            }
            float std = (float) Math.sqrt(var / experiences.size() + 1e-8f);

            for (Experience e : experiences) {
                float adv = (std > 1e-8f) ? (e.reward - mean) / std : 0f;

                int m = Math.min(e.obs.length, weights.length);
                int n = Math.min(actionDim, e.action.length);

                for (int j = 0; j < n; j++) {
                    float a = e.action[j];
                    float grad = (1f - a * a); // tanh'
                    for (int i = 0; i < m; i++) {
                        weights[i][j] += learningRate * adv * e.obs[i] * grad;
                    }
                }
            }

            experiences.clear();
        }

        public void setImitationTargets(float[] targets) { this.imitationTargets = targets; }

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
