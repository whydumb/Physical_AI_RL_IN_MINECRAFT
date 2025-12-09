package com.kAIS.KAIMyEntity.rl;

import com.kAIS.KAIMyEntity.urdf.URDFModel;
import com.kAIS.KAIMyEntity.urdf.URDFModelOpenGLWithSTL;
import com.kAIS.KAIMyEntity.urdf.control.URDFMotion;
import com.kAIS.KAIMyEntity.urdf.vmd.VMDLoader;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.io.File;

/**
 * URDF + VMD + RL 환경을 한 번에 세팅해 주는 헬퍼.
 * - 환경 초기화
 * - VMD → URDFMotion 로드
 * - referenceMotion 설정
 * - 기본 보상/환경 파라미터 설정
 * (학습 시작은 선택)
 */
public final class RLVMDIntegration {
    private static final Logger logger = LogManager.getLogger();

    private RLVMDIntegration() {}

    /**
     * renderer만 가지고 RL 환경 초기화.
     * 이미 초기화 돼 있으면 다시 안 함.
     */
    public static RLEnvironmentCore initEnvironment(URDFModelOpenGLWithSTL renderer) {
        if (renderer == null) {
            logger.error("Renderer is null. Cannot initialize RL.");
            return null;
        }
        RLEnvironmentCore env = RLEnvironmentCore.getInstance();
        if (!env.isInitialized()) {
            env.initialize(renderer);
            logger.info("RL environment initialized with renderer: {}", renderer);
        } else {
            logger.info("RL environment already initialized. Skipping re-init.");
        }
        return env;
    }

    /**
     * VMD 파일을 로드해서 referenceMotion으로 등록하고,
     * 모방/포즈 보상에 적합한 기본 Config를 세팅한다.
     *
     * @param renderer URDF 렌더러
     * @param vmdFile  VMD 파일
     * @param startLearning  true면 바로 LEARNING 모드로 학습 시작
     */
    public static void setupVmdImitation(
            URDFModelOpenGLWithSTL renderer,
            File vmdFile,
            boolean startLearning
    ) {
        if (renderer == null) {
            logger.error("Renderer is null. Cannot setup VMD imitation.");
            return;
        }
        if (vmdFile == null || !vmdFile.exists()) {
            logger.error("VMD file not found: {}", vmdFile);
            return;
        }

        // 1) 환경 초기화 (필요하면)
        RLEnvironmentCore env = initEnvironment(renderer);
        if (env == null || !env.isInitialized()) {
            logger.error("RLEnvironmentCore initialization failed.");
            return;
        }

        // 2) URDF 모델 (joint mapping용)
        URDFModel urdfModel = renderer.getRobotModel();
        if (urdfModel == null) {
            logger.warn("URDF model is null in renderer. VMD joint mapping may be limited.");
        }

        // 3) VMD → URDFMotion 로드
        URDFMotion motion;
        try {
            // urdfModel을 받는 오버로드가 있으면 사용
            if (urdfModel != null) {
                motion = VMDLoader.load(vmdFile, urdfModel);
            } else {
                motion = VMDLoader.load(vmdFile);
            }
        } catch (Exception e) {
            logger.error("Failed to load VMD as URDFMotion: {}", vmdFile.getName(), e);
            return;
        }

        if (motion == null || motion.keys == null || motion.keys.isEmpty()) {
            logger.error("Failed to load VMD or empty motion keys: {}", vmdFile.getName());
            return;
        }

        logger.info("Loaded VMD for RL imitation: {} ({} keyframes)", motion.name, motion.keys.size());

        // 4) referenceMotion으로 설정
        env.setReferenceMotion(motion);

        // 5) Config 세팅 (모방/포즈 위주 세팅 예시)
        RLEnvironmentCore.Config cfg = env.getConfig();

        // 기본 물리 파라미터는 그대로 두고, 보상 중 포즈/속도 관련만 조정
        cfg.poseMatchWeight   = 2.0f;   // 포즈 모방 비중
        cfg.speedMatchWeight  = 0.0f;   // 일단 속도 보상은 끔
        cfg.targetSpeed       = 0.0f;
        cfg.heightRewardWeight = 0.5f;
        cfg.aliveBonus        = 0.05f;

        // 에피소드 길이를 모션 길이에 맞추고 싶으면:
        // float duration = motion.getDuration(); // (samplePose 구현에 따라 다름)
        // cfg.maxEpisodeSteps = (int)(duration / cfg.timeStep);

        logger.info("RL Config updated for VMD imitation (poseMatchWeight={}, speedMatchWeight={})",
                cfg.poseMatchWeight, cfg.speedMatchWeight);

        // 6) 원하면 여기서 바로 학습 시작
        if (startLearning) {
            env.startTraining(RLEnvironmentCore.AgentMode.LEARNING);
            logger.info("✅ RL imitation learning started with motion: {}", motion.name);
        }
    }
}
