package com.kAIS.KAIMyEntity.urdf.control;

import com.kAIS.KAIMyEntity.urdf.URDFModelOpenGLWithSTL;
import com.kAIS.KAIMyEntity.webots.WebotsController;
import com.kAIS.KAIMyEntity.rl.RLEnvironmentCore;
import net.minecraft.world.entity.Entity;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;

public final class PosePipeline {
    private static final Logger LOGGER = LogManager.getLogger();
    private static volatile PosePipeline instance;

    // 입력 적용 순서 옵션 (deprecated - 이제 고정 순서: RL → VMD → Physics)
    @Deprecated
    private volatile boolean applyInputsBeforeModelUpdate = false;

    // Webots
    private volatile int statsIntervalTicks = 100; // 5초(20tps 가정)
    private int statsTickCounter = 0;

    private volatile boolean enableWebotsStats = true;
    private volatile boolean enableWebotsSend  = true;

    private WebotsController webots;
    private boolean webotsInitialized = false;

    // MotionEditorScreen이 채우는 frame 버퍼(재사용)
    private final Map<String, Float> frameScratch = new HashMap<>();

    private PosePipeline() {}

    public static PosePipeline getInstance() {
        if (instance == null) {
            synchronized (PosePipeline.class) {
                if (instance == null) instance = new PosePipeline();
            }
        }
        return instance;
    }

    @Deprecated
    public void setApplyInputsBeforeModelUpdate(boolean v) {
        // 이제 항상 고정 순서이므로 무시
        LOGGER.warn("setApplyInputsBeforeModelUpdate is deprecated - order is now fixed: RL → VMD → Physics");
    }

    public void setStatsIntervalTicks(int ticks) { this.statsIntervalTicks = Math.max(1, ticks); }
    public void setEnableWebotsStats(boolean v) { this.enableWebotsStats = v; }
    public void setEnableWebotsSend(boolean v) { this.enableWebotsSend = v; }

    /**
     * ClientTickLoop가 호출하는 단일 진입점.
     * - single + many 중복 인스턴스는 제거(Identity 기준)
     *
     * @param dt 델타 타임
     * @param entity Minecraft Entity (물리 포함 버전용, null 허용)
     * @param single 단일 URDF 모델
     * @param many 다중 URDF 모델 리스트
     */
    public void onClientTick(float dt, Entity entity, URDFModelOpenGLWithSTL single, List<URDFModelOpenGLWithSTL> many) {
        Map<URDFModelOpenGLWithSTL, Boolean> uniq = new IdentityHashMap<>();
        if (single != null) uniq.put(single, Boolean.TRUE);
        if (many != null) {
            for (URDFModelOpenGLWithSTL r : many) {
                if (r != null) uniq.put(r, Boolean.TRUE);
            }
        }

        for (URDFModelOpenGLWithSTL urdf : uniq.keySet()) {
            tickOne(dt, urdf, entity);
        }

        if (enableWebotsStats && ++statsTickCounter >= statsIntervalTicks) {
            statsTickCounter = 0;
            printWebotsStats();
        }
    }

    /**
     * 레거시 호환용 - Entity 없는 버전
     */
    @Deprecated
    public void onClientTick(float dt, URDFModelOpenGLWithSTL single, List<URDFModelOpenGLWithSTL> many) {
        onClientTick(dt, null, single, many);
    }

    /**
     * 단일 URDF에 대한 틱 처리
     * 고정 순서: RL action 주입 → VMD 적용 → 물리 step
     */
    private void tickOne(float dt, URDFModelOpenGLWithSTL urdf, Entity entity) {
        if (urdf == null) return;

        frameScratch.clear();

        // === 고정 순서 시작 ===

        // 1) RL action 주입 (물리 step 이전에 실행되어야 함)
        try {
            RLEnvironmentCore.getInstance().tick(dt);
        } catch (Exception e) {
            LOGGER.error("RL tick failed", e);
        }

        // 2) VMD/모션 적용 (내부에서 RL 활성 여부에 따라 preview/target 분기)
        try {
            MotionEditorScreen.tick(urdf);
        } catch (Exception e) {
            LOGGER.error("MotionEditorScreen tick failed", e);
        }

        // 3) 물리 step
        try {
            if (entity != null) {
                // 물리 포함 버전 (Entity의 위치/속도 동기화)
                urdf.tickUpdate(dt, entity);
            } else {
                // 물리 미포함 버전 (기존 호환)
                urdf.tickUpdate(dt);
            }
        } catch (Exception e) {
            LOGGER.error("URDF tickUpdate failed", e);
        }

        // === 고정 순서 끝 ===

        // Webots 전송 (✅ 수정: sendFrame → setJoints)
        if (enableWebotsSend && !frameScratch.isEmpty()) {
            WebotsController wc = getWebots();
            if (wc != null) {
                wc.setJoints(frameScratch);  // ✅ 수정됨
            }
        }
    }

    private WebotsController getWebots() {
        if (!webotsInitialized) {
            webotsInitialized = true;
            try {
                webots = WebotsController.getInstance();
            } catch (Exception e) {
                webots = null;
                LOGGER.debug("WebotsController init failed (ignored): {}", e.getMessage());
            }
        }
        return webots;
    }

    private void printWebotsStats() {
        WebotsController wc = getWebots();
        if (wc != null && wc.isConnected()) wc.printStats();
    }

    // ===== ClientTickLoop에서 제공하던 API 호환용 =====

    public void reconnectWebots(String ip, int port) {
        WebotsController wc = getWebots();
        if (wc != null) wc.reconnect(ip, port);
        else {
            try {
                webots = WebotsController.getInstance(ip, port);
                webotsInitialized = true;
            } catch (Exception ignored) {}
        }
    }

    public boolean isWebotsConnected() {
        WebotsController wc = getWebots();
        return wc != null && wc.isConnected();
    }

    public String getWebotsAddress() {
        WebotsController wc = getWebots();
        return wc != null ? wc.getRobotAddress() : "Not initialized";
    }
}
