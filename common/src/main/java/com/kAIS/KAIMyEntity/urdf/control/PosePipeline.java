package com.kAIS.KAIMyEntity.urdf.control;

import com.kAIS.KAIMyEntity.urdf.URDFModelOpenGLWithSTL;
import com.kAIS.KAIMyEntity.webots.WebotsController;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;

public final class PosePipeline {
    private static final Logger LOGGER = LogManager.getLogger();
    private static volatile PosePipeline instance;

    // 입력 적용 순서 옵션
    // false: urdf.tickUpdate(dt) -> MotionEditorScreen.tick(...)
    // true : MotionEditorScreen.tick(...) -> urdf.tickUpdate(dt)
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

    public void setApplyInputsBeforeModelUpdate(boolean v) { this.applyInputsBeforeModelUpdate = v; }
    public void setStatsIntervalTicks(int ticks) { this.statsIntervalTicks = Math.max(1, ticks); }
    public void setEnableWebotsStats(boolean v) { this.enableWebotsStats = v; }
    public void setEnableWebotsSend(boolean v) { this.enableWebotsSend = v; }

    /**
     * ClientTickLoop가 호출하는 단일 진입점.
     * - single + many 중복 인스턴스는 제거(Identity 기준)
     */
    public void onClientTick(float dt, URDFModelOpenGLWithSTL single, List<URDFModelOpenGLWithSTL> many) {
        Map<URDFModelOpenGLWithSTL, Boolean> uniq = new IdentityHashMap<>();
        if (single != null) uniq.put(single, Boolean.TRUE);
        if (many != null) {
            for (URDFModelOpenGLWithSTL r : many) {
                if (r != null) uniq.put(r, Boolean.TRUE);
            }
        }

        for (URDFModelOpenGLWithSTL urdf : uniq.keySet()) {
            tickOne(dt, urdf);
        }

        if (enableWebotsStats && ++statsTickCounter >= statsIntervalTicks) {
            statsTickCounter = 0;
            printWebotsStats();
        }
    }

    private void tickOne(float dt, URDFModelOpenGLWithSTL urdf) {
        if (urdf == null) return;

        frameScratch.clear();

        if (applyInputsBeforeModelUpdate) {
            MotionEditorScreen.tick(urdf, frameScratch);
            urdf.tickUpdate(dt);
        } else {
            urdf.tickUpdate(dt);
            MotionEditorScreen.tick(urdf, frameScratch);
        }

        if (enableWebotsSend && !frameScratch.isEmpty()) {
            WebotsController wc = getWebots();
            if (wc != null) {
                wc.sendFrame(frameScratch);
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