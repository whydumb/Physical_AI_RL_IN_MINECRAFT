package com.kAIS.KAIMyEntity.neoforge;

import com.kAIS.KAIMyEntity.coordinator.CoordinatorHttpServer;
import com.kAIS.KAIMyEntity.coordinator.MotionCoordinator;
import com.kAIS.KAIMyEntity.webots.WebotsController;

import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.client.event.ClientTickEvent;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * 통합 ClientTickLoop
 * - RobotListener 모드: WASD + 마우스 → RobotListener
 * - Coordinator heartbeat 전송
 * - Coordinator HTTP 서버 관리
 */
@EventBusSubscriber(
        modid = "kaimyentity",
        value = Dist.CLIENT
)
public final class ClientTickLoop {
    private static final Logger LOGGER = LogManager.getLogger();
    
    private static WebotsController webots;
    private static boolean webotsInitialized = false;
    
    // Coordinator HTTP 서버
    private static CoordinatorHttpServer coordinatorServer;
    private static boolean coordinatorInitialized = false;
    private static final int COORDINATOR_PORT = 8081;
    
    private static int tickCount = 0;
    private static final int STATS_INTERVAL = 100; // 5초마다
    
    // Heartbeat 주기 (10초 = 200틱)
    private static int heartbeatTickCount = 0;
    private static final int HEARTBEAT_INTERVAL = 200;

    @SubscribeEvent
    public static void onClientTick(ClientTickEvent.Post event) {
        // Coordinator HTTP 서버 시작 (최초 1회)
        ensureCoordinatorServer();
        
        // RobotListener 모드: WASD + 마우스
        WebotsController wc = getWebots();
        if (wc != null && wc.getMode() == WebotsController.Mode.ROBOTLISTENER) {
            wc.tick();
        }
        
        // Heartbeat 전송 (10초마다)
        if (++heartbeatTickCount >= HEARTBEAT_INTERVAL) {
            heartbeatTickCount = 0;
            if (wc != null && wc.isRobotListenerEnabled() && wc.hasLock()) {
                wc.sendHeartbeat();
            }
        }
        
        // 통계 출력 (5초마다)
        if (++tickCount >= STATS_INTERVAL) {
            tickCount = 0;
            if (wc != null && wc.isConnected()) {
                wc.printStats();
            }
        }
    }
    
    /**
     * Coordinator HTTP 서버 초기화
     */
    private static void ensureCoordinatorServer() {
        if (!coordinatorInitialized) {
            coordinatorInitialized = true;
            try {
                coordinatorServer = CoordinatorHttpServer.getInstance(COORDINATOR_PORT);
                coordinatorServer.start();
                LOGGER.info("Coordinator HTTP server started on port {}", COORDINATOR_PORT);
            } catch (Exception e) {
                LOGGER.error("Failed to start Coordinator HTTP server", e);
                coordinatorServer = null;
            }
        }
    }
    
    /**
     * WebotsController (지연 초기화)
     */
    private static WebotsController getWebots() {
        if (!webotsInitialized) {
            webotsInitialized = true;
            try {
                webots = WebotsController.getInstance();
            } catch (Exception e) {
                webots = null;
            }
        }
        return webots;
    }
    
    // ==================== 외부 제어 API ====================
    
    public static void reconnectWebots(String ip, int port) {
        WebotsController wc = getWebots();
        if (wc != null) {
            wc.reconnect(ip, port);
        } else {
            try {
                webots = WebotsController.getInstance(ip, port);
                webotsInitialized = true;
            } catch (Exception e) {
                // 실패 무시
            }
        }
    }
    
    public static void enableRobotListener(boolean enable) {
        WebotsController wc = getWebots();
        if (wc != null) {
            wc.enableRobotListener(enable);
        }
    }
    
    public static WebotsController.Mode getMode() {
        WebotsController wc = getWebots();
        return wc != null ? wc.getMode() : WebotsController.Mode.WEBOTS;
    }
    
    public static boolean isWebotsConnected() {
        WebotsController wc = getWebots();
        return wc != null && wc.isConnected();
    }
    
    public static boolean isRobotListenerEnabled() {
        WebotsController wc = getWebots();
        return wc != null && wc.isRobotListenerEnabled();
    }
    
    public static String getWebotsAddress() {
        WebotsController wc = getWebots();
        return wc != null ? wc.getRobotAddress() : "Not initialized";
    }
    
    // ==================== Coordinator API ====================
    
    public static MotionCoordinator.LockStatus getCoordinatorStatus() {
        return MotionCoordinator.getInstance().getStatus();
    }
    
    public static boolean isCoordinatorServerRunning() {
        return coordinatorServer != null && coordinatorServer.isRunning();
    }
    
    public static int getCoordinatorPort() {
        return COORDINATOR_PORT;
    }
    
    public static void forceReleaseLock(String reason) {
        MotionCoordinator.getInstance().forceRelease(reason);
    }
}
