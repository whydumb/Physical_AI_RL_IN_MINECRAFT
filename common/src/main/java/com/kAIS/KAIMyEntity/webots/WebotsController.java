// common/src/main/java/com/kAIS/KAIMyEntity/webots/WebotsController.java
package com.kAIS.KAIMyEntity.webots;

import net.minecraft.client.Minecraft;
import net.minecraft.client.player.LocalPlayer;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.*;

/**
 * 통합 Webots/RobotListener 컨트롤러
 * 
 * 기능:
 * 1. Webots 연동 (URDF → Webots HTTP API)
 * 2. RobotListener 연동 (Minecraft WASD + 마우스 → RobotListener)
 * 
 * 두 가지 모드:
 * - WEBOTS 모드: set_joint API 사용 (URDF 관절 제어)
 * - ROBOTLISTENER 모드: set_walk + set_head API 사용 (실시간 제어)
 */
public class WebotsController {
    private static final Logger LOGGER = LogManager.getLogger();
    private static WebotsController instance;

    // ==================== 모드 설정 ====================
    public enum Mode {
        WEBOTS,          // Webots 직접 제어 (set_joint)
        ROBOTLISTENER    // RobotListener를 통한 제어 (set_walk + set_head)
    }
    
    private Mode currentMode = Mode.WEBOTS;

    // ==================== 네트워크 ====================
    private final HttpClient httpClient;
    private String serverIp;
    private int serverPort;
    private String serverUrl;
    
    private final ExecutorService executor;
    private final ScheduledExecutorService scheduler;
    private final BlockingQueue<Command> commandQueue;
    
    private volatile boolean connected = false;
    private volatile int failureCount = 0;
    private static final int MAX_FAILURES = 10;

    // ==================== Webots 관련 ====================
    private final Map<String, Float> lastSentJoint;
    private static final float JOINT_DELTA_THRESHOLD = 0.01f;
    
    // 조인트 매핑 (URDF → Webots)
    private static final Map<String, JointMapping> JOINT_MAP = new HashMap<>();
    
    static {
        // 머리
        JOINT_MAP.put("head_pan",  new JointMapping("Neck",  18, -1.57f,  1.57f));
        JOINT_MAP.put("head_tilt", new JointMapping("Head",  19, -0.52f,  0.52f));
        
        // 오른팔
        JOINT_MAP.put("r_sho_pitch", new JointMapping("ShoulderR", 0, -1.57f,  0.52f));
        JOINT_MAP.put("r_sho_roll",  new JointMapping("ArmUpperR", 2, -0.68f,  2.30f));
        JOINT_MAP.put("r_el",        new JointMapping("ArmLowerR", 4, -1.57f, -0.10f));
        
        // 왼팔
        JOINT_MAP.put("l_sho_pitch", new JointMapping("ShoulderL", 1, -1.57f,  0.52f));
        JOINT_MAP.put("l_sho_roll",  new JointMapping("ArmUpperL", 3, -2.25f,  0.77f));
        JOINT_MAP.put("l_el",        new JointMapping("ArmLowerL", 5, -1.57f, -0.10f));
        
        // 역호환 (Webots 이름으로도 접근 가능)
        JOINT_MAP.put("Neck", new JointMapping("Neck", 18, -1.57f, 1.57f));
        JOINT_MAP.put("Head", new JointMapping("Head", 19, -0.52f, 0.52f));
        JOINT_MAP.put("ShoulderR", new JointMapping("ShoulderR", 0, -1.57f, 0.52f));
        JOINT_MAP.put("ShoulderL", new JointMapping("ShoulderL", 1, -1.57f, 0.52f));
        JOINT_MAP.put("ArmUpperR", new JointMapping("ArmUpperR", 2, -0.68f, 2.30f));
        JOINT_MAP.put("ArmUpperL", new JointMapping("ArmUpperL", 3, -2.25f, 0.77f));
        JOINT_MAP.put("ArmLowerR", new JointMapping("ArmLowerR", 4, -1.57f, -0.10f));
        JOINT_MAP.put("ArmLowerL", new JointMapping("ArmLowerL", 5, -1.57f, -0.10f));
    }

    // ==================== RobotListener 관련 ====================
    private boolean robotListenerEnabled = false;
    
    // WASD 이전 상태 (델타 감지)
    private boolean lastF = false, lastB = false, lastL = false, lastR = false;
    private float lastYaw = 0.0f;
    private float lastPitch = 0.0f;
    
    // 민감도
    private static final float YAW_SENSITIVITY = 0.01f;    // 0.57도
    private static final float PITCH_SENSITIVITY = 0.01f;
    
    // 모터 범위
    private static final float NECK_MIN = -1.57f;
    private static final float NECK_MAX = 1.57f;
    private static final float HEAD_MIN = -0.52f;
    private static final float HEAD_MAX = 0.52f;

    // ==================== 통계 ====================
    private final Stats stats = new Stats();

    // ==================== 생성자 ====================
    
    private WebotsController(String ip, int port) {
        this.serverIp = ip;
        this.serverPort = port;
        this.serverUrl = String.format("http://%s:%d", ip, port);

        this.httpClient = HttpClient.newBuilder()
                .connectTimeout(Duration.ofMillis(500))
                .build();

        this.executor = Executors.newSingleThreadExecutor(r -> {
            Thread t = new Thread(r, "Webots-Sender");
            t.setDaemon(true);
            return t;
        });

        this.scheduler = Executors.newScheduledThreadPool(1, r -> {
            Thread t = new Thread(r, "Webots-Scheduler");
            t.setDaemon(true);
            return t;
        });

        this.commandQueue = new LinkedBlockingQueue<>();
        this.lastSentJoint = new ConcurrentHashMap<>();

        scheduler.scheduleAtFixedRate(this::processQueue, 0, 20, TimeUnit.MILLISECONDS);
        testConnection();

        LOGGER.info("✅ WebotsController initialized: {}", serverUrl);
    }

    public static WebotsController getInstance() {
        if (instance == null) {
            try {
                WebotsConfigScreen.Config config = WebotsConfigScreen.Config.getInstance();
                instance = new WebotsController(config.getLastIp(), config.getLastPort());
            } catch (Exception e) {
                LOGGER.warn("Failed to load config, using defaults", e);
                instance = new WebotsController("localhost", 8080);
            }
        }
        return instance;
    }

    public static WebotsController getInstance(String ip, int port) {
        if (instance != null) {
            if (!instance.serverIp.equals(ip) || instance.serverPort != port) {
                LOGGER.info("🔄 Recreating WebotsController: {}:{}", ip, port);
                instance.shutdown();
                instance = new WebotsController(ip, port);
                
                try {
                    WebotsConfigScreen.Config config = WebotsConfigScreen.Config.getInstance();
                    config.update(ip, port);
                } catch (Exception e) {
                    LOGGER.warn("Failed to save config", e);
                }
            }
        } else {
            instance = new WebotsController(ip, port);
            
            try {
                WebotsConfigScreen.Config config = WebotsConfigScreen.Config.getInstance();
                config.update(ip, port);
            } catch (Exception e) {
                LOGGER.warn("Failed to save config", e);
            }
        }
        return instance;
    }

    // ==================== 모드 전환 ====================
    
    public void setMode(Mode mode) {
        this.currentMode = mode;
        LOGGER.info("Mode changed to: {}", mode);
    }
    
    public Mode getMode() {
        return currentMode;
    }

    // ==================== Webots 모드: 관절 제어 ====================
    
    /**
     * URDF 관절을 Webots로 전송 (Webots 모드)
     */
    public void setJoint(String jointName, float value) {
        if (currentMode != Mode.WEBOTS) return;
        
        JointMapping mapping = JOINT_MAP.get(jointName);
        if (mapping == null) {
            if (stats.unknownJointWarnings.computeIfAbsent(jointName, k -> 0) < 3) {
                LOGGER.warn("Unknown joint: {} (warning {} of 3)", jointName,
                           stats.unknownJointWarnings.merge(jointName, 1, Integer::sum));
            }
            return;
        }

        float webotsValue = convertUrdfToWebots(jointName, value);
        Float last = lastSentJoint.get(jointName);
        
        if (last != null && Math.abs(webotsValue - last) < JOINT_DELTA_THRESHOLD) {
            stats.deltaSkipped++;
            return;
        }

        float clamped = clamp(webotsValue, mapping.min, mapping.max);
        
        if (commandQueue.offer(new Command(CommandType.SET_JOINT, mapping.index, clamped))) {
            lastSentJoint.put(jointName, clamped);
            stats.queued++;
        } else {
            stats.queueFull++;
        }
    }

    public void setJoints(Map<String, Float> joints) {
        joints.forEach(this::setJoint);
    }

    // ==================== RobotListener 모드: 실시간 제어 ====================
    
    /**
     * RobotListener 모드 활성화
     */
    public void enableRobotListener(boolean enable) {
        this.robotListenerEnabled = enable;
        if (enable) {
            setMode(Mode.ROBOTLISTENER);
            LOGGER.info("🎮 RobotListener mode enabled");
        } else {
            setMode(Mode.WEBOTS);
            // 긴급 정지
            sendStopAll();
            LOGGER.info("🛑 RobotListener mode disabled");
        }
    }
    
    /**
     * 매 틱 호출 (RobotListener 모드)
     */
    public void tick() {
        if (currentMode != Mode.ROBOTLISTENER || !robotListenerEnabled || !connected) {
            return;
        }
        
        Minecraft mc = Minecraft.getInstance();
        LocalPlayer player = mc.player;
        if (player == null) return;
        
        // WASD 키 상태
        boolean f = mc.options.keyUp.isDown();
        boolean b = mc.options.keyDown.isDown();
        boolean l = mc.options.keyLeft.isDown();
        boolean r = mc.options.keyRight.isDown();
        
        // 마우스 에임
        float yaw = player.getYRot();
        float pitch = player.getXRot();
        
        // WASD 변화 감지
        if (f != lastF || b != lastB || l != lastL || r != lastR) {
            sendWalkCommand(f, b, l, r);
            lastF = f; lastB = b; lastL = l; lastR = r;
        }
        
        // 마우스 에임 변화 감지
        float yawDelta = Math.abs(yaw - lastYaw);
        float pitchDelta = Math.abs(pitch - lastPitch);
        
        if (yawDelta > YAW_SENSITIVITY * 57.3f || pitchDelta > PITCH_SENSITIVITY * 57.3f) {
            sendHeadCommand(yaw, pitch);
            lastYaw = yaw;
            lastPitch = pitch;
        }
    }
    
    /**
     * WASD 명령 전송
     */
    private void sendWalkCommand(boolean f, boolean b, boolean l, boolean r) {
        String url = String.format(
            "%s/?command=set_walk&f=%d&b=%d&l=%d&r=%d",
            serverUrl, f ? 1 : 0, b ? 1 : 0, l ? 1 : 0, r ? 1 : 0
        );
        
        sendAsyncDirect(url).thenAccept(success -> {
            if (success) stats.walkSent++;
            else stats.failed++;
        });
    }
    
    /**
     * 마우스 에임 명령 전송
     */
    private void sendHeadCommand(float yawDeg, float pitchDeg) {
        float yawRad = (float) Math.toRadians(yawDeg);
        float pitchRad = (float) Math.toRadians(-pitchDeg);
        
        yawRad = clamp(yawRad, NECK_MIN, NECK_MAX);
        pitchRad = clamp(pitchRad, HEAD_MIN, HEAD_MAX);
        
        String url = String.format(
            "%s/?command=set_head&yaw=%.3f&pitch=%.3f",
            serverUrl, yawRad, pitchRad
        );
        
        sendAsyncDirect(url).thenAccept(success -> {
            if (success) stats.headSent++;
            else stats.failed++;
        });
    }
    
    /**
     * 긴급 정지
     */
    private void sendStopAll() {
        String url = String.format("%s/?command=stop_all", serverUrl);
        sendAsyncDirect(url);
    }

    // ==================== HTTP 통신 ====================
    
    private void processQueue() {
        Command cmd = commandQueue.poll();
        if (cmd == null) return;
        
        if (cmd.type == CommandType.SET_JOINT) {
            executor.submit(() -> sendJointToWebots(cmd.motorIndex, cmd.value));
        }
    }

    private void sendJointToWebots(int index, float value) {
        if (!connected && failureCount > MAX_FAILURES) return;

        try {
            String url = String.format("%s/?command=set_joint&index=%d&value=%.4f",
                                      serverUrl, index, value);

            HttpRequest request = HttpRequest.newBuilder()
                    .uri(URI.create(url))
                    .timeout(Duration.ofMillis(100))
                    .GET()
                    .build();

            HttpResponse<String> response = httpClient.send(request,
                    HttpResponse.BodyHandlers.ofString());

            if (response.statusCode() == 200) {
                stats.sent++;
                failureCount = 0;
                if (!connected) {
                    connected = true;
                    LOGGER.info("✅ Reconnected to server");
                }
            } else {
                stats.failed++;
            }

        } catch (Exception e) {
            stats.failed++;
            failureCount++;

            if (failureCount == MAX_FAILURES) {
                connected = false;
                LOGGER.error("❌ Connection lost after {} failures", MAX_FAILURES);
            }
        }
    }
    
    /**
     * 직접 전송 (RobotListener 명령용)
     */
    private CompletableFuture<Boolean> sendAsyncDirect(String url) {
        HttpRequest request = HttpRequest.newBuilder()
                .uri(URI.create(url))
                .timeout(Duration.ofMillis(100))
                .GET()
                .build();
        
        return httpClient.sendAsync(request, HttpResponse.BodyHandlers.discarding())
                .thenApply(response -> {
                    boolean success = (response.statusCode() == 200);
                    if (success) {
                        connected = true;
                        failureCount = 0;
                    }
                    return success;
                })
                .exceptionally(e -> {
                    failureCount++;
                    if (failureCount >= MAX_FAILURES) {
                        connected = false;
                    }
                    return false;
                });
    }

    private void testConnection() {
        executor.submit(() -> {
            try {
                String url = serverUrl + "/?command=get_stats";
                HttpRequest request = HttpRequest.newBuilder()
                        .uri(URI.create(url))
                        .timeout(Duration.ofMillis(500))
                        .GET()
                        .build();

                HttpResponse<String> response = httpClient.send(request,
                        HttpResponse.BodyHandlers.ofString());

                if (response.statusCode() == 200) {
                    connected = true;
                    failureCount = 0;
                    LOGGER.info("✅ Connected to server: {}", serverUrl);
                }

            } catch (Exception e) {
                connected = false;
                LOGGER.error("❌ Failed to connect: {}", e.getMessage());
            }
        });
    }

    // ==================== 재연결 ====================
    
    public void reconnect(String ip, int port) {
        LOGGER.info("🔄 Reconnecting to {}:{}", ip, port);
        this.serverIp = ip;
        this.serverPort = port;
        this.serverUrl = String.format("http://%s:%d", ip, port);
        this.failureCount = 0;
        this.connected = false;

        commandQueue.clear();
        lastSentJoint.clear();

        testConnection();
        
        try {
            WebotsConfigScreen.Config.getInstance().update(ip, port);
        } catch (Exception e) {
            LOGGER.warn("Failed to save config", e);
        }
    }

    // ==================== 통계 ====================
    
    public void printStats() {
        LOGGER.info("=== WebotsController Stats ===");
        LOGGER.info("  Mode: {}", currentMode);
        LOGGER.info("  Target: {}:{} {}", serverIp, serverPort, connected ? "✅" : "❌");
        
        if (currentMode == Mode.WEBOTS) {
            LOGGER.info("  [Webots] Queued: {} | Sent: {} | Failed: {}", 
                       stats.queued, stats.sent, stats.failed);
            LOGGER.info("  [Webots] Delta Skipped: {} | Queue Full: {}", 
                       stats.deltaSkipped, stats.queueFull);
        } else {
            LOGGER.info("  [RobotListener] Walk: {} | Head: {} | Failed: {}", 
                       stats.walkSent, stats.headSent, stats.failed);
        }
    }

    public String getStatsJson() {
        try {
            String url = serverUrl + "/?command=get_stats";
            HttpRequest request = HttpRequest.newBuilder()
                    .uri(URI.create(url))
                    .timeout(Duration.ofMillis(200))
                    .GET()
                    .build();

            HttpResponse<String> response = httpClient.send(request,
                    HttpResponse.BodyHandlers.ofString());

            return response.body();

        } catch (Exception e) {
            return String.format("{\"error\": \"%s\"}", e.getMessage());
        }
    }

    // ==================== Getters ====================
    
    public boolean isConnected() { return connected; }
    public boolean isRobotListenerEnabled() { return robotListenerEnabled; }
    public String getRobotAddress() { return String.format("%s:%d", serverIp, serverPort); }
    public long getWalkSent() { return stats.walkSent; }
    public long getHeadSent() { return stats.headSent; }
    public long getErrors() { return stats.failed; }

    // ==================== 종료 ====================
    
    public void shutdown() {
        LOGGER.info("🛑 Shutting down WebotsController...");
        
        if (robotListenerEnabled) {
            sendStopAll();
        }
        
        scheduler.shutdown();
        executor.shutdown();
        try {
            if (!executor.awaitTermination(1, TimeUnit.SECONDS)) {
                executor.shutdownNow();
            }
        } catch (InterruptedException e) {
            executor.shutdownNow();
        }
        LOGGER.info("✅ WebotsController shutdown complete");
    }

    // ==================== 내부 클래스 ====================
    
    private enum CommandType {
        SET_JOINT
    }
    
    private static class Command {
        final CommandType type;
        final int motorIndex;
        final float value;
        
        Command(CommandType type, int motorIndex, float value) {
            this.type = type;
            this.motorIndex = motorIndex;
            this.value = value;
        }
    }

    private static class JointMapping {
        final String webotsName;
        final int index;
        final float min;
        final float max;

        JointMapping(String webotsName, int index, float min, float max) {
            this.webotsName = webotsName;
            this.index = index;
            this.min = min;
            this.max = max;
        }
    }

    private static class Stats {
        // Webots 모드
        long queued = 0;
        long sent = 0;
        long deltaSkipped = 0;
        long queueFull = 0;
        
        // RobotListener 모드
        long walkSent = 0;
        long headSent = 0;
        
        // 공통
        long failed = 0;
        final Map<String, Integer> unknownJointWarnings = new ConcurrentHashMap<>();
    }

    // ==================== 유틸리티 ====================
    
    private static float clamp(float v, float min, float max) {
        return v < min ? min : (v > max ? max : v);
    }
    
    private float map(float v, float fromLow, float fromHigh, float toLow, float toHigh) {
        if (v <= fromLow) return toLow;
        if (v >= fromHigh) return toHigh;
        return toLow + (v - fromLow) * (toHigh - toLow) / (fromHigh - fromLow);
    }

    private float convertUrdfToWebots(String jointName, float urdfValue) {
        return switch (jointName) {
            case "r_el" -> map(urdfValue, 0.0f, 2.7925f, -0.10f, -1.57f);
            case "l_el" -> map(urdfValue, -2.7925f, 0.0f, -1.57f, -0.10f);
            case "r_knee", "l_knee" -> map(urdfValue, -2.27f, 0.0f, 2.09f, -0.1f);
            case "head_pan" -> clamp(urdfValue, -1.57f, 1.57f);
            case "head_tilt" -> clamp(urdfValue, -0.52f, 0.52f);
            default -> urdfValue;
        };
    }
}
