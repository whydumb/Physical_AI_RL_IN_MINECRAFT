// common/src/main/java/com/kAIS/KAIMyEntity/webots/WebotsController.java
package com.kAIS.KAIMyEntity.webots;

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

public class WebotsController {
    private static final Logger LOGGER = LogManager.getLogger();
    private static WebotsController instance;

    private final HttpClient httpClient;
    private String webotsUrl;
    private String robotIp;
    private int robotPort;
    private final ExecutorService executor;
    private final ScheduledExecutorService scheduler;
    private final BlockingQueue<Command> commandQueue;
    private final Map<String, Float> lastSent;
    private static final float DELTA_THRESHOLD = 0.01f;

    private volatile boolean connected = false;
    private volatile int failureCount = 0;
    private static final int MAX_FAILURES = 10;

    private final Stats stats = new Stats();

    // ==================== ✅ 수정: 충돌 방지를 위한 JOINT_MAP ====================
    private static final Map<String, JointMapping> JOINT_MAP = new HashMap<>();

    static {
        // 머리
        JOINT_MAP.put("head_pan",  new JointMapping("Neck",  18, -1.57f,  1.57f));
        JOINT_MAP.put("head_tilt", new JointMapping("Head",  19, -0.52f,  0.52f));

        // ✅ 오른쪽 팔 - 충돌 방지를 위한 안전 범위
        JOINT_MAP.put("r_sho_pitch", new JointMapping("ShoulderR", 0, -1.57f,  0.52f));
        JOINT_MAP.put("r_sho_roll",  new JointMapping("ArmUpperR", 2, -0.15f,  2.30f));  // ✅ -0.68 → -0.15
        JOINT_MAP.put("r_el",        new JointMapping("ArmLowerR", 4, -1.57f, -0.10f));

        // ✅ 왼쪽 팔 - 대칭 처리
        JOINT_MAP.put("l_sho_pitch", new JointMapping("ShoulderL", 1, -1.57f,  0.52f));
        JOINT_MAP.put("l_sho_roll",  new JointMapping("ArmUpperL", 3, -2.25f,  0.15f));  // ✅ 0.77 → 0.15
        JOINT_MAP.put("l_el",        new JointMapping("ArmLowerL", 5, -1.57f, -0.10f));

        // 골반
        JOINT_MAP.put("r_hip_yaw",   new JointMapping("PelvYR", 6, -1.047f, 1.047f));
        JOINT_MAP.put("l_hip_yaw",   new JointMapping("PelvYL", 7, -0.69f,  2.50f));
        JOINT_MAP.put("r_hip_roll",  new JointMapping("PelvR",  8, -1.01f,  1.01f));
        JOINT_MAP.put("l_hip_roll",  new JointMapping("PelvL",  9, -0.35f,  0.35f));

        // 다리
        JOINT_MAP.put("r_hip_pitch", new JointMapping("LegUpperR", 10, -2.50f, 0.87f));
        JOINT_MAP.put("l_hip_pitch", new JointMapping("LegUpperL", 11, -2.50f, 0.87f));
        JOINT_MAP.put("r_hip_roll",  new JointMapping("LegLowerR", 12, -0.35f, 0.35f));
        JOINT_MAP.put("l_hip_roll",  new JointMapping("LegLowerL", 13, -0.35f, 0.35f));

        JOINT_MAP.put("r_knee", new JointMapping("KneeR", 14, -0.1f, 2.09f));
        JOINT_MAP.put("l_knee", new JointMapping("KneeL", 15, -0.1f, 2.09f));

        JOINT_MAP.put("r_ank_pitch", new JointMapping("AnkleR", 14, -0.87f, 0.87f));
        JOINT_MAP.put("l_ank_pitch", new JointMapping("AnkleL", 15, -1.39f, 1.22f));
        JOINT_MAP.put("r_ank_roll",  new JointMapping("FootR",  16, -0.87f, 0.87f));
        JOINT_MAP.put("l_ank_roll",  new JointMapping("FootL",  17, -0.87f, 0.87f));

        // 역호환용 Webots 이름
        JOINT_MAP.put("ShoulderR", new JointMapping("ShoulderR", 0, -1.57f, 0.52f));
        JOINT_MAP.put("ShoulderL", new JointMapping("ShoulderL", 1, -1.57f, 0.52f));
        JOINT_MAP.put("ArmUpperR", new JointMapping("ArmUpperR", 2, -0.15f, 2.30f));  // ✅ 수정
        JOINT_MAP.put("ArmUpperL", new JointMapping("ArmUpperL", 3, -2.25f, 0.15f));  // ✅ 수정
        JOINT_MAP.put("ArmLowerR", new JointMapping("ArmLowerR", 4, -1.57f, -0.10f));
        JOINT_MAP.put("ArmLowerL", new JointMapping("ArmLowerL", 5, -1.57f, -0.10f));
        JOINT_MAP.put("Neck",      new JointMapping("Neck",      18, -1.57f, 1.57f));
        JOINT_MAP.put("Head",      new JointMapping("Head",      19, -0.52f, 0.52f));
    }

    private WebotsController(String ip, int port) {
        this.robotIp = ip;
        this.robotPort = port;
        this.webotsUrl = String.format("http://%s:%d", ip, port);

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
        this.lastSent = new ConcurrentHashMap<>();

        scheduler.scheduleAtFixedRate(this::processQueue, 0, 20, TimeUnit.MILLISECONDS);
        testConnection();

        LOGGER.info("✅ WebotsController initialized: {}", webotsUrl);
    }

    /**
     * ✅ 개선: WebotsConfigScreen.Config에서 기본값 로드
     */
    public static WebotsController getInstance() {
        if (instance == null) {
            // WebotsConfigScreen.Config에서 마지막 저장된 IP/Port 가져오기
            try {
                WebotsConfigScreen.Config config = WebotsConfigScreen.Config.getInstance();
                instance = new WebotsController(config.getLastIp(), config.getLastPort());
            } catch (Exception e) {
                // Config 로드 실패 시 기본값 사용
                LOGGER.warn("Failed to load config, using defaults", e);
                instance = new WebotsController("localhost", 8080);
            }
        }
        return instance;
    }

    /**
     * ✅ 개선: Config 저장 포함
     */
    public static WebotsController getInstance(String ip, int port) {
        if (instance != null) {
            if (!instance.robotIp.equals(ip) || instance.robotPort != port) {
                LOGGER.info("🔄 Recreating WebotsController with new address: {}:{}", ip, port);
                instance.shutdown();
                instance = new WebotsController(ip, port);
                
                // ✅ Config에 저장
                try {
                    WebotsConfigScreen.Config config = WebotsConfigScreen.Config.getInstance();
                    config.update(ip, port);
                } catch (Exception e) {
                    LOGGER.warn("Failed to save config", e);
                }
            }
        } else {
            instance = new WebotsController(ip, port);
            
            // ✅ Config에 저장
            try {
                WebotsConfigScreen.Config config = WebotsConfigScreen.Config.getInstance();
                config.update(ip, port);
            } catch (Exception e) {
                LOGGER.warn("Failed to save config", e);
            }
        }
        return instance;
    }

    /**
     * ✅ 개선: Config 저장 포함
     */
    public void reconnect(String ip, int port) {
        LOGGER.info("🔄 Reconnecting to {}:{}", ip, port);
        this.robotIp = ip;
        this.robotPort = port;
        this.webotsUrl = String.format("http://%s:%d", ip, port);
        this.failureCount = 0;
        this.connected = false;

        commandQueue.clear();
        lastSent.clear();

        testConnection();
        
        // ✅ Config에 저장
        try {
            WebotsConfigScreen.Config config = WebotsConfigScreen.Config.getInstance();
            config.update(ip, port);
        } catch (Exception e) {
            LOGGER.warn("Failed to save config", e);
        }
    }

    private void testConnection() {
        executor.submit(() -> {
            try {
                String url = webotsUrl + "/?command=get_stats";
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
                    LOGGER.info("✅ Connected to Webots: {}", webotsUrl);
                } else {
                    LOGGER.warn("⚠️  Webots returned status {}", response.statusCode());
                }

            } catch (Exception e) {
                connected = false;
                LOGGER.error("❌ Failed to connect to Webots: {}", e.getMessage());
            }
        });
    }

    public void setJoint(String jointName, float value) {
        JointMapping mapping = JOINT_MAP.get(jointName);
        if (mapping == null) {
            if (stats.unknownJointWarnings.computeIfAbsent(jointName, k -> 0) < 3) {
                LOGGER.warn("Unknown joint: {} (warning {} of 3)", jointName,
                           stats.unknownJointWarnings.merge(jointName, 1, Integer::sum));
            }
            return;
        }

        // URDF → Webots 변환 (부호 반전 + 범위 매핑)
        float webotsValue = convertUrdfToWebots(jointName, value);

        Float last = lastSent.get(jointName);
        if (last != null && Math.abs(webotsValue - last) < DELTA_THRESHOLD) {
            stats.deltaSkipped++;
            return;
        }

        float clamped = clamp(webotsValue, mapping.min, mapping.max);
        if (Math.abs(clamped - value) > 0.001f) {
            stats.rangeClamped++;
        }

        if (commandQueue.offer(new Command(mapping.index, clamped))) {
            lastSent.put(jointName, clamped);
            stats.queued++;
        } else {
            stats.queueFull++;
        }
    }

    public void setJoints(Map<String, Float> joints) {
        joints.forEach(this::setJoint);
    }

    private void processQueue() {
        Command cmd = commandQueue.poll();
        if (cmd == null) return;

        executor.submit(() -> sendToWebots(cmd.index, cmd.value));
    }

    private void sendToWebots(int index, float value) {
        if (!connected && failureCount > MAX_FAILURES) {
            return;
        }

        try {
            String url = String.format("%s/?command=set_joint&index=%d&value=%.4f",
                                      webotsUrl, index, value);

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
                    LOGGER.info("✅ Reconnected to Webots");
                }
            } else {
                stats.failed++;
                LOGGER.warn("⚠️  Webots returned status {}", response.statusCode());
            }

        } catch (Exception e) {
            stats.failed++;
            failureCount++;

            if (failureCount == MAX_FAILURES) {
                connected = false;
                LOGGER.error("❌ Connection lost to Webots after {} failures", MAX_FAILURES);
            } else if (failureCount % 50 == 0) {
                LOGGER.warn("⚠️  Failed to send to Webots ({} failures): {}",
                           failureCount, e.getMessage());
            }
        }
    }

    public String getStatsJson() {
        try {
            String url = webotsUrl + "/?command=get_stats";
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

    public void printStats() {
        LOGGER.info("=== Webots Controller Stats ===");
        LOGGER.info("  Target: {}:{} {}", robotIp, robotPort, connected ? "✅" : "❌");
        LOGGER.info("  Queued: {} | Sent: {} | Failed: {}", stats.queued, stats.sent, stats.failed);
        LOGGER.info("  Delta Skipped: {} | Range Clamped: {} | Queue Full: {}",
                   stats.deltaSkipped, stats.rangeClamped, stats.queueFull);
        LOGGER.info("  Queue Size: {} | Failure Count: {}", commandQueue.size(), failureCount);

        String serverStats = getStatsJson();
        LOGGER.info("  Server Stats: {}", serverStats);
    }

    public boolean isConnected() {
        return connected;
    }

    public String getRobotAddress() {
        return String.format("%s:%d", robotIp, robotPort);
    }

    public void shutdown() {
        LOGGER.info("🛑 Shutting down WebotsController...");
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

    // ========== 내부 클래스 ==========

    private static class Command {
        final int index;
        final float value;
        final long timestamp;

        Command(int index, float value) {
            this.index = index;
            this.value = value;
            this.timestamp = System.currentTimeMillis();
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
        long queued = 0;
        long sent = 0;
        long failed = 0;
        long deltaSkipped = 0;
        long rangeClamped = 0;
        long queueFull = 0;
        final Map<String, Integer> unknownJointWarnings = new ConcurrentHashMap<>();
    }

    private static float clamp(float value, float min, float max) {
        return Math.max(min, Math.min(max, value));
    }

    // ========== 유틸리티 메서드 ==========

    public static String[] getSupportedJoints() {
        return JOINT_MAP.keySet().toArray(new String[0]);
    }

    public static JointMapping getJointMapping(String jointName) {
        return JOINT_MAP.get(jointName);
    }

    public static Integer getMotorIndex(String jointName) {
        JointMapping mapping = JOINT_MAP.get(jointName);
        return mapping != null ? mapping.index : null;
    }

    // ====================== URDF → Webots 변환기 ======================
    private float convertUrdfToWebots(String jointName, float urdfValue) {
        return switch (jointName) {
            // 팔꿈치 (기본 설정)
            case "r_el" -> map(urdfValue, 0.0f, 2.7925f, -0.10f, -1.57f);
            case "l_el" -> map(urdfValue, -2.7925f, 0.0f, -1.57f, -0.10f);

            // ⚠️ 만약 위 설정으로도 팔이 반대로 꺾인다면, 아래 주석을 해제하고 위 2줄을 주석 처리하세요:
            // case "r_el" -> map(urdfValue, 0.0f, 2.7925f, -1.57f, -0.10f);
            // case "l_el" -> map(urdfValue, -2.7925f, 0.0f, -0.10f, -1.57f);

            // 무릎 (역방향)
            case "r_knee", "l_knee" -> map(urdfValue, -2.27f, 0.0f, 2.09f, -0.1f);

            // 머리 (Webots가 더 좁음)
            case "head_pan"  -> clamp(urdfValue, -1.57f, 1.57f);
            case "head_tilt" -> clamp(urdfValue, -0.52f, 0.52f);

            // 기타 미세 차이
            case "l_ank_pitch" -> clamp(urdfValue, -1.39f, 1.22f);
            case "r_hip_yaw"   -> clamp(urdfValue, -1.047f, 1.047f);
            case "l_hip_yaw"   -> clamp(urdfValue, -0.69f, 2.50f);

            default -> urdfValue; // 나머지는 1:1
        };
    }

    private float map(float v, float fromLow, float fromHigh, float toLow, float toHigh) {
        if (v <= fromLow) return toLow;
        if (v >= fromHigh) return toHigh;
        return toLow + (v - fromLow) * (toHigh - toLow) / (fromHigh - fromLow);
    }
}