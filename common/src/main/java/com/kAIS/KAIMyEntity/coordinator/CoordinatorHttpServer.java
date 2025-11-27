package com.kAIS.KAIMyEntity.coordinator;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.URI;
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Executors;

/**
 * CoordinatorHttpServer - Coordinator HTTP API 서버
 * 
 * 엔드포인트:
 *   GET  /coord/status              - Lock 상태 조회
 *   POST /coord/acquire             - Lock 획득
 *   POST /coord/release             - Lock 해제
 *   POST /coord/heartbeat           - Heartbeat 갱신
 *   POST /coord/force_release       - 강제 해제 (관리자용)
 * 
 * Query 파라미터:
 *   owner       - 요청자 식별자 (필수)
 *   task        - 태스크 설명 (acquire 시 선택)
 *   duration_ms - 예상 소요 시간 (acquire 시 선택)
 *   reason      - 강제 해제 사유 (force_release 시 선택)
 * 
 * 응답 형식: JSON
 * 
 * 사용 예:
 *   curl "http://localhost:8081/coord/status"
 *   curl -X POST "http://localhost:8081/coord/acquire?owner=contral&task=manual_test"
 *   curl -X POST "http://localhost:8081/coord/heartbeat?owner=contral"
 *   curl -X POST "http://localhost:8081/coord/release?owner=contral"
 */
public class CoordinatorHttpServer {
    private static final Logger LOGGER = LogManager.getLogger();
    
    private static CoordinatorHttpServer instance;
    private HttpServer server;
    private final int port;
    private volatile boolean running = false;
    
    private final MotionCoordinator coordinator;
    
    // ==================== 생성자 & 싱글톤 ====================
    
    private CoordinatorHttpServer(int port) {
        this.port = port;
        this.coordinator = MotionCoordinator.getInstance();
    }
    
    public static synchronized CoordinatorHttpServer getInstance(int port) {
        if (instance == null || instance.port != port) {
            if (instance != null) {
                instance.stop();
            }
            instance = new CoordinatorHttpServer(port);
        }
        return instance;
    }
    
    public static synchronized CoordinatorHttpServer getInstance() {
        if (instance == null) {
            instance = new CoordinatorHttpServer(8081);  // 기본 포트
        }
        return instance;
    }
    
    // ==================== 서버 시작/종료 ====================
    
    public void start() {
        if (running) {
            LOGGER.warn("CoordinatorHttpServer already running on port {}", port);
            return;
        }
        
        try {
            server = HttpServer.create(new InetSocketAddress("0.0.0.0", port), 0);
            server.setExecutor(Executors.newFixedThreadPool(4));
            
            // 엔드포인트 등록
            server.createContext("/coord/status", new StatusHandler());
            server.createContext("/coord/acquire", new AcquireHandler());
            server.createContext("/coord/release", new ReleaseHandler());
            server.createContext("/coord/heartbeat", new HeartbeatHandler());
            server.createContext("/coord/force_release", new ForceReleaseHandler());
            
            // Health check
            server.createContext("/coord/health", exchange -> {
                sendJsonResponse(exchange, 200, "{\"status\":\"ok\"}");
            });
            
            server.start();
            running = true;
            LOGGER.info("CoordinatorHttpServer started on port {}", port);
            
        } catch (IOException e) {
            LOGGER.error("Failed to start CoordinatorHttpServer on port {}", port, e);
            throw new RuntimeException("Failed to start CoordinatorHttpServer", e);
        }
    }
    
    public void stop() {
        if (server != null) {
            server.stop(1);
            running = false;
            LOGGER.info("CoordinatorHttpServer stopped");
        }
    }
    
    public boolean isRunning() {
        return running;
    }
    
    public int getPort() {
        return port;
    }
    
    // ==================== HTTP 핸들러들 ====================
    
    /**
     * GET /coord/status - Lock 상태 조회
     */
    private class StatusHandler implements HttpHandler {
        @Override
        public void handle(HttpExchange exchange) throws IOException {
            if (!validateMethod(exchange, "GET")) return;
            
            MotionCoordinator.LockStatus status = coordinator.getStatus();
            sendJsonResponse(exchange, 200, status.toJson());
        }
    }
    
    /**
     * POST /coord/acquire - Lock 획득
     * Query: owner (필수), task (선택), duration_ms (선택)
     */
    private class AcquireHandler implements HttpHandler {
        @Override
        public void handle(HttpExchange exchange) throws IOException {
            if (!validateMethod(exchange, "POST", "GET")) return;  // GET도 허용 (테스트 편의)
            
            Map<String, String> params = parseQuery(exchange.getRequestURI());
            
            String owner = params.get("owner");
            if (owner == null || owner.isBlank()) {
                sendJsonResponse(exchange, 400, "{\"success\":false,\"message\":\"Missing 'owner' parameter\"}");
                return;
            }
            
            String task = params.getOrDefault("task", "unknown");
            long durationMs = 0;
            try {
                String durationStr = params.get("duration_ms");
                if (durationStr != null) {
                    durationMs = Long.parseLong(durationStr);
                }
            } catch (NumberFormatException ignored) {}
            
            MotionCoordinator.AcquireResult result = coordinator.acquire(owner, task, durationMs);
            int statusCode = result.success ? 200 : 409;  // 409 Conflict
            sendJsonResponse(exchange, statusCode, result.toJson());
        }
    }
    
    /**
     * POST /coord/release - Lock 해제
     * Query: owner (필수)
     */
    private class ReleaseHandler implements HttpHandler {
        @Override
        public void handle(HttpExchange exchange) throws IOException {
            if (!validateMethod(exchange, "POST", "GET")) return;
            
            Map<String, String> params = parseQuery(exchange.getRequestURI());
            
            String owner = params.get("owner");
            if (owner == null || owner.isBlank()) {
                sendJsonResponse(exchange, 400, "{\"success\":false,\"message\":\"Missing 'owner' parameter\"}");
                return;
            }
            
            MotionCoordinator.ReleaseResult result = coordinator.release(owner);
            int statusCode = result.success ? 200 : 403;  // 403 Forbidden (not owner)
            sendJsonResponse(exchange, statusCode, result.toJson());
        }
    }
    
    /**
     * POST /coord/heartbeat - Heartbeat 갱신
     * Query: owner (필수)
     */
    private class HeartbeatHandler implements HttpHandler {
        @Override
        public void handle(HttpExchange exchange) throws IOException {
            if (!validateMethod(exchange, "POST", "GET")) return;
            
            Map<String, String> params = parseQuery(exchange.getRequestURI());
            
            String owner = params.get("owner");
            if (owner == null || owner.isBlank()) {
                sendJsonResponse(exchange, 400, "{\"success\":false,\"message\":\"Missing 'owner' parameter\"}");
                return;
            }
            
            MotionCoordinator.HeartbeatResult result = coordinator.heartbeat(owner);
            int statusCode = result.success ? 200 : 403;
            sendJsonResponse(exchange, statusCode, result.toJson());
        }
    }
    
    /**
     * POST /coord/force_release - 강제 해제 (관리자/긴급용)
     * Query: reason (선택)
     */
    private class ForceReleaseHandler implements HttpHandler {
        @Override
        public void handle(HttpExchange exchange) throws IOException {
            if (!validateMethod(exchange, "POST", "GET")) return;
            
            Map<String, String> params = parseQuery(exchange.getRequestURI());
            String reason = params.getOrDefault("reason", "manual force release");
            
            coordinator.forceRelease(reason);
            sendJsonResponse(exchange, 200, "{\"success\":true,\"message\":\"Force released\"}");
        }
    }
    
    // ==================== 유틸리티 메서드 ====================
    
    private boolean validateMethod(HttpExchange exchange, String... allowedMethods) throws IOException {
        String method = exchange.getRequestMethod();
        for (String allowed : allowedMethods) {
            if (method.equalsIgnoreCase(allowed)) {
                return true;
            }
        }
        
        exchange.getResponseHeaders().add("Allow", String.join(", ", allowedMethods));
        sendJsonResponse(exchange, 405, 
            "{\"success\":false,\"message\":\"Method not allowed. Use: " + String.join(", ", allowedMethods) + "\"}");
        return false;
    }
    
    private Map<String, String> parseQuery(URI uri) {
        Map<String, String> params = new HashMap<>();
        String query = uri.getQuery();
        if (query == null || query.isEmpty()) {
            return params;
        }
        
        for (String param : query.split("&")) {
            String[] pair = param.split("=", 2);
            if (pair.length == 2) {
                try {
                    String key = URLDecoder.decode(pair[0], StandardCharsets.UTF_8);
                    String value = URLDecoder.decode(pair[1], StandardCharsets.UTF_8);
                    params.put(key, value);
                } catch (Exception e) {
                    LOGGER.warn("Failed to decode query param: {}", param);
                }
            } else if (pair.length == 1) {
                params.put(pair[0], "");
            }
        }
        
        return params;
    }
    
    private void sendJsonResponse(HttpExchange exchange, int statusCode, String json) throws IOException {
        exchange.getResponseHeaders().add("Content-Type", "application/json");
        exchange.getResponseHeaders().add("Access-Control-Allow-Origin", "*");  // CORS
        exchange.getResponseHeaders().add("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        
        byte[] bytes = json.getBytes(StandardCharsets.UTF_8);
        exchange.sendResponseHeaders(statusCode, bytes.length);
        
        try (OutputStream os = exchange.getResponseBody()) {
            os.write(bytes);
        }
    }
}
