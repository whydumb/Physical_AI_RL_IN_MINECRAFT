package com.kAIS.KAIMyEntity.coordinator;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicReference;

/**
 * MotionCoordinator - Action Lock 관리자
 * 
 * 역할:
 *  - 모션/걷기 명령의 제어권을 관리
 *  - blink/track/camera 같은 상태 명령은 Lock과 무관하게 항상 허용
 *  - heartbeat 타임아웃으로 죽은 클라이언트의 Lock 자동 해제
 * 
 * 사용 시나리오:
 *  - contral (WASD 테스트): 사용자가 수동으로 로봇 테스트
 *  - AI_TEST_2 (Agent): LLM 기반 자동 제어
 *  - External RL: 강화학습 시스템의 장기 태스크
 * 
 * Lock 범위:
 *  - 모션 명령 (?motion=N)
 *  - 걷기 명령 (set_walk, move_forward, etc.)
 *  - 머리 움직임 (set_head) - 선택적
 * 
 * Lock 제외:
 *  - blink_toggle
 *  - track_toggle  
 *  - /camera (읽기 전용)
 *  - /info (읽기 전용)
 */
public class MotionCoordinator {
    private static final Logger LOGGER = LogManager.getLogger();
    
    private static MotionCoordinator instance;
    
    // Lock 정보
    private final AtomicReference<LockInfo> lock = new AtomicReference<>(null);
    
    // 설정
    private static final long HEARTBEAT_TIMEOUT_MS = 30_000;  // 30초
    private static final long DEFAULT_MOTION_TIMEOUT_MS = 5_000;  // 단순 모션 기본 타임아웃
    
    // 콜백 리스너 (UI 업데이트용)
    private final ConcurrentHashMap<String, LockStateListener> listeners = new ConcurrentHashMap<>();
    
    // ==================== Lock 정보 구조체 ====================
    
    public static class LockInfo {
        public final String owner;
        public final long acquiredAt;
        public volatile long lastHeartbeat;
        public final String taskDescription;
        public final long expectedDurationMs;  // 예상 소요 시간 (0이면 무제한)
        
        public LockInfo(String owner, String taskDescription, long expectedDurationMs) {
            this.owner = owner;
            this.acquiredAt = System.currentTimeMillis();
            this.lastHeartbeat = this.acquiredAt;
            this.taskDescription = taskDescription;
            this.expectedDurationMs = expectedDurationMs;
        }
        
        public long getElapsedMs() {
            return System.currentTimeMillis() - acquiredAt;
        }
        
        public long getTimeSinceHeartbeat() {
            return System.currentTimeMillis() - lastHeartbeat;
        }
        
        public boolean isExpired(long timeoutMs) {
            return getTimeSinceHeartbeat() > timeoutMs;
        }
    }
    
    // ==================== 콜백 인터페이스 ====================
    
    public interface LockStateListener {
        void onLockAcquired(String owner, String taskDescription);
        void onLockReleased(String previousOwner);
        void onLockExpired(String owner);
    }
    
    // ==================== 싱글톤 ====================
    
    private MotionCoordinator() {
        LOGGER.info("MotionCoordinator initialized (heartbeat timeout: {}ms)", HEARTBEAT_TIMEOUT_MS);
    }
    
    public static synchronized MotionCoordinator getInstance() {
        if (instance == null) {
            instance = new MotionCoordinator();
        }
        return instance;
    }
    
    // ==================== 핵심 API ====================
    
    /**
     * 현재 Lock 상태 조회
     */
    public LockStatus getStatus() {
        checkAndExpire();
        LockInfo current = lock.get();
        if (current == null) {
            return new LockStatus(false, null, null, 0, 0);
        }
        return new LockStatus(
            true, 
            current.owner, 
            current.taskDescription,
            current.getElapsedMs(),
            current.expectedDurationMs
        );
    }
    
    /**
     * Lock 획득 시도
     * 
     * @param ownerId 요청자 식별자 (예: "contral", "agent", "external_rl")
     * @param taskDescription 태스크 설명 (UI 표시용)
     * @param expectedDurationMs 예상 소요 시간 (0이면 무제한)
     * @return 획득 결과
     */
    public AcquireResult acquire(String ownerId, String taskDescription, long expectedDurationMs) {
        if (ownerId == null || ownerId.isBlank()) {
            return new AcquireResult(false, "Owner ID cannot be empty", null);
        }
        
        checkAndExpire();
        
        LockInfo current = lock.get();
        
        // 이미 같은 owner가 가지고 있으면 갱신
        if (current != null && current.owner.equals(ownerId)) {
            current.lastHeartbeat = System.currentTimeMillis();
            LOGGER.debug("Lock renewed for {}", ownerId);
            return new AcquireResult(true, "renewed", current.owner);
        }
        
        // 다른 owner가 가지고 있으면 실패
        if (current != null) {
            LOGGER.debug("Lock acquire failed - {} owns it", current.owner);
            return new AcquireResult(false, "Lock held by " + current.owner, current.owner);
        }
        
        // 새로 획득
        LockInfo newLock = new LockInfo(ownerId, taskDescription, expectedDurationMs);
        if (lock.compareAndSet(null, newLock)) {
            LOGGER.info("Lock acquired: {} (task: {})", ownerId, taskDescription);
            notifyLockAcquired(ownerId, taskDescription);
            return new AcquireResult(true, "acquired", ownerId);
        }
        
        // CAS 실패 (동시 요청으로 다른 쪽이 먼저 획득)
        current = lock.get();
        return new AcquireResult(false, "Race condition - " + (current != null ? current.owner : "unknown") + " acquired first", 
                                 current != null ? current.owner : null);
    }
    
    /**
     * Lock 해제
     */
    public ReleaseResult release(String ownerId) {
        if (ownerId == null || ownerId.isBlank()) {
            return new ReleaseResult(false, "Owner ID cannot be empty");
        }
        
        LockInfo current = lock.get();
        
        if (current == null) {
            return new ReleaseResult(true, "already_free");
        }
        
        if (!current.owner.equals(ownerId)) {
            return new ReleaseResult(false, "Not owner. Current owner: " + current.owner);
        }
        
        if (lock.compareAndSet(current, null)) {
            LOGGER.info("Lock released: {}", ownerId);
            notifyLockReleased(ownerId);
            return new ReleaseResult(true, "released");
        }
        
        return new ReleaseResult(false, "Release failed (concurrent modification)");
    }
    
    /**
     * Heartbeat 갱신
     */
    public HeartbeatResult heartbeat(String ownerId) {
        if (ownerId == null || ownerId.isBlank()) {
            return new HeartbeatResult(false, "Owner ID cannot be empty");
        }
        
        LockInfo current = lock.get();
        
        if (current == null) {
            return new HeartbeatResult(false, "No lock held");
        }
        
        if (!current.owner.equals(ownerId)) {
            return new HeartbeatResult(false, "Not owner. Current owner: " + current.owner);
        }
        
        current.lastHeartbeat = System.currentTimeMillis();
        LOGGER.debug("Heartbeat from {}", ownerId);
        return new HeartbeatResult(true, "ok");
    }
    
    /**
     * 특정 owner가 명령을 실행할 수 있는지 빠른 체크
     * (Lock이 없거나, 본인이 Lock을 가지고 있으면 true)
     */
    public boolean canExecute(String ownerId) {
        checkAndExpire();
        LockInfo current = lock.get();
        return current == null || current.owner.equals(ownerId);
    }
    
    /**
     * Lock 없이 누구나 실행 가능한지 체크
     */
    public boolean isFree() {
        checkAndExpire();
        return lock.get() == null;
    }
    
    /**
     * 강제 해제 (관리자/긴급 상황용)
     */
    public void forceRelease(String reason) {
        LockInfo current = lock.get();
        if (current != null) {
            LOGGER.warn("Force releasing lock from {} (reason: {})", current.owner, reason);
            lock.set(null);
            notifyLockReleased(current.owner);
        }
    }
    
    // ==================== 내부 메서드 ====================
    
    private void checkAndExpire() {
        LockInfo current = lock.get();
        if (current != null && current.isExpired(HEARTBEAT_TIMEOUT_MS)) {
            if (lock.compareAndSet(current, null)) {
                LOGGER.warn("Lock expired for {} (no heartbeat for {}ms)", 
                           current.owner, current.getTimeSinceHeartbeat());
                notifyLockExpired(current.owner);
            }
        }
    }
    
    // ==================== 리스너 관리 ====================
    
    public void addListener(String id, LockStateListener listener) {
        listeners.put(id, listener);
    }
    
    public void removeListener(String id) {
        listeners.remove(id);
    }
    
    private void notifyLockAcquired(String owner, String taskDescription) {
        listeners.values().forEach(l -> {
            try {
                l.onLockAcquired(owner, taskDescription);
            } catch (Exception e) {
                LOGGER.error("Listener error on lockAcquired", e);
            }
        });
    }
    
    private void notifyLockReleased(String previousOwner) {
        listeners.values().forEach(l -> {
            try {
                l.onLockReleased(previousOwner);
            } catch (Exception e) {
                LOGGER.error("Listener error on lockReleased", e);
            }
        });
    }
    
    private void notifyLockExpired(String owner) {
        listeners.values().forEach(l -> {
            try {
                l.onLockExpired(owner);
            } catch (Exception e) {
                LOGGER.error("Listener error on lockExpired", e);
            }
        });
    }
    
    // ==================== 결과 클래스들 ====================
    
    public static class LockStatus {
        public final boolean locked;
        public final String owner;
        public final String taskDescription;
        public final long elapsedMs;
        public final long expectedDurationMs;
        
        public LockStatus(boolean locked, String owner, String taskDescription, 
                         long elapsedMs, long expectedDurationMs) {
            this.locked = locked;
            this.owner = owner;
            this.taskDescription = taskDescription;
            this.elapsedMs = elapsedMs;
            this.expectedDurationMs = expectedDurationMs;
        }
        
        public String toJson() {
            if (!locked) {
                return "{\"locked\":false,\"owner\":null}";
            }
            return String.format(
                "{\"locked\":true,\"owner\":\"%s\",\"task\":\"%s\",\"elapsed_ms\":%d,\"expected_ms\":%d}",
                owner, 
                taskDescription != null ? taskDescription.replace("\"", "\\\"") : "",
                elapsedMs,
                expectedDurationMs
            );
        }
    }
    
    public static class AcquireResult {
        public final boolean success;
        public final String message;
        public final String currentOwner;
        
        public AcquireResult(boolean success, String message, String currentOwner) {
            this.success = success;
            this.message = message;
            this.currentOwner = currentOwner;
        }
        
        public String toJson() {
            return String.format(
                "{\"success\":%b,\"message\":\"%s\",\"owner\":%s}",
                success,
                message != null ? message.replace("\"", "\\\"") : "",
                currentOwner != null ? "\"" + currentOwner + "\"" : "null"
            );
        }
    }
    
    public static class ReleaseResult {
        public final boolean success;
        public final String message;
        
        public ReleaseResult(boolean success, String message) {
            this.success = success;
            this.message = message;
        }
        
        public String toJson() {
            return String.format(
                "{\"success\":%b,\"message\":\"%s\"}",
                success,
                message != null ? message.replace("\"", "\\\"") : ""
            );
        }
    }
    
    public static class HeartbeatResult {
        public final boolean success;
        public final String message;
        
        public HeartbeatResult(boolean success, String message) {
            this.success = success;
            this.message = message;
        }
        
        public String toJson() {
            return String.format(
                "{\"success\":%b,\"message\":\"%s\"}",
                success,
                message != null ? message.replace("\"", "\\\"") : ""
            );
        }
    }
}
