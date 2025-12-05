// neoforge/src/main/java/com/kAIS/KAIMyEntity/neoforge/ClientTickLoop.java
package com.kAIS.KAIMyEntity.neoforge;

import com.kAIS.KAIMyEntity.urdf.URDFModelOpenGLWithSTL;
import com.kAIS.KAIMyEntity.urdf.control.MotionEditorScreen;
import com.kAIS.KAIMyEntity.webots.WebotsController; // ✅ 추가

import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.client.event.ClientTickEvent;

import java.util.ArrayList;
import java.util.List;

/**
 * 네오포지용 클라이언트 틱 루프
 * - 매 틱(20Hz)마다 URDF 모델 업데이트
 * - URDFModelOpenGLWithSTL.tickUpdate(dt) 호출
 * - VMC 데이터 처리 (MotionEditorScreen.tick)
 * 
 * ✅ 2025.11.21 Webots 연동 추가
 * - URDF 업데이트 후 자동으로 Webots 전송
 * - 기존 로직 완전 보존
 */
@EventBusSubscriber(
        modid = "kaimyentity",
        value = Dist.CLIENT
)
public final class ClientTickLoop {

    public static URDFModelOpenGLWithSTL renderer;              // 단일 모델
    public static final List<URDFModelOpenGLWithSTL> renderers = new ArrayList<>();
    
    // ✅ Webots 컨트롤러 (지연 초기화)
    private static WebotsController webots;
    private static boolean webotsInitialized = false;
    
    // 통계 출력용 카운터
    private static int tickCount = 0;
    private static final int STATS_INTERVAL = 100; // 5초마다 (100 ticks = 5초)

    @SubscribeEvent
    public static void onClientTick(ClientTickEvent.Post event) {
        float dt = 1.0f / 20.0f;

        // ✅ 기존 로직: URDF 업데이트 + VMC 처리
        if (renderer != null) {
            renderer.tickUpdate(dt);
            MotionEditorScreen.tick(renderer); // ★ VMC 데이터 처리
        }
        
        for (URDFModelOpenGLWithSTL r : renderers) {
            r.tickUpdate(dt);
            MotionEditorScreen.tick(r); // ★ VMC 데이터 처리
        }
        
        // ✅ 추가 로직: Webots 전송 (기존 로직에 영향 없음)
        // 참고: MotionEditorScreen.tick() 내부에서 이미 sendToWebots() 호출됨
        // 여기서는 추가로 직접 제어하는 경우에만 사용
        
        // 통계 출력 (5초마다)
        if (++tickCount >= STATS_INTERVAL) {
            tickCount = 0;
            printWebotsStats();
        }
    }
    
    // ✅ 새로운 메서드: Webots 컨트롤러 초기화 (지연 로딩)
    /**
     * Webots 컨트롤러를 지연 초기화
     * - 첫 호출 시에만 초기화
     * - 초기화 실패 시 null 반환 (에러 없이)
     */
    private static WebotsController getWebots() {
        if (!webotsInitialized) {
            webotsInitialized = true;
            try {
                webots = WebotsController.getInstance();
            } catch (Exception e) {
                // 초기화 실패 시 조용히 무시
                webots = null;
            }
        }
        return webots;
    }
    
    // ✅ 새로운 메서드: Webots 통계 출력
    /**
     * Webots 연결 상태 및 통계 출력
     * - 연결 안 되어 있으면 조용히 스킵
     */
    private static void printWebotsStats() {
        WebotsController wc = getWebots();
        if (wc != null && wc.isConnected()) {
            wc.printStats();
        }
    }
    
    // ✅ 새로운 메서드: 외부에서 Webots IP/Port 재설정
    /**
     * Webots 연결 재설정
     * @param ip Webots 서버 IP
     * @param port Webots 서버 Port
     */
    public static void reconnectWebots(String ip, int port) {
        WebotsController wc = getWebots();
        if (wc != null) {
            wc.reconnect(ip, port);
        } else {
            // 첫 연결
            try {
                webots = WebotsController.getInstance(ip, port);
                webotsInitialized = true;
            } catch (Exception e) {
                // 실패 시 조용히 무시
            }
        }
    }
    
    // ✅ 새로운 메서드: Webots 연결 상태 확인
    /**
     * Webots 연결 상태 확인
     * @return 연결 여부
     */
    public static boolean isWebotsConnected() {
        WebotsController wc = getWebots();
        return wc != null && wc.isConnected();
    }
    
    // ✅ 새로운 메서드: Webots 주소 조회
    /**
     * 현재 Webots 주소 조회
     * @return "IP:Port" 형식 또는 "Not initialized"
     */
    public static String getWebotsAddress() {
        WebotsController wc = getWebots();
        return wc != null ? wc.getRobotAddress() : "Not initialized";
    }
}
