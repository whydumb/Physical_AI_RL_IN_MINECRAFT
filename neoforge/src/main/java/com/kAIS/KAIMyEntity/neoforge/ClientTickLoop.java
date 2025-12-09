package com.kAIS.KAIMyEntity.neoforge;

import com.kAIS.KAIMyEntity.rl.RLEnvironmentCore;
import com.kAIS.KAIMyEntity.urdf.URDFModelOpenGLWithSTL;
import com.kAIS.KAIMyEntity.urdf.control.MotionEditorScreen;
import com.kAIS.KAIMyEntity.webots.WebotsController;

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
 *
 * ✅ Webots 연동
 *  - URDF 업데이트 후 Webots 전송/통계
 *
 * ✅ RL 연동
 *  - RLEnvironmentCore.tick(dt) 호출
 *  - RL 초기화/참조 모션/학습 시작은
 *    - MotionEditorScreen.RLControlGUI 에서 버튼으로
 *    - 또는 RLVMDIntegration.setupVmdImitation(...) 등에서
 *    한 번만 수행해 둔다.
 */
@EventBusSubscriber(
        modid = "kaimyentity",
        value = Dist.CLIENT
)
public final class ClientTickLoop {

    /** 기본(플레이어)용 URDF 렌더러 하나 */
    public static URDFModelOpenGLWithSTL renderer;

    /** 여러 엔티티용 렌더러 리스트 (필요시 사용) */
    public static final List<URDFModelOpenGLWithSTL> renderers = new ArrayList<>();

    // Webots 컨트롤러 (지연 초기화)
    private static WebotsController webots;
    private static boolean webotsInitialized = false;

    // 통계 출력용 카운터
    private static int tickCount = 0;
    private static final int STATS_INTERVAL = 100; // 5초마다 (100 ticks = 5초)

    @SubscribeEvent
    public static void onClientTick(ClientTickEvent.Post event) {
        // Minecraft 틱은 20Hz 기준
        float dt = 1.0f / 20.0f;

        // 1) URDF 물리/그래픽 업데이트
        if (renderer != null) {
            renderer.tickUpdate(dt);
        }
        for (URDFModelOpenGLWithSTL r : renderers) {
            r.tickUpdate(dt);
        }

        // 2) VMD / RL 간 충돌 제어 + VMD 재생
        //    (MotionEditorScreen.tick 안에서
        //     "RL이 관절을 제어 중이면 VMD가 setJointTarget을 안 하도록" 분기)
        if (renderer != null) {
            MotionEditorScreen.tick(renderer);
        }

        // 3) RL 환경 한 스텝 진행
        //    - RL이 초기화/학습 중일 때만 내부에서 동작
        RLEnvironmentCore env = RLEnvironmentCore.getInstance();
        env.tick(dt);

        // 4) Webots 통계 출력 (5초마다)
        if (++tickCount >= STATS_INTERVAL) {
            tickCount = 0;
            printWebotsStats();
        }
    }

    // Webots 컨트롤러 초기화 (지연 로딩)
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

    // Webots 통계 출력
    private static void printWebotsStats() {
        WebotsController wc = getWebots();
        if (wc != null && wc.isConnected()) {
            wc.printStats();
        }
    }

    // 외부에서 Webots IP/Port 재설정
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

    // Webots 연결 상태 확인
    public static boolean isWebotsConnected() {
        WebotsController wc = getWebots();
        return wc != null && wc.isConnected();
    }

    // 현재 Webots 주소 조회
    public static String getWebotsAddress() {
        WebotsController wc = getWebots();
        return wc != null ? wc.getRobotAddress() : "Not initialized";
    }
}
