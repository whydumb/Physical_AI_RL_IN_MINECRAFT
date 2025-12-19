package com.kAIS.KAIMyEntity.neoforge;

import com.kAIS.KAIMyEntity.urdf.URDFModelOpenGLWithSTL;
import com.kAIS.KAIMyEntity.urdf.control.PosePipeline;
import net.neoforged.api.distmarker.Dist;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.fml.common.EventBusSubscriber;
import net.neoforged.neoforge.client.event.ClientTickEvent;

import java.util.ArrayList;
import java.util.List;

/**
 * 클라이언트 전용 틱 루프
 * - "한 곳에서만" tick을 받아 PosePipeline에 위임
 * - Webots 관련 외부 호출 호환 메서드 제공
 */
@EventBusSubscriber(modid = "kaimyentity", value = Dist.CLIENT)
public final class ClientTickLoop {

    private ClientTickLoop() {}

    /** 기본(단일) 렌더러/모델 (레거시 호환) */
    public static URDFModelOpenGLWithSTL renderer;

    /** 여러 엔티티용 (필요 시) */
    public static final List<URDFModelOpenGLWithSTL> renderers = new ArrayList<>();

    // tick 통계(원하면 로그/디버그에서 사용)
    private static int tickCount = 0;
    private static final int STATS_INTERVAL = 100;

    @SubscribeEvent
    public static void onClientTick(ClientTickEvent.Post event) {
        // MC 클라 tick은 보통 20tps 기준으로 움직이므로 고정 dt 유지(네 기존 코드 유지)
        final float dt = 1.0f / 20.0f;

        // 핵심: 모든 업데이트는 PosePipeline로 위임
        PosePipeline.getInstance().onClientTick(dt, renderer, renderers);

        // (옵션) 주기적으로 Webots/파이프라인 통계 출력하고 싶으면 PosePipeline에 맡기거나 여기서 트리거
        if (++tickCount >= STATS_INTERVAL) {
            tickCount = 0;
            // PosePipeline 쪽에 stats 메서드가 있다면 여기서 호출하도록 연결 가능
            // PosePipeline.getInstance().printStats();
        }
    }

    // ==========================
    // Webots 관련 기존 외부 호출 호환
    // ==========================

    public static void reconnectWebots(String ip, int port) {
        PosePipeline.getInstance().reconnectWebots(ip, port);
    }

    public static boolean isWebotsConnected() {
        return PosePipeline.getInstance().isWebotsConnected();
    }

    public static String getWebotsAddress() {
        return PosePipeline.getInstance().getWebotsAddress();
    }
}
