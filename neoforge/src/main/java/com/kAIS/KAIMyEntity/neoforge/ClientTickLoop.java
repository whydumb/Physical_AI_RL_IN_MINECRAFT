// neoforge/src/main/java/com/kAIS/KAIMyEntity/neoforge/ClientTickLoop.java
package com.kAIS.KAIMyEntity.neoforge;

import com.kAIS.KAIMyEntity.urdf.URDFModelOpenGLWithSTL;

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
 */
@EventBusSubscriber( // bus 파라미터 제거
        modid = "kaimyentity",
        value = Dist.CLIENT
)
public final class ClientTickLoop {

    public static URDFModelOpenGLWithSTL renderer;              // 단일 모델
    public static final List<URDFModelOpenGLWithSTL> renderers = new ArrayList<>();

    @SubscribeEvent
    public static void onClientTick(ClientTickEvent.Post event) {
        float dt = 1.0f / 20.0f;

        if (renderer != null) {
            renderer.tickUpdate(dt);
        }
        for (URDFModelOpenGLWithSTL r : renderers) {
            r.tickUpdate(dt);
        }
    }
}
