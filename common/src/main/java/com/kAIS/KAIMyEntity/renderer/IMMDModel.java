package com.kAIS.KAIMyEntity.renderer;

import com.mojang.blaze3d.vertex.PoseStack;
import com.mojang.blaze3d.vertex.VertexConsumer;
import net.minecraft.resources.ResourceLocation;
import net.minecraft.world.entity.Entity;
import org.joml.Vector3f;

/**
 * MC 표준 파이프라인 지원을 위해 VertexConsumer 기반 메서드를 추가.
 * 기존 Render(...)는 하위호환을 위해 유지하고, renderToBuffer의 기본 구현에서 호출하도록 함.
 */
public interface IMMDModel {
    /** 레거시 경로(이전 코드 호환). 가능하면 새 renderToBuffer를 구현해서 사용하세요. */
    void Render(Entity entityIn, float entityYaw, float entityPitch,
                Vector3f entityTrans, float tickDelta, PoseStack mat, int packedLight);

    void ChangeAnim(long anim, long layer);
    void ResetPhysics();
    long GetModelLong();
    String GetModelDir();

    /** ✅ 새 경로: VertexConsumer로 버텍스를 기록해서 MC 렌더 파이프라인을 사용 */
    default void renderToBuffer(Entity entityIn,
                                float entityYaw, float entityPitch, Vector3f entityTrans, float tickDelta,
                                PoseStack pose,
                                VertexConsumer consumer,
                                int packedLight,
                                int overlay) {
        // 하위호환: 새 메서드를 아직 구현하지 않았다면 레거시 렌더를 호출
        Render(entityIn, entityYaw, entityPitch, entityTrans, tickDelta, pose, packedLight);
    }

    /** 선택: 텍스처가 있으면 반환(없으면 null). */
    default ResourceLocation getTexture() { return null; }
}
