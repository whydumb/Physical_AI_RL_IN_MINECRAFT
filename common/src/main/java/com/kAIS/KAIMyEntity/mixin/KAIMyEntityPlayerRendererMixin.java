package com.kAIS.KAIMyEntity.mixin;

import com.kAIS.KAIMyEntity.renderer.IMMDModel;
import com.kAIS.KAIMyEntity.renderer.MMDModelManager;
import com.kAIS.KAIMyEntity.urdf.URDFModelOpenGLWithSTL;
import com.mojang.blaze3d.vertex.PoseStack;
import com.mojang.blaze3d.vertex.VertexConsumer;

import net.minecraft.client.model.PlayerModel;
import net.minecraft.client.player.AbstractClientPlayer;
import net.minecraft.client.renderer.MultiBufferSource;
import net.minecraft.client.renderer.RenderType;
import net.minecraft.client.renderer.entity.EntityRendererProvider;
import net.minecraft.client.renderer.entity.LivingEntityRenderer;
import net.minecraft.client.renderer.entity.player.PlayerRenderer;
import net.minecraft.client.renderer.texture.OverlayTexture;
import net.minecraft.resources.ResourceLocation;

import org.joml.Quaternionf;
import org.joml.Vector3f;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.lang.reflect.Field;

@Mixin(PlayerRenderer.class)
public abstract class KAIMyEntityPlayerRendererMixin
        extends LivingEntityRenderer<AbstractClientPlayer, PlayerModel<AbstractClientPlayer>> {

    private static final Logger logger = LogManager.getLogger();
    private static int renderCallCount = 0;

    // ======== 업라이트/방향 스위치 ========
    /** URDF는 렌더러에서만 보정하므로 믹스인에서는 false (중복 방지) */
    private static final boolean APPLY_URDF_UPRIGHT_IN_MIXIN = false;
    /** URDF 전진축: -Z(기본) / +Z(반전) */
    private static final boolean FORWARD_NEG_Z_URDF = true;
    /** URDF가 거꾸로 보이면 true로 (Z축 180도) */
    private static final boolean ROLL_180_Z_URDF = false;

    /** MMD 경로 보정 */
    private static final boolean FORWARD_NEG_Z = true;
    private static final boolean ROLL_180_Z = false;

    private static final float HALF_PI = (float)(Math.PI / 2.0);
    private static final float PI      = (float)(Math.PI);

    public KAIMyEntityPlayerRendererMixin(EntityRendererProvider.Context ctx,
                                          PlayerModel<AbstractClientPlayer> model,
                                          float shadowRadius) {
        super(ctx, model, shadowRadius);
    }

    @Inject(method = "render", at = @At("HEAD"), cancellable = true)
    public void render(AbstractClientPlayer player, float entityYaw, float tickDelta,
                       PoseStack pose, MultiBufferSource buffers, int packedLight, CallbackInfo ci) {

        // 1) 모델 획득
        URDFModelOpenGLWithSTL urdfFromTickLoop = tryGetClientTickLoopRenderer();
        URDFModelOpenGLWithSTL urdfFromManager = null;
        IMMDModel generic = null;

        String playerName = player.getName().getString();
        MMDModelManager.Model m = MMDModelManager.GetModel("EntityPlayer_" + playerName);
        if (m == null) m = MMDModelManager.GetModel("EntityPlayer");
        if (m != null) {
            generic = m.model;
            if (generic instanceof URDFModelOpenGLWithSTL) {
                urdfFromManager = (URDFModelOpenGLWithSTL) generic;
            }
        }

        URDFModelOpenGLWithSTL urdf = (urdfFromTickLoop != null) ? urdfFromTickLoop : urdfFromManager;

        if (urdf == null && generic == null) return;

        // 2) 로그
        renderCallCount++;
        if (renderCallCount % 60 == 0) {
            if (urdf != null) {
                logger.debug("[URDF] using instance#{} (source: {}) uprightInMixin={}",
                        System.identityHashCode(urdf),
                        (urdf == urdfFromTickLoop) ? "ClientTickLoop" : "Manager",
                        APPLY_URDF_UPRIGHT_IN_MIXIN);
            } else {
                logger.debug("[MMD] render generic model");
            }
        }

        // 3) 텍스처
        ResourceLocation whiteTexture = ResourceLocation.parse("minecraft:textures/misc/white.png");
        ResourceLocation tex = (generic != null ? generic.getTexture() : null);
        RenderType renderType = (tex != null)
                ? RenderType.entitySolid(tex)
                : RenderType.entitySolid(whiteTexture);
        VertexConsumer vertexConsumer = buffers.getBuffer(renderType);

        pose.pushPose();

        // 위치 보정
        pose.translate(0.0f, 0.0f, 0.0f);

        // 좌표계 보정
        if (urdf == null) {
            Quaternionf q = new Quaternionf()
                    .rotateX(-HALF_PI)
                    .rotateY(FORWARD_NEG_Z ? +HALF_PI : -HALF_PI);
            if (ROLL_180_Z) q.rotateZ(PI);
            pose.mulPose(q);
        } else if (APPLY_URDF_UPRIGHT_IN_MIXIN) {
            Quaternionf q = new Quaternionf()
                    .rotateX(-HALF_PI)
                    .rotateY(FORWARD_NEG_Z_URDF ? +HALF_PI : -HALF_PI);
            if (ROLL_180_Z_URDF) q.rotateZ(PI);
            pose.mulPose(q);
        }

        // 스케일 (MMD만 적용)
        if (urdf == null && m != null && m.properties != null && m.properties.containsKey("modelScale")) {
            try {
                float modelScale = Float.parseFloat(m.properties.getProperty("modelScale"));
                pose.scale(modelScale, modelScale, modelScale);
            } catch (NumberFormatException ignored) { }
        }

        // 조명
        int blockLight = (packedLight & 0xFFFF);
        int skyLight   = (packedLight >> 16) & 0xFFFF;
        blockLight = Math.max(blockLight, 0xF0);
        skyLight  = Math.max(skyLight,  0xF0);
        int adjustedLight = (skyLight << 16) | blockLight;

        // 4) 실제 렌더링
        if (urdf != null) {
            urdf.tickUpdate(1.0f / 20.0f);
            urdf.Render(
                player,
                entityYaw,
                player.getXRot(),
                new Vector3f(0f, 0f, 0f),
                tickDelta,
                pose,
                adjustedLight
            );
        } else {
            generic.renderToBuffer(
                player,
                entityYaw,
                player.getXRot(),
                new Vector3f(0f, 0f, 0f),
                tickDelta,
                pose,
                vertexConsumer,
                adjustedLight,
                OverlayTexture.NO_OVERLAY
            );
        }

        pose.popPose();

        ci.cancel();
    }

    /** 런타임에 ClientTickLoop.renderer 조회 */
    private static URDFModelOpenGLWithSTL tryGetClientTickLoopRenderer() {
        try {
            Class<?> cls = Class.forName("com.kAIS.KAIMyEntity.neoforge.ClientTickLoop");
            Field f = cls.getField("renderer");
            Object o = f.get(null);
            if (o instanceof URDFModelOpenGLWithSTL) {
                return (URDFModelOpenGLWithSTL) o;
            }
        } catch (Throwable ignored) { }
        return null;
    }
}
