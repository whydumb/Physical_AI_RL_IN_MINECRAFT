package com.kAIS.KAIMyEntity.renderer;

import net.minecraft.world.entity.player.Player;

public class KAIMyEntityRendererPlayerHelper {

    KAIMyEntityRendererPlayerHelper() {
    }

    public static void ResetPhysics(Player player) {
        MMDModelManager.Model m = MMDModelManager.GetModel("EntityPlayer_" + player.getName().getString());
        if (m == null)
            m = MMDModelManager.GetModel("EntityPlayer");
        
        if (m != null && m.model != null) {
            m.model.ResetPhysics();
        }
    }

    public static void CustomAnim(Player player, String id) {
        // URDF는 애니메이션 없음
    }
}