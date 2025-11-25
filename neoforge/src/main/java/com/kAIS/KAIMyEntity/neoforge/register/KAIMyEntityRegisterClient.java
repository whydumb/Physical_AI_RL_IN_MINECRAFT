package com.kAIS.KAIMyEntity.neoforge.register;

import com.kAIS.KAIMyEntity.webots.WebotsController;
import com.kAIS.KAIMyEntity.webots.WebotsConfigScreen;

import com.mojang.blaze3d.platform.InputConstants;
import net.minecraft.client.Minecraft;
import net.minecraft.client.player.LocalPlayer;
import net.minecraft.client.KeyMapping;
import net.minecraft.network.chat.Component;

import net.neoforged.api.distmarker.Dist;
import net.neoforged.api.distmarker.OnlyIn;
import net.neoforged.bus.api.SubscribeEvent;
import net.neoforged.neoforge.client.settings.KeyConflictContext;
import net.neoforged.neoforge.client.settings.KeyModifier;
import net.neoforged.neoforge.client.event.InputEvent;
import net.neoforged.neoforge.client.event.RegisterKeyMappingsEvent;
import net.neoforged.fml.common.EventBusSubscriber;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.lwjgl.glfw.GLFW;

/**
 * 키 등록 & 처리
 * - T: Webots 통계 출력
 * - Y: Webots T-Pose 테스트
 * - U: Webots 설정 GUI
 */
@EventBusSubscriber(value = Dist.CLIENT)
public class KAIMyEntityRegisterClient {
    static final Logger logger = LogManager.getLogger();

    static KeyMapping keyWebotsStats  = new KeyMapping("key.webotsStats",  KeyConflictContext.IN_GAME, KeyModifier.NONE, InputConstants.Type.KEYSYM, GLFW.GLFW_KEY_T, "key.title");
    static KeyMapping keyWebotsTest   = new KeyMapping("key.webotsTest",   KeyConflictContext.IN_GAME, KeyModifier.NONE, InputConstants.Type.KEYSYM, GLFW.GLFW_KEY_Y, "key.title");
    static KeyMapping keyWebotsConfig = new KeyMapping("key.webotsConfig", KeyConflictContext.IN_GAME, KeyModifier.NONE, InputConstants.Type.KEYSYM, GLFW.GLFW_KEY_U, "key.title");

    @SubscribeEvent
    public static void onRegisterKeyMappings(RegisterKeyMappingsEvent e) {
        e.register(keyWebotsStats);
        e.register(keyWebotsTest);
        e.register(keyWebotsConfig);
        logger.info("KAIMyEntityRegisterClient: key mappings registered.");
    }

    public static void Register() {
        logger.info("KAIMyEntityRegisterClient.Register() called.");
    }

    @OnlyIn(Dist.CLIENT)
    @SubscribeEvent
    public static void onKeyPressed(InputEvent.Key event) {
        Minecraft mc = Minecraft.getInstance();
        LocalPlayer player = mc.player;
        if (player == null) return;

        // T: Webots 통계 출력
        if (keyWebotsStats.consumeClick()) {
            try {
                WebotsController.getInstance().printStats();
                mc.gui.getChat().addMessage(Component.literal("§a[Webots] Stats printed to console"));
            } catch (Exception e) {
                mc.gui.getChat().addMessage(Component.literal("§c[Webots] Error: " + e.getMessage()));
            }
        }

        // Y: Webots T-Pose 테스트
        if (keyWebotsTest.consumeClick()) {
            testWebotsConnection(mc);
        }

        // U: Webots 설정 GUI
        if (keyWebotsConfig.consumeClick()) {
            mc.setScreen(new WebotsConfigScreen(mc.screen));
        }
    }

    /**
     * Webots T-Pose 테스트
     */
    private static void testWebotsConnection(Minecraft mc) {
        try {
            var webots = WebotsController.getInstance();

            // T-Pose 자세 전송
            webots.setJoint("r_sho_pitch", 0.3f);
            webots.setJoint("r_sho_roll", 1.57f);
            webots.setJoint("r_el", -0.1f);

            webots.setJoint("l_sho_pitch", 0.3f);
            webots.setJoint("l_sho_roll", -1.57f);
            webots.setJoint("l_el", -0.1f);

            mc.gui.getChat().addMessage(Component.literal("§a[Webots] T-Pose sent! Check Webots simulation."));

        } catch (Exception e) {
            mc.gui.getChat().addMessage(Component.literal("§c[Webots] Connection failed: " + e.getMessage()));
            logger.error("Webots test failed", e);
        }
    }
}
