package com.kAIS.KAIMyEntity.neoforge.register;

import com.kAIS.KAIMyEntity.renderer.KAIMyEntityRendererPlayerHelper;
import com.kAIS.KAIMyEntity.renderer.MMDModelManager;

// URDF 쪽
import com.kAIS.KAIMyEntity.neoforge.ClientTickLoop;
import com.kAIS.KAIMyEntity.urdf.URDFModelOpenGLWithSTL;
import com.kAIS.KAIMyEntity.urdf.control.MotionEditorScreen;
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

import java.io.File;
import java.util.Objects;

/**
 * 클라이언트 키 등록 & 처리
 *
 * - V/B/N/M: 기존 커스텀 애니메이션
 * - G: RL Control 패널 열기 (없으면 URDF 자동 로드 시도)
 *   - Ctrl + G: URDF 모델 리로드
 * - H: URDF 물리 리셋
 * - T: Webots 통계 출력
 * - Y: Webots T-Pose 테스트
 * - U: Webots 설정 GUI
 *
 * VMD 관련(키, GUI, 로더)은 전부 제거됨.
 */
@EventBusSubscriber(modid = "kaimyentity", bus = EventBusSubscriber.Bus.MOD, value = Dist.CLIENT)
public class KAIMyEntityRegisterClient {
    static final Logger logger = LogManager.getLogger();

    // === 키맵 정의 ===
    static KeyMapping keyCustomAnim1       = new KeyMapping("key.customAnim1",       KeyConflictContext.IN_GAME, KeyModifier.NONE, InputConstants.Type.KEYSYM, GLFW.GLFW_KEY_V, "key.title");
    static KeyMapping keyCustomAnim2       = new KeyMapping("key.customAnim2",       KeyConflictContext.IN_GAME, KeyModifier.NONE, InputConstants.Type.KEYSYM, GLFW.GLFW_KEY_B, "key.title");
    static KeyMapping keyCustomAnim3       = new KeyMapping("key.customAnim3",       KeyConflictContext.IN_GAME, KeyModifier.NONE, InputConstants.Type.KEYSYM, GLFW.GLFW_KEY_N, "key.title");
    static KeyMapping keyCustomAnim4       = new KeyMapping("key.customAnim4",       KeyConflictContext.IN_GAME, KeyModifier.NONE, InputConstants.Type.KEYSYM, GLFW.GLFW_KEY_M, "key.title");
    static KeyMapping keyMotionGuiOrReload = new KeyMapping("key.motionGuiOrReload", KeyConflictContext.IN_GAME, KeyModifier.NONE, InputConstants.Type.KEYSYM, GLFW.GLFW_KEY_G, "key.title");
    static KeyMapping keyResetPhysics      = new KeyMapping("key.resetPhysics",      KeyConflictContext.IN_GAME, KeyModifier.NONE, InputConstants.Type.KEYSYM, GLFW.GLFW_KEY_H, "key.title");
    static KeyMapping keyWebotsStats       = new KeyMapping("key.webotsStats",       KeyConflictContext.IN_GAME, KeyModifier.NONE, InputConstants.Type.KEYSYM, GLFW.GLFW_KEY_T, "key.title");
    static KeyMapping keyWebotsTest        = new KeyMapping("key.webotsTest",        KeyConflictContext.IN_GAME, KeyModifier.NONE, InputConstants.Type.KEYSYM, GLFW.GLFW_KEY_Y, "key.title");
    static KeyMapping keyWebotsConfig      = new KeyMapping("key.webotsConfig",      KeyConflictContext.IN_GAME, KeyModifier.NONE, InputConstants.Type.KEYSYM, GLFW.GLFW_KEY_U, "key.title");

    // === 키맵 등록 (MOD BUS 이벤트) ===
    @SubscribeEvent
    public static void onRegisterKeyMappings(RegisterKeyMappingsEvent e) {
        e.register(keyCustomAnim1);
        e.register(keyCustomAnim2);
        e.register(keyCustomAnim3);
        e.register(keyCustomAnim4);
        e.register(keyMotionGuiOrReload);
        e.register(keyResetPhysics);
        e.register(keyWebotsStats);
        e.register(keyWebotsTest);
        e.register(keyWebotsConfig);
        logger.info("KAIMyEntityRegisterClient: key mappings registered.");
    }

    public static void Register() {
        // 예전 Forge 스타일과의 호환을 위한 더미 메서드
        logger.info("KAIMyEntityRegisterClient.Register() called (no-op, use event-based registration).");
    }
}

// === 키 입력 처리 (NeoForge 버스) ===
@EventBusSubscriber(modid = "kaimyentity", value = Dist.CLIENT)
class KAIMyEntityKeyHandler {
    private static final Logger logger = LogManager.getLogger();

    @OnlyIn(Dist.CLIENT)
    @SubscribeEvent
    public static void onKeyPressed(InputEvent.Key event) {
        Minecraft MC = Minecraft.getInstance();
        LocalPlayer player = MC.player;
        if (player == null) return;

        // ==== 커스텀 애니메이션 ====
        handleCustomAnim(player);

        // ==== G: RL Control 패널 열기 / Ctrl+G: URDF 리로드 ====
        if (KAIMyEntityRegisterClient.keyMotionGuiOrReload.consumeClick()) {
            long win = MC.getWindow().getWindow();
            boolean ctrl = GLFW.glfwGetKey(win, GLFW.GLFW_KEY_LEFT_CONTROL) == GLFW.GLFW_PRESS
                        || GLFW.glfwGetKey(win, GLFW.GLFW_KEY_RIGHT_CONTROL) == GLFW.GLFW_PRESS;

            if (ctrl) {
                // URDF 모델 리로드
                try {
                    MMDModelManager.ReloadModel();
                    MC.gui.getChat().addMessage(Component.literal("[URDF] models reloaded"));
                } catch (Throwable t) {
                    MC.gui.getChat().addMessage(Component.literal("[URDF] reload failed: " + t.getMessage()));
                }
                ensureActiveRenderer(MC);
            } else {
                // RL 패널 열기
                if (ClientTickLoop.renderer == null) ensureActiveRenderer(MC);
                if (ClientTickLoop.renderer != null) {
                    MotionEditorScreen.open(ClientTickLoop.renderer);
                } else {
                    MC.gui.getChat().addMessage(Component.literal("[URDF] No active renderer."));
                }
            }
        }

        // ==== H: 물리 리셋 ====
        if (KAIMyEntityRegisterClient.keyResetPhysics.consumeClick()) {
            var m = MMDModelManager.GetModel("EntityPlayer_" + player.getName().getString());
            if (m != null) {
                KAIMyEntityRendererPlayerHelper.ResetPhysics(player);
                net.neoforged.neoforge.network.PacketDistributor.sendToServer(
                        new com.kAIS.KAIMyEntity.neoforge.network.KAIMyEntityNetworkPack(2, player.getGameProfile(), 0));
                MC.gui.getChat().addMessage(Component.literal("URDF physics reset"));
            }
        }

        // ==== T: Webots 통계 출력 ====
        if (KAIMyEntityRegisterClient.keyWebotsStats.consumeClick()) {
            try {
                WebotsController.getInstance().printStats();
                MC.gui.getChat().addMessage(Component.literal("§a[Webots] Stats printed to console"));
            } catch (Exception e) {
                MC.gui.getChat().addMessage(Component.literal("§c[Webots] Error: " + e.getMessage()));
            }
        }

        // ==== Y: Webots 테스트 자세 ====
        if (KAIMyEntityRegisterClient.keyWebotsTest.consumeClick()) {
            testWebotsConnection(MC);
        }

        // ==== U: Webots 설정 GUI ====
        if (KAIMyEntityRegisterClient.keyWebotsConfig.consumeClick()) {
            MC.setScreen(new WebotsConfigScreen(MC.screen));
        }
    }

    // === 커스텀 애니메이션 처리 ===
    private static void handleCustomAnim(LocalPlayer player) {
        var m = MMDModelManager.GetModel("EntityPlayer_" + player.getName().getString());
        if (m == null) return;
        if (KAIMyEntityRegisterClient.keyCustomAnim1.isDown()) sendAnim(player, 1);
        if (KAIMyEntityRegisterClient.keyCustomAnim2.isDown()) sendAnim(player, 2);
        if (KAIMyEntityRegisterClient.keyCustomAnim3.isDown()) sendAnim(player, 3);
        if (KAIMyEntityRegisterClient.keyCustomAnim4.isDown()) sendAnim(player, 4);
    }

    private static void sendAnim(LocalPlayer player, int index) {
        KAIMyEntityRendererPlayerHelper.CustomAnim(player, String.valueOf(index));
        net.neoforged.neoforge.network.PacketDistributor.sendToServer(
                new com.kAIS.KAIMyEntity.neoforge.network.KAIMyEntityNetworkPack(1, player.getGameProfile(), index));
    }

    // === Webots 연결 테스트 (T-Pose) ===
    private static void testWebotsConnection(Minecraft mc) {
        try {
            var webots = WebotsController.getInstance();

            webots.setJoint("r_sho_pitch", 0.3f);
            webots.setJoint("r_sho_roll", 1.57f);
            webots.setJoint("r_el", -0.1f);

            webots.setJoint("l_sho_pitch", 0.3f);
            webots.setJoint("l_sho_roll", -1.57f);
            webots.setJoint("l_el", -0.1f);

            mc.gui.getChat().addMessage(Component.literal("§a[Webots] T-Pose sent!"));

        } catch (Exception e) {
            mc.gui.getChat().addMessage(Component.literal("§c[Webots] Connection failed: " + e.getMessage()));
            logger.error("Webots test failed", e);
        }
    }

    // === 렌더러 자동 로드 ===
    private static void ensureActiveRenderer(Minecraft mc) {
        if (ClientTickLoop.renderer != null) return;

        File gameDir = mc.gameDirectory;
        File urdf = findFirstUrdf(gameDir, 2);
        if (urdf == null) {
            mc.gui.getChat().addMessage(Component.literal("[URDF] No *.urdf found under ./KAIMyEntity or ./config"));
            return;
        }
        File modelDir = guessModelDir(urdf);
        if (modelDir == null || !modelDir.isDirectory()) {
            mc.gui.getChat().addMessage(Component.literal("[URDF] Guessing modelDir failed"));
            return;
        }
        URDFModelOpenGLWithSTL r = URDFModelOpenGLWithSTL.Create(urdf.getAbsolutePath(), modelDir.getAbsolutePath());
        if (r == null) {
            mc.gui.getChat().addMessage(Component.literal("[URDF] Parse failed: " + urdf.getAbsolutePath()));
            return;
        }
        ClientTickLoop.renderer = r;
        mc.gui.getChat().addMessage(Component.literal("[URDF] Active renderer set: " + urdf.getName()));
    }

    private static File findFirstUrdf(File root, int maxDepth) {
        if (root == null || !root.exists() || maxDepth < 0) return null;
        File[] list = root.listFiles();
        if (list == null) return null;
        for (File f : Objects.requireNonNull(list)) {
            if (f.isFile() && f.getName().toLowerCase().endsWith(".urdf")) return f;
        }
        for (File f : list) {
            if (f.isDirectory()) {
                File r = findFirstUrdf(f, maxDepth - 1);
                if (r != null) return r;
            }
        }
        return null;
    }

    private static File guessModelDir(File urdfFile) {
        File parent = urdfFile.getParentFile();
        if (parent == null) return null;
        File meshes = new File(parent, "meshes");
        if (meshes.exists() && meshes.isDirectory()) return meshes;
        return parent;
    }
}
