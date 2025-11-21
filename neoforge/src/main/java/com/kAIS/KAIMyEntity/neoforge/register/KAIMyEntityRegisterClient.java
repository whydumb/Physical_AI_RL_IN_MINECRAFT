package com.kAIS.KAIMyEntity.neoforge.register;

import com.kAIS.KAIMyEntity.renderer.KAIMyEntityRendererPlayerHelper;
import com.kAIS.KAIMyEntity.renderer.MMDModelManager;

// URDF 쪽
import com.kAIS.KAIMyEntity.neoforge.ClientTickLoop;
import com.kAIS.KAIMyEntity.urdf.URDFModelOpenGLWithSTL;
import com.kAIS.KAIMyEntity.urdf.control.MotionEditorScreen;
import com.kAIS.KAIMyEntity.urdf.control.VMCListenerController;
import com.kAIS.KAIMyEntity.webots.WebotsController; // ✅ 추가

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
 * - V/B/N/M: 기존 커스텀 애니메이션 (유지)
 * - G: 조인트 에디터 열기 (없으면 자동 로드 시도)
 * - Ctrl+G: URDF 리로드 + 자동 로드 시도
 * - H: 물리 리셋
 * - K: VMC 매핑 에디터 열기
 * - T: Webots 통계 출력 (디버깅용) ✅ 추가
 * - Y: Webots T-Pose 테스트 ✅ 추가
 */
@EventBusSubscriber(value = Dist.CLIENT)
public class KAIMyEntityRegisterClient {
    static final Logger logger = LogManager.getLogger();

    // === 키맵 ===
    static KeyMapping keyCustomAnim1 = new KeyMapping("key.customAnim1", KeyConflictContext.IN_GAME, KeyModifier.NONE, InputConstants.Type.KEYSYM, GLFW.GLFW_KEY_V, "key.title");
    static KeyMapping keyCustomAnim2 = new KeyMapping("key.customAnim2", KeyConflictContext.IN_GAME, KeyModifier.NONE, InputConstants.Type.KEYSYM, GLFW.GLFW_KEY_B, "key.title");
    static KeyMapping keyCustomAnim3 = new KeyMapping("key.customAnim3", KeyConflictContext.IN_GAME, KeyModifier.NONE, InputConstants.Type.KEYSYM, GLFW.GLFW_KEY_N, "key.title");
    static KeyMapping keyCustomAnim4 = new KeyMapping("key.customAnim4", KeyConflictContext.IN_GAME, KeyModifier.NONE, InputConstants.Type.KEYSYM, GLFW.GLFW_KEY_M, "key.title");
    static KeyMapping keyMotionGuiOrReload = new KeyMapping("key.motionGuiOrReload", KeyConflictContext.IN_GAME, KeyModifier.NONE, InputConstants.Type.KEYSYM, GLFW.GLFW_KEY_G, "key.title");
    static KeyMapping keyOpenVmcMapping    = new KeyMapping("key.openVmcMapping",  KeyConflictContext.IN_GAME, KeyModifier.NONE, InputConstants.Type.KEYSYM, GLFW.GLFW_KEY_K, "key.title");
    static KeyMapping keyResetPhysics      = new KeyMapping("key.resetPhysics",    KeyConflictContext.IN_GAME, KeyModifier.NONE, InputConstants.Type.KEYSYM, GLFW.GLFW_KEY_H, "key.title");
    static KeyMapping keyWebotsStats       = new KeyMapping("key.webotsStats",     KeyConflictContext.IN_GAME, KeyModifier.NONE, InputConstants.Type.KEYSYM, GLFW.GLFW_KEY_T, "key.title"); // ✅ 추가
    static KeyMapping keyWebotsTest        = new KeyMapping("key.webotsTest",      KeyConflictContext.IN_GAME, KeyModifier.NONE, InputConstants.Type.KEYSYM, GLFW.GLFW_KEY_Y, "key.title"); // ✅ 추가

    // === 키 등록 ===
    @SubscribeEvent
    public static void onRegisterKeyMappings(RegisterKeyMappingsEvent e) {
        e.register(keyCustomAnim1);
        e.register(keyCustomAnim2);
        e.register(keyCustomAnim3);
        e.register(keyCustomAnim4);
        e.register(keyMotionGuiOrReload);
        e.register(keyOpenVmcMapping);
        e.register(keyResetPhysics);
        e.register(keyWebotsStats);  // ✅ 추가
        e.register(keyWebotsTest);   // ✅ 추가
        logger.info("KAIMyEntityRegisterClient: key mappings registered.");
    }

    // === 하위호환용 Register() (초기화 호출 호환) ===
    public static void Register() {
        logger.info("KAIMyEntityRegisterClient.Register() called (no-op, use event-based registration).");
    }

    @OnlyIn(Dist.CLIENT)
    @SubscribeEvent
    public static void onKeyPressed(InputEvent.Key event) {
        Minecraft MC = Minecraft.getInstance();
        LocalPlayer player = MC.player;
        if (player == null) return;

        // ==== 커스텀 애니메이션 유지 ====
        handleCustomAnim(player);

        // ==== G: 조인트 에디터 / Ctrl+G: 리로드 ====
        if (keyMotionGuiOrReload.consumeClick()) {
            long win = MC.getWindow().getWindow();
            boolean ctrl = org.lwjgl.glfw.GLFW.glfwGetKey(win, GLFW.GLFW_KEY_LEFT_CONTROL)  == GLFW.GLFW_PRESS
                    || org.lwjgl.glfw.GLFW.glfwGetKey(win, GLFW.GLFW_KEY_RIGHT_CONTROL) == GLFW.GLFW_PRESS;

            if (ctrl) {
                // Ctrl+G → 리로드
                try {
                    MMDModelManager.ReloadModel();
                    MC.gui.getChat().addMessage(Component.literal("[URDF] models reloaded"));
                } catch (Throwable t) {
                    MC.gui.getChat().addMessage(Component.literal("[URDF] reload failed: " + t.getMessage()));
                }
                ensureActiveRenderer(MC);
            } else {
                // G → 조인트 에디터 열기
                if (ClientTickLoop.renderer == null) ensureActiveRenderer(MC);
                if (ClientTickLoop.renderer != null) {
                    MotionEditorScreen.open(ClientTickLoop.renderer);
                } else {
                    MC.gui.getChat().addMessage(Component.literal("[URDF] No active renderer. Put a *.urdf under ./KAIMyEntity or ./config and press G again."));
                }
            }
        }

        // ==== K: VMC 매핑 에디터 열기 ====
        if (keyOpenVmcMapping.consumeClick()) {
            if (ClientTickLoop.renderer == null) ensureActiveRenderer(MC);
            if (ClientTickLoop.renderer != null) {
                MC.setScreen(new VMCListenerController(MC.screen, ClientTickLoop.renderer));
            } else {
                MC.gui.getChat().addMessage(Component.literal("[URDF] No active renderer. Put a *.urdf under ./KAIMyEntity or ./config and press K again."));
            }
        }

        // ==== H: 물리 리셋 ====
        if (keyResetPhysics.isDown()) {
            var m = MMDModelManager.GetModel("EntityPlayer_" + player.getName().getString());
            if (m != null) {
                KAIMyEntityRendererPlayerHelper.ResetPhysics(player);
                net.neoforged.neoforge.network.PacketDistributor.sendToServer(
                        new com.kAIS.KAIMyEntity.neoforge.network.KAIMyEntityNetworkPack(2, player.getGameProfile(), 0));
                MC.gui.getChat().addMessage(Component.literal("URDF physics reset"));
            }
        }

        // ✅ ==== T: Webots 통계 출력 ====
        if (keyWebotsStats.consumeClick()) {
            try {
                WebotsController.getInstance().printStats();
                MC.gui.getChat().addMessage(Component.literal("§a[Webots] Stats printed to console"));
            } catch (Exception e) {
                MC.gui.getChat().addMessage(Component.literal("§c[Webots] Error: " + e.getMessage()));
            }
        }

        // ✅ ==== Y: Webots 테스트 자세 ====
        if (keyWebotsTest.consumeClick()) {
            testWebotsConnection(MC);
        }
    }

    // === 커스텀 애니메이션 처리 ===
    private static void handleCustomAnim(LocalPlayer player) {
        var m = MMDModelManager.GetModel("EntityPlayer_" + player.getName().getString());
        if (m == null) return;
        if (keyCustomAnim1.isDown()) sendAnim(player, 1);
        if (keyCustomAnim2.isDown()) sendAnim(player, 2);
        if (keyCustomAnim3.isDown()) sendAnim(player, 3);
        if (keyCustomAnim4.isDown()) sendAnim(player, 4);
    }

    private static void sendAnim(LocalPlayer player, int index) {
        KAIMyEntityRendererPlayerHelper.CustomAnim(player, String.valueOf(index));
        net.neoforged.neoforge.network.PacketDistributor.sendToServer(
                new com.kAIS.KAIMyEntity.neoforge.network.KAIMyEntityNetworkPack(1, player.getGameProfile(), index));
    }

    // ✅ === Webots 연결 테스트 (T-Pose) ===
    private static void testWebotsConnection(Minecraft mc) {
        try {
            var webots = WebotsController.getInstance();
            
            // T-Pose 자세 전송
            webots.setJoint("r_sho_pitch", 0.3f);   // 오른쪽 어깨 앞으로 약간
            webots.setJoint("r_sho_roll", 1.57f);   // 오른팔 벌리기 (90도)
            webots.setJoint("r_el", -0.1f);         // 팔꿈치 살짝 구부림
            
            webots.setJoint("l_sho_pitch", 0.3f);   // 왼쪽 어깨
            webots.setJoint("l_sho_roll", -1.57f);  // 왼팔 벌리기
            webots.setJoint("l_el", -0.1f);
            
            mc.gui.getChat().addMessage(Component.literal("§a[Webots] T-Pose sent! Check Webots simulation."));
            
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
            mc.gui.getChat().addMessage(Component.literal("[URDF] Guessing modelDir failed: " + (modelDir == null ? "null" : modelDir)));
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
