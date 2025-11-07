package com.kAIS.KAIMyEntity.client.command;

import com.kAIS.KAIMyEntity.urdf.URDFModelOpenGLWithSTL;
import com.kAIS.KAIMyEntity.urdf.control.MotionEditorScreen;
import com.kAIS.KAIMyEntity.urdf.control.URDFMappingScreen;
import com.kAIS.KAIMyEntity.urdf.control.URDFMotionEditor;

import com.mojang.brigadier.Command;
import com.mojang.brigadier.context.CommandContext;
import net.minecraft.client.Minecraft;
import net.minecraft.client.gui.components.Button;
import net.minecraft.client.gui.screens.Screen;
import net.minecraft.commands.CommandSourceStack;
import net.minecraft.network.chat.Component;

/**
 * 클라이언트 커맨드에서 모션 관련 GUI를 엽니다.
 * - 존재하지 않는 옛 GUI( EditorSelectionScreen / client.gui.* ) 의존성 제거
 * - 파일 내부에 간단한 선택 화면을 내장(두 버튼: Joint / VMC)
 */
public class OpenMotionGuiCommand {

    public static int execute(CommandContext<CommandSourceStack> ctx) {
        Minecraft mc = Minecraft.getInstance();

        // ClientTickLoop에서 renderer 가져오기 (리플렉션)
        URDFModelOpenGLWithSTL renderer = getRendererFromClientTickLoop();
        if (renderer == null) {
            ctx.getSource().sendFailure(Component.literal("URDF 렌더러를 찾을 수 없습니다."));
            return 0;
        }

        // 선택 화면 열기 (메인 스레드 보장)
        mc.execute(() -> mc.setScreen(new EditorSelectionScreenLite(mc.screen, renderer)));
        return Command.SINGLE_SUCCESS;
    }

    /** ClientTickLoop.renderer 정적 필드에서 렌더러를 가져온다. */
    private static URDFModelOpenGLWithSTL getRendererFromClientTickLoop() {
        // 패키지 경로가 환경에 따라 다를 수 있어 두 경로 모두 시도
        String[] candidates = new String[] {
            "com.kAIS.KAIMyEntity.neoforge.loop.ClientTickLoop",
            "com.kAIS.KAIMyEntity.neoforge.ClientTickLoop"
        };
        for (String fqcn : candidates) {
            try {
                Class<?> cls = Class.forName(fqcn);
                java.lang.reflect.Field f = cls.getField("renderer");
                Object o = f.get(null);
                if (o instanceof URDFModelOpenGLWithSTL) return (URDFModelOpenGLWithSTL) o;
            } catch (Throwable ignored) { }
        }
        return null;
    }

    /** 파일 내부 간단 선택 화면 (두 버튼만) */
    private static class EditorSelectionScreenLite extends Screen {
        private final Screen parent;
        private final URDFModelOpenGLWithSTL renderer;

        protected EditorSelectionScreenLite(Screen parent, URDFModelOpenGLWithSTL renderer) {
            super(Component.literal("Open Motion GUI"));
            this.parent = parent;
            this.renderer = renderer;
        }

        @Override
        protected void init() {
            int w = 160, h = 20;
            int cx = this.width / 2;
            int cy = this.height / 2;

            // Joint Editor 버튼
            this.addRenderableWidget(
                Button.builder(Component.literal("Open Joint Editor"), b -> {
                    // 조인트 즉시 조작 화면 (parent를 넘기는 생성자 사용)
                    this.minecraft.setScreen(new MotionEditorScreen(this, renderer));
                }).bounds(cx - w - 6, cy - 10, w, h).build()
            );

            // VMC Mapping 버튼
            this.addRenderableWidget(
                Button.builder(Component.literal("Open VMC Mapping"), b -> {
                    // VMC-URDF 매핑 에디터 (3D 뷰포트)
                    URDFMotionEditor me = renderer.getMotionEditor();
                    this.minecraft.setScreen(new URDFMappingScreen(this, renderer, me));
                }).bounds(cx + 6, cy - 10, w, h).build()
            );

            // 닫기
            this.addRenderableWidget(
                Button.builder(Component.literal("Close"), b -> this.minecraft.setScreen(parent))
                      .bounds(cx - 40, cy + 22, 80, 20).build()
            );
        }

        @Override
        public void onClose() {
            this.minecraft.setScreen(parent);
        }

        @Override
        public void render(net.minecraft.client.gui.GuiGraphics g, int mouseX, int mouseY, float partialTicks) {
            this.renderBackground(g, mouseX, mouseY, partialTicks);
            super.render(g, mouseX, mouseY, partialTicks);
            g.drawCenteredString(this.font, "Select Motion GUI", this.width / 2, this.height / 2 - 38, 0xFFFFFF);
        }
    }
}
