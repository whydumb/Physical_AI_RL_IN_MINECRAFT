package com.kAIS.KAIMyEntity.urdf.control;

import com.kAIS.KAIMyEntity.urdf.URDFJoint;
import com.kAIS.KAIMyEntity.urdf.URDFRobotModel;
import net.minecraft.client.Minecraft;
import net.minecraft.client.gui.GuiGraphics;
import net.minecraft.client.gui.components.AbstractSliderButton;
import net.minecraft.client.gui.components.Button;
import net.minecraft.client.gui.screens.Screen;
import net.minecraft.network.chat.Component;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;

/**
 * URDF Joint Editor (즉시 적용 전용, 리플렉션 호환)
 *
 * - 페이지 분할(Prev/Next)
 * - 각 관절 행: [-] 슬라이더 [+]  (리미트 기반 스케일)
 * - 상단: Prev / Next / Page, Reset All
 * - 하단: Exit
 *
 * renderer 필요 메서드(리플렉션):
 *   · getRobotModel(): URDFRobotModel
 *   · setJointPreview(String, float)
 *   · setJointTarget(String, float)
 */
public class MotionEditorScreen extends Screen {
    private final Object renderer;

    private final List<Row> rows = new ArrayList<>();
    private int page = 0;
    private final int perPage = 14;

    private final Screen parent;

    public MotionEditorScreen(Screen parent, Object renderer) {
        super(Component.literal("URDF Joint Editor"));
        this.parent = parent;
        this.renderer = renderer;
    }
    public MotionEditorScreen(Object renderer) { this(null, renderer); }

    @Override
    protected void init() { rebuild(); }

    private void rebuild() {
        clearWidgets();
        rows.clear();

        URDFRobotModel model = reflectGetRobotModel(renderer);
        if (model == null || model.joints == null || model.joints.isEmpty()) {
            buildHeaderButtons(0);
            buildExitButton();
            return;
        }

        int total = model.joints.size();
        buildHeaderButtons(total);

        int listTop  = 42;
        int leftX    = 20;

        int start = page * perPage;
        int end   = Math.min(total, start + perPage);

        int y = listTop;

        for (int i = start; i < end; i++) {
            URDFJoint j = model.joints.get(i);

            float lo = (j.limit != null && j.limit.upper > j.limit.lower)
                    ? (float) j.limit.lower
                    : (float) Math.toRadians(-180.0);
            float hi = (j.limit != null && j.limit.upper > j.limit.lower)
                    ? (float) j.limit.upper
                    : (float) Math.toRadians( 180.0);
            if (hi <= lo) { lo = (float) Math.toRadians(-180.0); hi = (float) Math.toRadians(180.0); }

            final URDFJoint jRef = j;
            final float loF = lo, hiF = hi;

            addRenderableWidget(Button.builder(Component.literal("-"), b -> {
                float step = (float) Math.toRadians(2.0);
                float v = clamp(jRef.currentPosition - step, loF, hiF);
                reflectSetJointPreview(renderer, jRef.name, v);
                reflectSetJointTarget(renderer, jRef.name, v);
                syncRow(jRef.name, v);
            }).bounds(leftX, y, 20, 20).build());

            JointSlider slider = new JointSlider(leftX + 24, y, 260, 20,
                    jRef.name, jRef.currentPosition, loF, hiF, renderer);
            rows.add(new Row(jRef.name, slider));
            addRenderableWidget(slider);

            addRenderableWidget(Button.builder(Component.literal("+"), b -> {
                float step = (float) Math.toRadians(2.0);
                float v = clamp(jRef.currentPosition + step, loF, hiF);
                reflectSetJointPreview(renderer, jRef.name, v);
                reflectSetJointTarget(renderer, jRef.name, v);
                syncRow(jRef.name, v);
            }).bounds(leftX + 288, y, 20, 20).build());

            y += 24;
        }

        buildExitButton();
    }

    private void buildHeaderButtons(int totalJoints) {
        int headerY = 10;
        int leftX   = 20;

        int pages  = Math.max(1, (int) Math.ceil(totalJoints / (double) perPage));
        page = Math.min(page, pages - 1);
        page = Math.max(page, 0);

        addRenderableWidget(Button.builder(Component.literal("< Prev"), b -> {
            if (page > 0) { page--; rebuild(); }
        }).bounds(leftX, headerY, 60, 20).build());

        addRenderableWidget(Button.builder(Component.literal("Next >"), b -> {
            if (page < pages - 1) { page++; rebuild(); }
        }).bounds(leftX + 66, headerY, 60, 20).build());

        Button pageLabel = Button.builder(Component.literal("Page " + (page + 1) + "/" + pages), b -> {})
                .bounds(leftX + 132, headerY, 90, 20).build();
        pageLabel.active = false;
        addRenderableWidget(pageLabel);

        addRenderableWidget(Button.builder(Component.literal("Reset All"), b -> {
            URDFRobotModel model = reflectGetRobotModel(renderer);
            if (model != null && model.joints != null) {
                for (URDFJoint j : model.joints) {
                    reflectSetJointPreview(renderer, j.name, 0f);
                    reflectSetJointTarget(renderer, j.name, 0f);
                }
            }
            for (Row r : rows) r.slider.setFromRadians(0f);
        }).bounds(width - 100, headerY, 80, 20).build());
    }

    private void buildExitButton() {
        addRenderableWidget(Button.builder(Component.literal("Exit"), b -> {
            if (parent != null) minecraft.setScreen(parent);
            else Minecraft.getInstance().setScreen(null);
        }).bounds(width - 70, height - 30, 50, 20).build());
    }

    @Override
    public void render(GuiGraphics g, int mouseX, int mouseY, float partialTicks) {
        renderBackground(g, mouseX, mouseY, partialTicks);
        super.render(g, mouseX, mouseY, partialTicks);
        g.drawCenteredString(font, "URDF Joint Editor (Immediate)", width / 2, 2, 0xFFFFFF);
    }

    private void syncRow(String jointName, float radians) {
        for (Row r : rows) if (r.jointName.equals(jointName)) { r.slider.setFromRadians(radians); break; }
    }

    private record Row(String jointName, JointSlider slider) {}

    private static class JointSlider extends AbstractSliderButton {
        private final String jointName;
        private final Object renderer;
        private final float lo, hi;

        public JointSlider(int x, int y, int w, int h,
                           String jointName, float currentRad, float lo, float hi,
                           Object renderer) {
            super(x, y, w, h, Component.literal(""), normalize(currentRad, lo, hi));
            this.jointName = jointName;
            this.renderer = renderer;
            this.lo = lo; this.hi = hi;
            updateMessage();
        }

        @Override
        protected void updateMessage() {
            float rad = denorm((float) value);
            int deg = Math.round((float) Math.toDegrees(rad));
            setMessage(Component.literal(jointName + ": " + deg + "°"));
        }

        @Override
        protected void applyValue() {
            float rad = denorm((float) value);
            reflectSetJointPreview(renderer, jointName, rad);
            reflectSetJointTarget(renderer, jointName, rad);
        }

        @Override
        public boolean mouseDragged(double mx, double my, int button, double dx, double dy) {
            boolean r = super.mouseDragged(mx, my, button, dx, dy);
            float rad = denorm((float) value);
            reflectSetJointPreview(renderer, jointName, rad);
            reflectSetJointTarget(renderer, jointName, rad);
            return r;
        }

        public void setFromRadians(float rad) { this.value = normalize(rad, lo, hi); updateMessage(); }
        private float denorm(float v01) { return lo + v01 * (hi - lo); }
        private static float normalize(float v, float lo, float hi) {
            if (hi - lo <= 1e-6f) return 0.5f;
            float t = (v - lo) / (hi - lo);
            return t < 0 ? 0 : Math.min(1, t);
        }
    }

    private static URDFRobotModel reflectGetRobotModel(Object rend) {
        if (rend == null) return null;
        try {
            Method m = rend.getClass().getMethod("getRobotModel");
            Object o = m.invoke(rend);
            return (o instanceof URDFRobotModel) ? (URDFRobotModel) o : null;
        } catch (Throwable ignored) { return null; }
    }
    private static void reflectSetJointPreview(Object rend, String name, float rad) {
        if (rend == null || name == null) return;
        try {
            Method m = rend.getClass().getMethod("setJointPreview", String.class, float.class);
            m.invoke(rend, name, rad);
        } catch (Throwable ignored) { }
    }
    private static void reflectSetJointTarget(Object rend, String name, float rad) {
        if (rend == null || name == null) return;
        try {
            Method m = rend.getClass().getMethod("setJointTarget", String.class, float.class);
            m.invoke(rend, name, rad);
        } catch (Throwable ignored) { }
    }

    private static float clamp(float v, float lo, float hi) { return v < lo ? lo : Math.min(hi, v); }

    @Override
    public void onClose() {
        if (parent != null) minecraft.setScreen(parent);
        else super.onClose();
    }
}
