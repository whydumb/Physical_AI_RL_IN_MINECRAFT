package com.kAIS.KAIMyEntity.urdf.control;

import com.kAIS.KAIMyEntity.urdf.URDFJoint;
import com.kAIS.KAIMyEntity.urdf.URDFRobotModel;

import net.minecraft.client.Minecraft;
import net.minecraft.client.gui.GuiGraphics;
import net.minecraft.client.gui.components.Button;
import net.minecraft.client.gui.screens.Screen;
import net.minecraft.network.chat.Component;

import org.joml.Quaternionf;
import org.joml.Vector3f;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.*;

/**
 * URDFMotionEditor (이름 유지) — 간단 인스펙터/자료 페이지
 *
 * - VMC가 없어도 좌측에 "표준 VRM 본"을 항상 표시 (자료용)
 * - VMC가 연결되면 자동으로 실제 본 상태로 교체 표시
 * - 우측에는 URDF 조인트(모터) 목록/현재값/제한을 표시
 * - F5 새로고침, 1초 간격 자동 새로고침
 *
 * ※ 외부 의존성: 전부 리플렉션으로 접근 (컴파일 타임 의존성 없음)
 */
public class URDFMotionEditor extends Screen {

    // -------- 시각/색상(불투명) --------
    private static final int BG_COLOR    = 0xFF0E0E10; // 전체 배경
    private static final int PANEL_COLOR = 0xFF1D1F24; // 좌/우 패널 박스
    private static final int TITLE_COLOR = 0xFFFFD770;
    private static final int TXT_MAIN    = 0xFFFFFFFF;
    private static final int TXT_SUB_L   = 0xFFA0E0FF;
    private static final int TXT_SUB_R   = 0xFFB0FFA0;

    // -------- 기본 VRM 본 목록(소문자 camelCase) --------
    private static final String[] DEFAULT_VRM_BONES = {
            // 몸통
            "hips","spine","chest","upperChest","neck","head",
            // 왼팔
            "leftShoulder","leftUpperArm","leftLowerArm","leftHand",
            // 오른팔
            "rightShoulder","rightUpperArm","rightLowerArm","rightHand",
            // 왼다리
            "leftUpperLeg","leftLowerLeg","leftFoot","leftToes",
            // 오른다리
            "rightUpperLeg","rightLowerLeg","rightFoot","rightToes",
            // 손가락(생략 가능) - 필요시 아래 주석 해제
            //"leftThumbProximal","leftThumbIntermediate","leftThumbDistal",
            //"leftIndexProximal","leftIndexIntermediate","leftIndexDistal",
            //"leftMiddleProximal","leftMiddleIntermediate","leftMiddleDistal",
            //"leftRingProximal","leftRingIntermediate","leftRingDistal",
            //"leftLittleProximal","leftLittleIntermediate","leftLittleDistal",
            //"rightThumbProximal","rightThumbIntermediate","rightThumbDistal",
            //"rightIndexProximal","rightIndexIntermediate","rightIndexDistal",
            //"rightMiddleProximal","rightMiddleIntermediate","rightMiddleDistal",
            //"rightRingProximal","rightRingIntermediate","rightRingDistal",
            //"rightLittleProximal","rightLittleIntermediate","rightLittleDistal"
    };

    // -------- 상태/데이터 --------
    private final Screen parent;
    private final Object renderer; // getRobotModel()

    private final List<Row> boneRows  = new ArrayList<>();
    private final List<Row> jointRows = new ArrayList<>();

    // 페이지네이션
    private int perPageLeft  = 18;
    private int perPageRight = 18;
    private int bonePage = 0;
    private int jointPage = 0;

    // 레이아웃
    private final int margin = 8;
    private final int titleH = 16;
    private int listTop;
    private int listHeight;
    private int colWidth;
    private int leftX, rightX;

    // 버튼
    private Button bonePrevBtn, boneNextBtn, bonePageBtn;
    private Button jointPrevBtn, jointNextBtn, jointPageBtn;
    private Button refreshBtn;

    // 자동 새로고침
    private int autoRefreshTicker = 0;

    public URDFMotionEditor(Screen parent, Object renderer) {
        super(Component.literal("VMC & URDF Inspector"));
        this.parent = parent;
        this.renderer = renderer;
    }

    /** 하위호환: 예전 호출 (new URDFMotionEditor(robotModel, ctrl))용 어댑터 */
    public URDFMotionEditor(URDFRobotModel model, URDFSimpleController ctrl) {
        this(Minecraft.getInstance() != null ? Minecraft.getInstance().screen : null,
             new LegacyRendererAdapter(model, ctrl));
    }
    private static final class LegacyRendererAdapter {
        private final URDFRobotModel model;
        private final URDFSimpleController ctrl;
        LegacyRendererAdapter(URDFRobotModel model, URDFSimpleController ctrl) {
            this.model = model; this.ctrl = ctrl;
        }
        public URDFRobotModel getRobotModel() { return model; }
        public void setJointPreview(String name, float value) {
            if (model == null || model.joints == null) return;
            for (URDFJoint j : model.joints) {
                if (j != null && name.equals(j.name)) {
                    j.currentPosition = value;
                    if (ctrl != null) {
                        try {
                            ctrl.getClass().getMethod("setJointPosition", String.class, float.class)
                                .invoke(ctrl, name, value);
                        } catch (Throwable ignored) {}
                    }
                    break;
                }
            }
        }
        public String GetModelDir() { return "."; }
    }

    @Override
    protected void init() {
        super.init();
        computeLayout();
        buildData(/*force*/true);
        buildHeaderControls();
        updatePageLabels();
    }

    private void computeLayout() {
        listTop = margin + titleH + 6;
        listHeight = Math.max(120, this.height - listTop - 20);
        colWidth = (this.width - margin * 3) / 2;
        leftX = margin;
        rightX = leftX + colWidth + margin;

        int rowH = 12;
        perPageLeft = Math.max(6, listHeight / rowH);
        perPageRight = perPageLeft;
    }

    private void buildHeaderControls() {
        // 좌측
        bonePrevBtn = addRenderableWidget(Button.builder(Component.literal("< Prev"), b -> {
            if (bonePage > 0) bonePage--;
            updatePageLabels();
        }).bounds(leftX, listTop - 26, 60, 20).build());

        boneNextBtn = addRenderableWidget(Button.builder(Component.literal("Next >"), b -> {
            int pages = Math.max(1, (int)Math.ceil(boneRows.size() / (double)perPageLeft));
            if (bonePage < pages - 1) bonePage++;
            updatePageLabels();
        }).bounds(leftX + 66, listTop - 26, 60, 20).build());

        bonePageBtn = addRenderableWidget(Button.builder(Component.literal("Page"), b -> {})
                .bounds(leftX + 132, listTop - 26, 90, 20).build());
        bonePageBtn.active = false;

        // 우측
        jointPrevBtn = addRenderableWidget(Button.builder(Component.literal("< Prev"), b -> {
            if (jointPage > 0) jointPage--;
            updatePageLabels();
        }).bounds(rightX, listTop - 26, 60, 20).build());

        jointNextBtn = addRenderableWidget(Button.builder(Component.literal("Next >"), b -> {
            int pages = Math.max(1, (int)Math.ceil(jointRows.size() / (double)perPageRight));
            if (jointPage < pages - 1) jointPage++;
            updatePageLabels();
        }).bounds(rightX + 66, listTop - 26, 60, 20).build());

        jointPageBtn = addRenderableWidget(Button.builder(Component.literal("Page"), b -> {})
                .bounds(rightX + 132, listTop - 26, 90, 20).build());
        jointPageBtn.active = false;

        // 중앙 상단 새로고침
        refreshBtn = addRenderableWidget(Button.builder(Component.literal("F5 / Refresh"), b -> {
            buildData(true);
            updatePageLabels();
        }).bounds(this.width/2 - 60, listTop - 26, 120, 20).build());
    }

    private void updatePageLabels() {
        int bonePages  = Math.max(1, (int)Math.ceil(boneRows.size() / (double)perPageLeft));
        int jointPages = Math.max(1, (int)Math.ceil(jointRows.size() / (double)perPageRight));
        bonePage = clamp(bonePage, 0, bonePages - 1);
        jointPage = clamp(jointPage, 0, jointPages - 1);

        bonePageBtn.setMessage(Component.literal("Page " + (bonePage+1) + "/" + bonePages));
        jointPageBtn.setMessage(Component.literal("Page " + (jointPage+1) + "/" + jointPages));
    }

    /** 데이터 빌드: VMC가 없으면 VRM 기본 리스트를 채움 */
    private void buildData(boolean force) {
        // 이미 채워져 있고 강제 아님이면 스킵
        if (!force && !boneRows.isEmpty() && !jointRows.isEmpty()) return;

        boneRows.clear();
        jointRows.clear();

        // ---- 본 목록 (VMC 있으면 실제값, 없으면 VRM 기본) ----
        Map<String, Object> bones = reflectCollectBoneMap(reflectGetVmcState());
        if (bones.isEmpty()) {
            // VMC가 없어도 자료 페이지로 떠야 하므로 VRM 기본 본 표출
            for (String n : DEFAULT_VRM_BONES) {
                boneRows.add(new Row(n, "(no VMC)"));
            }
        } else {
            List<String> names = new ArrayList<>(bones.keySet());
            names.sort(String.CASE_INSENSITIVE_ORDER);
            for (String name : names) {
                Object tr = bones.get(name);
                float[] p = extractPos(tr);
                float[] e = extractEuler(tr);
                String detail = String.format(Locale.ROOT,
                        "X:%.2f Y:%.2f Z:%.2f | p:%.2f y:%.2f r:%.2f",
                        p[0], p[1], p[2], e[0], e[1], e[2]);
                boneRows.add(new Row(name, detail));
            }
        }

        // ---- 조인트 목록 ----
        URDFRobotModel model = reflectGetRobotModel();
        if (model == null || model.joints == null || model.joints.isEmpty()) {
            jointRows.add(new Row("(no URDF model)", ""));
        } else {
            for (URDFJoint j : model.joints) {
                String name = (j != null && j.name != null) ? j.name : "(unnamed)";
                float curDeg = (float)Math.toDegrees(j.currentPosition);
                String lim;
                if (j.limit != null && j.limit.hasLimits() && j.limit.upper > j.limit.lower) {
                    int lo = Math.round((float)Math.toDegrees(j.limit.lower));
                    int hi = Math.round((float)Math.toDegrees(j.limit.upper));
                    lim = String.format(Locale.ROOT, "cur:%d° | lim:[%d°, %d°]", Math.round(curDeg), lo, hi);
                } else {
                    lim = String.format(Locale.ROOT, "cur:%d° | lim:(none)", Math.round(curDeg));
                }
                jointRows.add(new Row(name, lim));
            }
        }

        bonePage = 0;
        jointPage = 0;
    }

    // ===== 렌더 =====
    @Override
    public void render(GuiGraphics g, int mouseX, int mouseY, float partialTicks) {
        // 배경/패널
        g.fill(0, 0, this.width, this.height, BG_COLOR);
        g.fill(leftX,  listTop, leftX  + colWidth, listTop + listHeight, PANEL_COLOR);
        g.fill(rightX, listTop, rightX + colWidth, listTop + listHeight, PANEL_COLOR);

        // 기본 위젯
        super.render(g, mouseX, mouseY, partialTicks);

        // 텍스트는 위로
        g.pose().pushPose();
        g.pose().translate(0, 0, 1000.0f);

        // 타이틀
        g.drawString(this.font, "VMC Bones (Humanoid / VRM) — shows defaults when VMC is off", leftX, 6, TITLE_COLOR, false);
        g.drawString(this.font, "URDF Joints (Motors)",                                      rightX, 6, TITLE_COLOR, false);

        // 좌측 리스트
        int y = listTop + 4;
        int start = bonePage * perPageLeft;
        int end = Math.min(boneRows.size(), start + perPageLeft);
        for (int i = start; i < end; i++) {
            Row r = boneRows.get(i);
            g.drawString(this.font, r.name, leftX + 4, y, TXT_MAIN, false);
            int nx = leftX + 4 + this.font.width(r.name) + 6;
            g.drawString(this.font, r.detail, nx, y, TXT_SUB_L, false);
            y += 12;
        }

        // 우측 리스트
        y = listTop + 4;
        start = jointPage * perPageRight;
        end = Math.min(jointRows.size(), start + perPageRight);
        for (int i = start; i < end; i++) {
            Row r = jointRows.get(i);
            g.drawString(this.font, r.name, rightX + 4, y, TXT_MAIN, false);
            int nx = rightX + 4 + this.font.width(r.name) + 6;
            g.drawString(this.font, r.detail, nx, y, TXT_SUB_R, false);
            y += 12;
        }

        g.pose().popPose();

        // 자동 새로고침 (1초 간격)
        if (++autoRefreshTicker >= 20) {
            autoRefreshTicker = 0;
            Object st = reflectGetVmcState();
            if (st != null) { // VMC가 붙었으면 실제 데이터로 갱신
                buildData(true);
                updatePageLabels();
            } else if (boneRows.isEmpty()) { // 안전: 비었으면 기본값 채우기
                buildData(true);
                updatePageLabels();
            }
        }
    }

    // 스크롤: 패널별 페이지 이동
    @Override
    public boolean mouseScrolled(double mx, double my, double deltaX, double deltaY) {
        double delta = (Math.abs(deltaY) > 0.0) ? deltaY : deltaX;
        boolean inLeft  = mx >= leftX  && mx < leftX  + colWidth && my >= listTop && my < listTop + listHeight;
        boolean inRight = mx >= rightX && mx < rightX + colWidth && my >= listTop && my < listTop + listHeight;

        if (inLeft) {
            int pages = Math.max(1, (int)Math.ceil(boneRows.size() / (double)perPageLeft));
            bonePage = clamp(bonePage - (int)Math.signum(delta), 0, pages - 1);
            updatePageLabels();
            return true;
        } else if (inRight) {
            int pages = Math.max(1, (int)Math.ceil(jointRows.size() / (double)perPageRight));
            jointPage = clamp(jointPage - (int)Math.signum(delta), 0, pages - 1);
            updatePageLabels();
            return true;
        }
        return super.mouseScrolled(mx, my, deltaX, deltaY);
    }

    // F5 새로고침
    @Override
    public boolean keyPressed(int keyCode, int scanCode, int modifiers) {
        // GLFW.GLFW_KEY_F5 = 294 (매핑에 따라 다를 수 있으니 숫자 사용)
        if (keyCode == 294) {
            buildData(true);
            updatePageLabels();
            return true;
        }
        return super.keyPressed(keyCode, scanCode, modifiers);
    }

    @Override
    public void resize(Minecraft mc, int w, int h) {
        super.resize(mc, w, h);
        computeLayout();
        updateHeaderBounds();
        updatePageLabels();
    }

    private void updateHeaderBounds() {
        // 좌측
        bonePrevBtn.setX(leftX);
        bonePrevBtn.setY(listTop - 26);
        boneNextBtn.setX(leftX + 66);
        boneNextBtn.setY(listTop - 26);
        bonePageBtn.setX(leftX + 132);
        bonePageBtn.setY(listTop - 26);
        // 우측
        jointPrevBtn.setX(rightX);
        jointPrevBtn.setY(listTop - 26);
        jointNextBtn.setX(rightX + 66);
        jointNextBtn.setY(listTop - 26);
        jointPageBtn.setX(rightX + 132);
        jointPageBtn.setY(listTop - 26);
        // 중앙
        refreshBtn.setX(this.width/2 - 60);
        refreshBtn.setY(listTop - 26);
    }

    @Override
    public void onClose() {
        this.minecraft.setScreen(this.parent);
    }

    // ===== 작은 구조체 =====
    private static class Row {
        final String name, detail;
        Row(String n, String d) { name = n; detail = d; }
    }

    // ===== VMC (리플렉션) =====
    private Object reflectGetVmcState() {
        try {
            Class<?> mgr = Class.forName("top.fifthlight.armorstand.vmc.VmcMarionetteManager");

            // getState() 시도
            try {
                Method getState = mgr.getMethod("getState");
                Object s = getState.invoke(null);
                if (s != null) return s;
            } catch (NoSuchMethodException ignored) {}

            // public static field state 시도 + state.value
            try {
                Field f = mgr.getField("state");
                Object v = f.get(null);
                if (v != null) {
                    try {
                        Field fv = v.getClass().getField("value");
                        Object sv = fv.get(v);
                        if (sv != null) return sv;
                    } catch (Throwable ignored2) {}
                    return v;
                }
            } catch (Throwable ignored) {}
        } catch (Throwable ignored) {}
        return null;
    }

    @SuppressWarnings("unchecked")
    private Map<String, Object> reflectCollectBoneMap(Object vmcState) {
        Map<String, Object> out = new HashMap<>();
        if (vmcState == null) return out;

        Object mapObj = null;

        // 필드 후보
        String[] fieldCandidates = { "boneTransforms", "bones", "transforms" };
        for (String fname : fieldCandidates) {
            try {
                Field f;
                try {
                    f = vmcState.getClass().getField(fname);
                } catch (NoSuchFieldException e) {
                    f = vmcState.getClass().getDeclaredField(fname);
                    f.setAccessible(true);
                }
                mapObj = f.get(vmcState);
                if (mapObj != null) break;
            } catch (Throwable ignored) {}
        }

        // 메서드 후보
        if (mapObj == null) {
            String[] methodCandidates = { "getBoneTransforms", "boneTransforms", "getBones", "getTransforms" };
            for (String mname : methodCandidates) {
                try {
                    Method m = vmcState.getClass().getMethod(mname);
                    mapObj = m.invoke(vmcState);
                    if (mapObj != null) break;
                } catch (Throwable ignored) {}
            }
        }

        if (!(mapObj instanceof Map)) return out;

        Map<Object, Object> m = (Map<Object, Object>) mapObj;
        for (Map.Entry<Object, Object> e : m.entrySet()) {
            Object key = e.getKey();
            String name = null;

            // Enum이면 name() 우선
            if (key instanceof Enum<?>) name = ((Enum<?>) key).name();

            if (name == null) {
                try {
                    Method nameM = key.getClass().getMethod("name");
                    Object n = nameM.invoke(key);
                    if (n != null) name = n.toString();
                } catch (Throwable ignored) {}
            }
            if (name == null) name = (key != null ? key.toString() : "(null)");

            out.put(name, e.getValue());
        }
        return out;
    }

    // ===== VMC 트랜스폼 유틸 =====
    private static float[] extractPos(Object transform) {
        float[] r = {0, 0, 0};
        if (transform == null) return r;
        try {
            Object pos = transform.getClass().getField("position").get(transform);
            Method mx = pos.getClass().getMethod("x");
            Method my = pos.getClass().getMethod("y");
            Method mz = pos.getClass().getMethod("z");
            r[0] = (Float) mx.invoke(pos);
            r[1] = (Float) my.invoke(pos);
            r[2] = (Float) mz.invoke(pos);
        } catch (Throwable ignored) {}
        return r;
    }

    private static float[] extractEuler(Object transform) {
        float[] r = {0, 0, 0};
        if (transform == null) return r;
        try {
            Object rot = transform.getClass().getField("rotation").get(transform);
            Method mx = rot.getClass().getMethod("x");
            Method my = rot.getClass().getMethod("y");
            Method mz = rot.getClass().getMethod("z");
            Method mw = rot.getClass().getMethod("w");
            float qx = (Float) mx.invoke(rot);
            float qy = (Float) my.invoke(rot);
            float qz = (Float) mz.invoke(rot);
            float qw = (Float) mw.invoke(rot);
            Vector3f e = new Vector3f();
            new Quaternionf(qx, qy, qz, qw).getEulerAnglesXYZ(e);
            r[0] = e.x; r[1] = e.y; r[2] = e.z;
        } catch (Throwable ignored) {}
        return r;
    }

    // ===== URDF (리플렉션) =====
    private URDFRobotModel reflectGetRobotModel() {
        try {
            Method m = renderer.getClass().getMethod("getRobotModel");
            return (URDFRobotModel) m.invoke(renderer);
        } catch (Throwable ignored) {}
        return null;
    }

    // ===== 유틸 =====
    private static int clamp(int v, int lo, int hi) {
        return v < lo ? lo : Math.min(v, hi);
    }
}
