package com.kAIS.KAIMyEntity.urdf.control;

import com.kAIS.KAIMyEntity.urdf.URDFJoint;
import com.kAIS.KAIMyEntity.urdf.URDFRobotModel;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import net.minecraft.client.Minecraft;
import net.minecraft.client.gui.GuiGraphics;
import net.minecraft.client.gui.components.Button;
import net.minecraft.client.gui.screens.Screen;
import net.minecraft.network.chat.Component;
import org.joml.Quaternionf;
import org.joml.Vector3f;

import java.io.IOException;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;

/**
 * URDFMotionEditor (이름 유지) — VMC/URDF 인스펙터 + 반자동 매핑 위저드
 *
 * 추가된 것:
 * - 매핑 스키마(VMCURDFMapping) + JSON 저장/불러오기
 * - 본 하나 선택 후, 사용자가 "URDF 관절을 한 축으로만 왕복" → 회귀해서 multiplier/offset/축/성분 추정
 * - 버튼: [위저드 시작] [기록/정지] [이 단계 적용] [저장] [불러오기]
 *
 * 시각:
 * - 좌/우 패널 불투명 배경, 텍스트 오버레이 (간단 프리뷰)
 */
public class URDFMotionEditor extends Screen {

    // ---------- 색상(불투명) ----------
    private static final int BG_COLOR    = 0xFF0E0E10; // 화면 전체 배경
    private static final int PANEL_COLOR = 0xFF1D1F24; // 좌/우 패널 박스
    private static final int TITLE_LEFT_COLOR  = 0xFFFFD770;
    private static final int TITLE_RIGHT_COLOR = 0xFFFFD770;
    private static final int TEXT_LEFT_COLOR   = 0xFFFFFFFF;
    private static final int TEXT_LEFT_SUB     = 0xFFA0E0FF;
    private static final int TEXT_RIGHT_COLOR  = 0xFFFFFFFF;
    private static final int TEXT_RIGHT_SUB    = 0xFFB0FFA0;

    private final Screen parent;
    private final Object renderer; // getRobotModel(), setJointPreview(), GetModelDir()

    // 좌/우 리스트 데이터
    private final List<BoneRow> boneRows = new ArrayList<>();
    private final List<JointRow> jointRows = new ArrayList<>();

    // 페이지 상태
    private int bonePage = 0;
    private int jointPage = 0;
    private int perPageLeft = 18;
    private int perPageRight = 18;

    // 레이아웃 캐시
    private int margin = 8;
    private int titleH = 16;
    private int listTop;
    private int colWidth;
    private int leftX, rightX;
    private int listHeight;

    // UI 버튼 (상단 컨트롤 + 하단 위저드/저장)
    private Button bonePrevBtn, boneNextBtn, bonePageBtn;
    private Button jointPrevBtn, jointNextBtn, jointPageBtn;

    private Button wizardStartBtn, recordToggleBtn, applyStepBtn, saveBtn, loadBtn;

    // 선택 상태
    private String selectedBone = null;
    private String selectedJoint = null;

    // 상태 메시지
    private String status = "";

    // ------- 매핑 스키마 -------
    private final Gson gson = new GsonBuilder().setPrettyPrinting().create();
    private VMCURDFMapping mapping = new VMCURDFMapping();

    public static final class VMCURDFMapping {
        public String robotName = "UNKNOWN";
        public List<BoneMapping> mappings = new ArrayList<>();

        public static final class BoneMapping {
            public String vmcBone;
            public List<JointComponent> joints = new ArrayList<>(); // ★ 순서 = 체인 순서
            public float[] preRotCorr = {0,0,0,1};
            public float   extraOffsetRad = 0f;
        }
        public static final class JointComponent {
            public String jointName;
            public RotationComponent component = RotationComponent.PITCH;
            public ExtractMode mode = ExtractMode.AUTO;   // EULER / AXIS_PROJ / MAGNITUDE
            public float multiplier = 1f;
            public float offset = 0f;
            public float[] axis = null;                   // AXIS_PROJ 시 사용
            public EulerOrder eulerOrder = EulerOrder.ZYX;
        }
        public enum RotationComponent { YAW, PITCH, ROLL, MAGNITUDE }
        public enum ExtractMode { AUTO, EULER, AXIS_PROJ, MAGNITUDE }
        public enum EulerOrder { ZYX, XYZ, XZY, YXZ, YZX, ZXY }
    }

    // ------- 위저드(기록/회귀) -------
    private MappingWizard wizard = null;
    private boolean recording = false;
    private int autoRefreshTicker = 0;

    public URDFMotionEditor(Screen parent, Object renderer) {
        super(Component.literal("VMC & URDF Mapper"));
        this.parent = parent;
        this.renderer = renderer;
    }

    // 하위호환: 예전 호출(new URDFMotionEditor(robotModel, ctrl))
    public URDFMotionEditor(URDFRobotModel model, URDFSimpleController ctrl) {
        this(Minecraft.getInstance() != null ? Minecraft.getInstance().screen : null,
             new LegacyRendererAdapter(model));
    }
    private static final class LegacyRendererAdapter {
        private final URDFRobotModel model;
        LegacyRendererAdapter(URDFRobotModel model) { this.model = model; }
        public URDFRobotModel getRobotModel() { return model; }
        public String GetModelDir() { return "."; }
        public void setJointPreview(String name, float v) { if (model==null||model.joints==null) return; for (URDFJoint j:model.joints) if (name.equals(j.name)) j.currentPosition=v; }
    }

    @Override
    protected void init() {
        super.init();
        computeLayout();
        buildData();
        buildHeaderControls();
        buildWizardButtons();
        updatePageLabels();
    }

    private void computeLayout() {
        listTop = margin + titleH + 4;
        listHeight = Math.max(120, this.height - listTop - 60); // 아래에 버튼 영역 확보
        colWidth = (this.width - margin * 3) / 2;
        leftX = margin;
        rightX = leftX + colWidth + margin;

        perPageLeft = Math.max(5, listHeight / 12);
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
    }

    private void buildWizardButtons() {
        int y = this.height - 28;

        wizardStartBtn = addRenderableWidget(Button.builder(Component.literal("위저드 시작"), b -> {
            if (selectedBone == null) { status = "본을 먼저 선택하세요."; return; }
            wizard = new MappingWizard(selectedBone, this.renderer);
            status = "위저드 시작: " + selectedBone;
        }).bounds(this.width - 520, y, 100, 20).build());

        recordToggleBtn = addRenderableWidget(Button.builder(Component.literal("기록/정지"), b -> {
            if (wizard == null) { status = "위저드를 먼저 시작하세요."; return; }
            if (selectedJoint == null) { status = "관절을 먼저 선택하세요."; return; }
            if (!recording) {
                wizard.beginJoint(selectedJoint);
                recording = true;
                status = "기록중… " + selectedJoint + " (해당 관절만 한 축으로 왕복)";
            } else {
                recording = false;
                status = "기록 정지";
            }
        }).bounds(this.width - 414, y, 100, 20).build());

        applyStepBtn = addRenderableWidget(Button.builder(Component.literal("이 단계 적용"), b -> {
            if (wizard == null) { status = "위저드를 먼저 시작하세요."; return; }
            VMCURDFMapping.JointComponent jc = wizard.finalizeStep();
            if (jc == null) { status = "유효한 상관 부족: 더 크게/한 축으로 천천히 움직이세요."; return; }
            addJointComponentToMapping(selectedBone, jc);
            status = "추가됨: " + selectedBone + " → " + jc.jointName
                    + String.format(Locale.ROOT, " (m=%.3f, b=%.3f, %s)", jc.multiplier, jc.offset, jc.mode);
        }).bounds(this.width - 308, y, 110, 20).build());

        saveBtn = addRenderableWidget(Button.builder(Component.literal("저장"), b -> saveMappingJson())
                .bounds(this.width - 194, y, 80, 20).build());

        loadBtn = addRenderableWidget(Button.builder(Component.literal("불러오기"), b -> loadMappingJson())
                .bounds(this.width - 108, y, 100, 20).build());
    }

    private void updatePageLabels() {
        int bonePages  = Math.max(1, (int)Math.ceil(boneRows.size()  / (double)perPageLeft));
        int jointPages = Math.max(1, (int)Math.ceil(jointRows.size() / (double)perPageRight));
        bonePage = clamp(bonePage, 0, bonePages - 1);
        jointPage = clamp(jointPage, 0, jointPages - 1);

        if (bonePageBtn != null)  bonePageBtn.setMessage(Component.literal("Page " + (bonePage+1) + "/" + bonePages));
        if (jointPageBtn != null) jointPageBtn.setMessage(Component.literal("Page " + (jointPage+1) + "/" + jointPages));
    }

    private void buildData() {
        boneRows.clear();
        jointRows.clear();

        // VMC 본 덤프
        Map<String, Object> bones = reflectCollectBoneMap(reflectGetVmcState());
        List<String> boneNames = new ArrayList<>(bones.keySet());
        boneNames.sort(String.CASE_INSENSITIVE_ORDER);

        if (boneNames.isEmpty()) {
            boneRows.add(new BoneRow("(no VMC state)", "-", null));
        } else {
            for (String name : boneNames) {
                Object tr = bones.get(name);
                float[] p = extractPos(tr);
                float[] e = extractEuler(tr);
                String line = String.format(Locale.ROOT,
                        "X:%.2f Y:%.2f Z:%.2f | p:%.2f y:%.2f r:%.2f",
                        p[0], p[1], p[2], e[0], e[1], e[2]);
                boneRows.add(new BoneRow(name, line, name));
            }
        }

        // URDF 조인트 덤프
        URDFRobotModel model = reflectGetRobotModel(renderer);
        if (model == null || model.joints == null || model.joints.isEmpty()) {
            jointRows.add(new JointRow("(no URDF model)", "", null));
        } else {
            for (URDFJoint j : model.joints) {
                String name = j.name != null ? j.name : "(unnamed)";
                float curDeg = (float) Math.toDegrees(j.currentPosition);
                String limTxt;
                if (j.limit != null && (j.limit.upper > j.limit.lower)) {
                    int lo = Math.round((float) Math.toDegrees(j.limit.lower));
                    int hi = Math.round((float) Math.toDegrees(j.limit.upper));
                    limTxt = String.format(Locale.ROOT, "cur:%d° | lim:[%d°, %d°]", Math.round(curDeg), lo, hi);
                } else {
                    limTxt = String.format(Locale.ROOT, "cur:%d° | lim:(none)", Math.round(curDeg));
                }
                jointRows.add(new JointRow(name, limTxt, name));
            }
        }

        bonePage = 0;
        jointPage = 0;
    }

    // ---------- 입력/틱 ----------
    @Override
    public void tick() {
        super.tick();
        // 자동 새로고침(약 1초)
        if (++autoRefreshTicker >= 20) {
            autoRefreshTicker = 0;
            buildData();
            updatePageLabels();
        }
        // 기록 중이면 샘플 수집
        if (recording && wizard != null && selectedJoint != null) {
            Object vmc = reflectGetVmcState();
            Object tr = wizard.pickVmcTransform(vmc);
            float jointAngle = wizard.readJointAngleRad(selectedJoint);
            wizard.onTick(tr, jointAngle);
        }
    }

    // ---------- 렌더 ----------
    @Override
    public void render(GuiGraphics g, int mouseX, int mouseY, float partialTicks) {
        // 1) 배경
        g.fill(0, 0, this.width, this.height, BG_COLOR);

        // 2) 좌/우 패널
        g.fill(leftX,  listTop, leftX  + colWidth, listTop + listHeight, PANEL_COLOR);
        g.fill(rightX, listTop, rightX + colWidth, listTop + listHeight, PANEL_COLOR);

        // 3) 기본 위젯
        super.render(g, mouseX, mouseY, partialTicks);

        // 4) 최상층 텍스트
        g.pose().pushPose();
        g.pose().translate(0, 0, 1000.0f);

        // 제목
        g.drawString(this.font, "VMC Bones (클릭: 선택) — 본을 고른 뒤, 오른쪽에서 관절을 골라 위저드를 진행", leftX, 6, TITLE_LEFT_COLOR, false);
        g.drawString(this.font, "URDF Joints (클릭: 선택) — 관절을 한 축으로만 왕복해서 매핑", rightX, 6, TITLE_RIGHT_COLOR, false);

        // 좌측 리스트
        int y = listTop + 4;
        int start = bonePage * perPageLeft;
        int end = Math.min(boneRows.size(), start + perPageLeft);
        for (int i = start; i < end; i++) {
            BoneRow r = boneRows.get(i);
            int col = (Objects.equals(selectedBone, r.selKey) ? 0xFFFFC040 : TEXT_LEFT_COLOR);
            g.drawString(this.font, r.name, leftX + 4, y, col, false);
            int nx = leftX + 4 + this.font.width(r.name) + 6;
            g.drawString(this.font, r.detail, nx, y, TEXT_LEFT_SUB, false);

            // 클릭 검출(간단 hit)
            if (mouseY >= y && mouseY < y + 12 && mouseX >= leftX && mouseX < leftX + colWidth && isMouseDown()) {
                selectedBone = r.selKey;
                status = "선택된 본: " + selectedBone;
            }
            y += 12;
        }

        // 우측 리스트
        y = listTop + 4;
        start = jointPage * perPageRight;
        end = Math.min(jointRows.size(), start + perPageRight);
        for (int i = start; i < end; i++) {
            JointRow r = jointRows.get(i);
            int col = (Objects.equals(selectedJoint, r.selKey) ? 0xFF80FF80 : TEXT_RIGHT_COLOR);
            g.drawString(this.font, r.name, rightX + 4, y, col, false);
            int nx = rightX + 4 + this.font.width(r.name) + 6;
            g.drawString(this.font, r.detail, nx, y, TEXT_RIGHT_SUB, false);

            if (mouseY >= y && mouseY < y + 12 && mouseX >= rightX && mouseX < rightX + colWidth && isMouseDown()) {
                selectedJoint = r.selKey;
                status = "선택된 관절: " + selectedJoint;
            }
            y += 12;
        }

        // 상태
        if (!status.isEmpty()) g.drawString(this.font, status, 8, this.height - 48, 0x80FF80, false);

        // 현재 선택 표시
        String sel = "선택: VMC=" + (selectedBone==null?"(없음)":selectedBone)
                + " / URDF=" + (selectedJoint==null?"(없음)":selectedJoint)
                + (recording ? "  [기록중]" : "");
        g.drawString(this.font, sel, 8, this.height - 64, 0xA0A0A0, false);

        g.pose().popPose();
    }

    private boolean mouseDownCache = false;
    private boolean isMouseDown() {
        boolean now = Minecraft.getInstance().mouseHandler.isLeftPressed();
        boolean ret = now && !mouseDownCache;
        mouseDownCache = now;
        return ret;
    }

    @Override
    public void onClose() {
        this.minecraft.setScreen(this.parent);
    }

    /* ---------------- 데이터 행 ---------------- */
    private static class BoneRow {
        final String name, detail, selKey;
        BoneRow(String n, String d, String k) { name = n; detail = d; selKey = k; }
    }
    private static class JointRow {
        final String name, detail, selKey;
        JointRow(String n, String d, String k) { name = n; detail = d; selKey = k; }
    }

    /* ---------------- VMC reflection ---------------- */
    private Object reflectGetVmcState() {
        try {
            Class<?> mgr = Class.forName("top.fifthlight.armorstand.vmc.VmcMarionetteManager");
            Method getState = mgr.getMethod("getState");
            return getState.invoke(null);
        } catch (Throwable ignored) {}
        return null;
    }

    @SuppressWarnings("unchecked")
    private Map<String, Object> reflectCollectBoneMap(Object vmcState) {
        Map<String, Object> out = new HashMap<>();
        if (vmcState == null) return out;

        Object mapObj = null;

        // 1) 필드 후보
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

        // 2) 메서드 후보
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
            if (key != null) {
                try { if (key instanceof Enum) name = ((Enum<?>) key).name(); } catch (Throwable ignored) {}
                if (name == null) {
                    try { Method nameM = key.getClass().getMethod("name"); Object n = nameM.invoke(key); if (n != null) name = n.toString(); } catch (Throwable ignored) {}
                }
                if (name == null) name = key.toString();
            } else name = "(null)";
            out.put(name, e.getValue());
        }
        return out;
    }

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

    /* ---------------- URDF access ---------------- */
    private URDFRobotModel reflectGetRobotModel(Object renderer) {
        try {
            Method m = renderer.getClass().getMethod("getRobotModel");
            return (URDFRobotModel) m.invoke(renderer);
        } catch (Throwable ignored) {}
        return null;
    }

    private String reflectGetModelDir() {
        try {
            Method m = renderer.getClass().getMethod("GetModelDir");
            Object r = m.invoke(renderer);
            return r == null ? null : r.toString();
        } catch (Throwable ignored) {}
        return null;
    }

    /* ---------------- 매핑 I/O ---------------- */
    private Path mappingPath() {
        String dir = reflectGetModelDir();
        return Paths.get(dir == null ? "." : dir, "vmc_mapping.json");
    }

    private void saveMappingJson() {
        try {
            Path p = mappingPath();
            Files.createDirectories(p.getParent());
            String json = gson.toJson(mapping);
            Files.writeString(p, json);
            status = "매핑 저장: " + p;
        } catch (IOException e) {
            status = "저장 실패: " + e.getMessage();
        }
    }

    private void loadMappingJson() {
        try {
            Path p = mappingPath();
            if (!Files.exists(p)) { status = "파일 없음: " + p; return; }
            String json = Files.readString(p);
            VMCURDFMapping m = new Gson().fromJson(json, VMCURDFMapping.class);
            if (m != null) mapping = m;
            status = "매핑 불러오기: " + p;
        } catch (IOException e) {
            status = "불러오기 실패: " + e.getMessage();
        }
    }

    /* ---------------- 매핑 갱신 ---------------- */
    private void addJointComponentToMapping(String vmcBone, VMCURDFMapping.JointComponent jc) {
        if (vmcBone == null || jc == null) return;
        VMCURDFMapping.BoneMapping bm = null;
        for (var b : mapping.mappings) if (vmcBone.equals(b.vmcBone)) { bm = b; break; }
        if (bm == null) {
            bm = new VMCURDFMapping.BoneMapping();
            bm.vmcBone = vmcBone;
            mapping.mappings.add(bm);
        }
        bm.joints.add(jc); // ★ 순서가 곧 체인 순서
    }

    /* ---------------- 매핑 위저드 ---------------- */
    private static float axisProjectedAngle(Quaternionf q, float ax, float ay, float az) {
        float w=q.w(), x=q.x(), y=q.y(), z=q.z();
        float dot = x*ax + y*ay + z*az;
        return (float)(2.0 * Math.atan2(dot, w));
    }

    private static final class Stats {
        static class Fit { float m,b,r; }
        static Fit fit(float[] x, float[] y) {
            int n=Math.min(x.length,y.length);
            double sx=0,sy=0,sxx=0,syy=0,sxy=0;
            for(int i=0;i<n;i++){ sx+=x[i]; sy+=y[i]; sxx+=x[i]*x[i]; syy+=y[i]*y[i]; sxy+=x[i]*y[i]; }
            double denom = n*sxx - sx*sx;
            Fit f=new Fit();
            if (Math.abs(denom)<1e-9){ f.m=0; f.b=(float)(sy/n); f.r=0; return f; }
            f.m=(float)((n*sxy - sx*sy)/denom);
            f.b=(float)((sy - f.m*sx)/n);
            double cov = sxy/n - (sx/n)*(sy/n);
            double vx  = sxx/n - (sx/n)*(sx/n);
            double vy  = syy/n - (sy/n)*(sy/n);
            f.r=(float)(cov/(Math.sqrt(vx)*Math.sqrt(vy)+1e-9));
            return f;
        }
    }

    private final class MappingWizard {
        final String vmcBone;
        final Object renderer;
        final List<Float> xs=new ArrayList<>(), ys=new ArrayList<>();
        URDFJoint currentJoint; float[] useAxis;

        MappingWizard(String vmcBone, Object renderer){ this.vmcBone=vmcBone; this.renderer=renderer; }

        void beginJoint(String jointName) {
            currentJoint = findJoint(jointName);
            useAxis = null;
            if (currentJoint != null && currentJoint.axis != null && currentJoint.axis.xyz != null) {
                float ax=currentJoint.axis.xyz.x, ay=currentJoint.axis.xyz.y, az=currentJoint.axis.xyz.z;
                float n=(float)Math.sqrt(ax*ax+ay*ay+az*az);
                if (n>=1e-6) useAxis = new float[]{ax/n, ay/n, az/n};
            }
            xs.clear(); ys.clear();
        }

        void onTick(Object vmcTransform, float jointAngleRad) {
            if (currentJoint==null || vmcTransform==null) return;
            Quaternionf q = readQuat(vmcTransform); if (q==null) return;

            float x;
            if (useAxis!=null) {
                x = axisProjectedAngle(q, useAxis[0], useAxis[1], useAxis[2]);
            } else {
                Vector3f e = new Vector3f(); q.getEulerAnglesZYX(e); // (x=roll,y=pitch,z=yaw)
                x = e.y; // 축 정보 없으면 일단 pitch 성분
            }
            xs.add(x);
            ys.add(jointAngleRad);
        }

        VMCURDFMapping.JointComponent finalizeStep() {
            if (currentJoint==null || xs.size()<20) return null;
            float[] x=new float[xs.size()], y=new float[ys.size()];
            for(int i=0;i<x.length;i++){ x[i]=xs.get(i); y[i]=ys.get(i); }
            Stats.Fit fit = Stats.fit(x,y);
            if (Math.abs(fit.r) < 0.6f) return null;

            VMCURDFMapping.JointComponent jc = new VMCURDFMapping.JointComponent();
            jc.jointName = currentJoint.name;
            if (useAxis != null) {
                jc.mode = VMCURDFMapping.ExtractMode.AXIS_PROJ;
                jc.axis = useAxis;
                // 의미 레이블 힌트(이름 기반)
                String n = currentJoint.name.toLowerCase(Locale.ROOT);
                if (n.contains("yaw")||n.contains("_z")) jc.component=VMCURDFMapping.RotationComponent.YAW;
                else if (n.contains("roll")||n.contains("_x")) jc.component=VMCURDFMapping.RotationComponent.ROLL;
                else jc.component=VMCURDFMapping.RotationComponent.PITCH;
            } else {
                jc.mode = VMCURDFMapping.ExtractMode.EULER;
                jc.eulerOrder = VMCURDFMapping.EulerOrder.ZYX;
                jc.component = VMCURDFMapping.RotationComponent.PITCH;
            }
            jc.multiplier = fit.m;
            jc.offset     = fit.b;
            return jc;
        }

        Object pickVmcTransform(Object vmcState) {
            Map<String,Object> m = reflectCollectBoneMap(vmcState);
            return m.getOrDefault(vmcBone, null);
        }

        float readJointAngleRad(String name) {
            URDFRobotModel model = reflectGetRobotModel(renderer);
            if (model==null || model.joints==null) return 0f;
            for (URDFJoint j : model.joints) if (name.equals(j.name)) return j.currentPosition;
            return 0f;
        }

        URDFJoint findJoint(String n){
            URDFRobotModel m=reflectGetRobotModel(renderer);
            if(m==null) return null;
            for(URDFJoint j:m.joints) if(n.equals(j.name)) return j;
            return null;
        }
    }

    /* ---------------- 유틸 ---------------- */
    private static int clamp(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
    private static Vector3f readPos(Object tr) {
        try {
            Object p = tr.getClass().getField("position").get(tr);
            float x = (float)p.getClass().getMethod("x").invoke(p);
            float y = (float)p.getClass().getMethod("y").invoke(p);
            float z = (float)p.getClass().getMethod("z").invoke(p);
            return new Vector3f(x,y,z);
        } catch (Throwable ignored) { return null; }
    }
    private static Quaternionf readQuat(Object tr) {
        try {
            Object r = tr.getClass().getField("rotation").get(tr);
            float x = (float)r.getClass().getMethod("x").invoke(r);
            float y = (float)r.getClass().getMethod("y").invoke(r);
            float z = (float)r.getClass().getMethod("z").invoke(r);
            float w = (float)r.getClass().getMethod("w").invoke(r);
            return new Quaternionf(x,y,z,w);
        } catch (Throwable ignored) { return null; }
    }
}
