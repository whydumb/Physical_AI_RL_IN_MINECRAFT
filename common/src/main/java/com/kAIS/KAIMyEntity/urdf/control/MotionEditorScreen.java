package com.kAIS.KAIMyEntity.urdf.control;

import com.kAIS.KAIMyEntity.rl.RLEnvironmentCore;
import com.kAIS.KAIMyEntity.rl.RLVMDIntegration;
import com.kAIS.KAIMyEntity.urdf.URDFModelOpenGLWithSTL;
import com.kAIS.KAIMyEntity.urdf.URDFModelOpenGLWithSTL.JointControlSource;
import com.kAIS.KAIMyEntity.urdf.vmd.VMDLoader;
import com.kAIS.KAIMyEntity.rl.RLEnvironmentCore.AgentMode;
import net.minecraft.client.Minecraft;
import net.minecraft.client.gui.GuiGraphics;
import net.minecraft.client.gui.screens.Screen;
import net.minecraft.network.chat.Component;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.io.File;
import java.util.*;

public final class MotionEditorScreen {
    private static final Logger logger = LogManager.getLogger();

    private MotionEditorScreen() {}

    // =========================
    //  외부에서 호출하는 진입점
    // =========================

    public static void open(URDFModelOpenGLWithSTL renderer) {
        Minecraft.getInstance().setScreen(new RLControlGUI(Minecraft.getInstance().screen, renderer));
    }

    /**
     * 매 틱마다 호출되는 정적 tick.
     * 여기서 VMDPlayer가 무조건 관절을 덮어쓰지 않도록,
     * RL이 관절을 제어 중인지 확인해서 분기한다.
     */
    public static void tick(URDFModelOpenGLWithSTL renderer) {
        if (renderer == null) return;

        RLEnvironmentCore env = RLEnvironmentCore.getInstance();

        boolean rlActive =
                env != null
                        && env.isInitialized()
                        && env.isTraining()
                        && env.getAgentMode() != AgentMode.MANUAL;

        // 디버그 옵션: RL이 죽었든 말든, 최소한 “뭔가 움직이는 거”는 보이게 하고 싶다면 true
        final boolean FORCE_SHOW_VMD = false;   // 필요하면 true로 바꿔서 테스트

        if (!rlActive || FORCE_SHOW_VMD) {
            // RL이 관절 안 잡고 있으면: VMD가 진짜로 관절을 움직인다 (preview + target)
            VMDPlayer.getInstance().tick(renderer, 1f / 20f, true, true);
        } else {
            // RL이 관절을 제어 중이면:
            //  - VMD는 ghost 용으로 preview만 뿌린다 (target은 RL이 담당)
            VMDPlayer.getInstance().tick(renderer, 1f / 20f, true, false);
        }
    }

    // =========================
    //  RL 제어 GUI
    // =========================

    public static class RLControlGUI extends Screen {
        private static final Logger logger = LogManager.getLogger();

        private static final int COL_BG_DARK = 0xF0181818;
        private static final int COL_BG_PANEL = 0xF0252525;
        private static final int COL_BG_HEADER = 0xF0353535;
        private static final int COL_BG_ITEM = 0xF02A2A2A;
        private static final int COL_BG_HOVER = 0xF0404040;
        private static final int COL_BG_ACTIVE = 0xF0505050;
        private static final int COL_BORDER = 0xFF3A3A3A;
        private static final int COL_TEXT = 0xFFE8E8E8;
        private static final int COL_TEXT_DIM = 0xFF888888;
        private static final int COL_ACCENT = 0xFF4CAF50;
        private static final int COL_ACCENT_HOVER = 0xFF66BB6A;
        private static final int COL_WARNING = 0xFFFF9800;
        private static final int COL_ERROR = 0xFFF44336;
        private static final int COL_INFO = 0xFF2196F3;

        private static final int PANEL_WIDTH = 280;
        private static final int PANEL_MARGIN = 8;
        private static final int PADDING = 10;
        private static final int LINE_H = 18;
        private static final int HEADER_H = 22;
        private static final int BTN_H = 18;
        private static final int SECTION_GAP = 6;

        private final Screen parent;
        private final URDFModelOpenGLWithSTL renderer;
        private RLEnvironmentCore rlEnv;

        private SimState simState = SimState.STOPPED;
        private float simTime = 0f;
        private float simSpeed = 1.0f;
        private int stepCount = 0;

        private boolean serverRunning = false;
        private boolean pythonConnected = false;
        private String serverPort = "5555";
        private float episodeReward = 0f;
        private float lastReward = 0f;

        private boolean simExpanded = true;
        private boolean rlExpanded = true;
        private boolean jointExpanded = true;
        private boolean sensorExpanded = false;

        private final LinkedHashMap<String, JointInfo> joints = new LinkedHashMap<>();
        private String selectedJoint = null;
        private String draggingJoint = null;

        private final List<LogEntry> logs = new ArrayList<>();
        private static final int MAX_LOGS = 100;

        private int jointScrollOffset = 0;
        private static final int MAX_VISIBLE_JOINTS = 6;

        private boolean editingPort = false;
        private StringBuilder portBuffer = new StringBuilder("5555");

        private final Map<String, int[]> buttonBounds = new HashMap<>();
        private final Map<String, int[]> sliderBounds = new HashMap<>();

        public RLControlGUI(Screen parent, URDFModelOpenGLWithSTL renderer) {
            super(Component.literal("RL Control Panel"));
            this.parent = parent;
            this.renderer = renderer;

            // RL 환경 초기화는 헬퍼를 통해 수행 (이미 초기화되어 있으면 재초기화 방지)
            this.rlEnv = RLVMDIntegration.initEnvironment(renderer);
            if (this.rlEnv != null) {
                this.rlEnv.setLogCallback(msg -> log(LogLevel.INFO, msg));
                // 기본 모드를 LEARNING으로 설정
                this.rlEnv.setAgentMode(AgentMode.LEARNING);
                log(LogLevel.INFO, "Agent mode set to LEARNING");
            }

            loadJoints();
            log(LogLevel.INFO, "RL Control Panel opened");
        }

        private void loadJoints() {
            joints.clear();
            var robot = renderer.getRobotModel();
            if (robot == null || robot.joints == null) {
                log(LogLevel.WARN, "No robot model loaded");
                return;
            }
            for (var joint : robot.joints) {
                if (joint.isMovable()) {
                    float lower = (joint.limit != null) ? joint.limit.lower : -3.14f;
                    float upper = (joint.limit != null) ? joint.limit.upper : 3.14f;
                    joints.put(joint.name, new JointInfo(joint.name, joint.currentPosition, lower, upper));
                }
            }
            log(LogLevel.INFO, "Loaded " + joints.size() + " joints");
        }

        @Override
        protected void init() {
            super.init();
        }

        @Override
        public void render(GuiGraphics g, int mouseX, int mouseY, float delta) {
            buttonBounds.clear();
            sliderBounds.clear();

            int panelX = this.width - PANEL_WIDTH - PANEL_MARGIN;
            int panelY = PANEL_MARGIN;
            int panelH = this.height - PANEL_MARGIN * 2 - 80;

            renderMainPanel(g, panelX, panelY, PANEL_WIDTH, panelH, mouseX, mouseY);
            renderLogPanel(g, PANEL_MARGIN, this.height - 75, this.width - PANEL_WIDTH - PANEL_MARGIN * 3, 65);
            renderStatusBar(g);
            renderHints(g);
        }

        private void renderMainPanel(GuiGraphics g, int x, int y, int w, int h, int mx, int my) {
            fillRect(g, x, y, w, h, COL_BG_PANEL);
            drawBorder(g, x, y, w, h, COL_BORDER);

            fillRect(g, x, y, w, HEADER_H, COL_BG_DARK);
            drawCenteredString(g, "RL Control Panel", x + w / 2, y + 6, COL_TEXT);

            int cy = y + HEADER_H + PADDING;

            cy = renderSectionHeader(g, x, cy, w, "Simulation", simExpanded, "sim_toggle", mx, my);
            if (simExpanded) {
                cy = renderSimulationSection(g, x + PADDING, cy, w - PADDING * 2, mx, my);
                cy += SECTION_GAP;
            }

            cy = renderSectionHeader(g, x, cy, w, "RL Environment", rlExpanded, "rl_toggle", mx, my);
            if (rlExpanded) {
                cy = renderRLSection(g, x + PADDING, cy, w - PADDING * 2, mx, my);
                cy += SECTION_GAP;
            }

            String jointTitle = String.format("Joints (%d)", joints.size());
            cy = renderSectionHeader(g, x, cy, w, jointTitle, jointExpanded, "joint_toggle", mx, my);
            if (jointExpanded) {
                cy = renderJointSection(g, x + PADDING, cy, w - PADDING * 2, mx, my);
                cy += SECTION_GAP;
            }

            cy = renderSectionHeader(g, x, cy, w, "Sensors", sensorExpanded, "sensor_toggle", mx, my);
            if (sensorExpanded) {
                cy = renderSensorSection(g, x + PADDING, cy, w - PADDING * 2, mx, my);
            }
        }

        private int renderSectionHeader(GuiGraphics g, int x, int y, int w, String title, boolean expanded, String id, int mx, int my) {
            int h = HEADER_H - 2;
            boolean hover = isInside(mx, my, x + 4, y, w - 8, h);
            fillRect(g, x + 4, y, w - 8, h, hover ? COL_BG_HOVER : COL_BG_HEADER);
            String arrow = expanded ? "[-] " : "[+] ";
            g.drawString(font, arrow + title, x + PADDING, y + 5, hover ? COL_TEXT : COL_TEXT_DIM, false);
            buttonBounds.put(id, new int[]{x + 4, y, w - 8, h});
            return y + h + 4;
        }

        private int renderSimulationSection(GuiGraphics g, int x, int y, int w, int mx, int my) {
            int btnW = 45;
            int gap = 6;
            int bx = x;

            bx = renderButton(g, bx, y, btnW, BTN_H, ">", "sim_play", mx, my, simState == SimState.RUNNING);
            bx = renderButton(g, bx + gap, y, btnW, BTN_H, "||", "sim_pause", mx, my, simState == SimState.PAUSED);
            bx = renderButton(g, bx + gap, y, btnW, BTN_H, "R", "sim_reset", mx, my, false);
            bx = renderButton(g, bx + gap, y, btnW, BTN_H, ">|", "sim_step", mx, my, false);
            y += BTN_H + 8;

            renderButton(g, x, y, 90, BTN_H, "Load VMD", "load_vmd", mx, my, false);
            g.drawString(font, String.format("%.1fx", simSpeed), x + 100, y + 4, COL_TEXT_DIM, false);
            y += BTN_H + 8;

            g.drawString(font, String.format("Time: %.2fs", simTime), x, y, COL_TEXT_DIM, false);
            g.drawString(font, String.format("Steps: %d", stepCount), x + 100, y, COL_TEXT_DIM, false);
            y += LINE_H;

            var vmd = VMDPlayer.getInstance();
            String vmdStatus;
            int vmdColor;
            if (!vmd.hasMotion()) {
                vmdStatus = "No Motion";
                vmdColor = COL_TEXT_DIM;
            } else if (vmd.isPlaying()) {
                vmdStatus = "Playing";
                vmdColor = COL_ACCENT;
            } else {
                vmdStatus = "Loaded";
                vmdColor = COL_WARNING;
            }
            g.drawString(font, "VMD: " + vmdStatus, x, y, vmdColor, false);

            if (vmd.hasMotion()) {
                var st = vmd.getStatus();
                g.drawString(font, String.format("%.1f/%.1fs", st.currentTime(), st.duration()), x + 100, y, COL_TEXT_DIM, false);
            }
            return y + LINE_H;
        }

        private int renderRLSection(GuiGraphics g, int x, int y, int w, int mx, int my) {
            if (rlEnv == null || !rlEnv.isInitialized()) {
                g.drawString(font, "RL Env: Not initialized", x, y, COL_WARNING, false);
                return y + LINE_H;
            }

            // 모드 선택 버튼
            g.drawString(font, "Mode:", x, y + 4, COL_TEXT_DIM, false);
            int bx = x + 40;
            var mode = rlEnv.getAgentMode();

            bx = renderButton(g, bx, y, 35, BTN_H, "MAN", "mode_manual", mx, my,
                    mode == AgentMode.MANUAL);
            bx = renderButton(g, bx + 2, y, 35, BTN_H, "RND", "mode_random", mx, my,
                    mode == AgentMode.RANDOM);
            bx = renderButton(g, bx + 2, y, 35, BTN_H, "LRN", "mode_learn", mx, my,
                    mode == AgentMode.LEARNING);
            bx = renderButton(g, bx + 2, y, 35, BTN_H, "IMI", "mode_imitate", mx, my,
                    mode == AgentMode.IMITATION);
            y += BTN_H + 6;

            // 학습 상태
            boolean training = rlEnv.isTraining();
            g.drawString(font, "Status:", x, y + 4, COL_TEXT_DIM, false);
            String statusText = training ? "Training" : "Stopped";
            int statusColor = training ? COL_ACCENT : COL_TEXT_DIM;
            g.drawString(font, statusText, x + 50, y + 4, statusColor, false);

            // Start/Stop 버튼
            String btnText = training ? "Stop" : "Start";
            renderButton(g, x + 110, y, 50, BTN_H, btnText, "training_toggle", mx, my, training);
            y += BTN_H + 6;

            // 에피소드 정보
            var stats = rlEnv.getStats();
            g.drawString(font, String.format("Episode: %d", rlEnv.getEpisodeCount()), x, y, COL_TEXT_DIM, false);
            g.drawString(font, String.format("Step: %d", rlEnv.getStepCount()), x + 90, y, COL_TEXT_DIM, false);
            y += LINE_H;

            // 보상 정보
            int rewardColor = lastReward > 0 ? COL_ACCENT : (lastReward < 0 ? COL_ERROR : COL_TEXT);
            g.drawString(font, String.format("Reward: %.2f", rlEnv.getEpisodeReward()), x, y, COL_TEXT_DIM, false);
            g.drawString(font, String.format("(%.4f)", lastReward), x + 85, y, rewardColor, false);
            y += LINE_H;

            // 통계
            g.drawString(font, String.format("Avg: %.2f  Best: %.2f",
                    stats.getAverageReward(), stats.getBestReward()), x, y, COL_TEXT_DIM, false);
            y += LINE_H;

            // 건강 상태
            boolean healthy = rlEnv.isHealthy();
            g.drawString(font, "Health: ", x, y, COL_TEXT_DIM, false);
            g.drawString(font, healthy ? "OK" : "FALLEN", x + 45, y, healthy ? COL_ACCENT : COL_ERROR, false);
            y += LINE_H;

            // 공간 정보
            g.drawString(font, String.format("Obs: %d  Act: %d",
                    rlEnv.getObservationDim(), rlEnv.getActionDim()), x, y, COL_TEXT_DIM, false);

            return y + LINE_H;
        }

        private int renderJointSection(GuiGraphics g, int x, int y, int w, int mx, int my) {
            if (joints.isEmpty()) {
                g.drawString(font, "No movable joints", x, y, COL_TEXT_DIM, false);
                return y + LINE_H;
            }

            int scrollBtnW = 30;
            if (joints.size() > MAX_VISIBLE_JOINTS) {
                renderButton(g, x + w - scrollBtnW * 2 - 4, y - 2, scrollBtnW, 14, "UP", "joint_scroll_up", mx, my, false);
                renderButton(g, x + w - scrollBtnW, y - 2, scrollBtnW, 14, "DN", "joint_scroll_down", mx, my, false);
            }
            y += 2;

            List<String> jointNames = new ArrayList<>(joints.keySet());
            int endIdx = Math.min(jointScrollOffset + MAX_VISIBLE_JOINTS, jointNames.size());

            for (int i = jointScrollOffset; i < endIdx; i++) {
                String name = jointNames.get(i);
                JointInfo ji = joints.get(name);
                String displayName = name.length() > 12 ? name.substring(0, 10) + ".." : name;
                boolean isSelected = name.equals(selectedJoint);
                int nameColor = isSelected ? COL_INFO : COL_TEXT_DIM;
                g.drawString(font, displayName, x, y + 3, nameColor, false);

                int sliderX = x + 85;
                int sliderW = w - 130;
                y = renderSlider(g, sliderX, y, sliderW, ji, name, mx, my);
            }

            if (joints.size() > MAX_VISIBLE_JOINTS) {
                g.drawString(font, String.format("(%d/%d shown)", Math.min(MAX_VISIBLE_JOINTS, joints.size()), joints.size()), x, y, COL_TEXT_DIM, false);
                y += LINE_H;
            }
            return y;
        }

        private int renderSlider(GuiGraphics g, int x, int y, int w, JointInfo ji, String jointId, int mx, int my) {
            int h = 14;
            int handleW = 8;
            boolean hover = isInside(mx, my, x, y, w, h);
            boolean active = jointId.equals(draggingJoint);

            fillRect(g, x, y + 2, w, h - 4, COL_BG_ITEM);

            float norm = (ji.value - ji.min) / (ji.max - ji.min);
            norm = Math.max(0, Math.min(1, norm));
            int handleX = x + (int) (norm * (w - handleW));

            fillRect(g, x, y + 2, handleX - x + handleW / 2, h - 4, 0x60000000 | (COL_ACCENT & 0xFFFFFF));

            int handleColor = active ? COL_ACCENT_HOVER : (hover ? COL_ACCENT : COL_BG_HOVER);
            fillRect(g, handleX, y, handleW, h, handleColor);

            g.drawString(font, String.format("%.2f", ji.value), x + w + 4, y + 2, COL_TEXT, false);
            sliderBounds.put("slider_" + jointId, new int[]{x, y, w, h});
            return y + LINE_H;
        }

        private int renderSensorSection(GuiGraphics g, int x, int y, int w, int mx, int my) {
            g.drawString(font, "IMU:", x, y, COL_TEXT_DIM, false);
            g.drawString(font, "[0.0, -9.8, 0.0]", x + 35, y, COL_TEXT, false);
            y += LINE_H;

            g.drawString(font, "Contact:", x, y, COL_TEXT_DIM, false);
            g.drawString(font, "L:ON R:ON", x + 55, y, COL_ACCENT, false);
            y += LINE_H;

            g.drawString(font, "Force:", x, y, COL_TEXT_DIM, false);
            g.drawString(font, "0.0 N", x + 45, y, COL_TEXT, false);
            return y + LINE_H;
        }

        private int renderButton(GuiGraphics g, int x, int y, int w, int h, String text, String id, int mx, int my, boolean active) {
            boolean hover = isInside(mx, my, x, y, w, h);
            int bgColor;
            if (active) {
                bgColor = COL_ACCENT;
            } else if (hover) {
                bgColor = COL_BG_HOVER;
            } else {
                bgColor = COL_BG_ITEM;
            }
            fillRect(g, x, y, w, h, bgColor);
            drawBorder(g, x, y, w, h, hover ? COL_ACCENT : COL_BORDER);
            int textColor = active ? 0xFFFFFFFF : (hover ? COL_TEXT : COL_TEXT_DIM);
            drawCenteredString(g, text, x + w / 2, y + (h - 8) / 2, textColor);
            buttonBounds.put(id, new int[]{x, y, w, h});
            return x + w;
        }

        private void renderLogPanel(GuiGraphics g, int x, int y, int w, int h) {
            fillRect(g, x, y, w, h, COL_BG_PANEL);
            drawBorder(g, x, y, w, h, COL_BORDER);

            fillRect(g, x, y, w, 16, COL_BG_DARK);
            g.drawString(font, "Console", x + 6, y + 4, COL_TEXT, false);

            int logY = y + 20;
            int maxLines = (h - 24) / 10;
            int start = Math.max(0, logs.size() - maxLines);

            for (int i = start; i < logs.size(); i++) {
                LogEntry entry = logs.get(i);
                String prefix = switch (entry.level) {
                    case ERROR -> "[E] ";
                    case WARN -> "[W] ";
                    case INFO -> "> ";
                    case DEBUG -> "[D] ";
                };
                int color = switch (entry.level) {
                    case ERROR -> COL_ERROR;
                    case WARN -> COL_WARNING;
                    case INFO -> COL_TEXT;
                    case DEBUG -> COL_TEXT_DIM;
                };
                g.drawString(font, prefix + entry.msg, x + 6, logY, color, false);
                logY += 10;
            }
        }

        private void renderStatusBar(GuiGraphics g) {
            int y = 0;
            int h = 18;
            fillRect(g, 0, y, this.width, h, COL_BG_DARK);

            String stateStr = switch (simState) {
                case RUNNING -> "Running";
                case PAUSED -> "Paused";
                case STOPPED -> "Stopped";
            };
            int stateColor = switch (simState) {
                case RUNNING -> COL_ACCENT;
                case PAUSED -> COL_WARNING;
                case STOPPED -> COL_TEXT_DIM;
            };
            g.drawString(font, stateStr, 10, 5, stateColor, false);

            if (renderer != null && renderer.hasManualJointLocks()) {
                g.drawString(font, "Manual Override", 80, 5, COL_WARNING, false);
            }

            int fps = Minecraft.getInstance().getFps();
            g.drawString(font, "FPS: " + fps, 180, 5, COL_TEXT_DIM, false);
        }

        private void renderHints(GuiGraphics g) {
            int hx = this.width - PANEL_WIDTH - PANEL_MARGIN - 180;
            int hy = PANEL_MARGIN + 30;

            fillRect(g, hx, hy, 170, 50, COL_BG_PANEL);
            drawBorder(g, hx, hy, 170, 50, COL_BORDER);

            g.drawString(font, "Move with W, A, S and D", hx + 8, hy + 8, COL_TEXT, false);
            g.drawString(font, "Jump with Space", hx + 8, hy + 22, COL_TEXT, false);
            g.drawString(font, "ESC to close", hx + 8, hy + 36, COL_TEXT_DIM, false);
        }

        private void fillRect(GuiGraphics g, int x, int y, int w, int h, int color) {
            g.fill(x, y, x + w, y + h, color);
        }

        private void drawBorder(GuiGraphics g, int x, int y, int w, int h, int color) {
            g.fill(x, y, x + w, y + 1, color);
            g.fill(x, y + h - 1, x + w, y + h, color);
            g.fill(x, y, x + 1, y + h, color);
            g.fill(x + w - 1, y, x + w, y + h, color);
        }

        private void drawCenteredString(GuiGraphics g, String text, int cx, int y, int color) {
            int tw = font.width(text);
            g.drawString(font, text, cx - tw / 2, y, color, false);
        }

        private boolean isInside(int mx, int my, int x, int y, int w, int h) {
            return mx >= x && mx < x + w && my >= y && my < y + h;
        }

        private void log(LogLevel level, String msg) {
            logs.add(new LogEntry(level, msg));
            if (logs.size() > MAX_LOGS) logs.remove(0);
            logger.info("[{}] {}", level, msg);
        }

        @Override
        public boolean mouseClicked(double mouseX, double mouseY, int button) {
            int mx = (int) mouseX;
            int my = (int) mouseY;

            for (var entry : buttonBounds.entrySet()) {
                int[] b = entry.getValue();
                if (isInside(mx, my, b[0], b[1], b[2], b[3])) {
                    handleButtonClick(entry.getKey());
                    return true;
                }
            }

            for (var entry : sliderBounds.entrySet()) {
                int[] b = entry.getValue();
                if (isInside(mx, my, b[0], b[1], b[2], b[3])) {
                    String jointName = entry.getKey().replace("slider_", "");
                    draggingJoint = jointName;
                    selectedJoint = jointName;
                    updateSliderValue(jointName, mx, b[0], b[2]);
                    return true;
                }
            }

            if (editingPort) {
                int[] portBox = buttonBounds.get("port_input");
                if (portBox != null && !isInside(mx, my, portBox[0], portBox[1], portBox[2], portBox[3])) {
                    finishPortEdit();
                }
            }
            return super.mouseClicked(mouseX, mouseY, button);
        }

        @Override
        public boolean mouseReleased(double mouseX, double mouseY, int button) {
            draggingJoint = null;
            return super.mouseReleased(mouseX, mouseY, button);
        }

        @Override
        public boolean mouseDragged(double mouseX, double mouseY, int button, double dragX, double dragY) {
            if (draggingJoint != null) {
                int[] b = sliderBounds.get("slider_" + draggingJoint);
                if (b != null) {
                    updateSliderValue(draggingJoint, (int) mouseX, b[0], b[2]);
                    return true;
                }
            }
            return super.mouseDragged(mouseX, mouseY, button, dragX, dragY);
        }

        @Override
        public boolean mouseScrolled(double mouseX, double mouseY, double scrollX, double scrollY) {
            int panelX = this.width - PANEL_WIDTH - PANEL_MARGIN;
            if (mouseX >= panelX && jointExpanded) {
                if (scrollY > 0 && jointScrollOffset > 0) {
                    jointScrollOffset--;
                    return true;
                } else if (scrollY < 0 && jointScrollOffset < joints.size() - MAX_VISIBLE_JOINTS) {
                    jointScrollOffset++;
                    return true;
                }
            }
            return super.mouseScrolled(mouseX, mouseY, scrollX, scrollY);
        }

        @Override
        public boolean keyPressed(int keyCode, int scanCode, int modifiers) {
            if (editingPort) {
                if (keyCode == 259 && portBuffer.length() > 0) {
                    portBuffer.deleteCharAt(portBuffer.length() - 1);
                    return true;
                } else if (keyCode == 257 || keyCode == 335) {
                    finishPortEdit();
                    return true;
                } else if (keyCode == 256) {
                    editingPort = false;
                    portBuffer = new StringBuilder(serverPort);
                    return true;
                }
                return true;
            }
            if (keyCode == 256) {
                onClose();
                return true;
            }
            return super.keyPressed(keyCode, scanCode, modifiers);
        }

        @Override
        public boolean charTyped(char chr, int modifiers) {
            if (editingPort && Character.isDigit(chr) && portBuffer.length() < 5) {
                portBuffer.append(chr);
                return true;
            }
            return super.charTyped(chr, modifiers);
        }

        private void handleButtonClick(String id) {
            switch (id) {
                case "sim_toggle" -> simExpanded = !simExpanded;
                case "rl_toggle" -> rlExpanded = !rlExpanded;
                case "joint_toggle" -> jointExpanded = !jointExpanded;
                case "sensor_toggle" -> sensorExpanded = !sensorExpanded;
                case "sim_play" -> play();
                case "sim_pause" -> pause();
                case "sim_reset" -> reset();
                case "sim_step" -> step();
                case "load_vmd" -> openVmdDialog();
                case "server_toggle" -> toggleServer();
                case "port_input" -> startPortEdit();
                case "joint_scroll_up" -> {
                    if (jointScrollOffset > 0) jointScrollOffset--;
                }
                case "joint_scroll_down" -> {
                    if (jointScrollOffset < joints.size() - MAX_VISIBLE_JOINTS) jointScrollOffset++;
                }
                case "mode_manual" -> {
                    if (rlEnv != null) rlEnv.setAgentMode(AgentMode.MANUAL);
                }
                case "mode_random" -> {
                    if (rlEnv != null) rlEnv.setAgentMode(AgentMode.RANDOM);
                }
                case "mode_learn" -> {
                    if (rlEnv != null) rlEnv.setAgentMode(AgentMode.LEARNING);
                }
                case "mode_imitate" -> {
                    if (rlEnv != null) rlEnv.setAgentMode(AgentMode.IMITATION);
                }
                case "training_toggle" -> {
                    if (rlEnv != null) {
                        if (rlEnv.isTraining()) {
                            rlEnv.stopTraining();
                        } else {
                            rlEnv.startTraining(rlEnv.getAgentMode());
                        }
                    }
                }
            }
        }

        private void updateSliderValue(String jointName, int mouseX, int sliderX, int sliderW) {
            JointInfo ji = joints.get(jointName);
            if (ji == null) return;
            float norm = (float) (mouseX - sliderX) / sliderW;
            norm = Math.max(0, Math.min(1, norm));
            float value = ji.min + norm * (ji.max - ji.min);
            ji.value = value;
            if (rlEnv != null && rlEnv.isInitialized() && rlEnv.isTraining()) {
                rlEnv.stopTraining();
                rlEnv.setAgentMode(AgentMode.MANUAL);
                log(LogLevel.WARN, "RL training paused: manual override on " + jointName);
            }
            renderer.setManualJointTarget(jointName, value);
        }

        private void startPortEdit() {
            editingPort = true;
            portBuffer = new StringBuilder(serverPort);
        }

        private void finishPortEdit() {
            editingPort = false;
            if (portBuffer.length() > 0) {
                serverPort = portBuffer.toString();
            }
        }

        private void play() {
            simState = SimState.RUNNING;
            VMDPlayer.getInstance().play();
            log(LogLevel.INFO, "Simulation started");
        }

        private void pause() {
            simState = SimState.PAUSED;
            VMDPlayer.getInstance().pause();
            log(LogLevel.INFO, "Simulation paused");
        }

        private void reset() {
            simState = SimState.STOPPED;
            simTime = 0f;
            stepCount = 0;
            episodeReward = 0f;
            lastReward = 0f;

            VMDPlayer.getInstance().stop();
            loadJoints();
            if (renderer != null) {
                renderer.clearManualJointLocks();
            }

            // RL 환경 리셋
            if (rlEnv != null && rlEnv.isInitialized()) {
                rlEnv.reset();
            }

            log(LogLevel.INFO, "Environment reset");
        }

        private void step() {
            if (simState == SimState.RUNNING) return;

            // RL 환경 수동 스텝
            if (rlEnv != null && rlEnv.isInitialized()) {
                rlEnv.manualStep();
                stepCount = rlEnv.getStepCount();
                episodeReward = rlEnv.getEpisodeReward();
                lastReward = rlEnv.getLastReward();
            }

            simTime += 0.05f;
        }

        /**
         * VMD 로딩:
         *  - 게임폴더/motions/walk.vmd 를 기준으로 로드
         *  - VMDPlayer에는 시각화/재생용으로 등록
         *  - RL 환경에는 referenceMotion으로 등록하고, Config는 간단히 조정
         */
        private void openVmdDialog() {
            if (renderer == null) {
                log(LogLevel.WARN, "[RLControlGUI] renderer is null, cannot load VMD");
                return;
            }

            var model = renderer.getRobotModel();
            if (model == null) {
                log(LogLevel.WARN, "[RLControlGUI] robot model is null, cannot load VMD");
            }

            // <게임폴더>/motions/walk.vmd
            File gameDir = Minecraft.getInstance().gameDirectory;
            File baseDir = new File(gameDir, "motions");
            File vmdFile = new File(baseDir, "walk.vmd");

            log(LogLevel.INFO, "[RLControlGUI] Trying to load VMD: " + vmdFile.getAbsolutePath());
            if (!vmdFile.exists()) {
                log(LogLevel.ERROR, "[RLControlGUI] VMD file does not exist: " + vmdFile.getAbsolutePath());
                return;
            }

            try {
                // 1) VMD → URDFMotion
                URDFMotion motion;
                if (model != null) {
                    motion = VMDLoader.load(vmdFile, model);
                } else {
                    motion = VMDLoader.load(vmdFile);
                }

                if (motion == null || motion.keys == null || motion.keys.isEmpty()) {
                    log(LogLevel.ERROR, "[RLControlGUI] VMD load failed or no keys: " + vmdFile.getName());
                    return;
                }

                log(LogLevel.INFO, String.format(
                        "[RLControlGUI] VMD loaded: name=%s, keys=%d",
                        motion.name, motion.keys.size()
                ));

                // 2) VMD 재생기에 등록
                MotionEditorScreen.VMDPlayer.getInstance().loadMotion(motion);

                // 3) RL referenceMotion + Config
                if (rlEnv != null) {
                    rlEnv.setReferenceMotion(motion);

                    RLEnvironmentCore.Config cfg = rlEnv.getConfig();
                    cfg.poseMatchWeight  = 0.10f;
                    cfg.targetSpeed      = 0.20f;
                    cfg.speedMatchWeight = 0.50f;
                    cfg.actionMode       = RLEnvironmentCore.ActionMode.POSITION;

                    log(LogLevel.INFO, "[RLControlGUI] Reference motion set for RL: " + motion.name);
                } else {
                    log(LogLevel.WARN, "[RLControlGUI] rlEnv is null, cannot set referenceMotion");
                }

            } catch (Exception e) {
                logger.error("[RLControlGUI] Failed to load VMD", e);
                log(LogLevel.ERROR, "Failed to load VMD: " + e.getMessage());
            }
        }

        private void toggleServer() {
            serverRunning = !serverRunning;
            if (serverRunning) {
                log(LogLevel.INFO, "Server started on port " + serverPort);
            } else {
                log(LogLevel.INFO, "Server stopped");
                pythonConnected = false;
            }
        }

        @Override
        public void tick() {
            super.tick();

            // RL 환경 틱
            if (rlEnv != null && rlEnv.isInitialized()) {
                rlEnv.tick(0.05f); // 50ms per tick

                // 상태 동기화
                episodeReward = rlEnv.getEpisodeReward();
                lastReward = rlEnv.getLastReward();
                stepCount = rlEnv.getStepCount();
            }

            if (simState == SimState.RUNNING) {
                simTime += 0.05f * simSpeed;
                stepCount++;
            }

            // 관절 상태 동기화 (렌더러 → GUI)
            var robot = renderer.getRobotModel();
            if (robot != null && robot.joints != null) {
                for (var joint : robot.joints) {
                    if (joint.isMovable() && joints.containsKey(joint.name)) {
                        if (!joint.name.equals(draggingJoint)) {
                            joints.get(joint.name).value = joint.currentPosition;
                        }
                    }
                }
            }
        }

        @Override
        public void onClose() {
            Minecraft.getInstance().setScreen(parent);
        }

        @Override
        public boolean isPauseScreen() {
            return false;
        }

        private enum SimState { STOPPED, RUNNING, PAUSED }
        private enum LogLevel { DEBUG, INFO, WARN, ERROR }
        private record LogEntry(LogLevel level, String msg) {}

        private static class JointInfo {
            String name;
            float value, min, max;
            JointInfo(String name, float value, float min, float max) {
                this.name = name;
                this.value = value;
                this.min = min;
                this.max = max;
            }
        }
    }

    // =========================
    //  VMD 플레이어 (Webots 제거 버전)
    // =========================

    public static final class VMDPlayer {
        private static final Logger logger = LogManager.getLogger();
        private static volatile VMDPlayer instance;

        private volatile boolean playing = false;
        private volatile URDFMotion currentMotion = null;
        private float currentTime = 0f;
        private int activeJointCount = 0;
        private int debugCounter = 0;

        private VMDPlayer() {}

        public static VMDPlayer getInstance() {
            if (instance == null) {
                synchronized (VMDPlayer.class) {
                    if (instance == null) instance = new VMDPlayer();
                }
            }
            return instance;
        }

        public void loadMotion(URDFMotion motion) {
            currentMotion = motion;
            currentTime = 0f;
            playing = false;
            logger.info("VMD Motion loaded: {} ({} keyframes)", motion.name, motion.keys.size());
        }

        public void loadFromFile(File vmdFile) {
            URDFMotion motion = VMDLoader.load(vmdFile);
            if (motion != null) {
                loadMotion(motion);
            }
        }

        public void play() {
            if (currentMotion != null) {
                playing = true;
                logger.info("VMD Playback started");
            }
        }

        public void stop() {
            playing = false;
            currentTime = 0f;
        }

        public void pause() {
            playing = false;
        }

        public boolean isPlaying() {
            return playing;
        }

        public boolean hasMotion() {
            return currentMotion != null;
        }

        /**
         * 기존 tick 유지 (하위 호환).
         * preview + target 둘 다 적용.
         */
        public void tick(URDFModelOpenGLWithSTL renderer, float deltaTime) {
            tick(renderer, deltaTime, true, true);
        }

        /**
         * 옵션 버전:
         *  - applyPreview: setJointPreview 호출 여부
         *  - applyTarget : setJointTarget 호출 여부
         */
        public void tick(URDFModelOpenGLWithSTL renderer,
                         float deltaTime,
                         boolean applyPreview,
                         boolean applyTarget) {
            if (!playing) return;
            URDFMotion motion = currentMotion;
            if (motion == null || motion.keys.isEmpty()) return;

            currentTime += deltaTime;
            float maxTime = motion.keys.get(motion.keys.size() - 1).t;
            if (maxTime <= 0) maxTime = 1f;

            if (motion.loop && currentTime > maxTime) {
                currentTime = currentTime % maxTime;
            } else if (!motion.loop && currentTime > maxTime) {
                playing = false;
                return;
            }

            URDFMotion.Key prevKey = null, nextKey = null;
            for (URDFMotion.Key key : motion.keys) {
                if (key.t <= currentTime) prevKey = key;
                else { nextKey = key; break; }
            }
            if (prevKey == null) prevKey = motion.keys.get(0);

            float alpha = 0f;
            if (nextKey != null && nextKey.t > prevKey.t) {
                alpha = (currentTime - prevKey.t) / (nextKey.t - prevKey.t);
                if ("cubic".equals(prevKey.interp)) {
                    alpha = alpha * alpha * (3f - 2f * alpha);
                }
            }

            activeJointCount = 0;
            for (Map.Entry<String, Float> entry : prevKey.pose.entrySet()) {
                String jointName = entry.getKey();
                float value = entry.getValue();
                if (nextKey != null && nextKey.pose.containsKey(jointName)) {
                    value = lerp(value, nextKey.pose.get(jointName), alpha);
                }

                if (applyPreview) {
                    renderer.setJointPreview(jointName, value, JointControlSource.VMD);
                }
                if (applyTarget) {
                    renderer.setJointTarget(jointName, value, JointControlSource.VMD);
                }
                activeJointCount++;
            }

            if (++debugCounter >= 20) {
                debugCounter = 0;
                logger.debug("VMD: t={}/{}, joints={}", currentTime, maxTime, activeJointCount);
            }
        }

        private float lerp(float a, float b, float t) {
            return a + (b - a) * t;
        }

        public Status getStatus() {
            URDFMotion motion = currentMotion;
            if (motion == null) return new Status(null, 0, 0f, 0f, false, 0);
            float maxTime = motion.keys.isEmpty() ? 0f : motion.keys.get(motion.keys.size() - 1).t;
            return new Status(motion.name, motion.keys.size(), maxTime, currentTime, playing, activeJointCount);
        }

        public record Status(String motionName, int keyframeCount, float duration,
                             float currentTime, boolean playing, int activeJoints) {}
    }
}
