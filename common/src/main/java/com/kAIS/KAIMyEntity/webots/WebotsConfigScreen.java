// common/src/main/java/com/kAIS/KAIMyEntity/webots/WebotsConfigScreen.java
package com.kAIS.KAIMyEntity.webots;

import net.minecraft.client.Minecraft;
import net.minecraft.client.gui.GuiGraphics;
import net.minecraft.client.gui.components.Button;
import net.minecraft.client.gui.components.EditBox;
import net.minecraft.client.gui.screens.Screen;
import net.minecraft.network.chat.Component;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.io.*;
import java.util.Properties;

/**
 * Webots 연결 설정 GUI + 설정 관리
 * - IP 주소 변경
 * - 포트 변경
 * - 연결 테스트
 * - 통계 확인
 * - 설정 자동 저장/로드
 */
public class WebotsConfigScreen extends Screen {
    private static final Logger LOGGER = LogManager.getLogger();
    
    // UI 색상
    private static final int BG_COLOR = 0xFF0E0E10;
    private static final int PANEL_COLOR = 0xFF1D1F24;
    private static final int TITLE_COLOR = 0xFFFFD770;
    private static final int TEXT_COLOR = 0xFFFFFFFF;
    private static final int CONNECTED_COLOR = 0xFF55FF55;
    private static final int DISCONNECTED_COLOR = 0xFFFF5555;
    
    private final Screen parent;
    private WebotsController controller;
    
    // UI 컴포넌트
    private EditBox ipBox;
    private EditBox portBox;
    private Button connectButton;
    private Button testButton;
    private Button closeButton;
    
    private String statusMessage = "";
    private int statusColor = TEXT_COLOR;
    private int autoRefreshTicker = 0;
    
    public WebotsConfigScreen(Screen parent) {
        super(Component.literal("Webots Connection Settings"));
        this.parent = parent;
        
        try {
            this.controller = WebotsController.getInstance();
        } catch (Exception e) {
            LOGGER.warn("WebotsController not initialized yet");
        }
    }
    
    @Override
    protected void init() {
        super.init();
        
        int centerX = this.width / 2;
        int startY = 80;
        
        // === IP 주소 입력 ===
        this.ipBox = new EditBox(this.font, centerX - 100, startY, 200, 20, 
                Component.literal("IP Address"));
        
        // ✅ Config에서 마지막 저장된 IP 로드
        Config config = Config.getInstance();
        this.ipBox.setValue(config.getLastIp());
        this.ipBox.setMaxLength(50);
        addRenderableWidget(this.ipBox);
        
        startY += 30;
        
        // === 포트 입력 ===
        this.portBox = new EditBox(this.font, centerX - 100, startY, 200, 20,
                Component.literal("Port"));
        
        // ✅ Config에서 마지막 저장된 Port 로드
        this.portBox.setValue(String.valueOf(config.getLastPort()));
        this.portBox.setMaxLength(5);
        addRenderableWidget(this.portBox);
        
        startY += 35;
        
        // === 연결/재연결 버튼 ===
        this.connectButton = Button.builder(Component.literal("Connect / Reconnect"), b -> {
            handleConnect();
        }).bounds(centerX - 100, startY, 200, 20).build();
        addRenderableWidget(this.connectButton);
        
        startY += 25;
        
        // === T-Pose 테스트 버튼 ===
        this.testButton = Button.builder(Component.literal("Test T-Pose"), b -> {
            handleTest();
        }).bounds(centerX - 100, startY, 200, 20).build();
        addRenderableWidget(this.testButton);
        
        // === 닫기 버튼 ===
        this.closeButton = Button.builder(Component.literal("Close"), b -> {
            Minecraft.getInstance().setScreen(parent);
        }).bounds(centerX - 50, this.height - 30, 100, 20).build();
        addRenderableWidget(this.closeButton);
        
        updateButtonStates();
    }
    
    private void handleConnect() {
        String ip = ipBox.getValue().trim();
        int port;
        
        try {
            port = Integer.parseInt(portBox.getValue().trim());
            if (port < 1 || port > 65535) {
                setStatus("Invalid port number (1-65535)", DISCONNECTED_COLOR);
                return;
            }
        } catch (NumberFormatException e) {
            setStatus("Invalid port format", DISCONNECTED_COLOR);
            return;
        }
        
        if (ip.isEmpty()) {
            setStatus("IP address cannot be empty", DISCONNECTED_COLOR);
            return;
        }
        
        try {
            if (controller == null) {
                controller = WebotsController.getInstance(ip, port);
                setStatus("✓ Connected to " + ip + ":" + port, CONNECTED_COLOR);
            } else {
                controller.reconnect(ip, port);
                setStatus("✓ Reconnected to " + ip + ":" + port, CONNECTED_COLOR);
            }
            
            // ✅ Config에 저장
            Config.getInstance().update(ip, port);
            
            LOGGER.info("Connected to Webots: {}:{}", ip, port);
        } catch (Exception e) {
            setStatus("✗ Connection failed: " + e.getMessage(), DISCONNECTED_COLOR);
            LOGGER.error("Failed to connect to Webots", e);
        }
    }
    
    private void handleTest() {
        if (controller == null) {
            setStatus("✗ Not connected. Click 'Connect' first.", DISCONNECTED_COLOR);
            return;
        }
        
        if (!controller.isConnected()) {
            setStatus("✗ Connection lost. Try reconnecting.", DISCONNECTED_COLOR);
            return;
        }
        
        try {
            // T-Pose 자세 전송
            controller.setJoint("r_sho_pitch", 0.3f);
            controller.setJoint("r_sho_roll", 1.57f);
            controller.setJoint("r_el", -0.1f);
            
            controller.setJoint("l_sho_pitch", 0.3f);
            controller.setJoint("l_sho_roll", -1.57f);
            controller.setJoint("l_el", -0.1f);
            
            setStatus("✓ T-Pose sent! Check Webots simulation.", CONNECTED_COLOR);
            LOGGER.info("T-Pose test sent successfully");
        } catch (Exception e) {
            setStatus("✗ Test failed: " + e.getMessage(), DISCONNECTED_COLOR);
            LOGGER.error("T-Pose test failed", e);
        }
    }
    
    private void setStatus(String message, int color) {
        this.statusMessage = message;
        this.statusColor = color;
    }
    
    private void updateButtonStates() {
        boolean hasController = (controller != null);
        boolean connected = hasController && controller.isConnected();
        
        testButton.active = connected;
    }
    
    @Override
    public void render(GuiGraphics graphics, int mouseX, int mouseY, float partialTicks) {
        // 배경
        graphics.fill(0, 0, this.width, this.height, BG_COLOR);
        
        // 메인 패널
        int panelX = this.width / 2 - 250;
        int panelY = 50;
        int panelW = 500;
        int panelH = this.height - 100;
        graphics.fill(panelX, panelY, panelX + panelW, panelY + panelH, PANEL_COLOR);
        
        super.render(graphics, mouseX, mouseY, partialTicks);
        
        // 제목
        graphics.pose().pushPose();
        graphics.pose().translate(0, 0, 1000.0f);
        graphics.drawCenteredString(this.font, "Webots Connection Settings", 
                this.width / 2, 20, TITLE_COLOR);
        
        // 라벨
        graphics.drawString(this.font, "IP Address:", this.width / 2 - 100, 68, TEXT_COLOR, false);
        graphics.drawString(this.font, "Port:", this.width / 2 - 100, 98, TEXT_COLOR, false);
        
        // 연결 상태
        int statusY = 145;
        if (controller != null) {
            boolean connected = controller.isConnected();
            String connStatus = connected ? "§a● CONNECTED" : "§c● DISCONNECTED";
            graphics.drawCenteredString(this.font, connStatus, this.width / 2, statusY, 
                    connected ? CONNECTED_COLOR : DISCONNECTED_COLOR);
            
            if (connected) {
                String address = "Address: " + controller.getRobotAddress();
                graphics.drawCenteredString(this.font, address, this.width / 2, 
                        statusY + 12, TEXT_COLOR);
            }
        } else {
            graphics.drawCenteredString(this.font, "§c● NOT INITIALIZED", 
                    this.width / 2, statusY, DISCONNECTED_COLOR);
        }
        
        // 상태 메시지
        if (!statusMessage.isEmpty()) {
            graphics.drawCenteredString(this.font, statusMessage, 
                    this.width / 2, statusY + 30, statusColor);
        }
        
        // 통계 패널
        if (controller != null && controller.isConnected()) {
            int statsY = statusY + 55;
            graphics.drawString(this.font, "=== Statistics ===", 
                    panelX + 20, statsY, TITLE_COLOR, false);
            
            String statsJson = controller.getStatsJson();
            if (!statsJson.contains("error")) {
                graphics.drawString(this.font, "Server: OK", 
                        panelX + 20, statsY + 15, CONNECTED_COLOR, false);
            } else {
                graphics.drawString(this.font, "Server: " + statsJson, 
                        panelX + 20, statsY + 15, DISCONNECTED_COLOR, false);
            }
        }
        
        graphics.pose().popPose();
        
        // 주기적 업데이트
        if (++autoRefreshTicker >= 20) {
            autoRefreshTicker = 0;
            updateButtonStates();
        }
    }
    
    @Override
    public void tick() {
        super.tick();
        // EditBox.tick() removed in modern versions
    }
    
    @Override
    public void onClose() {
        Minecraft.getInstance().setScreen(parent);
    }
    
    @Override
    public boolean isPauseScreen() {
        return false;
    }
    
    // ========================================================================
    // ✅ 내부 Config 클래스 (WebotsConfig.java 내용 통합)
    // ========================================================================
    
    /**
     * Webots 연결 설정 관리 (내부 클래스)
     * - 기본 IP/Port 저장 및 로드
     * - 마지막 연결 정보 기억
     * - 설정 파일 자동 생성
     */
    public static class Config {
        private static final Logger CONFIG_LOGGER = LogManager.getLogger();
        
        // 기본값
        private static final String DEFAULT_IP = "localhost";
        private static final int DEFAULT_PORT = 8080;
        
        // 현재 설정값
        private String lastIp;
        private int lastPort;
        
        // 설정 파일 경로
        private final File configFile;
        
        // 싱글톤
        private static Config instance;
        
        private Config() {
            File gameDirectory = Minecraft.getInstance().gameDirectory;
            File configDir = new File(gameDirectory, "config");
            if (!configDir.exists()) {
                configDir.mkdirs();
            }
            this.configFile = new File(configDir, "webots_connection.properties");
            
            // 설정 로드
            load();
        }
        
        public static Config getInstance() {
            if (instance == null) {
                instance = new Config();
            }
            return instance;
        }
        
        /**
         * 설정 파일에서 로드
         */
        private void load() {
            if (!configFile.exists()) {
                // 파일 없으면 기본값 사용
                lastIp = DEFAULT_IP;
                lastPort = DEFAULT_PORT;
                save(); // 기본값으로 파일 생성
                CONFIG_LOGGER.info("Created default Webots config: {}:{}", lastIp, lastPort);
                return;
            }
            
            Properties props = new Properties();
            try (FileInputStream fis = new FileInputStream(configFile)) {
                props.load(fis);
                lastIp = props.getProperty("ip", DEFAULT_IP);
                lastPort = Integer.parseInt(props.getProperty("port", String.valueOf(DEFAULT_PORT)));
                CONFIG_LOGGER.info("Loaded Webots config: {}:{}", lastIp, lastPort);
            } catch (Exception e) {
                CONFIG_LOGGER.warn("Failed to load Webots config, using defaults", e);
                lastIp = DEFAULT_IP;
                lastPort = DEFAULT_PORT;
            }
        }
        
        /**
         * 설정 파일에 저장
         */
        public void save() {
            Properties props = new Properties();
            props.setProperty("ip", lastIp);
            props.setProperty("port", String.valueOf(lastPort));
            
            try (FileOutputStream fos = new FileOutputStream(configFile)) {
                props.store(fos, "Webots Connection Settings");
                CONFIG_LOGGER.info("Saved Webots config: {}:{}", lastIp, lastPort);
            } catch (Exception e) {
                CONFIG_LOGGER.error("Failed to save Webots config", e);
            }
        }
        
        /**
         * 현재 설정 업데이트 및 저장
         */
        public void update(String ip, int port) {
            this.lastIp = ip;
            this.lastPort = port;
            save();
        }
        
        // Getters
        public String getLastIp() {
            return lastIp;
        }
        
        public int getLastPort() {
            return lastPort;
        }
        
        public String getDefaultIp() {
            return DEFAULT_IP;
        }
        
        public int getDefaultPort() {
            return DEFAULT_PORT;
        }
    }
}