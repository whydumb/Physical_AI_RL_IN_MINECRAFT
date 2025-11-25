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
 * 통합 Webots 설정 GUI
 */
public class WebotsConfigScreen extends Screen {
    private static final Logger LOGGER = LogManager.getLogger();
    
    private static final int BG_COLOR = 0xFF0E0E10;
    private static final int PANEL_COLOR = 0xFF1D1F24;
    private static final int TITLE_COLOR = 0xFFFFD770;
    private static final int TEXT_COLOR = 0xFFFFFFFF;
    private static final int CONNECTED_COLOR = 0xFF55FF55;
    private static final int DISCONNECTED_COLOR = 0xFFFF5555;
    
    private final Screen parent;
    private WebotsController controller;
    
    private EditBox ipBox;
    private EditBox portBox;
    private Button connectButton;
    private Button modeButton;
    private Button testButton;
    private Button closeButton;
    
    private String statusMessage = "";
    private int statusColor = TEXT_COLOR;
    private int autoRefreshTicker = 0;
    
    public WebotsConfigScreen(Screen parent) {
        super(Component.literal("Webots/RobotListener Settings"));
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
        
        this.ipBox = new EditBox(this.font, centerX - 100, startY, 200, 20, 
                Component.literal("IP Address"));
        
        Config config = Config.getInstance();
        this.ipBox.setValue(config.getLastIp());
        this.ipBox.setMaxLength(50);
        addRenderableWidget(this.ipBox);
        
        startY += 30;
        
        this.portBox = new EditBox(this.font, centerX - 100, startY, 200, 20,
                Component.literal("Port"));
        this.portBox.setValue(String.valueOf(config.getLastPort()));
        this.portBox.setMaxLength(5);
        addRenderableWidget(this.portBox);
        
        startY += 35;
        
        this.connectButton = Button.builder(Component.literal("Connect / Reconnect"), b -> {
            handleConnect();
        }).bounds(centerX - 100, startY, 200, 20).build();
        addRenderableWidget(this.connectButton);
        
        startY += 25;
        
        updateModeButtonText();
        this.modeButton = Button.builder(Component.literal("Mode: Webots"), b -> {
            handleModeToggle();
        }).bounds(centerX - 100, startY, 200, 20).build();
        addRenderableWidget(this.modeButton);
        
        startY += 25;
        
        this.testButton = Button.builder(Component.literal("Test T-Pose"), b -> {
            handleTest();
        }).bounds(centerX - 100, startY, 200, 20).build();
        addRenderableWidget(this.testButton);
        
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
            
            Config.getInstance().update(ip, port);
            
            LOGGER.info("Connected to server: {}:{}", ip, port);
        } catch (Exception e) {
            setStatus("✗ Connection failed: " + e.getMessage(), DISCONNECTED_COLOR);
            LOGGER.error("Failed to connect", e);
        }
    }
    
    private void handleModeToggle() {
        if (controller == null) {
            setStatus("✗ Not connected. Click 'Connect' first.", DISCONNECTED_COLOR);
            return;
        }
        
        if (!controller.isConnected()) {
            setStatus("✗ Connection lost. Try reconnecting.", DISCONNECTED_COLOR);
            return;
        }
        
        try {
            WebotsController.Mode currentMode = controller.getMode();
            
            if (currentMode == WebotsController.Mode.WEBOTS) {
                controller.enableRobotListener(true);
                setStatus("✓ Mode: RobotListener (WASD + Mouse)", CONNECTED_COLOR);
            } else {
                controller.enableRobotListener(false);
                setStatus("✓ Mode: Webots (URDF Joint Control)", CONNECTED_COLOR);
            }
            
            updateModeButtonText();
            
        } catch (Exception e) {
            setStatus("✗ Mode switch failed: " + e.getMessage(), DISCONNECTED_COLOR);
            LOGGER.error("Mode switch failed", e);
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
            controller.setJoint("r_sho_pitch", 0.3f);
            controller.setJoint("r_sho_roll", 1.57f);
            controller.setJoint("r_el", -0.1f);
            
            controller.setJoint("l_sho_pitch", 0.3f);
            controller.setJoint("l_sho_roll", -1.57f);
            controller.setJoint("l_el", -0.1f);
            
            setStatus("✓ T-Pose sent! Check simulation.", CONNECTED_COLOR);
            LOGGER.info("T-Pose test sent successfully");
        } catch (Exception e) {
            setStatus("✗ Test failed: " + e.getMessage(), DISCONNECTED_COLOR);
            LOGGER.error("T-Pose test failed", e);
        }
    }
    
    private void updateModeButtonText() {
        if (controller != null && modeButton != null) {
            WebotsController.Mode mode = controller.getMode();
            String text = mode == WebotsController.Mode.WEBOTS 
                ? "Mode: Webots → RobotListener" 
                : "Mode: RobotListener → Webots";
            modeButton.setMessage(Component.literal(text));
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
        modeButton.active = connected;
    }
    
    @Override
    public void render(GuiGraphics graphics, int mouseX, int mouseY, float partialTicks) {
        graphics.fill(0, 0, this.width, this.height, BG_COLOR);
        
        int panelX = this.width / 2 - 250;
        int panelY = 50;
        int panelW = 500;
        int panelH = this.height - 100;
        graphics.fill(panelX, panelY, panelX + panelW, panelY + panelH, PANEL_COLOR);
        
        super.render(graphics, mouseX, mouseY, partialTicks);
        
        graphics.pose().pushPose();
        graphics.pose().translate(0, 0, 1000.0f);
        graphics.drawCenteredString(this.font, "Webots/RobotListener Settings", 
                this.width / 2, 20, TITLE_COLOR);
        
        graphics.drawString(this.font, "IP Address:", this.width / 2 - 100, 68, TEXT_COLOR, false);
        graphics.drawString(this.font, "Port:", this.width / 2 - 100, 98, TEXT_COLOR, false);
        
        int statusY = 195;
        if (controller != null) {
            boolean connected = controller.isConnected();
            WebotsController.Mode mode = controller.getMode();
            
            String connStatus = connected ? "● CONNECTED" : "● DISCONNECTED";
            graphics.drawCenteredString(this.font, connStatus, this.width / 2, statusY, 
                    connected ? CONNECTED_COLOR : DISCONNECTED_COLOR);
            
            if (connected) {
                String address = "Address: " + controller.getRobotAddress();
                graphics.drawCenteredString(this.font, address, this.width / 2, 
                        statusY + 12, TEXT_COLOR);
                
                String modeStr = "Mode: " + (mode == WebotsController.Mode.WEBOTS ? "Webots" : "RobotListener");
                graphics.drawCenteredString(this.font, modeStr, this.width / 2, 
                        statusY + 24, TITLE_COLOR);
            }
        } else {
            graphics.drawCenteredString(this.font, "● NOT INITIALIZED", 
                    this.width / 2, statusY, DISCONNECTED_COLOR);
        }
        
        if (!statusMessage.isEmpty()) {
            graphics.drawCenteredString(this.font, statusMessage, 
                    this.width / 2, statusY + 45, statusColor);
        }
        
        if (controller != null && controller.isConnected()) {
            int statsY = statusY + 75;
            graphics.drawString(this.font, "=== Statistics ===", 
                    panelX + 20, statsY, TITLE_COLOR, false);
            
            if (controller.getMode() == WebotsController.Mode.ROBOTLISTENER) {
                graphics.drawString(this.font, "Walk commands: " + controller.getWalkSent(), 
                        panelX + 20, statsY + 15, TEXT_COLOR, false);
                graphics.drawString(this.font, "Head commands: " + controller.getHeadSent(), 
                        panelX + 20, statsY + 30, TEXT_COLOR, false);
                graphics.drawString(this.font, "Errors: " + controller.getErrors(), 
                        panelX + 20, statsY + 45, 
                        controller.getErrors() > 0 ? DISCONNECTED_COLOR : CONNECTED_COLOR, false);
                
                int helpY = statsY + 70;
                graphics.drawString(this.font, "=== Controls ===", 
                        panelX + 20, helpY, TITLE_COLOR, false);
                graphics.drawString(this.font, "• WASD: Move robot", 
                        panelX + 20, helpY + 15, 0xFFAAAAAA, false);
                graphics.drawString(this.font, "• Mouse: Aim head", 
                        panelX + 20, helpY + 30, 0xFFAAAAAA, false);
            } else {
                graphics.drawString(this.font, "URDF Joint Control Active", 
                        panelX + 20, statsY + 15, CONNECTED_COLOR, false);
                graphics.drawString(this.font, "Use U key to toggle mode", 
                        panelX + 20, statsY + 30, 0xFFAAAAAA, false);
            }
        }
        
        graphics.pose().popPose();
        
        if (++autoRefreshTicker >= 20) {
            autoRefreshTicker = 0;
            updateButtonStates();
            updateModeButtonText();
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
    
    // ========================================================================
    // Config 클래스
    // ========================================================================
    
    public static class Config {
        private static final Logger CONFIG_LOGGER = LogManager.getLogger();
        private static final String DEFAULT_IP = "localhost";
        private static final int DEFAULT_PORT = 8080;
        
        private String lastIp;
        private int lastPort;
        private final File configFile;
        private static Config instance;
        
        private Config() {
            File gameDirectory = Minecraft.getInstance().gameDirectory;
            File configDir = new File(gameDirectory, "config");
            if (!configDir.exists()) {
                configDir.mkdirs();
            }
            this.configFile = new File(configDir, "webots_connection.properties");
            load();
        }
        
        public static Config getInstance() {
            if (instance == null) {
                instance = new Config();
            }
            return instance;
        }
        
        private void load() {
            if (!configFile.exists()) {
                lastIp = DEFAULT_IP;
                lastPort = DEFAULT_PORT;
                save();
                CONFIG_LOGGER.info("Created default config: {}:{}", lastIp, lastPort);
                return;
            }
            
            Properties props = new Properties();
            try (FileInputStream fis = new FileInputStream(configFile)) {
                props.load(fis);
                lastIp = props.getProperty("ip", DEFAULT_IP);
                lastPort = Integer.parseInt(props.getProperty("port", String.valueOf(DEFAULT_PORT)));
                CONFIG_LOGGER.info("Loaded config: {}:{}", lastIp, lastPort);
            } catch (Exception e) {
                CONFIG_LOGGER.warn("Failed to load config, using defaults", e);
                lastIp = DEFAULT_IP;
                lastPort = DEFAULT_PORT;
            }
        }
        
        public void save() {
            Properties props = new Properties();
            props.setProperty("ip", lastIp);
            props.setProperty("port", String.valueOf(lastPort));
            
            try (FileOutputStream fos = new FileOutputStream(configFile)) {
                props.store(fos, "Webots Connection Settings");
                CONFIG_LOGGER.info("Saved config: {}:{}", lastIp, lastPort);
            } catch (Exception e) {
                CONFIG_LOGGER.error("Failed to save config", e);
            }
        }
        
        public void update(String ip, int port) {
            this.lastIp = ip;
            this.lastPort = port;
            save();
        }
        
        public String getLastIp() { return lastIp; }
        public int getLastPort() { return lastPort; }
    }
}
