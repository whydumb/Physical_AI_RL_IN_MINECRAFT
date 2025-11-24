package com.kAIS.KAIMyEntity;

import net.minecraft.client.Minecraft;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.io.File;

/**
 * 최소 클라이언트 초기화
 * - 폴더 생성만 하고, 렌더러/MMD/URDF 전부 사용 안 함
 * - WASD + 마우스 → WebotsController 로 보내는 용도만 남김
 */
public class KAIMyEntityClient {
    public static final Logger logger = LogManager.getLogger();
    static final Minecraft MCinstance = Minecraft.getInstance();
    static final String gameDirectory = MCinstance.gameDirectory.getAbsolutePath();

    public static void initClient() {
        checkKAIMyEntityFolder();
        logger.info("KAIMyEntityClient initialized (RobotListener only)");
    }

    private static void checkKAIMyEntityFolder() {
        File KAIMyEntityFolder = new File(gameDirectory + "/KAIMyEntity");
        if (!KAIMyEntityFolder.exists()) {
            logger.info("KAIMyEntity folder not found, creating...");
            KAIMyEntityFolder.mkdir();
        }
    }
}
