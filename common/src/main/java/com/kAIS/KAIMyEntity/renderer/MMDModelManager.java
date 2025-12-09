package com.kAIS.KAIMyEntity.renderer;

import com.kAIS.KAIMyEntity.KAIMyEntityClient;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.HashMap;
import java.util.Map;
import java.util.Properties;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import net.minecraft.client.Minecraft;

public class MMDModelManager {
    static final Logger logger = LogManager.getLogger();
    static final Minecraft MCinstance = Minecraft.getInstance();
    static Map<String, Model> models;
    static String gameDirectory = MCinstance.gameDirectory.getAbsolutePath();

    public static void Init() {
        models = new HashMap<>();
        logger.info("MMDModelManager.Init() finished");
    }

    /**
     * 모델 로딩 - URDF만 지원
     */
    public static IMMDModel LoadModel(String modelName) {
        File modelDir = new File(gameDirectory + "/KAIMyEntity/" + modelName);
        String modelDirStr = modelDir.getAbsolutePath();

        if (!modelDir.exists()) {
            logger.error("Model directory not found: " + modelDirStr);
            return null;
        }

        // URDF만 체크
        File urdfFile = new File(modelDir, "robot.urdf");
        if (urdfFile.isFile()) {
            logger.info("Loading URDF: " + modelName);
            IMMDModel urdfModel = com.kAIS.KAIMyEntity.urdf.URDFModelOpenGLWithSTL.Create(
                    urdfFile.getAbsolutePath(),
                    modelDirStr
            );
            if (urdfModel != null) {
                logger.info("✓ URDF loaded: " + modelName);
                return urdfModel;
            }
        }

        logger.error("No robot.urdf found in: " + modelDirStr);
        return null;
    }

    /**
     * 모델 가져오기 (캐시 포함)
     */
    public static Model GetModel(String modelName, String uuid) {
        String fullName = modelName + uuid;
        Model model = models.get(fullName);

        if (model == null) {
            IMMDModel m = LoadModel(modelName);
            if (m == null) {
                return null;
            }

            // URDF 모델 등록
            URDFModelData urdfData = new URDFModelData();
            urdfData.entityName = fullName;
            urdfData.model = m;
            urdfData.modelName = modelName;

            // ★ 여기 한 줄만 제거/주석 처리
            // m.ResetPhysics();

            models.put(fullName, urdfData);
            logger.info("✓ Model registered: " + fullName);

            model = urdfData;
        }
        return model;
    }

    public static Model GetModel(String modelName){
        return GetModel(modelName, "");
    }

    public static void ReloadModel() {
        models.clear();
    }

    // ========== 모델 클래스 ==========

    public static abstract class Model {
        public IMMDModel model;
        public String entityName;
        public String modelName;
        public Properties properties = new Properties();
        boolean isPropertiesLoaded = false;

        public void loadModelProperties(boolean forceReload){
            if (isPropertiesLoaded && !forceReload)
                return;
            String path2Properties = gameDirectory + "/KAIMyEntity/" + modelName + "/model.properties";
            try {
                InputStream istream = new FileInputStream(path2Properties);
                properties.load(istream);
            } catch (IOException e) {
                // properties 없어도 OK
            }
            isPropertiesLoaded = true;
            //           KAIMyEntityClient.reloadProperties = false;
        }

        public boolean isURDFModel() { return true; }
    }

    /**
     * URDF 모델
     */
    public static class URDFModelData extends Model {
        // 기본 구현
    }
}