package com.kAIS.KAIMyEntity.urdf.vmd;

import com.kAIS.KAIMyEntity.urdf.URDFJoint;
import com.kAIS.KAIMyEntity.urdf.URDFModel;
import com.kAIS.KAIMyEntity.urdf.control.URDFMotion;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.io.File;
import java.nio.file.Files;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class VMDLoader {
    private static final Logger logger = LogManager.getLogger();

    public static URDFMotion load(File vmdFile) {
        return load(vmdFile, null);
    }

    /**
     * VMD 파일을 URDFMotion으로 변환 (URDF 모델 참조)
     */
    public static URDFMotion load(File vmdFile, URDFModel robotModel) {
        try {
            // URDF 관절 이름 설정
            if (robotModel != null) {
                Set<String> jointNames = new HashSet<>();
                for (URDFJoint j : robotModel.joints) {
                    jointNames.add(j.name);
                }
                VMDParser.setKnownUrdfJoints(jointNames);
                logger.info("URDF joints for VMD mapping: {}", jointNames);
            }

            byte[] vmdData = Files.readAllBytes(vmdFile.toPath());
            List<VMDParser.VMDFrame> frames = VMDParser.parse(vmdData);

            if (frames == null || frames.isEmpty()) {
                logger.error("No frames parsed from VMD: {}", vmdFile.getName());
                return null;
            }

            URDFMotion motion = new URDFMotion();
            motion.name = vmdFile.getName();
            motion.fps = 30f;
            motion.loop = true;

            for (VMDParser.VMDFrame frame : frames) {
                if (frame.jointAngles.isEmpty()) continue;

                URDFMotion.Key key = new URDFMotion.Key();
                key.t = frame.frameNum / 30f;
                key.pose = new HashMap<>(frame.jointAngles);
                key.interp = "cubic";
                motion.keys.add(key);
            }

            if (motion.keys.isEmpty()) {
                logger.error("No valid keyframes in VMD: {}", vmdFile.getName());
                return null;
            }

            logger.info("✅ Loaded VMD: {} ({} keyframes)", vmdFile.getName(), motion.keys.size());
            return motion;

        } catch (Exception e) {
            logger.error("Failed to load VMD: {}", vmdFile.getName(), e);
            return null;
        }
    }
}
