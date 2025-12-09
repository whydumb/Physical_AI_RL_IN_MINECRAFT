package com.kAIS.KAIMyEntity.urdf.vmd;

import org.joml.Quaternionf;
import org.joml.Vector3f;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.Charset;
import java.util.*;

public class VMDParser {
    private static final Logger logger = LogManager.getLogger();

    // VMD 본 이름 → URDF 관절 이름 매핑
    private static final Map<String, String[]> VMD_TO_URDF = new HashMap<>();
    static {
        // ===== 몸통/척추 =====
        VMD_TO_URDF.put("センター", new String[]{"torso", "Hips", "hips", "Pelvis", "pelvis", "base_link"});
        VMD_TO_URDF.put("下半身", new String[]{"hip", "Hips", "hips", "Pelvis", "pelvis", "lower_body"});
        VMD_TO_URDF.put("上半身", new String[]{"torso", "Spine", "spine", "Torso", "chest"});
        VMD_TO_URDF.put("上半身2", new String[]{"chest", "Chest", "Spine1", "Spine2", "upper_chest"});
        
        // ===== 머리/목 =====
        VMD_TO_URDF.put("首", new String[]{"head_pan", "Neck", "neck", "HeadYaw", "head_yaw"});
        VMD_TO_URDF.put("頭", new String[]{"head_tilt", "Head", "head", "HeadPitch", "head_pitch"});

        // ===== 왼팔 =====
        VMD_TO_URDF.put("左肩", new String[]{"l_sho_pitch", "LShoulderPitch", "LeftShoulder", "left_shoulder_pitch", "l_shoulder_pitch"});
        VMD_TO_URDF.put("左腕", new String[]{"l_sho_roll", "LShoulderRoll", "LeftUpperArm", "left_shoulder_roll", "l_shoulder_roll"});
        VMD_TO_URDF.put("左ひじ", new String[]{"l_el", "LElbowYaw", "LElbowRoll", "LeftLowerArm", "left_elbow", "l_elbow"});
        VMD_TO_URDF.put("左手首", new String[]{"l_wrist", "LWristYaw", "LeftHand", "left_wrist", "l_wrist_yaw"});

        // ===== 오른팔 =====
        VMD_TO_URDF.put("右肩", new String[]{"r_sho_pitch", "RShoulderPitch", "RightShoulder", "right_shoulder_pitch", "r_shoulder_pitch"});
        VMD_TO_URDF.put("右腕", new String[]{"r_sho_roll", "RShoulderRoll", "RightUpperArm", "right_shoulder_roll", "r_shoulder_roll"});
        VMD_TO_URDF.put("右ひじ", new String[]{"r_el", "RElbowYaw", "RElbowRoll", "RightLowerArm", "right_elbow", "r_elbow"});
        VMD_TO_URDF.put("右手首", new String[]{"r_wrist", "RWristYaw", "RightHand", "right_wrist", "r_wrist_yaw"});

        // ===== 왼다리 (추가!) =====
        VMD_TO_URDF.put("左足", new String[]{"l_hip_yaw", "l_hip_pitch", "LHipYawPitch", "LHipPitch", "LeftUpLeg", "left_hip", "l_leg"});
        VMD_TO_URDF.put("左ひざ", new String[]{"l_knee", "LKneePitch", "LeftLeg", "left_knee", "l_knee_pitch"});
        VMD_TO_URDF.put("左足首", new String[]{"l_ank_pitch", "LAnklePitch", "LeftFoot", "left_ankle", "l_ankle"});
        VMD_TO_URDF.put("左つま先", new String[]{"l_ank_roll", "LAnkleRoll", "LeftToeBase", "left_toe", "l_toe"});

        // ===== 오른다리 (추가!) =====
        VMD_TO_URDF.put("右足", new String[]{"r_hip_yaw", "r_hip_pitch", "RHipYawPitch", "RHipPitch", "RightUpLeg", "right_hip", "r_leg"});
        VMD_TO_URDF.put("右ひざ", new String[]{"r_knee", "RKneePitch", "RightLeg", "right_knee", "r_knee_pitch"});
        VMD_TO_URDF.put("右足首", new String[]{"r_ank_pitch", "RAnklePitch", "RightFoot", "right_ankle", "r_ankle"});
        VMD_TO_URDF.put("右つま先", new String[]{"r_ank_roll", "RAnkleRoll", "RightToeBase", "right_toe", "r_toe"});

        // ===== IK 본 (위치 기반 - 참고용) =====
        VMD_TO_URDF.put("左足ＩＫ", new String[]{"l_ank_pitch", "LAnklePitch", "LeftFoot"});
        VMD_TO_URDF.put("右足ＩＫ", new String[]{"r_ank_pitch", "RAnklePitch", "RightFoot"});
        VMD_TO_URDF.put("左つま先ＩＫ", new String[]{"l_ank_roll", "LAnkleRoll", "LeftToeBase"});
        VMD_TO_URDF.put("右つま先ＩＫ", new String[]{"r_ank_roll", "RAnkleRoll", "RightToeBase"});
    }

    // 실제 URDF에서 발견된 관절 이름 캐시
    private static Set<String> knownUrdfJoints = new HashSet<>();

    public static void setKnownUrdfJoints(Set<String> joints) {
        knownUrdfJoints = joints;
        logger.info("VMDParser: Known URDF joints set: {}", joints);
    }

    public static class VMDFrame {
        public int frameNum;
        public Map<String, Float> jointAngles = new HashMap<>();

        @Override
        public String toString() {
            return "Frame#" + frameNum + ": " + jointAngles;
        }
    }

    public static List<VMDFrame> parse(byte[] vmdData) {
        ByteBuffer buffer = ByteBuffer.wrap(vmdData).order(ByteOrder.LITTLE_ENDIAN);

        try {
            // 1. VMD 헤더 검증
            byte[] magic = new byte[30];
            buffer.get(magic);
            String magicStr = new String(magic, Charset.forName("Shift-JIS"))
                    .replace("\0", "").trim();

            logger.info("VMD Magic: '{}'", magicStr);

            if (!magicStr.startsWith("Vocaloid Motion Data")) {
                throw new IllegalArgumentException("Not a valid VMD file: " + magicStr);
            }

            // 모델 이름
            byte[] modelName = new byte[20];
            buffer.get(modelName);
            String modelNameStr = new String(modelName, Charset.forName("Shift-JIS"))
                    .replace("\0", "").trim();
            logger.info("VMD Model: '{}'", modelNameStr);

            // 2. 모션 프레임 개수
            int motionCount = buffer.getInt();
            logger.info("VMD Motion count: {}", motionCount);

            if (motionCount <= 0 || motionCount > 1000000) {
                throw new IllegalArgumentException("Invalid motion count: " + motionCount);
            }

            Map<Integer, VMDFrame> frameMap = new TreeMap<>();
            int mappedBones = 0;
            Set<String> unmappedNames = new HashSet<>();
            Set<String> mappedJointNames = new HashSet<>(); // 디버그용

            // 3. 각 모션 프레임 파싱
            for (int i = 0; i < motionCount; i++) {
                // 본 이름 (15바이트)
                byte[] nameBytes = new byte[15];
                buffer.get(nameBytes);
                String boneName = new String(nameBytes, Charset.forName("Shift-JIS"))
                        .replace("\0", "").trim();

                // 프레임 번호
                int frameNum = buffer.getInt();

                // 위치 (X, Y, Z)
                float px = buffer.getFloat();
                float py = buffer.getFloat();
                float pz = buffer.getFloat();

                // 회전 (Quaternion)
                float qx = buffer.getFloat();
                float qy = buffer.getFloat();
                float qz = buffer.getFloat();
                float qw = buffer.getFloat();

                // 보간 파라미터 (64바이트)
                buffer.position(buffer.position() + 64);

                // URDF 관절 이름으로 변환
                String[] possibleNames = VMD_TO_URDF.get(boneName);
                if (possibleNames == null) {
                    unmappedNames.add(boneName);
                    continue;
                }

                // 실제 URDF에 존재하는 이름 찾기
                String urdfJointName = findMatchingJoint(possibleNames);
                if (urdfJointName == null) {
                    // 첫 번째 이름을 기본값으로 사용
                    urdfJointName = possibleNames[0];
                }

                mappedBones++;
                mappedJointNames.add(urdfJointName);

                // Quaternion → Euler
                Quaternionf q = new Quaternionf(qx, qy, qz, qw);
                Vector3f euler = new Vector3f();
                q.getEulerAnglesXYZ(euler);

                // MMD → URDF 변환
                float angle = convertToUrdfAngle(boneName, euler);

                // 프레임에 추가
                VMDFrame frame = frameMap.computeIfAbsent(frameNum, k -> {
                    VMDFrame f = new VMDFrame();
                    f.frameNum = k;
                    return f;
                });

                frame.jointAngles.put(urdfJointName, angle);
            }

            logger.info("✅ Mapped bones: {}, Unmapped: {}", mappedBones, unmappedNames.size());
            logger.info("✅ Mapped joint names: {}", mappedJointNames);
            if (!unmappedNames.isEmpty()) {
                logger.debug("Unmapped bone names: {}", unmappedNames);
            }

            List<VMDFrame> frames = new ArrayList<>(frameMap.values());
            logger.info("Total unique frames: {}", frames.size());

            return frames;

        } catch (Exception e) {
            logger.error("VMD parse error at position: {}", buffer.position(), e);
            throw new RuntimeException("Failed to parse VMD", e);
        }
    }

    private static String findMatchingJoint(String[] possibleNames) {
        if (knownUrdfJoints.isEmpty()) {
            return null; // URDF 관절 목록이 없으면 첫 번째 사용
        }
        for (String name : possibleNames) {
            if (knownUrdfJoints.contains(name)) {
                return name;
            }
            // 대소문자 무시 검색
            for (String urdfJoint : knownUrdfJoints) {
                if (urdfJoint.equalsIgnoreCase(name)) {
                    return urdfJoint;
                }
            }
        }
        return null;
    }

    /**
     * VMD 본의 Euler 각도를 URDF 관절 각도로 변환
     */
    private static float convertToUrdfAngle(String vmdBoneName, Vector3f euler) {
        switch (vmdBoneName) {
            // ===== 머리 =====
            case "首":  // Neck - Y축 (좌우)
                return -euler.y;
            case "頭":  // Head - X축 (상하)
                return -euler.x;

            // ===== 어깨 =====
            case "左肩":
            case "右肩":  // Shoulder pitch - X축
                return -euler.x;

            // ===== 팔 =====
            case "左腕":  // Left arm roll - Z축
                return euler.z + (float) Math.toRadians(30); // A-Pose 보정
            case "右腕":  // Right arm roll - Z축
                return euler.z - (float) Math.toRadians(30);

            // ===== 팔꿈치 =====
            case "左ひじ":
                return -Math.abs(euler.x);  // 음수로 굽힘
            case "右ひじ":
                return Math.abs(euler.x);   // 양수로 굽힘 (좌우 대칭)

            // ===== 다리 (엉덩이) =====
            case "左足":  // Left hip
                return -euler.x;  // 앞뒤 움직임
            case "右足":  // Right hip
                return -euler.x;

            // ===== 무릎 =====
            case "左ひざ":  // Left knee
                return euler.x;  // 무릎은 양수로 굽힘
            case "右ひざ":  // Right knee
                return euler.x;

            // ===== 발목 =====
            case "左足首":  // Left ankle
                return -euler.x;
            case "右足首":  // Right ankle
                return -euler.x;

            // ===== 발끝 =====
            case "左つま先":
            case "右つま先":
                return euler.z;

            // ===== 몸통 =====
            case "センター":
            case "下半身":
            case "上半身":
            case "上半身2":
                return euler.y;  // Y축 회전 (좌우 비틀기)

            default:
                return euler.z;
        }
    }
}
