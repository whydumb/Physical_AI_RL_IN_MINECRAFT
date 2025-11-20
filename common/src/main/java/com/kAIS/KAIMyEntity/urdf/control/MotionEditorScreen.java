package com.kAIS.KAIMyEntity.urdf.control;

import com.kAIS.KAIMyEntity.urdf.URDFJoint;
import com.kAIS.KAIMyEntity.urdf.URDFModelOpenGLWithSTL;
import net.minecraft.client.Minecraft;
import org.joml.*;

import java.util.*;

import static java.lang.Math.*;

/**
 * 2025.11.20 VMC Direct Retargeting (IK 제거, 순수 데이터 기반)
 * - /VMC/Ext/Bone/Pos/Local의 로컬 회전을 직접 사용
 * - IK 계산 완전 제거
 * - 팔만 움직임 (다리 필요 없음)
 */
public final class MotionEditorScreen {
    private MotionEditorScreen() {}

    // VMC 본 이름 정규화
    static {
        VMCListenerController.VmcListener listener = VMCListenerController.VmcListener.getInstance();
        listener.setBoneNameNormalizer(original -> {
            if (original == null) return null;
            String lower = original.toLowerCase().trim();

            return switch (lower) {
                // 왼팔
                case "leftupperarm", "leftarm", "left_arm", "upperarm_left", "arm.l", "leftshoulder" ->
                        "LeftUpperArm";
                case "leftlowerarm", "leftforearm", "lowerarm_left", "forearm.l", "leftelbow" ->
                        "LeftLowerArm";
                case "lefthand", "hand.l", "hand_left", "left_wrist", "left_hand" ->
                        "LeftHand";
                
                // 오른팔
                case "rightupperarm", "rightarm", "right_arm", "upperarm_right", "arm.r", "rightshoulder" ->
                        "RightUpperArm";
                case "rightlowerarm", "rightforearm", "lowerarm_right", "forearm.r", "rightelbow" ->
                        "RightLowerArm";
                case "righthand", "hand.r", "hand_right", "right_wrist", "right_hand" ->
                        "RightHand";
                
                default -> original;
            };
        });
    }

    public static void open(URDFModelOpenGLWithSTL renderer) {
        open(renderer, 39539);
    }

    public static void open(URDFModelOpenGLWithSTL renderer, int vmcPort) {
        VMCListenerController.VmcListener listener = VMCListenerController.VmcListener.getInstance();
        listener.start("0.0.0.0", vmcPort);
        Minecraft.getInstance().setScreen(new VMCListenerController(Minecraft.getInstance().screen, renderer));
    }

    public static void tick(URDFModelOpenGLWithSTL renderer) {
        VmcDrive.tick(renderer);
    }
}

/* ======================== VmcDrive ======================== */
final class VmcDrive {
    private static final JointControlBus BUS = new JointControlBus(0.35f);
    private static final URDFArmRetargeter RETARGETER = new URDFArmRetargeter();

    static void tick(URDFModelOpenGLWithSTL renderer) {
        var listener = VMCListenerController.VmcListener.getInstance();
        Map<String, VMCListenerController.VmcListener.BoneTransform> raw = listener.getBones();

        if (raw.isEmpty()) {
            BUS.resolveAndApply(renderer);
            return;
        }

        // VMC 본 데이터를 맵으로 변환
        Map<String, Object> bones = new HashMap<>();
        raw.forEach((k, v) -> bones.put(k, v));

        // 직접 리타게팅만 수행 (IK 제거)
        var commands = RETARGETER.commands(bones);
        if (!commands.isEmpty()) {
            BUS.push("retarget", JointControlBus.Priority.RETARGET, commands);
        }

        BUS.resolveAndApply(renderer);
    }
}

/* ======================== URDFArmRetargeter ======================== */
final class URDFArmRetargeter {
    public static final String L_SHO_PITCH = "l_sho_pitch";
    public static final String L_SHO_ROLL  = "l_sho_roll";
    public static final String L_ELBOW     = "l_el";
    public static final String R_SHO_PITCH = "r_sho_pitch";
    public static final String R_SHO_ROLL  = "r_sho_roll";
    public static final String R_ELBOW     = "r_el";

    public Map<String, Float> commands(Map<String, Object> bones) {
        Map<String, Float> out = new HashMap<>();
        solve(bones, true, out);   // 왼팔
        solve(bones, false, out);  // 오른팔
        return out;
    }

    private void solve(Map<String, Object> b, boolean left, Map<String, Float> out) {
        // VMC 본 데이터 읽기
        var upper = readBone(b.get(left ? "LeftUpperArm" : "RightUpperArm"));
        var lower = readBone(b.get(left ? "LeftLowerArm" : "RightLowerArm"));
        
        if (upper == null || lower == null) return;

        // 로컬 회전에서 오일러 각도 추출
        Vector3f upperEuler = toEulerZYX(upper.rotation);
        Vector3f lowerEuler = toEulerZYX(lower.rotation);

        // URDF 조인트로 매핑 (좌표계 변환 포함)
        // VMC Unity 좌표계 -> URDF ROS 좌표계 변환
        float shoPitch = -upperEuler.y;  // Y축 회전 -> Pitch
        float shoRoll  = upperEuler.x;   // X축 회전 -> Roll
        float elbow    = -lowerEuler.y * (left ? 1f : -1f);  // 팔꿈치 굽힘

        if (left) {
            out.put(L_SHO_PITCH, shoPitch);
            out.put(L_SHO_ROLL,  shoRoll);
            out.put(L_ELBOW,     elbow);
        } else {
            out.put(R_SHO_PITCH, shoPitch);
            out.put(R_SHO_ROLL,  shoRoll);
            out.put(R_ELBOW,     elbow);
        }
    }

    /**
     * Quaternion -> Euler (ZYX 순서)
     */
    private Vector3f toEulerZYX(Quaternionf q) {
        // ZYX Euler: Roll(X) -> Pitch(Y) -> Yaw(Z)
        float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
        float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
        float roll = (float) atan2(sinr_cosp, cosr_cosp);

        float sinp = 2 * (q.w * q.y - q.z * q.x);
        float pitch = (float) (abs(sinp) >= 1 ? copySign(PI / 2, sinp) : asin(sinp));

        float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        float yaw = (float) atan2(siny_cosp, cosy_cosp);

        return new Vector3f(roll, pitch, yaw);
    }

    private VMCListenerController.VmcListener.BoneTransform readBone(Object t) {
        return t instanceof VMCListenerController.VmcListener.BoneTransform bt ? bt : null;
    }
}

/* ======================== JointControlBus ======================== */
final class JointControlBus {
    public enum Priority { 
        MANUAL(100), 
        RETARGET(10); 
        
        final int v; 
        Priority(int v) { this.v = v; } 
        int value() { return v; } 
    }

    private final Map<String, Float> manual = new HashMap<>();
    private final List<SourceBuf> sources = new ArrayList<>();
    private final Map<String, Float> ema = new HashMap<>();
    private final float alpha;

    JointControlBus(float a) { 
        this.alpha = a; 
    }

    void push(String name, Priority prio, Map<String, Float> cmds) {
        if (cmds == null || cmds.isEmpty()) return;
        sources.add(new SourceBuf(name, prio.value(), new HashMap<>(cmds)));
    }

    void resolveAndApply(URDFModelOpenGLWithSTL r) {
        sources.sort((a, b) -> Integer.compare(b.prio, a.prio));

        for (URDFJoint j : r.getRobotModel().joints) {
            String n = j.name;
            Float target = manual.get(n);

            // 우선순위에 따라 타겟 값 선택
            if (target == null) {
                for (SourceBuf s : sources) {
                    Float v = s.cmds.get(n);
                    if (v != null) { 
                        target = v; 
                        break; 
                    }
                }
            }
            if (target == null) continue;

            // 관절 제한 적용
            float lo = Float.NEGATIVE_INFINITY, hi = Float.POSITIVE_INFINITY;
            if (n.equals("l_sho_pitch") || n.equals("r_sho_pitch")) { 
                lo = (float) toRadians(-250); 
                hi = (float) toRadians(250); 
            } else if (n.equals("l_sho_roll") || n.equals("r_sho_roll")) { 
                lo = (float) toRadians(-100); 
                hi = (float) toRadians(100); 
            } else if (n.equals("l_el")) { 
                lo = (float) toRadians(-160); 
                hi = 0f; 
            } else if (n.equals("r_el")) { 
                lo = 0f; 
                hi = (float) toRadians(160); 
            } else if (j.limit != null && j.limit.hasLimits()) { 
                lo = j.limit.lower; 
                hi = j.limit.upper; 
            }

            target = max(lo, min(hi, target));

            // EMA 스무딩
            Float prev = ema.get(n);
            float y = prev != null ? prev + (target - prev) * alpha : target;
            ema.put(n, y);

            // 조인트 적용
            r.setJointTarget(n, y);
            r.setJointPreview(n, y);
        }
        sources.clear();
    }

    private static final class SourceBuf {
        final String name; 
        final int prio; 
        final Map<String, Float> cmds;
        
        SourceBuf(String n, int p, Map<String, Float> c) { 
            name = n; 
            prio = p; 
            cmds = c; 
        }
    }
}
