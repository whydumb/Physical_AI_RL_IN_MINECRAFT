package com.kAIS.KAIMyEntity.urdf.control;

import com.kAIS.KAIMyEntity.urdf.URDFJoint;
import net.minecraft.util.Mth;

import java.util.HashMap;
import java.util.Map;
import java.util.Collection;

public final class URDFSimpleController {
    private final Map<String, URDFJoint> joints;
    private final Map<String, Float> target = new HashMap<>();
    private float kp = 30f;
    private float kd = 6f;
    private float defaultMaxVel = 4.0f;   // [rad/s] or [m/s]
    private float defaultMaxAcc = 12.0f;  // [rad/s^2] or [m/s^2]

    public URDFSimpleController(Collection<URDFJoint> allJoints) {
        Map<String, URDFJoint> m = new HashMap<>();
        for (URDFJoint j : allJoints) {
            m.put(j.name, j);
            target.put(j.name, j.currentPosition);
        }
        this.joints = m;
    }

    public void setTarget(String name, float value) {
        URDFJoint j = joints.get(name);
        if (j == null) return;
        if (j.type == URDFJoint.JointType.CONTINUOUS) value = wrapToPi(value);
        if (j.type == URDFJoint.JointType.REVOLUTE || j.type == URDFJoint.JointType.PRISMATIC) {
            if (j.limit != null && j.limit.hasLimits()) {
                value = Mth.clamp(value, j.limit.lower, j.limit.upper);
            }
        }
        target.put(name, value);
    }

    public void setTargets(Map<String, Float> targets) {
        for (var e : targets.entrySet()) setTarget(e.getKey(), e.getValue());
    }

    public float getTarget(String name) { return target.getOrDefault(name, 0f); }
    public void setGains(float kp, float kd){ this.kp = kp; this.kd = kd; }
    public void setLimits(float maxVel, float maxAcc){ this.defaultMaxVel = maxVel; this.defaultMaxAcc = maxAcc; }

    /** call every tick; dt ≈ 1/20f */
    public void update(float dt) {
        for (URDFJoint j : joints.values()) {
            float tgt = target.getOrDefault(j.name, j.currentPosition);
            float pos = j.currentPosition;
            float vel = j.currentVelocity;

            if (j.type == URDFJoint.JointType.CONTINUOUS) {
                float d = (float)Math.atan2(Math.sin(tgt - pos), Math.cos(tgt - pos));
                tgt = pos + d;
            }

            float err = tgt - pos;
            float acc = kp * err - kd * vel;

            float maxVel = (j.limit != null && j.limit.velocity > 0f) ? j.limit.velocity : defaultMaxVel;
            float maxAcc = defaultMaxAcc;

            acc = Mth.clamp(acc, -maxAcc, maxAcc);
            vel += acc * dt;
            vel  = Mth.clamp(vel, -maxVel, maxVel);
            pos += vel * dt;

            if (j.type == URDFJoint.JointType.REVOLUTE || j.type == URDFJoint.JointType.PRISMATIC) {
                if (j.limit != null && j.limit.hasLimits()) {
                    if (pos < j.limit.lower) { pos = j.limit.lower; vel = 0f; }
                    if (pos > j.limit.upper) { pos = j.limit.upper; vel = 0f; }
                }
            } else if (j.type == URDFJoint.JointType.CONTINUOUS) {
                pos = wrapToPi(pos);
            }

            j.currentVelocity = vel;
            j.currentPosition = pos;
        }
    }

    private static float wrapToPi(float a) {
        float twoPi = (float)(Math.PI * 2.0);
        a = (float)(a % twoPi);
        if (a >  Math.PI) a -= twoPi;
        if (a < -Math.PI) a += twoPi;
        return a;
    }
}
