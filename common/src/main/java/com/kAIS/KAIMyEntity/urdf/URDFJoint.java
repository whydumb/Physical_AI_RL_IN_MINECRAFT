package com.kAIS.KAIMyEntity.urdf;

import org.joml.Vector3f;

public class URDFJoint {
    public String name;
    public JointType type;
    public String parentLinkName;
    public String childLinkName;
    public Origin origin;
    public Axis axis;
    public Limit limit;
    public Dynamics dynamics;

    // Runtime state (updated from Webots)
    public float currentPosition;
    public float currentVelocity;

    public URDFJoint(String name, JointType type) {
        this.name = name;
        this.type = type;
        this.origin = new Origin();
        this.axis = new Axis();
        this.currentPosition = 0.0f;
        this.currentVelocity = 0.0f;
    }

    public enum JointType {
        REVOLUTE,    // 회전 관절 (제한 있음)
        CONTINUOUS,  // 회전 관절 (제한 없음)
        PRISMATIC,   // 직선 관절
        FIXED,       // 고정 (움직이지 않음)
        FLOATING,    // 자유 (6DOF)
        PLANAR       // 평면 (3DOF)
    }

    public static class Origin {
        public Vector3f xyz;
        public Vector3f rpy; // roll, pitch, yaw

        public Origin() {
            this.xyz = new Vector3f(0.0f, 0.0f, 0.0f);
            this.rpy = new Vector3f(0.0f, 0.0f, 0.0f);
        }
    }

    public static class Axis {
        public Vector3f xyz;

        public Axis() {
            // Default axis is X (URDF spec)
            this.xyz = new Vector3f(1.0f, 0.0f, 0.0f); // ★ 변경: 기존 (0,0,1) -> (1,0,0)
        }

        public void normalize() {
            xyz.normalize();
        }
    }

    public static class Limit {
        public float lower;    // 최소 각도/위치 (radians or meters)
        public float upper;    // 최대 각도/위치
        public float effort;   // 최대 힘/토크
        public float velocity; // 최대 속도

        public Limit() {
            this.lower = 0.0f;
            this.upper = 0.0f;
            this.effort = 0.0f;
            this.velocity = 0.0f;
        }

        public boolean hasLimits() {
            return lower != 0.0f || upper != 0.0f;
        }
    }

    public static class Dynamics {
        public float damping;
        public float friction;

        public Dynamics() {
            this.damping = 0.0f;
            this.friction = 0.0f;
        }
    }

    public boolean isMovable() {
        return type != JointType.FIXED;
    }

    public void updatePosition(float position) {
        if (limit != null && limit.hasLimits()) {
            // Clamp to limits
            this.currentPosition = Math.max(limit.lower, Math.min(limit.upper, position));
        } else {
            this.currentPosition = position;
        }
    }
}
