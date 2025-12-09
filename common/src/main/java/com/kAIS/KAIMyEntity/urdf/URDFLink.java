package com.kAIS.KAIMyEntity.urdf;

import org.joml.Vector3f;
import org.joml.Quaternionf;

public class URDFLink {
    public String name;
    public Visual visual;
    public Collision collision;
    public Inertial inertial;
    
    public URDFLink(String name) {
        this.name = name;
    }
    
    public static class Visual {
        public Geometry geometry;
        public Origin origin;
        public Material material;
        
        public Visual() {
            this.origin = new Origin();
        }
    }
    
    public static class Collision {
        public Geometry geometry;
        public Origin origin;
        
        public Collision() {
            this.origin = new Origin();
        }
    }
    
    public static class Inertial {
        public Origin origin;
        public Mass mass;
        public Inertia inertia;
        
        public Inertial() {
            this.origin = new Origin();
            this.mass = new Mass();
            this.inertia = new Inertia();
        }
        
        public static class Mass {
            public float value = 1.0f;
            
            public Mass() {}
            
            public Mass(float value) {
                this.value = value;
            }
            
            public double getValue() {
                return (double) value;
            }
            
            public boolean isValid() {
                return value > 0 && Float.isFinite(value);
            }
        }
        
        public static class Inertia {
            public float ixx = 0.001f;
            public float ixy = 0f;
            public float ixz = 0f;
            public float iyy = 0.001f;
            public float iyz = 0f;
            public float izz = 0.001f;
            
            public Inertia() {}
            
            public float[][] toMatrix() {
                return new float[][] {
                    {ixx, ixy, ixz},
                    {ixy, iyy, iyz},
                    {ixz, iyz, izz}
                };
            }
            
            public double[] toArray() {
                return new double[] {
                    ixx, ixy, ixz,
                    ixy, iyy, iyz,
                    ixz, iyz, izz
                };
            }
        }
        
        public float getMassValue() {
            return mass != null ? mass.value : 1.0f;
        }
        
        public double getMassAsDouble() {
            return mass != null ? (double) mass.value : 1.0;
        }
        
        public boolean isValid() {
            return mass != null && mass.isValid();
        }
    }
    
    public static class Geometry {
        public GeometryType type;
        public String meshFilename;
        public Vector3f scale;
        
        public Vector3f boxSize;
        public float cylinderRadius;
        public float cylinderLength;
        public float sphereRadius;
        
        public enum GeometryType {
            MESH, BOX, CYLINDER, SPHERE
        }
        
        public Geometry() {
            this.scale = new Vector3f(1.0f, 1.0f, 1.0f);
        }
        
        public float estimateVolume() {
            switch (type) {
                case BOX:
                    if (boxSize != null) {
                        return boxSize.x * boxSize.y * boxSize.z;
                    }
                    break;
                case CYLINDER:
                    return (float) (Math.PI * cylinderRadius * cylinderRadius * cylinderLength);
                case SPHERE:
                    return (float) (4.0 / 3.0 * Math.PI * sphereRadius * sphereRadius * sphereRadius);
                case MESH:
                default:
                    return 0.001f;
            }
            return 0.001f;
        }
    }
    
    public static class Origin {
        public Vector3f xyz;
        public Vector3f rpy;
        
        public Origin() {
            this.xyz = new Vector3f(0.0f, 0.0f, 0.0f);
            this.rpy = new Vector3f(0.0f, 0.0f, 0.0f);
        }
        
        public Origin(float x, float y, float z) {
            this.xyz = new Vector3f(x, y, z);
            this.rpy = new Vector3f(0.0f, 0.0f, 0.0f);
        }
        
        public Origin(float x, float y, float z, float roll, float pitch, float yaw) {
            this.xyz = new Vector3f(x, y, z);
            this.rpy = new Vector3f(roll, pitch, yaw);
        }
        
        public Quaternionf getQuaternion() {
            Quaternionf qx = new Quaternionf().rotateX(rpy.x);
            Quaternionf qy = new Quaternionf().rotateY(rpy.y);
            Quaternionf qz = new Quaternionf().rotateZ(rpy.z);
            return qz.mul(qy).mul(qx);
        }
        
        public boolean isZero() {
            return xyz.x == 0 && xyz.y == 0 && xyz.z == 0 &&
                   rpy.x == 0 && rpy.y == 0 && rpy.z == 0;
        }
    }
    
    public static class Material {
        public String name;
        public Vector4f color;
        public String textureFilename;
        
        public Material() {
            this.color = new Vector4f(0.8f, 0.8f, 0.8f, 1.0f);
        }
        
        public Material(String name) {
            this();
            this.name = name;
        }
        
        public static class Vector4f {
            public float x, y, z, w;
            
            public Vector4f() {
                this(1.0f, 1.0f, 1.0f, 1.0f);
            }
            
            public Vector4f(float x, float y, float z, float w) {
                this.x = x;
                this.y = y;
                this.z = z;
                this.w = w;
            }
            
            public int[] toIntRGBA() {
                return new int[] {
                    (int)(x * 255),
                    (int)(y * 255),
                    (int)(z * 255),
                    (int)(w * 255)
                };
            }
        }
    }
    
    public boolean hasVisual() {
        return visual != null && visual.geometry != null;
    }
    
    public boolean hasCollision() {
        return collision != null && collision.geometry != null;
    }
    
    public boolean hasInertial() {
        return inertial != null && inertial.isValid();
    }
    
    public float getMass() {
        if (inertial != null && inertial.mass != null) {
            return inertial.mass.value;
        }
        return 1.0f;
    }
}