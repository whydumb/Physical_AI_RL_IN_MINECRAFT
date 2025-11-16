package com.kAIS.KAIMyEntity.urdf.control;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public final class URDFMotion {
    public String name = "motion";
    public float fps = 60f;
    public boolean loop = true;

    public static final class Key {
        public float t; // seconds
        public Map<String, Float> pose = new HashMap<>(); // joint -> value(rad/m)
        public String interp = "cubic"; // "linear" | "cubic"
    }

    public final List<Key> keys = new ArrayList<>();
}
