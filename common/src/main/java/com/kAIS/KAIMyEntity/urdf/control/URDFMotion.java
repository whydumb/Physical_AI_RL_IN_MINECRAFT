package com.kAIS.KAIMyEntity.urdf.control;

import java.util.*;

/**
 * URDF 관절 기반 모션 (VMD 등에서 변환된 결과)
 */
public final class URDFMotion {
    public String name = "motion";
    public float fps = 60f;
    public boolean loop = true;

    public static final class Key {
        /** 시간 (초 단위) */
        public float t; // seconds
        /** 조인트 이름 -> 값(rad 또는 m) */
        public Map<String, Float> pose = new HashMap<>();
        /** 보간 타입 ("linear" | "cubic" 등) - 현재 구현은 linear만 사용 */
        public String interp = "cubic";
    }

    /** 시간 순 정렬된 키프레임 목록 */
    public final List<Key> keys = new ArrayList<>();

    /**
     * 모션 전체 길이 (초)
     */
    public float getDuration() {
        if (keys.isEmpty()) return 0f;
        return keys.get(keys.size() - 1).t;
    }

    /**
     * 주어진 시간 t 에서의 포즈(joint -> value)를 반환.
     * - loop == true: duration 기준으로 t를 모듈로 연장
     * - loop == false: 양 끝으로 클램프
     * - 현재는 선형 보간(linear)만 사용
     */
    public Map<String, Float> samplePose(float t) {
        if (keys.isEmpty()) return Collections.emptyMap();

        // --- 1) 시간 범위 처리 (loop/clamp) ---
        float duration = getDuration();
        if (duration <= 0f) {
            // 키가 1개만 있는 경우 등
            return new HashMap<>(keys.get(0).pose);
        }

        if (loop) {
            // 0 ~ duration 범위로 래핑
            t = t % duration;
            if (t < 0f) t += duration;
        } else {
            // 양 끝으로 클램프
            if (t <= keys.get(0).t) {
                return new HashMap<>(keys.get(0).pose);
            }
            if (t >= duration) {
                return new HashMap<>(keys.get(keys.size() - 1).pose);
            }
        }

        // --- 2) t가 속한 구간 [prev, next] 찾기 ---
        Key prev = keys.get(0);
        Key next = keys.get(keys.size() - 1);

        for (int i = 0; i < keys.size() - 1; i++) {
            Key k0 = keys.get(i);
            Key k1 = keys.get(i + 1);
            if (k0.t <= t && t <= k1.t) {
                prev = k0;
                next = k1;
                break;
            }
        }

        // --- 3) 선형 보간 계수 계산 ---
        float dt = next.t - prev.t;
        float alpha = (dt > 1e-6f) ? (t - prev.t) / dt : 0f;
        if (alpha < 0f) alpha = 0f;
        if (alpha > 1f) alpha = 1f;

        // --- 4) 조인트별 선형 보간 ---
        Map<String, Float> result = new HashMap<>();

        // prev 기준으로 돌면서 next에 값 있으면 보간, 없으면 prev 값 유지
        for (Map.Entry<String, Float> e : prev.pose.entrySet()) {
            String joint = e.getKey();
            float v0 = e.getValue();
            float v1 = next.pose.getOrDefault(joint, v0);
            float v = v0 + alpha * (v1 - v0);
            result.put(joint, v);
        }

        // prev에는 없고 next에만 있는 조인트도 포함시킬지 여부는 선택인데,
        // 보통은 rare case라 안 넣어도 큰 문제는 없음.
        // 넣고 싶으면 아래 주석 해제:
        /*
        for (Map.Entry<String, Float> e : next.pose.entrySet()) {
            String joint = e.getKey();
            if (result.containsKey(joint)) continue;
            float v1 = e.getValue();
            float v0 = prev.pose.getOrDefault(joint, v1);
            float v = v0 + alpha * (v1 - v0);
            result.put(joint, v);
        }
        */

        return result;
    }
}
