package com.kAIS.KAIMyEntity.urdf.control;

import java.util.HashSet;
import java.util.Set;

public final class URDFMotionPlayer {
    private URDFMotion motion;
    private float time;
    private boolean playing;

    public void load(URDFMotion m) {
        this.motion = m;
        this.time = 0f;
        this.playing = (m != null && !m.keys.isEmpty());
    }

    public void play(){ if (motion != null && !motion.keys.isEmpty()) playing = true; }
    public void pause(){ playing = false; }
    public void stop(){ playing = false; time = 0f; }
    public boolean isPlaying(){ return playing; }
    public float getTime(){ return time; }
    public void setTime(float t){ this.time = Math.max(0f, t); }
    public URDFMotion getMotion(){ return motion; }

    /** dt마다 호출. setTarget(name, value)로 컨트롤러에 목표 전달 */
    public void update(float dt, java.util.function.BiConsumer<String, Float> setTarget) {
        if (!playing || motion == null || motion.keys.isEmpty()) return;
        time += dt;

        float end = motion.keys.get(motion.keys.size()-1).t;
        if (time > end) {
            if (motion.loop) time = (end > 1e-6f) ? (time % end) : 0f;
            else { time = end; playing = false; }
        }

        // 키 구간 찾기
        URDFMotion.Key a = motion.keys.get(0), b = motion.keys.get(motion.keys.size()-1);
        for (int i=1;i<motion.keys.size();i++){
            if (time <= motion.keys.get(i).t) { a = motion.keys.get(i-1); b = motion.keys.get(i); break; }
        }
        float s = (time - a.t) / Math.max(1e-6f, (b.t - a.t));

        // 두 키의 조인트 합집합
        Set<String> names = new HashSet<>(a.pose.keySet());
        names.addAll(b.pose.keySet());

        for (String n : names) {
            float pa = a.pose.getOrDefault(n, b.pose.getOrDefault(n, 0f));
            float pb = b.pose.getOrDefault(n, pa);
            float p;

            String interp = (b.interp != null ? b.interp : "cubic");
            if ("linear".equalsIgnoreCase(interp)) {
                p = pa + (pb - pa) * s;
            } else {
                // 간단 Hermite (정지-정지 가정)
                float h00 = (2*s*s*s - 3*s*s + 1);
                float h01 = (-2*s*s*s + 3*s*s);
                p = h00*pa + h01*pb;
            }

            setTarget.accept(n, p);
        }
    }
}
