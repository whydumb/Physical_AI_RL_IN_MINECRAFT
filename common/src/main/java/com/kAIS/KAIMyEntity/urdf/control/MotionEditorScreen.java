package com.kAIS.KAIMyEntity.urdf.control;

import com.kAIS.KAIMyEntity.urdf.URDFJoint;
import com.kAIS.KAIMyEntity.urdf.URDFModelOpenGLWithSTL;
import net.minecraft.client.Minecraft;
import net.minecraft.client.gui.GuiGraphics;
import net.minecraft.client.gui.components.Button;
import net.minecraft.client.gui.screens.Screen;
import net.minecraft.network.chat.Component;
import org.joml.*;

import java.io.Closeable;
import java.net.*;
import java.nio.charset.StandardCharsets;
import java.util.*;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.ReentrantLock;

/**
 * VMC Motion Editor – 한 파일에 모든 로직 통합
 * -------------------------------------------------
 * 1. VMC UDP/OSC 수신 (VmcListenerManager)
 * 2. 좌표계 변환 (URDFVmcMapper)
 * 3. 3-DoF Arm IK (ArmIK3DoF)
 * 4. Retarget / IK 명령 생성 (URDFArmRetargeter, VmcIk)
 * 5. JointControlBus (우선순위·EMA 스무딩)
 * 6. 화면 (연결 상태·진단·Exit)
 *
 * 사용법
 *   new VmcMotionEditor(renderer, port).open();
 */
public final class VmcMotionEditor {
    private VmcMotionEditor() {}

    private final URDFModelOpenGLWithSTL renderer;
    private final int vmcPort;
    private final JointControlBus bus = new JointControlBus(0.30f);

    private VmcMotionEditor(URDFModelOpenGLWithSTL renderer, int port) {
        this.renderer = renderer;
        this.vmcPort = port;
    }

    /** 화면 열고 VMC 리스너 시작 */
    public static void open(URDFModelOpenGLWithSTL renderer, int port) {
        VmcListenerManager.start(port);
        Minecraft.getInstance().setScreen(new EditorScreen(renderer));
    }

    /** 프레임마다 호출 (Minecraft tick) */
    public static void tick(URDFModelOpenGLWithSTL renderer) {
        VmcDrive.tick(renderer);
    }

    /* --------------------------------------------------------------------- *
     *  화면 – 슬라이더 전부 제거, 진단 정보만 표시
     * --------------------------------------------------------------------- */
    private static final class EditorScreen extends Screen {
        private final URDFModelOpenGLWithSTL renderer;
        private int page = 0;
        private final int perPage = 14;

        EditorScreen(URDFModelOpenGLWithSTL r) {
            super(Component.literal("VMC Motion Editor"));
            this.renderer = r;
        }

        @Override protected void init() { rebuild(); }

        private void rebuild() {
            clearWidgets();
            int y = 10;
            int left = 20;

            // ----- 페이지 -----
            addRenderableWidget(Button.builder(Component.literal("< Prev"), b -> {
                if (page > 0) { page--; rebuild(); }
            }).bounds(left, y, 60, 20).build());

            int total = renderer.getRobotModel().joints.size();
            int pages = Math.max(1, (total + perPage - 1) / perPage);
            addRenderableWidget(Button.builder(Component.literal("Next >"), b -> {
                if (page < pages - 1) { page++; rebuild(); }
            }).bounds(left + 66, y, 60, 20).build());

            Button pageLabel = Button.builder(
                    Component.literal("Page " + (page + 1) + "/" + pages), b -> {})
                    .bounds(left + 132, y, 90, 20).build();
            pageLabel.active = false;
            addRenderableWidget(pageLabel);
            y += 30;

            // ----- 진단 -----
            VmcListenerManager.Diagnostics diag = VmcListenerManager.getDiagnostics();
            List<String> lines = new ArrayList<>();
            lines.add("Running: " + diag.running);
            lines.add("Last packet: " + (System.currentTimeMillis() - diag.lastPacketMs) + " ms ago");
            lines.add("Total: " + diag.totalPackets + "  VMC: " + diag.vmcMsgCount +
                    "  Other: " + diag.nonVmcMsgCount);
            if (!diag.recentAddresses.isEmpty())
                lines.add("Recent: " + String.join(" | ", diag.recentAddresses));

            for (String s : lines) {
                addRenderableWidget(Button.builder(Component.literal(s), b -> {})
                        .bounds(left, y, width - 40, 20).build()).active = false;
                y += 22;
            }
            y += 10;

            // ----- 관절 리스트 (이름만) -----
            int start = page * perPage;
            int end = Math.min(total, start + perPage);
            List<URDFJoint> joints = renderer.getRobotModel().joints;
            for (int i = start; i < end; i++) {
                URDFJoint j = joints.get(i);
                addRenderableWidget(Button.builder(Component.literal(j.name), b -> {})
                        .bounds(left, y, width - 40, 20).build()).active = false;
                y += 22;
            }

            // ----- Exit -----
            addRenderableWidget(Button.builder(Component.literal("Exit"), b -> {
                VmcListenerManager.stop();
                Minecraft.getInstance().setScreen(null);
            }).bounds(width - 70, height - 30, 50, 20).build());
        }

        @Override
        public void render(GuiGraphics g, int mouseX, int mouseY, float pt) {
            renderBackground(g, mouseX, mouseY, pt);
            super.render(g, mouseX, mouseY, pt);
            g.drawCenteredString(font, "VMC Motion Editor – Auto IK/Retarget", width / 2, 2, 0xFFFFFF);
        }
    }

    /* --------------------------------------------------------------------- *
     *  1. VMC UDP/OSC Listener (VmcListenerManager)
     * --------------------------------------------------------------------- */
    private static final class VmcListenerManager {
        private VmcListenerManager() {}
        private static final StateHolder STATE = new StateHolder();
        private static volatile UdpLoop loop;

        public static boolean isRunning() {
            UdpLoop l = loop;
            return l != null && l.isAlive() && !l.isClosed();
        }

        public static synchronized void start(int port) {
            if (isRunning()) return;
            loop = new UdpLoop(port, STATE);
            loop.start();
            System.out.println("[VMC] listener starting on UDP " + port);
        }

        public static synchronized void stop() {
            if (loop != null) { loop.close(); loop = null; }
            System.out.println("[VMC] listener stopped");
        }

        public static Map<String, Object> getBones() {
            Snapshot s = STATE.read();
            return (s == null) ? Collections.emptyMap() : s.boneTransforms;
        }

        public static Snapshot getSnapshot() { return STATE.read(); }

        /* ---------- diagnostics ---------- */
        private static volatile long lastPacketMs = 0;
        private static volatile int totalPackets = 0, vmcMsgCount = 0, nonVmcMsgCount = 0;
        private static final int MAX_ADDR_SAMPLES = 8;
        private static final Deque<String> recentAddrs = new ArrayDeque<>();

        public static final class Diagnostics {
            public final boolean running;
            public final long lastPacketMs;
            public final int totalPackets, vmcMsgCount, nonVmcMsgCount;
            public final List<String> recentAddresses;
            Diagnostics(boolean r, long lp, int t, int v, int n, List<String> a){
                this.running=r; this.lastPacketMs=lp; this.totalPackets=t;
                this.vmcMsgCount=v; this.nonVmcMsgCount=n; this.recentAddresses=a;
            }
        }
        public static Diagnostics getDiagnostics() {
            return new Diagnostics(isRunning(), lastPacketMs, totalPackets,
                    vmcMsgCount, nonVmcMsgCount, new ArrayList<>(recentAddrs));
        }

        /* ---------- state ---------- */
        public static final class Transform {
            public final Vector3f position = new Vector3f();
            public final Quaternionf rotation = new Quaternionf();
        }
        public static final class Snapshot {
            public final Transform rootTransform;
            public final Map<String, Object> boneTransforms;
            public final Map<String, Float> blendShapes;
            public final long lastUpdateNanos;
            Snapshot(Transform r, Map<String,Object> b, Map<String,Float> bl, long t){
                this.rootTransform=r; this.boneTransforms=b; this.blendShapes=bl; this.lastUpdateNanos=t;
            }
        }
        private static final class StateHolder {
            private final ReentrantLock writeLock = new ReentrantLock();
            private Transform rootW = null;
            private final Map<String, Transform> bonesW = new HashMap<>();
            private final Map<String, Float> blendsW = new HashMap<>();
            private final Map<String, Float> blendsPending = new HashMap<>();
            private long lastUpdateW = 0L;
            private final AtomicReference<Snapshot> readRef = new AtomicReference<>(null);

            void write(java.util.function.Consumer<StateHolder> block){
                writeLock.lock();
                try{
                    block.accept(this);
                    Transform rootR = rootW==null?null:new Transform();
                    if(rootR!=null){ rootR.position.set(rootW.position); rootR.rotation.set(rootW.rotation); }
                    Map<String,Object> bonesR = new HashMap<>(bonesW.size());
                    for(var e:bonesW.entrySet()){
                        Transform t=e.getValue(), c=new Transform();
                        c.position.set(t.position); c.rotation.set(t.rotation);
                        bonesR.put(e.getKey(),c);
                    }
                    Map<String,Float> blendsR = new HashMap<>(blendsW);
                    readRef.set(new Snapshot(rootR,
                            Collections.unmodifiableMap(bonesR),
                            Collections.unmodifiableMap(blendsR), lastUpdateW));
                }finally{ writeLock.unlock(); }
            }
            Snapshot read(){ return readRef.get(); }

            void setRoot(float px,float py,float pz,float qx,float qy,float qz,float qw){
                if(rootW==null) rootW=new Transform();
                rootW.position.set(px,py,pz);
                rootW.rotation.set(qx,qy,qz,qw).normalize();
                lastUpdateW=System.nanoTime();
            }
            void setBone(String name,float px,float py,float pz,float qx,float qy,float qz,float qw){
                String key=normalizeBoneName(name);
                Transform t=bonesW.computeIfAbsent(key,k->new Transform());
                t.position.set(px,py,pz);
                t.rotation.set(qx,qy,qz,qw).normalize();
                lastUpdateW=System.nanoTime();
            }
            void setBlendPending(String n,float v){ blendsPending.put(n,v); }
            void applyBlends(){
                if(!blendsPending.isEmpty()){
                    blendsW.putAll(blendsPending); blendsPending.clear();
                    lastUpdateW=System.nanoTime();
                }
            }
            private static String normalizeBoneName(String src){
                if(src==null||src.isEmpty()) return src;
                char c0=Character.toLowerCase(src.charAt(0));
                return src.length()==1?String.valueOf(c0):c0+src.substring(1);
            }
        }

        /* ---------- UDP loop ---------- */
        private static final class UdpLoop extends Thread implements Closeable {
            private final int port; private final StateHolder state;
            private volatile boolean closed=false;
            private DatagramSocket socket;
            UdpLoop(int p,StateHolder s){ this.port=p; this.state=s; }
            public boolean isClosed(){ return closed; }
            @Override public void run(){
                byte[] buf=new byte[2048];
                try{ socket=new DatagramSocket(port); }
                catch(BindException be){
                    System.out.println("[VMC] UDP bind failed on "+port+" : "+be);
                    throw new RuntimeException(be);
                }
                catch(Exception e){ if(!closed) e.printStackTrace(); return; }
                DatagramPacket pkt=new DatagramPacket(buf,buf.length);
                while(!closed){
                    try{ socket.receive(pkt); }
                    catch(SocketException se){ if(closed) break; continue; }
                    int len=pkt.getLength(); if(len<=0) continue;
                    try{ processPacket(buf,len); }
                    catch(Throwable t){ System.out.println("[VMC] decode error: "+t); }
                }
                if(socket!=null) socket.close();
            }
            private void processPacket(byte[] data,int len){
                state.write(sh->{
                    Osc.decode(data,0,len,(addr,args)->{
                        lastPacketMs=System.currentTimeMillis();
                        totalPackets++;
                        boolean isVmc=addr.startsWith("/VMC/Ext/");
                        if(isVmc) vmcMsgCount++; else nonVmcMsgCount++;
                        if(recentAddrs.size()>=MAX_ADDR_SAMPLES) recentAddrs.removeFirst();
                        recentAddrs.addLast(addr);

                        switch(addr){
                            case "/VMC/Ext/Root/Pos":
                            case "/VMC/Ext/Root/Pos/Local":
                                if(args.length>=8 && args[0] instanceof String){
                                    float px=f(args,1),py=f(args,2),pz=f(args,3);
                                    float qx=f(args,4),qy=f(args,5),qz=f(args,6),qw=f(args,7);
                                    sh.setRoot(px,py,pz,qx,qy,qz,qw);
                                }break;
                            case "/VMC/Ext/Bone/Pos":
                            case "/VMC/Ext/Bone/Pos/Local":
                                if(args.length>=8 && args[0] instanceof String bone){
                                    float px=f(args,1),py=f(args,2),pz=f(args,3);
                                    float qx=f(args,4),qy=f(args,5),qz=f(args,6),qw=f(args,7);
                                    sh.setBone(bone,px,py,pz,qx,qy,qz,qw);
                                }break;
                            case "/VMC/Ext/Blend/Val":
                                if(args.length>=2 && args[0] instanceof String name){
                                    sh.setBlendPending(name,f(args,1));
                                }break;
                            case "/VMC/Ext/Blend/Apply":
                                sh.applyBlends(); break;
                        }
                    });
                });
            }
            private static float f(Object[] a,int i){
                Object v=i<a.length?a[i]:0f;
                if(v instanceof Float) return (Float)v;
                if(v instanceof Double) return ((Double)v).floatValue();
                if(v instanceof Number) return ((Number)v).floatValue();
                return 0f;
            }
            @Override public void close(){ closed=true; if(socket!=null) socket.close(); }
        }

        /* ---------- tiny OSC parser ---------- */
        private static final class Osc{
            interface Handler{ void onMessage(String address,Object[] args); }
            static void decode(byte[] buf,int off,int len,Handler h){
                if(startsWith(buf,off,len,"#bundle")){
                    int p=off+padLen("#bundle"); p+=8;
                    while(p+4<=off+len){
                        int sz=readInt(buf,p); p+=4;
                        if(sz<=0||p+sz>off+len) break;
                        decode(buf,p,sz,h); p+=sz;
                    }
                }else decodeMessage(buf,off,len,h);
            }
            private static void decodeMessage(byte[] buf,int off,int len,Handler h){
                int p=off;
                String addr=readString(buf,p,off+len); if(addr==null) return;
                p+=padLen(addr);
                String types=readString(buf,p,off+len);
                if(types==null||types.isEmpty()||types.charAt(0)!=',') return;
                p+=padLen(types);
                List<Object> args=new ArrayList<>(types.length()-1);
                for(int i=1;i<types.length();i++){
                    char t=types.charAt(i);
                    switch(t){
                        case 's':{ String s=readString(buf,p,off+len); if(s==null) return;
                            args.add(s); p+=padLen(s); break; }
                        case 'f':{ if(p+4>off+len) return;
                            args.add(Float.intBitsToFloat(readInt(buf,p))); p+=4; break; }
                        case 'i':{ if(p+4>off+len) return;
                            args.add(readInt(buf,p)); p+=4; break; }
                        default:return;
                    }
                }
                h.onMessage(addr,args.toArray());
            }
            private static boolean startsWith(byte[] b,int off,int len,String s){
                byte[] a=s.getBytes(StandardCharsets.US_ASCII);
                if(len<a.length) return false;
                for(int i=0;i<a.length;i++) if(b[off+i]!=a[i]) return false;
                return true;
            }
            private static int readInt(byte[] b,int p){
                return ((b[p]&0xFF)<<24)|((b[p+1]&0xFF)<<16)|((b[p+2]&0xFF)<<8)|(b[p+3]&0xFF);
            }
            private static String readString(byte[] b,int p,int end){
                int q=p; while(q<end&&b[q]!=0) q++;
                if(q>=end) return null;
                return new String(b,p,q-p,StandardCharsets.US_ASCII);
            }
            private static int padLen(String s){
                int n=s.getBytes(StandardCharsets.US_ASCII).length+1;
                return n+((4-(n%4))&3);
            }
        }
    }

    /* --------------------------------------------------------------------- *
     *  2. 좌표계 변환 (URDFVmcMapper)
     * --------------------------------------------------------------------- */
    private static final class URDFVmcMapper {
        private URDFVmcMapper(){}
        private static volatile float globalScale=1f;
        private static volatile boolean enableCoordTransform=true;
        private static volatile Vector3f positionOffset=new Vector3f();
        private static volatile Quaternionf rotationOffset=new Quaternionf();

        public static void setGlobalScale(float s){ globalScale=s; }
        public static void setEnableCoordTransform(boolean e){ enableCoordTransform=e; }
        public static void setPositionOffset(Vector3f o){ positionOffset=o==null?new Vector3f():new Vector3f(o); }
        public static void setRotationOffset(Quaternionf o){ rotationOffset=o==null?new Quaternionf():new Quaternionf(o).normalize(); }

        public static final class WristTarget{
            public final Vector3f position; public final Quaternionf rotation;
            public WristTarget(Vector3f p,Quaternionf r){ this.position=p; this.rotation=r; }
        }

        private static final Matrix3f C_UNITY_TO_ROS=new Matrix3f(
                0f,0f,1f,
                -1f,0f,0f,
                0f,1f,0f);
        private static final Matrix3f C_T=new Matrix3f(C_UNITY_TO_ROS).transpose();

        private static Vector3f unityToRos(Vector3f v){ return new Vector3f(v.z,-v.x,v.y); }
        private static Quaternionf unityToRos(Quaternionf q){
            Matrix3f rU=new Matrix3f(); q.get(rU);
            Matrix3f rR=new Matrix3f(C_UNITY_TO_ROS).mul(rU).mul(C_T);
            return quatFromMatrix(rR);
        }
        private static Quaternionf quatFromMatrix(Matrix3f m){
            float t=m.m00+m.m11+m.m22;
            float x,y,z,w;
            if(t>0f){
                float S=(float)Math.sqrt(t+1.0f)*2f;
                w=0.25f*S; x=(m.m21-m.m12)/S; y=(m.m02-m.m20)/S; z=(m.m10-m.m01)/S;
            }else if(m.m00>m.m11&&m.m00>m.m22){
                float S=(float)Math.sqrt(1f+m.m00-m.m11-m.m22)*2f;
                w=(m.m21-m.m12)/S; x=0.25f*S; y=(m.m01+m.m10)/S; z=(m.m02+m.m20)/S;
            }else if(m.m11>m.m22){
                float S=(float)Math.sqrt(1f+m.m11-m.m00-m.m22)*2f;
                w=(m.m02-m.m20)/S; x=(m.m01+m.m10)/S; y=0.25f*S; z=(m.m12+m.m21)/S;
            }else{
                float S=(float)Math.sqrt(1f+m.m22-m.m00-m.m11)*2f;
                w=(m.m10-m.m01)/S; x=(m.m02+m.m20)/S; y=(m.m12+m.m21)/S; z=0.25f*S;
            }
            return new Quaternionf(x,y,z,w).normalize();
        }

        public static WristTarget vmcToWristTarget(Object vmc){
            if(vmc==null) return null;
            try{
                Object posObj=reflect(vmc,"position");
                Object rotObj=reflect(vmc,"rotation");
                Vector3f p=reflectVector3f(posObj);
                Quaternionf r=reflectQuaternionf(rotObj);
                if(enableCoordTransform){ p=unityToRos(p); r=unityToRos(r); }
                if(globalScale!=1f) p.mul(globalScale);
                if(rotationOffset!=null) r=new Quaternionf(rotationOffset).mul(r).normalize();
                if(positionOffset!=null) p.add(positionOffset);
                return new WristTarget(p,r);
            }catch(Throwable t){ return null; }
        }
        private static Object reflect(Object o,String name) throws Exception{
            Class<?> c=o.getClass();
            try{ var f=c.getField(name); f.setAccessible(true); return f.get(o); }
            catch(NoSuchFieldException ignored){}
            try{ var m=c.getMethod("get"+cap(name)); return m.invoke(o); }
            catch(NoSuchMethodException ignored){}
            try{ var m=c.getMethod(name); return m.invoke(o); }
            catch(NoSuchMethodException ignored){}
            throw new NoSuchFieldException(name);
        }
        private static Vector3f reflectVector3f(Object o) throws Exception{
            Float x=rc(o,"x"), y=rc(o,"y"), z=rc(o,"z");
            return new Vector3f(x!=null?x:0f,y!=null?y:0f,z!=null?z:0f);
        }
        private static Quaternionf reflectQuaternionf(Object o) throws Exception{
            Float x=rc(o,"x"), y=rc(o,"y"), z=rc(o,"z"), w=rc(o,"w");
            return new Quaternionf(x!=null?x:0f,y!=null?y:0f,z!=null?z:0f,w!=null?w:1f).normalize();
        }
        private static Float rc(Object o,String n) throws Exception{
            Class<?> c=o.getClass();
            try{ var f=c.getField(n); f.setAccessible(true); return castF(f.get(o)); }
            catch(NoSuchFieldException ignored){}
            try{ var m=c.getMethod(n); return castF(m.invoke(o)); }
            catch(NoSuchMethodException ignored){}
            try{ var m=c.getMethod("get"+cap(n)); return castF(m.invoke(o)); }
            catch(NoSuchMethodException ignored){}
            return null;
        }
        private static Float castF(Object v){
            if(v instanceof Float) return (Float)v;
            if(v instanceof Double) return ((Double)v).floatValue();
            if(v instanceof Number) return ((Number)v).floatValue();
            return null;
        }
        private static String cap(String s){
            return s==null||s.isEmpty()?s:Character.toUpperCase(s.charAt(0))+s.substring(1);
        }
    }

    /* --------------------------------------------------------------------- *
     *  3. 3-DoF Arm IK (ArmIK3DoF)
     * --------------------------------------------------------------------- */
    private static final class ArmIK3DoF{
        private ArmIK3DoF(){}
        public static class Result{ public final float yaw,pitch,elbow;
            Result(float y,float p,float e){ yaw=y; pitch=p; elbow=e; } }

        public static Result solve(Vector3f shoulder,Vector3f target,
                                   float[] q0,float L1,float L2,int iters,float lambda){
            float[] q={q0!=null&&q0.length>0?q0[0]:0f,
                       q0!=null&&q0.length>1?q0[1]:0f,
                       q0!=null&&q0.length>2?q0[2]:0f};
            for(int k=0;k<iters;k++){
                Vector3f p=fk(shoulder,q,L1,L2);
                Vector3f e=new Vector3f(target).sub(p);
                if(e.lengthSquared()<1e-6f) break;
                float h=1e-4f; float[][] J=new float[3][3];
                for(int i=0;i<3;i++){
                    float old=q[i];
                    q[i]=old+h; Vector3f pp=fk(shoulder,q,L1,L2);
                    q[i]=old-h; Vector3f pm=fk(shoulder,q,L1,L2);
                    q[i]=old;
                    Vector3f d=pp.sub(pm).mul(1f/(2f*h));
                    J[0][i]=d.x; J[1][i]=d.y; J[2][i]=d.z;
                }
                float[][] JT=tr(J), A=add(mm(JT,J),diag(lambda*lambda));
                float[][] Ai=inv3(A);
                float[] rhs=mv(JT,e), dq=mv(Ai,rhs);
                q[0]+=dq[0]; q[1]+=dq[1]; q[2]+=dq[2];
            }
            return new Result(q[0],q[1],q[2]);
        }
        private static Vector3f fk(Vector3f sh,float[] q,float L1,float L2){
            float yaw=q[0],pitch=q[1],elbow=q[2];
            Matrix3f R=new Matrix3f().rotationZ(yaw).rotateY(pitch);
            Vector3f e=new Vector3f(L1,0,0); R.transform(e).add(sh);
            Matrix3f Re=new Matrix3f(R).rotateY(elbow);
            Vector3f w=new Vector3f(L2,0,0); Re.transform(w);
            return e.add(w);
        }
        private static float[][] tr(float[][]M){ return new float[][]{
                {M[0][0],M[1][0],M[2][0]},{M[0][1],M[1][1],M[2][1]},{M[0][2],M[1][2],M[2][2]} }; }
        private static float[][] mm(float[][]A,float[][]B){ float[][]C=new float[3][3];
            for(int i=0;i<3;i++)for(int j=0;j<3;j++)
                C[i][j]=A[i][0]*B[0][j]+A[i][1]*B[1][j]+A[i][2]*B[2][j]; return C; }
        private static float[] mv(float[][]A,Vector3f v){ return new float[]{
                A[0][0]*v.x+A[0][1]*v.y+A[0][2]*v.z,
                A[1][0]*v.x+A[1][1]*v.y+A[1][2]*v.z,
                A[2][0]*v.x+A[2][1]*v.y+A[2][2]*v.z }; }
        private static float[] mv(float[][]A,float[]x){ return new float[]{
                A[0][0]*x[0]+A[0][1]*x[1]+A[0][2]*x[2],
                A[1][0]*x[0]+A[1][1]*x[1]+A[1][2]*x[2],
                A[2][0]*x[0]+A[2][1]*x[1]+A[2][2]*x[2] }; }
        private static float[][] add(float[][]A,float[][]B){ float[][]C=new float[3][3];
            for(int i=0;i<3;i++)for(int j=0;j<3;j++) C[i][j]=A[i][j]+B[i][j]; return C; }
        private static float[][] diag(float d){ return new float[][]{{d,0,0},{0,d,0},{0,0,d}}; }
        private static float[][] inv3(float[][]a){
            float a00=a[0][0],a01=a[0][1],a02=a[0][2],a10=a[1][0],a11=a[1][1],a12=a[1][2],
                  a20=a[2][0],a21=a[2][1],a22=a[2][2];
            float det=a00*(a11*a22-a12*a21)-a01*(a10*a22-a12*a20)+a02*(a10*a21-a11*a20);
            float id=1f/det;
            return new float[][]{
                    {(a11*a22-a12*a21)*id,(a02*a21-a01*a22)*id,(a01*a12-a02*a11)*id},
                    {(a12*a20-a10*a22)*id,(a00*a22-a02*a20)*id,(a02*a10-a00*a12)*id},
                    {(a10*a21-a11*a20)*id,(a01*a20-a00*a21)*id,(a00*a11-a01*a10)*id} };
        }
    }

    /* --------------------------------------------------------------------- *
     *  4. Retarget / IK 명령 생성 (URDFArmRetargeter + VmcIk)
     * --------------------------------------------------------------------- */
    private static final class URDFArmRetargeter{
        public static final String L_YAW="left_shoulder_yaw_joint",
                L_PITCH="left_shoulder_pitch_joint", L_ELBOW="left_elbow_joint",
                R_YAW="right_shoulder_yaw_joint", R_PITCH="right_shoulder_pitch_joint",
                R_ELBOW="right_elbow_joint";
        private static final float ELBOW_SIGN_LEFT=+1f, ELBOW_SIGN_RIGHT=+1f;

        public Map<String,Float> commands(Map<String,Object> bones){
            Map<String,Float> out=new HashMap<>();
            solve(bones,true,out); solve(bones,false,out);
            return out;
        }
        private void solve(Map<String,Object> b,boolean left,Map<String,Float> out){
            URDFVmcMapper.WristTarget chest=read(b.get("chest"));
            URDFVmcMapper.WristTarget upper=read(b.get(left?"leftUpperArm":"rightUpperArm"));
            URDFVmcMapper.WristTarget lower=read(b.get(left?"leftLowerArm":"rightLowerArm"));
            URDFVmcMapper.WristTarget hand =read(b.get(left?"leftHand":"rightHand"));
            if(upper==null||lower==null||hand==null) return;
            if(chest==null) chest=new URDFVmcMapper.WristTarget(new Vector3f(),new Quaternionf());

            Vector3f sh=upper.position, el=lower.position, wr=hand.position;
            Vector3f u=el.sub(sh,new Vector3f()).normalize();
            Vector3f l=wr.sub(el,new Vector3f()).normalize();

            Vector3f uL=toLocal(u,chest.rotation);
            float yaw  =(float)Math.atan2(uL.y,uL.x);
            float pitch=(float)Math.atan2(uL.z,Math.hypot(uL.x,uL.y));
            float elbow=(float)Math.acos(clamp(u.dot(l),-1f,1f));
            elbow *= left?ELBOW_SIGN_LEFT:ELBOW_SIGN_RIGHT;

            if(left){ out.put(L_YAW,yaw); out.put(L_PITCH,pitch); out.put(L_ELBOW,elbow); }
            else   { out.put(R_YAW,yaw); out.put(R_PITCH,pitch); out.put(R_ELBOW,elbow); }
        }
        private Vector3f toLocal(Vector3f world,Quaternionf parentWorld){
            Matrix3f R=new Matrix3f().set(parentWorld).transpose();
            return new Vector3f(world).mul(R).normalize();
        }
        private float clamp(float v,float lo,float hi){ return v<lo?lo:Math.min(hi,v); }
        private URDFVmcMapper.WristTarget read(Object t){ return t==null?null:URDFVmcMapper.vmcToWristTarget(t); }
    }

    private static final class VmcIk{
        private static float[] qL={0f,0f,0f}, qR={0f,0f,0f};
        public static Map<String,Float> commandsFromBones(Map<String,Object> bones){
            Map<String,Float> out=new HashMap<>();
            solve(bones,true,out); solve(bones,false,out);
            return out;
        }
        private static void solve(Map<String,Object> b,boolean left,Map<String,Float> out){
            URDFVmcMapper.WristTarget up=read(b.get(left?"leftUpperArm":"rightUpperArm"));
            URDFVmcMapper.WristTarget lo=read(b.get(left?"leftLowerArm":"rightLowerArm"));
            URDFVmcMapper.WristTarget hd=read(b.get(left?"leftHand":"rightHand"));
            if(up==null||lo==null||hd==null) return;
            Vector3f sh=up.position, el=lo.position, wr=hd.position;
            float L1=sh.distance(el); if(L1<1e-4f) L1=0.25f;
            float L2=el.distance(wr); if(L2<1e-4f) L2=0.25f;
            float[] q0=left?qL:qR;
            ArmIK3DoF.Result r=ArmIK3DoF.solve(sh,wr,q0,L1,L2,10,1e-3f);
            if(left){ qL=new float[]{r.yaw,r.pitch,r.elbow};
                out.put(URDFArmRetargeter.L_YAW,r.yaw);
                out.put(URDFArmRetargeter.L_PITCH,r.pitch);
                out.put(URDFArmRetargeter.L_ELBOW,r.elbow);
            }else{ qR=new float[]{r.yaw,r.pitch,r.elbow};
                out.put(URDFArmRetargeter.R_YAW,r.yaw);
                out.put(URDFArmRetargeter.R_PITCH,r.pitch);
                out.put(URDFArmRetargeter.R_ELBOW,r.elbow);
            }
        }
        private static URDFVmcMapper.WristTarget read(Object t){
            return t==null?null:URDFVmcMapper.vmcToWristTarget(t);
        }
    }

    /* --------------------------------------------------------------------- *
     *  5. JointControlBus
     * --------------------------------------------------------------------- */
    private static final class JointControlBus{
        public enum Priority{ MANUAL(100), IK(50), RETARGET(10);
            final int v; Priority(int v){this.v=v;} int value(){return v;} }

        private final Map<String,Float> manual=new HashMap<>();
        private final List<SourceBuf> frameSources=new ArrayList<>();
        private final Map<String,Float> ema=new HashMap<>();
        private final float alpha;

        JointControlBus(float a){ this.alpha=a; }

        void push(String name,Priority prio,Map<String,Float> cmds){
            if(cmds==null||cmds.isEmpty()) return;
            frameSources.add(new SourceBuf(name,prio.value(),new HashMap<>(cmds)));
        }
        void setManual(String j,Float rad){
            if(rad==null) manual.remove(j); else manual.put(j,rad);
        }
        void clearManualAll(){ manual.clear(); }

        void resolveAndApply(URDFModelOpenGLWithSTL r){
            frameSources.sort((a,b)->Integer.compare(b.prio,a.prio));
            for(URDFJoint j:r.getRobotModel().joints){
                String n=j.name; Float target=manual.get(n);
                if(target==null){
                    for(SourceBuf s:frameSources){
                        Float v=s.cmds.get(n);
                        if(v!=null){ target=v; break; }
                    }
                }
                if(target==null) continue;
                float lo=j.limit!=null&&j.limit.hasLimits()?j.limit.lower:(float)Math.toRadians(-180);
                float hi=j.limit!=null&&j.limit.hasLimits()?j.limit.upper:(float)Math.toRadians(180);
                if(hi<=lo){ lo=(float)Math.toRadians(-180); hi=(float)Math.toRadians(180); }
                target=Math.max(lo,Math.min(hi,target));
                float y=target;
                Float prev=ema.get(n);
                if(prev!=null) y=prev+(target-prev)*alpha;
                ema.put(n,y);
                r.setJointTarget(n,y);
                r.setJointPreview(n,y);
            }
            frameSources.clear();
        }
        private static final class SourceBuf{
            final String name; final int prio; final Map<String,Float> cmds;
            SourceBuf(String n,int p,Map<String,Float> c){name=n;prio=p;cmds=c;}
        }
    }

    /* --------------------------------------------------------------------- *
     *  6. VmcDrive – 프레임마다 호출
     * --------------------------------------------------------------------- */
    private static final class VmcDrive{
        private static final JointControlBus BUS = new JointControlBus(0.30f);
        private static final URDFArmRetargeter RETARGETER = new URDFArmRetargeter();

        static void tick(URDFModelOpenGLWithSTL renderer){
            Map<String,Object> bones = VmcListenerManager.getBones();
            if(!bones.isEmpty()){
                // 1. Retarget (간단한 방향 기반)
                Map<String,Float> ret = RETARGETER.commands(bones);
                if(!ret.isEmpty())
                    BUS.push("retarget",JointControlBus.Priority.RETARGET,ret);

                // 2. IK (위치 기반)
                Map<String,Float> ik = VmcIk.commandsFromBones(bones);
                if(!ik.isEmpty())
                    BUS.push("ik",JointControlBus.Priority.IK,ik);
            }
            BUS.resolveAndApply(renderer);
        }
    }
}