package com.kAIS.KAIMyEntity.urdf;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.joml.Vector3f;

import java.io.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

/**
 * STL (STereoLithography) 파일 로더
 * Binary와 ASCII 형식 모두 지원
 */
public class STLLoader {
    private static final Logger logger = LogManager.getLogger();
    
    /**
     * STL 메시 데이터
     */
    public static class STLMesh {
        public List<Triangle> triangles;
        public Vector3f minBounds;
        public Vector3f maxBounds;
        
        public STLMesh() {
            this.triangles = new ArrayList<>();
            this.minBounds = new Vector3f(Float.MAX_VALUE);
            this.maxBounds = new Vector3f(-Float.MAX_VALUE);
        }
        
        public int getTriangleCount() {
            return triangles.size();
        }
        
        public int getVertexCount() {
            return triangles.size() * 3;
        }
        
        public void computeBounds() {
            minBounds.set(Float.MAX_VALUE);
            maxBounds.set(-Float.MAX_VALUE);
            
            for (Triangle tri : triangles) {
                for (int i = 0; i < 3; i++) {
                    Vector3f v = tri.vertices[i];
                    minBounds.x = Math.min(minBounds.x, v.x);
                    minBounds.y = Math.min(minBounds.y, v.y);
                    minBounds.z = Math.min(minBounds.z, v.z);
                    maxBounds.x = Math.max(maxBounds.x, v.x);
                    maxBounds.y = Math.max(maxBounds.y, v.y);
                    maxBounds.z = Math.max(maxBounds.z, v.z);
                }
            }
        }
        
        public Vector3f getCenter() {
            return new Vector3f(
                (minBounds.x + maxBounds.x) / 2,
                (minBounds.y + maxBounds.y) / 2,
                (minBounds.z + maxBounds.z) / 2
            );
        }
        
        public Vector3f getSize() {
            return new Vector3f(
                maxBounds.x - minBounds.x,
                maxBounds.y - minBounds.y,
                maxBounds.z - minBounds.z
            );
        }
    }
    
    /**
     * 삼각형 (법선 + 3개 정점)
     */
    public static class Triangle {
        public Vector3f normal;
        public Vector3f[] vertices;
        
        public Triangle() {
            this.normal = new Vector3f();
            this.vertices = new Vector3f[3];
            for (int i = 0; i < 3; i++) {
                vertices[i] = new Vector3f();
            }
        }
        
        /**
         * 법선 벡터 자동 계산 (반시계 방향 기준)
         */
        public void computeNormal() {
            Vector3f v1 = new Vector3f(vertices[1]).sub(vertices[0]);
            Vector3f v2 = new Vector3f(vertices[2]).sub(vertices[0]);
            normal = v1.cross(v2).normalize();
        }
    }
    
    /**
     * STL 파일 로드 (자동으로 Binary/ASCII 감지)
     */
    public static STLMesh load(String filepath) {
        File file = new File(filepath);
        if (!file.exists()) {
            logger.error("STL file not found: " + filepath);
            return null;
        }
        
        try {
            if (isBinarySTL(file)) {
                logger.info("Loading binary STL: " + filepath);
                return loadBinarySTL(file);
            } else {
                logger.info("Loading ASCII STL: " + filepath);
                return loadASCIISTL(file);
            }
        } catch (IOException e) {
            logger.error("Failed to load STL: " + filepath, e);
            return null;
        }
    }
    
    /**
     * Binary STL인지 확인
     */
    private static boolean isBinarySTL(File file) throws IOException {
        try (RandomAccessFile raf = new RandomAccessFile(file, "r")) {
            if (file.length() < 84) {
                return false; // 너무 작으면 ASCII
            }
            
            // ASCII는 "solid"로 시작
            byte[] header = new byte[5];
            raf.read(header);
            String headerStr = new String(header, StandardCharsets.US_ASCII);
            
            if (headerStr.equals("solid")) {
                // 하지만 Binary도 "solid"로 시작할 수 있음
                // Triangle 개수로 재확인
                raf.seek(80);
                byte[] countBytes = new byte[4];
                raf.read(countBytes);
                int triangleCount = ByteBuffer.wrap(countBytes)
                    .order(ByteOrder.LITTLE_ENDIAN)
                    .getInt();
                
                // Binary STL 크기 = 80(header) + 4(count) + 50 * triangleCount
                long expectedSize = 84L + (50L * triangleCount);
                return file.length() == expectedSize;
            }
            
            return true; // "solid"가 아니면 Binary
        }
    }
    
    /**
     * Binary STL 로드
     */
    private static STLMesh loadBinarySTL(File file) throws IOException {
        STLMesh mesh = new STLMesh();
        
        try (FileInputStream fis = new FileInputStream(file);
             BufferedInputStream bis = new BufferedInputStream(fis)) {
            
            // 80바이트 헤더 스킵
            bis.skip(80);
            
            // Triangle 개수 읽기 (4바이트, little-endian)
            byte[] countBytes = new byte[4];
            bis.read(countBytes);
            int triangleCount = ByteBuffer.wrap(countBytes)
                .order(ByteOrder.LITTLE_ENDIAN)
                .getInt();
            
            logger.debug("Binary STL triangle count: " + triangleCount);
            
            // 각 Triangle 읽기 (50바이트씩)
            for (int i = 0; i < triangleCount; i++) {
                Triangle tri = readBinaryTriangle(bis);
                if (tri != null) {
                    mesh.triangles.add(tri);
                }
            }
            
            mesh.computeBounds();
            logger.info("Loaded " + mesh.getTriangleCount() + " triangles");
            
            return mesh;
        }
    }
    
    /**
     * Binary Triangle 읽기 (50바이트)
     */
    private static Triangle readBinaryTriangle(InputStream is) throws IOException {
        byte[] data = new byte[50];
        int read = is.read(data);
        if (read != 50) {
            return null;
        }
        
        ByteBuffer buffer = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN);
        
        Triangle tri = new Triangle();
        
        // Normal (12바이트)
        tri.normal.x = buffer.getFloat();
        tri.normal.y = buffer.getFloat();
        tri.normal.z = buffer.getFloat();
        
        // Vertex 1 (12바이트)
        tri.vertices[0].x = buffer.getFloat();
        tri.vertices[0].y = buffer.getFloat();
        tri.vertices[0].z = buffer.getFloat();
        
        // Vertex 2 (12바이트)
        tri.vertices[1].x = buffer.getFloat();
        tri.vertices[1].y = buffer.getFloat();
        tri.vertices[1].z = buffer.getFloat();
        
        // Vertex 3 (12바이트)
        tri.vertices[2].x = buffer.getFloat();
        tri.vertices[2].y = buffer.getFloat();
        tri.vertices[2].z = buffer.getFloat();
        
        // Attribute byte count (2바이트) - 무시
        
        // Normal이 0이면 계산
        if (tri.normal.lengthSquared() < 0.0001f) {
            tri.computeNormal();
        }
        
        return tri;
    }
    
    /**
     * ASCII STL 로드
     */
    private static STLMesh loadASCIISTL(File file) throws IOException {
        STLMesh mesh = new STLMesh();
        
        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String line;
            Triangle currentTriangle = null;
            int vertexIndex = 0;
            
            while ((line = reader.readLine()) != null) {
                line = line.trim();
                
                if (line.startsWith("facet normal")) {
                    // 새 삼각형 시작
                    currentTriangle = new Triangle();
                    String[] parts = line.split("\\s+");
                    if (parts.length >= 5) {
                        currentTriangle.normal.x = Float.parseFloat(parts[2]);
                        currentTriangle.normal.y = Float.parseFloat(parts[3]);
                        currentTriangle.normal.z = Float.parseFloat(parts[4]);
                    }
                    vertexIndex = 0;
                    
                } else if (line.startsWith("vertex")) {
                    // 정점 읽기
                    if (currentTriangle != null && vertexIndex < 3) {
                        String[] parts = line.split("\\s+");
                        if (parts.length >= 4) {
                            currentTriangle.vertices[vertexIndex].x = Float.parseFloat(parts[1]);
                            currentTriangle.vertices[vertexIndex].y = Float.parseFloat(parts[2]);
                            currentTriangle.vertices[vertexIndex].z = Float.parseFloat(parts[3]);
                            vertexIndex++;
                        }
                    }
                    
                } else if (line.startsWith("endfacet")) {
                    // 삼각형 완료
                    if (currentTriangle != null) {
                        if (currentTriangle.normal.lengthSquared() < 0.0001f) {
                            currentTriangle.computeNormal();
                        }
                        mesh.triangles.add(currentTriangle);
                        currentTriangle = null;
                    }
                }
            }
            
            mesh.computeBounds();
            logger.info("Loaded " + mesh.getTriangleCount() + " triangles (ASCII)");
            
            return mesh;
        }
    }
    
    /**
     * 메시 스케일 적용
     */
    public static void scaleMesh(STLMesh mesh, Vector3f scale) {
        for (Triangle tri : mesh.triangles) {
            for (int i = 0; i < 3; i++) {
                tri.vertices[i].mul(scale);
            }
        }
        mesh.computeBounds();
    }
    
    /**
     * 메시 센터를 원점으로 이동
     */
    public static void centerMesh(STLMesh mesh) {
        Vector3f center = mesh.getCenter();
        for (Triangle tri : mesh.triangles) {
            for (int i = 0; i < 3; i++) {
                tri.vertices[i].sub(center);
            }
        }
        mesh.computeBounds();
    }
    
    /**
     * 간단한 메시 통계
     */
    public static void printMeshStats(STLMesh mesh) {
        logger.info("=== STL Mesh Statistics ===");
        logger.info("Triangles: " + mesh.getTriangleCount());
        logger.info("Vertices: " + mesh.getVertexCount());
        logger.info("Bounds: " + mesh.minBounds + " to " + mesh.maxBounds);
        logger.info("Size: " + mesh.getSize());
        logger.info("Center: " + mesh.getCenter());
    }
}