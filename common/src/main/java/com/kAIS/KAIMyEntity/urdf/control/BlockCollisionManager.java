package com.kAIS.KAIMyEntity.urdf.control;

import com.kAIS.KAIMyEntity.PhysicsManager;
import net.minecraft.core.BlockPos;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.Vec3;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.lang.reflect.Method;
import java.util.*;

/**
 * 마인크래프트 블록과 ODE4J 물리 충돌 연동
 *
 * 기능:
 * - 엔티티 주변 블록을 ODE4J static geometry로 변환
 * - 동적으로 충돌 영역 업데이트 (성능 최적화)
 * - 범위 밖 블록 자동 제거
 * - Body-Block 직접 충돌 검사 (ODE4J Geom 없이도 동작)
 */
public class BlockCollisionManager {
    private static final Logger logger = LogManager.getLogger();

    private final PhysicsManager physics;
    private final Map<BlockPos, Object> blockGeoms = new HashMap<>();

    // 더 이상 ODE4J space 를 직접 만들지 않음 (PhysicsManager.space만 사용)
    private boolean odeGeomSupported = false;

    // 설정
    private int scanRadius = 8;
    private int updateInterval = 5;
    private int tickCounter = 0;

    // 마인크래프트 1블록 = ODE4J 1미터
    private static final double BLOCK_SIZE = 1.0;

    // 충돌 응답 설정
    private double bounciness = 0.3;           // 반발 계수
    private double friction = 0.8;             // 마찰 계수
    private double penetrationCorrection = 0.8; // 관통 보정 강도

    // 통계
    private int totalBlocksCreated = 0;
    private int totalBlocksRemoved = 0;
    private int collisionsThisTick = 0;

    // 캐시된 블록 정보 (매 틱 재스캔 방지)
    private BlockPos lastCenterPos = null;
    private Set<BlockPos> cachedSolidBlocks = new HashSet<>();

    public BlockCollisionManager() {
        this.physics = PhysicsManager.GetInst();

        if (physics == null || !physics.isInitialized()) {
            logger.warn("PhysicsManager not available - using fallback collision mode");
            odeGeomSupported = false;
        } else {
            odeGeomSupported = true; // PhysicsManager.space 사용
        }

        logger.info(
            "BlockCollisionManager created (scan radius: {}, update interval: {}, ODE geom: {})",
            scanRadius, updateInterval, odeGeomSupported ? "supported" : "fallback"
        );
    }

    /**
     * 엔티티 주변 블록 충돌 업데이트 (메인 업데이트 메서드)
     */
    public void updateCollisionArea(Level level, double entityX, double entityY, double entityZ) {
        if (level == null) return;

        tickCounter++;
        if (tickCounter < updateInterval) {
            return;
        }
        tickCounter = 0;
        collisionsThisTick = 0;

        BlockPos centerPos = BlockPos.containing(entityX, entityY, entityZ);

        // 위치가 크게 변하지 않았으면 캐시 사용
        if (lastCenterPos != null && lastCenterPos.closerThan(centerPos, 2)) {
            // 기존 캐시 유지, 새 블록만 확인
            updateIncrementally(level, centerPos);
            return;
        }

        // 전체 재스캔
        fullScan(level, centerPos);
        lastCenterPos = centerPos;
    }

    /**
     * 전체 영역 스캔
     */
    private void fullScan(Level level, BlockPos centerPos) {
        Set<BlockPos> currentBlocks = new HashSet<>();

        for (int x = -scanRadius; x <= scanRadius; x++) {
            for (int y = -scanRadius; y <= scanRadius; y++) {
                for (int z = -scanRadius; z <= scanRadius; z++) {
                    BlockPos pos = centerPos.offset(x, y, z);
                    BlockState state = level.getBlockState(pos);

                    if (isSolidForCollision(state)) {
                        BlockPos immutablePos = pos.immutable();
                        currentBlocks.add(immutablePos);

                        if (!blockGeoms.containsKey(immutablePos) && odeGeomSupported) {
                            createBlockGeom(immutablePos, state);
                        }
                    }
                }
            }
        }

        // 범위 밖 블록 제거
        removeOutOfRangeBlocks(currentBlocks);

        // 캐시 업데이트
        cachedSolidBlocks = currentBlocks;
    }

    /**
     * 증분 업데이트 (위치가 조금만 변했을 때)
     */
    private void updateIncrementally(Level level, BlockPos centerPos) {
        // 경계 영역만 체크
        Set<BlockPos> toAdd = new HashSet<>();
        Set<BlockPos> toRemove = new HashSet<>();

        for (int x = -scanRadius; x <= scanRadius; x++) {
            for (int y = -scanRadius; y <= scanRadius; y++) {
                for (int z = -scanRadius; z <= scanRadius; z++) {
                    // 경계 근처만 체크 (최적화)
                    if (Math.abs(x) < scanRadius - 1 &&
                        Math.abs(y) < scanRadius - 1 &&
                        Math.abs(z) < scanRadius - 1) {
                        continue;
                    }

                    BlockPos pos = centerPos.offset(x, y, z).immutable();
                    BlockState state = level.getBlockState(pos);

                    if (isSolidForCollision(state)) {
                        if (!cachedSolidBlocks.contains(pos)) {
                            toAdd.add(pos);
                        }
                    } else {
                        if (cachedSolidBlocks.contains(pos)) {
                            toRemove.add(pos);
                        }
                    }
                }
            }
        }

        // 적용
        for (BlockPos pos : toAdd) {
            cachedSolidBlocks.add(pos);
            if (odeGeomSupported && !blockGeoms.containsKey(pos)) {
                createBlockGeom(pos, level.getBlockState(pos));
            }
        }

        for (BlockPos pos : toRemove) {
            cachedSolidBlocks.remove(pos);
            if (blockGeoms.containsKey(pos)) {
                removeBlockGeom(blockGeoms.remove(pos));
            }
        }
    }

    /**
     * 블록이 충돌 처리 대상인지 확인
     */
    private boolean isSolidForCollision(BlockState state) {
        if (state.isAir()) return false;

        // 완전한 고체 블록
        if (state.isSolid()) return true;

        // 반블록, 계단 등도 포함할 수 있음 (옵션)
        // if (state.getBlock() instanceof SlabBlock) return true;

        return false;
    }

    /**
     * 범위 밖 블록 제거
     */
    private void removeOutOfRangeBlocks(Set<BlockPos> currentBlocks) {
        Iterator<Map.Entry<BlockPos, Object>> iterator = blockGeoms.entrySet().iterator();
        while (iterator.hasNext()) {
            Map.Entry<BlockPos, Object> entry = iterator.next();
            if (!currentBlocks.contains(entry.getKey())) {
                removeBlockGeom(entry.getValue());
                iterator.remove();
            }
        }
    }

    /**
     * PhysicsManager.space 안에 ODE4J Box Geometry 생성
     */
    private void createBlockGeom(BlockPos pos, BlockState state) {
        if (!odeGeomSupported || physics == null) return;

        try {
            // PhysicsManager 래퍼를 통해 Box geom 생성
            Object geom = physics.createBoxGeom(BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE);
            if (geom == null) {
                logger.warn("Failed to create geom for block at {}", pos);
                return;
            }

            // 위치 설정 (블록 중심)
            physics.setGeomPosition(
                geom,
                pos.getX() + 0.5,
                pos.getY() + 0.5,
                pos.getZ() + 0.5
            );

            blockGeoms.put(pos, geom);
            totalBlocksCreated++;

        } catch (Exception e) {
            logger.debug("Failed to create block geom at {}: {}", pos, e.getMessage());
        }
    }

    /**
     * PhysicsManager 래퍼로 ODE4J Geometry 제거
     */
    private void removeBlockGeom(Object geom) {
        if (geom == null || physics == null || !odeGeomSupported) return;

        try {
            physics.destroyGeom(geom);
            totalBlocksRemoved++;
        } catch (Exception e) {
            logger.debug("Failed to destroy geom: {}", e.getMessage());
        }
    }

    // ========== 직접 충돌 검사 API (ODE4J Geom 없이도 사용 가능) ==========

    /**
     * Body와 주변 블록 충돌 검사 및 응답 적용
     * HybridJointController에서 호출
     *
     * @param body      ODE4J Body 객체
     * @param worldPos  엔티티의 월드 좌표
     * @param level     월드
     * @param bodyRadius Body의 대략적인 반경 (충돌 검사용)
     */
    public void handleBodyBlockCollision(Object body, Vec3 worldPos, Level level, double bodyRadius) {
        if (body == null || level == null) return;

        try {
            // Body 위치 가져오기
            double[] bodyPos = physics != null ? physics.getBodyPosition(body) : null;
            if (bodyPos == null) return;

            Vec3 absolutePos = worldPos.add(bodyPos[0], bodyPos[1], bodyPos[2]);

            // Body AABB 계산
            AABB bodyAABB = new AABB(
                absolutePos.x - bodyRadius, absolutePos.y - bodyRadius, absolutePos.z - bodyRadius,
                absolutePos.x + bodyRadius, absolutePos.y + bodyRadius, absolutePos.z + bodyRadius
            );

            // 주변 블록과 충돌 검사
            BlockPos minPos = BlockPos.containing(bodyAABB.minX - 1, bodyAABB.minY - 1, bodyAABB.minZ - 1);
            BlockPos maxPos = BlockPos.containing(bodyAABB.maxX + 1, bodyAABB.maxY + 1, bodyAABB.maxZ + 1);

            for (BlockPos pos : BlockPos.betweenClosed(minPos, maxPos)) {
                BlockState state = level.getBlockState(pos);

                if (!isSolidForCollision(state)) continue;

                // 블록 AABB
                AABB blockAABB = new AABB(
                    pos.getX(), pos.getY(), pos.getZ(),
                    pos.getX() + 1, pos.getY() + 1, pos.getZ() + 1
                );

                // AABB 충돌 검사
                if (bodyAABB.intersects(blockAABB)) {
                    resolveCollision(body, absolutePos, bodyRadius, blockAABB, pos);
                    collisionsThisTick++;
                }
            }

        } catch (Exception e) {
            logger.debug("Body-block collision check failed: {}", e.getMessage());
        }
    }

    /**
     * 충돌 해결 (위치 보정 + 속도 반영)
     */
    private void resolveCollision(Object body, Vec3 bodyPos, double bodyRadius,
                                  AABB blockAABB, BlockPos blockPos) {
        try {
            // 침투 깊이와 방향 계산
            Vec3 blockCenter = new Vec3(
                blockAABB.minX + 0.5,
                blockAABB.minY + 0.5,
                blockAABB.minZ + 0.5
            );

            Vec3 toBody = bodyPos.subtract(blockCenter);

            // 가장 가까운 면 찾기
            double overlapX = (0.5 + bodyRadius) - Math.abs(toBody.x);
            double overlapY = (0.5 + bodyRadius) - Math.abs(toBody.y);
            double overlapZ = (0.5 + bodyRadius) - Math.abs(toBody.z);

            if (overlapX <= 0 || overlapY <= 0 || overlapZ <= 0) return;

            // 최소 침투 축 선택
            Vec3 normal;
            double penetration;

            if (overlapX < overlapY && overlapX < overlapZ) {
                normal = new Vec3(Math.signum(toBody.x), 0, 0);
                penetration = overlapX;
            } else if (overlapY < overlapZ) {
                normal = new Vec3(0, Math.signum(toBody.y), 0);
                penetration = overlapY;
            } else {
                normal = new Vec3(0, 0, Math.signum(toBody.z));
                penetration = overlapZ;
            }

            // 1. 위치 보정
            applyPositionCorrection(body, normal, penetration);

            // 2. 속도 반영 (반발 + 마찰)
            applyVelocityResponse(body, normal);

        } catch (Exception e) {
            logger.debug("Collision resolution failed: {}", e.getMessage());
        }
    }

    /**
     * 위치 보정 적용
     */
    private void applyPositionCorrection(Object body, Vec3 normal, double penetration) {
        if (physics == null) return;

        try {
            double[] currentPos = physics.getBodyPosition(body);
            if (currentPos == null) return;

            double correction = penetration * penetrationCorrection;

            Method setPosition = body.getClass().getMethod(
                "setPosition",
                double.class, double.class, double.class
            );

            setPosition.invoke(
                body,
                currentPos[0] + normal.x * correction,
                currentPos[1] + normal.y * correction,
                currentPos[2] + normal.z * correction
            );

        } catch (Exception e) {
            logger.debug("Position correction failed: {}", e.getMessage());
        }
    }

    /**
     * 속도 응답 적용 (반발 + 마찰)
     */
    private void applyVelocityResponse(Object body, Vec3 normal) {
        if (physics == null) return;

        try {
            double[] vel = physics.getBodyLinearVel(body);
            if (vel == null) return;

            Vec3 velocity = new Vec3(vel[0], vel[1], vel[2]);

            // 법선 방향 속도 성분
            double normalSpeed = velocity.dot(normal);

            // 블록을 향해 이동 중일 때만 반발
            if (normalSpeed < 0) {
                // 법선 방향 속도 분해
                Vec3 normalVel = normal.scale(normalSpeed);
                Vec3 tangentVel = velocity.subtract(normalVel);

                // 새 속도 = 반발된 법선 속도 + 마찰 적용된 접선 속도
                Vec3 newNormalVel = normal.scale(-normalSpeed * bounciness);
                Vec3 newTangentVel = tangentVel.scale(1.0 - friction * 0.1); // 약한 마찰

                Vec3 newVel = newNormalVel.add(newTangentVel);

                Method setLinearVel = body.getClass().getMethod(
                    "setLinearVel",
                    double.class, double.class, double.class
                );
                setLinearVel.invoke(body, newVel.x, newVel.y, newVel.z);
            }

        } catch (Exception e) {
            logger.debug("Velocity response failed: {}", e.getMessage());
        }
    }

    /**
     * 간단한 바닥 충돌 검사 (Y축만)
     * 빠른 지면 체크용
     */
    public double getGroundHeight(Level level, double x, double z, double startY) {
        if (level == null) return startY;

        BlockPos.MutableBlockPos checkPos = new BlockPos.MutableBlockPos();

        for (int y = (int) startY; y >= startY - scanRadius && y >= level.getMinBuildHeight(); y--) {
            checkPos.set((int) Math.floor(x), y, (int) Math.floor(z));
            BlockState state = level.getBlockState(checkPos);

            if (isSolidForCollision(state)) {
                return y + 1.0; // 블록 위쪽 면
            }
        }

        return level.getMinBuildHeight();
    }

    /**
     * 레이캐스트 (특정 방향으로 블록 검사)
     */
    public Optional<BlockPos> raycastToBlock(Level level, Vec3 start, Vec3 direction, double maxDistance) {
        if (level == null) return Optional.empty();

        Vec3 normalizedDir = direction.normalize();
        double step = 0.25; // 정밀도

        for (double d = 0; d < maxDistance; d += step) {
            Vec3 point = start.add(normalizedDir.scale(d));
            BlockPos pos = BlockPos.containing(point);

            if (isSolidForCollision(level.getBlockState(pos))) {
                return Optional.of(pos);
            }
        }

        return Optional.empty();
    }

    // ========== 정리 ==========

    public void cleanup() {
        logger.info("Cleaning up BlockCollisionManager ({} active geoms)", blockGeoms.size());

        for (Object geom : blockGeoms.values()) {
            removeBlockGeom(geom);
        }
        blockGeoms.clear();
        cachedSolidBlocks.clear();

        // 더 이상 space 직접 destroy 안 함 (PhysicsManager가 책임짐)
        logger.info("BlockCollisionManager cleaned up");
    }

    // ========== 설정 ==========

    public void setScanRadius(int radius) {
        this.scanRadius = Math.max(1, Math.min(radius, 16));
    }

    public void setUpdateInterval(int ticks) {
        this.updateInterval = Math.max(1, Math.min(ticks, 20));
    }

    public void setBounciness(double bounce) {
        this.bounciness = Math.max(0, Math.min(bounce, 1.0));
    }

    public void setFriction(double fric) {
        this.friction = Math.max(0, Math.min(fric, 1.0));
    }

    public void forceUpdate(Level level, double entityX, double entityY, double entityZ) {
        tickCounter = updateInterval;
        lastCenterPos = null; // 캐시 무효화
        updateCollisionArea(level, entityX, entityY, entityZ);
    }

    // ========== 정보 조회 ==========

    public int getActiveBlockCount() {
        return odeGeomSupported ? blockGeoms.size() : cachedSolidBlocks.size();
    }

    public int getCollisionsThisTick() {
        return collisionsThisTick;
    }

    public boolean isOdeGeomSupported() {
        return odeGeomSupported;
    }

    public int getScanRadius() {
        return scanRadius;
    }

    public int getUpdateInterval() {
        return updateInterval;
    }

    public Set<BlockPos> getActiveBlockPositions() {
        return new HashSet<>(cachedSolidBlocks);
    }

    /**
     * 디버그 정보 반환
     */
    public Map<String, Object> getDebugInfo() {
        Map<String, Object> info = new HashMap<>();
        info.put("activeBlocks", getActiveBlockCount());
        info.put("odeGeomSupported", odeGeomSupported);
        info.put("scanRadius", scanRadius);
        info.put("updateInterval", updateInterval);
        info.put("collisionsThisTick", collisionsThisTick);
        info.put("totalCreated", totalBlocksCreated);
        info.put("totalRemoved", totalBlocksRemoved);
        info.put("bounciness", bounciness);
        info.put("friction", friction);
        return info;
    }
}
