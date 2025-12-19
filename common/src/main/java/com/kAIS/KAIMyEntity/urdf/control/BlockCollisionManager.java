package com.kAIS.KAIMyEntity.urdf.control;

import com.kAIS.KAIMyEntity.PhysicsManager;
import net.minecraft.core.BlockPos;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.block.state.BlockState;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.util.*;

/**
 * 마인크래프트 블록과 ODE4J 물리 충돌 연동
 */
public class BlockCollisionManager {
    private static final Logger logger = LogManager.getLogger();

    private final PhysicsManager physics;
    private final Map<BlockPos, Object> blockGeoms = new HashMap<>();

    private boolean odeGeomSupported = false;

    // 설정
    private int scanRadius = 8;
    private int updateInterval = 5;
    private int tickCounter = 0;

    private static final double BLOCK_SIZE = 1.0;

    // 통계
    private int totalBlocksCreated = 0;
    private int totalBlocksRemoved = 0;

    // 캐시
    private BlockPos lastCenterPos = null;
    private Set<BlockPos> cachedSolidBlocks = new HashSet<>();

    public BlockCollisionManager() {
        this.physics = PhysicsManager.GetInst();

        if (physics == null || !physics.isInitialized()) {
            logger.warn("PhysicsManager not available - block collision disabled");
            odeGeomSupported = false;
        } else {
            odeGeomSupported = true;
        }

        logger.info(
                "BlockCollisionManager created (scan radius: {}, update interval: {}, ODE geom: {})",
                scanRadius, updateInterval, odeGeomSupported ? "supported" : "disabled"
        );
    }

    public void updateCollisionArea(Level level, double entityX, double entityY, double entityZ) {
        if (level == null || !odeGeomSupported) return;

        BlockPos centerPos = BlockPos.containing(entityX, entityY, entityZ);

        // 최초 1회는 무조건 전체 스캔
        if (lastCenterPos == null) {
            fullScan(level, centerPos);
            lastCenterPos = centerPos;
            tickCounter = 0;
            return;
        }

        tickCounter++;
        if (tickCounter < updateInterval) return;
        tickCounter = 0;

        if (lastCenterPos.closerThan(centerPos, 2)) {
            updateIncrementally(level, centerPos);
            // ✅ PATCH: 증분 업데이트 후에도 centerPos 갱신해야 캐시가 따라감
            lastCenterPos = centerPos;
        } else {
            fullScan(level, centerPos);
            lastCenterPos = centerPos;
        }
    }

    private void fullScan(Level level, BlockPos centerPos) {
        Set<BlockPos> currentBlocks = new HashSet<>();

        for (int x = -scanRadius; x <= scanRadius; x++) {
            for (int y = -scanRadius; y <= scanRadius; y++) {
                for (int z = -scanRadius; z <= scanRadius; z++) {
                    BlockPos pos = centerPos.offset(x, y, z);
                    BlockState state = level.getBlockState(pos);

                    if (isSolidForCollision(level, pos, state)) {
                        BlockPos immutablePos = pos.immutable();
                        currentBlocks.add(immutablePos);

                        if (!blockGeoms.containsKey(immutablePos)) {
                            createBlockGeom(immutablePos, state);
                        }
                    }
                }
            }
        }

        removeOutOfRangeBlocks(currentBlocks);
        cachedSolidBlocks = currentBlocks;
    }

    /**
     * ✅ PATCH: 증분 업데이트 개선
     * - 경계만 체크해서 add/remove
     * - 그리고 현재 center 기준으로 scanRadius 밖으로 나간 cachedSolidBlocks도 제거
     */
    private void updateIncrementally(Level level, BlockPos centerPos) {
        Set<BlockPos> toAdd = new HashSet<>();
        Set<BlockPos> toRemove = new HashSet<>();

        for (int x = -scanRadius; x <= scanRadius; x++) {
            for (int y = -scanRadius; y <= scanRadius; y++) {
                for (int z = -scanRadius; z <= scanRadius; z++) {
                    if (Math.abs(x) < scanRadius - 1 &&
                            Math.abs(y) < scanRadius - 1 &&
                            Math.abs(z) < scanRadius - 1) {
                        continue;
                    }

                    BlockPos pos = centerPos.offset(x, y, z).immutable();
                    BlockState state = level.getBlockState(pos);

                    if (isSolidForCollision(level, pos, state)) {
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

        for (BlockPos pos : toAdd) {
            cachedSolidBlocks.add(pos);
            if (!blockGeoms.containsKey(pos)) {
                createBlockGeom(pos, level.getBlockState(pos));
            }
        }

        for (BlockPos pos : toRemove) {
            cachedSolidBlocks.remove(pos);
            Object geom = blockGeoms.remove(pos);
            if (geom != null) removeBlockGeom(geom);
        }

        // ✅ PATCH: scanRadius 밖으로 벗어난 블록 정리 (center가 조금씩 이동할 때 누적 방지)
        pruneOutsideCube(centerPos);
    }

    /**
     * ✅ PATCH: isSolid() 대신 collision shape로 판정
     * isSolid()가 false인 바닥에서도 충돌 shape가 있으면 geom을 생성하게 함.
     */
    private boolean isSolidForCollision(Level level, BlockPos pos, BlockState state) {
        if (state.isAir()) return false;
        return !state.getCollisionShape(level, pos).isEmpty();
    }

    private void pruneOutsideCube(BlockPos centerPos) {
        if (cachedSolidBlocks.isEmpty()) return;

        int r = scanRadius;

        Iterator<BlockPos> it = cachedSolidBlocks.iterator();
        while (it.hasNext()) {
            BlockPos p = it.next();
            int dx = Math.abs(p.getX() - centerPos.getX());
            int dy = Math.abs(p.getY() - centerPos.getY());
            int dz = Math.abs(p.getZ() - centerPos.getZ());

            if (dx > r || dy > r || dz > r) {
                it.remove();
                Object geom = blockGeoms.remove(p);
                if (geom != null) removeBlockGeom(geom);
            }
        }
    }

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

    private void createBlockGeom(BlockPos pos, BlockState state) {
        if (!odeGeomSupported || physics == null) return;

        try {
            Object geom = physics.createBoxGeom(BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE);
            if (geom == null) {
                logger.warn("Failed to create geom for block at {}", pos);
                return;
            }

            physics.setGeomPosition(
                    geom,
                    pos.getX() + 0.5,
                    pos.getY() + 0.5,
                    pos.getZ() + 0.5
            );

            physics.registerStaticGeom(geom);

            blockGeoms.put(pos, geom);
            totalBlocksCreated++;

        } catch (Exception e) {
            logger.debug("Failed to create block geom at {}: {}", pos, e.getMessage());
        }
    }

    private void removeBlockGeom(Object geom) {
        if (geom == null || physics == null || !odeGeomSupported) return;

        try {
            physics.destroyGeom(geom);
            totalBlocksRemoved++;
        } catch (Exception e) {
            logger.debug("Failed to destroy geom: {}", e.getMessage());
        }
    }

    public void cleanup() {
        logger.info("Cleaning up BlockCollisionManager ({} active geoms)", blockGeoms.size());

        for (Object geom : blockGeoms.values()) {
            removeBlockGeom(geom);
        }
        blockGeoms.clear();
        cachedSolidBlocks.clear();
        lastCenterPos = null;

        logger.info("BlockCollisionManager cleaned up (created: {}, removed: {})",
                totalBlocksCreated, totalBlocksRemoved);
    }

    public void setScanRadius(int radius) {
        this.scanRadius = Math.max(1, Math.min(radius, 16));
    }

    public void setUpdateInterval(int ticks) {
        this.updateInterval = Math.max(1, Math.min(ticks, 20));
    }

    public void forceUpdate(Level level, double entityX, double entityY, double entityZ) {
        tickCounter = updateInterval;
        lastCenterPos = null; // 캐시 무효화
        updateCollisionArea(level, entityX, entityY, entityZ);
    }

    public int getActiveBlockCount() {
        return blockGeoms.size();
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

    public Map<String, Object> getDebugInfo() {
        Map<String, Object> info = new HashMap<>();
        info.put("activeBlocks", getActiveBlockCount());
        info.put("odeGeomSupported", odeGeomSupported);
        info.put("scanRadius", scanRadius);
        info.put("updateInterval", updateInterval);
        info.put("totalCreated", totalBlocksCreated);
        info.put("totalRemoved", totalBlocksRemoved);
        return info;
    }
}
