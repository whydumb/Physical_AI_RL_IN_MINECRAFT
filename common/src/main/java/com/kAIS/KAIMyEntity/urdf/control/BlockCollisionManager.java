ackage com.kAIS.KAIMyEntity.urdf.control;

import com.kAIS.KAIMyEntity.PhysicsManager;
import net.minecraft.core.BlockPos;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.shapes.Shapes;
import net.minecraft.world.phys.shapes.VoxelShape;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.util.*;

/**
 * 마인크래프트 블록과 ODE4J 물리 충돌 연동
 *
 * ✅ PATCH:
 * - block collisionShape(VoxelShape) 기반으로 ODE Box Geom 생성 (부분 블록 대응)
 * - 한 블록이 여러 AABB를 가질 수 있으므로 blockGeoms: Map<BlockPos, List<Object>>
 * - 너무 많은 AABB는 bounding box로 fallback (성능/안정성)
 * - BlockState 변경(문/트랩도어 등) 시 geom 재생성 (fullScan에서)
 */
public class BlockCollisionManager {
    private static final Logger logger = LogManager.getLogger();

    private final PhysicsManager physics;

    // ✅ PATCH: 블록당 여러 Geom 지원
    private final Map<BlockPos, List<Object>> blockGeoms = new HashMap<>();
    // ✅ PATCH: state 변경 감지
    private final Map<BlockPos, BlockState> blockStates = new HashMap<>();

    private boolean odeGeomSupported = false;

    // 설정
    private int scanRadius = 8;
    private int updateInterval = 5;
    private int tickCounter = 0;

    private static final double BLOCK_SIZE = 1.0;

    // ✅ PATCH: shape->AABB가 너무 많을 때 제한
    private int maxBoxesPerBlock = 8;
    private boolean fallbackToBoundingBox = true;

    // 너무 얇은 박스 방지 (ODE에 0 크기 박스 들어가는 것 방지)
    private static final double MIN_GEOM_SIZE = 1.0e-6;

    // 통계 (이제 "blocks"보다 "geoms" 개념)
    private int totalGeomsCreated = 0;
    private int totalGeomsRemoved = 0;

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
            // ✅ PATCH: 증분 업데이트 후에도 centerPos 갱신
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

                        // ✅ PATCH: 없으면 생성, 있으면 state 변경 여부 체크
                        if (!blockGeoms.containsKey(immutablePos)) {
                            createBlockGeoms(level, immutablePos, state);
                        } else {
                            BlockState old = blockStates.get(immutablePos);
                            if (old != state) {
                                // state가 바뀌면 shape가 바뀔 수 있음 -> 재생성
                                rebuildBlockGeoms(level, immutablePos, state);
                            }
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
                        } else {
                            // ✅ PATCH: (경계에 한해) state 변경 감지
                            BlockState old = blockStates.get(pos);
                            if (old != null && old != state) {
                                rebuildBlockGeoms(level, pos, state);
                            }
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
                createBlockGeoms(level, pos, level.getBlockState(pos));
            }
        }

        for (BlockPos pos : toRemove) {
            cachedSolidBlocks.remove(pos);
            List<Object> geoms = blockGeoms.remove(pos);
            blockStates.remove(pos);
            if (geoms != null) removeBlockGeoms(geoms);
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
                List<Object> geoms = blockGeoms.remove(p);
                blockStates.remove(p);
                if (geoms != null) removeBlockGeoms(geoms);
            }
        }
    }

    private void removeOutOfRangeBlocks(Set<BlockPos> currentBlocks) {
        Iterator<Map.Entry<BlockPos, List<Object>>> iterator = blockGeoms.entrySet().iterator();
        while (iterator.hasNext()) {
            Map.Entry<BlockPos, List<Object>> entry = iterator.next();
            if (!currentBlocks.contains(entry.getKey())) {
                removeBlockGeoms(entry.getValue());
                blockStates.remove(entry.getKey());
                iterator.remove();
            }
        }
    }

    /**
     * ✅ PATCH: 블록의 collisionShape(VoxelShape)를 여러 AABB로 쪼개 ODE BoxGeom으로 생성
     * - full cube면 1개 박스로 생성
     * - AABB가 너무 많으면 bounding box로 fallback (성능/안정성)
     */
    private void createBlockGeoms(Level level, BlockPos pos, BlockState state) {
        if (!odeGeomSupported || physics == null) return;

        try {
            VoxelShape shape = state.getCollisionShape(level, pos);
            if (shape.isEmpty()) return;

            List<Object> geoms = new ArrayList<>(2);

            // 1) 가장 흔한 케이스: full cube
            if (shape == Shapes.block()) {
                Object geom = physics.createBoxGeom(BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE);
                if (geom != null) {
                    physics.setGeomPosition(
                            geom,
                            pos.getX() + 0.5,
                            pos.getY() + 0.5,
                            pos.getZ() + 0.5
                    );
                    physics.registerStaticGeom(geom);
                    geoms.add(geom);
                }
            } else {
                // 2) 부분/복합 shape: AABB 목록으로 생성
                List<AABB> boxes = shape.toAabbs();
                if (boxes.isEmpty()) return;

                // 너무 많은 박스는 성능/메모리 폭탄 -> 제한/폴백
                if (boxes.size() > maxBoxesPerBlock) {
                    if (fallbackToBoundingBox) {
                        boxes = Collections.singletonList(shape.bounds());
                    } else {
                        boxes = boxes.subList(0, maxBoxesPerBlock);
                    }
                }

                for (AABB bb : boxes) {
                    double sx = bb.maxX - bb.minX;
                    double sy = bb.maxY - bb.minY;
                    double sz = bb.maxZ - bb.minZ;

                    if (sx < MIN_GEOM_SIZE || sy < MIN_GEOM_SIZE || sz < MIN_GEOM_SIZE) {
                        continue;
                    }

                    Object geom = physics.createBoxGeom(sx, sy, sz);
                    if (geom == null) continue;

                    // AABB는 블록 로컬 좌표(0~1) -> 월드 좌표 변환
                    double cx = pos.getX() + (bb.minX + bb.maxX) * 0.5;
                    double cy = pos.getY() + (bb.minY + bb.maxY) * 0.5;
                    double cz = pos.getZ() + (bb.minZ + bb.maxZ) * 0.5;

                    physics.setGeomPosition(geom, cx, cy, cz);
                    physics.registerStaticGeom(geom);
                    geoms.add(geom);
                }
            }

            if (!geoms.isEmpty()) {
                blockGeoms.put(pos, geoms);
                blockStates.put(pos, state);
                totalGeomsCreated += geoms.size();
            }
        } catch (Exception e) {
            logger.debug("Failed to create block geoms at {}: {}", pos, e.getMessage());
        }
    }

    private void rebuildBlockGeoms(Level level, BlockPos pos, BlockState newState) {
        List<Object> old = blockGeoms.remove(pos);
        if (old != null) removeBlockGeoms(old);
        blockStates.remove(pos);

        // 새 state로 재생성
        createBlockGeoms(level, pos, newState);
    }

    private void removeBlockGeoms(List<Object> geoms) {
        if (geoms == null || geoms.isEmpty()) return;
        for (Object g : geoms) {
            removeBlockGeom(g);
        }
    }

    private void removeBlockGeom(Object geom) {
        if (geom == null || physics == null || !odeGeomSupported) return;

        try {
            physics.destroyGeom(geom);
            totalGeomsRemoved++;
        } catch (Exception e) {
            logger.debug("Failed to destroy geom: {}", e.getMessage());
        }
    }

    public void cleanup() {
        logger.info("Cleaning up BlockCollisionManager ({} active blocks, {} active geoms)",
                blockGeoms.size(), getActiveGeomCount());

        for (List<Object> geoms : blockGeoms.values()) {
            removeBlockGeoms(geoms);
        }
        blockGeoms.clear();
        blockStates.clear();
        cachedSolidBlocks.clear();
        lastCenterPos = null;

        logger.info("BlockCollisionManager cleaned up (geoms created: {}, removed: {})",
                totalGeomsCreated, totalGeomsRemoved);
    }

    public void setScanRadius(int radius) {
        this.scanRadius = Math.max(1, Math.min(radius, 16));
    }

    public void setUpdateInterval(int ticks) {
        this.updateInterval = Math.max(1, Math.min(ticks, 20));
    }

    // ✅ PATCH: 과도한 박스 생성 제한 (stairs/fence 같은 복합 shape 대비)
    public void setMaxBoxesPerBlock(int max) {
        this.maxBoxesPerBlock = Math.max(1, Math.min(max, 32));
    }

    public void setFallbackToBoundingBox(boolean enabled) {
        this.fallbackToBoundingBox = enabled;
    }

    public void forceUpdate(Level level, double entityX, double entityY, double entityZ) {
        tickCounter = updateInterval;
        lastCenterPos = null; // 캐시 무효화
        updateCollisionArea(level, entityX, entityY, entityZ);
    }

    public int getActiveBlockCount() {
        return blockGeoms.size();
    }

    public int getActiveGeomCount() {
        int count = 0;
        for (List<Object> list : blockGeoms.values()) {
            if (list != null) count += list.size();
        }
        return count;
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
        info.put("activeGeoms", getActiveGeomCount());
        info.put("odeGeomSupported", odeGeomSupported);
        info.put("scanRadius", scanRadius);
        info.put("updateInterval", updateInterval);
        info.put("maxBoxesPerBlock", maxBoxesPerBlock);
        info.put("fallbackToBoundingBox", fallbackToBoundingBox);
        info.put("totalGeomsCreated", totalGeomsCreated);
        info.put("totalGeomsRemoved", totalGeomsRemoved);
        return info;
    }
}
