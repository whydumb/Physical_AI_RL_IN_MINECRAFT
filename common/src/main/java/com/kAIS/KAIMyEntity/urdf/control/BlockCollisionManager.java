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
 *
 * 좌표계:
 * - ODE World = 엔티티 로컬 좌표계
 * - 블록 지오메트리도 엔티티 기준 로컬 좌표로 변환하여 ODE에 넣는다.
 *
 * 기능:
 * - 엔티티 주변 블록을 ODE4J static geometry로 변환
 * - 동적으로 충돌 영역 업데이트 (성능 최적화)
 * - 범위 밖 블록 자동 제거
 * - ODE4J의 space.collide가 자동으로 충돌 처리
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

    // 마인크래프트 1블록 = ODE4J 1미터
    private static final double BLOCK_SIZE = 1.0;

    // 통계
    private int totalBlocksCreated = 0;
    private int totalBlocksRemoved = 0;

    // 캐시된 블록 정보 (매 틱 재스캔 방지)
    private BlockPos lastCenterPos = null;
    private Set<BlockPos> cachedSolidBlocks = new HashSet<>();

    // ★ 추가: 마지막으로 updateCollisionArea가 호출됐을 때의 엔티티 월드 좌표
    // ODE World = 엔티티 로컬 좌표계이기 때문에,
    // 블록 지오메트리를 넣을 때 (블록 월드) - (엔티티 월드) 로 변환해서 사용한다.
    private double lastEntityX = 0.0;
    private double lastEntityY = 0.0;
    private double lastEntityZ = 0.0;

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

    /**
     * 엔티티 주변 블록 충돌 업데이트 (메인 업데이트 메서드)
     * static ODE geom만 관리, 실제 충돌은 physics.step 내부에서 처리됨
     *
     * @param level   현재 월드
     * @param entityX 엔티티 월드 X
     * @param entityY 엔티티 월드 Y
     * @param entityZ 엔티티 월드 Z
     */
    public void updateCollisionArea(Level level, double entityX, double entityY, double entityZ) {
        if (level == null || !odeGeomSupported) return;

        // ★ 엔티티의 월드 좌표를 항상 최신으로 저장
        // 이후 createBlockGeom 에서 "월드 → 엔티티 로컬" 변환에 사용
        this.lastEntityX = entityX;
        this.lastEntityY = entityY;
        this.lastEntityZ = entityZ;

        tickCounter++;
        if (tickCounter < updateInterval) {
            return;
        }
        tickCounter = 0;

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

                        if (!blockGeoms.containsKey(immutablePos)) {
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
            if (!blockGeoms.containsKey(pos)) {
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
     * ODE의 space.collide가 자동으로 이 geom을 인식함
     *
     * 여기서 **ODE World = 엔티티 로컬 좌표계**를 만족시키기 위해
     * (블록 월드 좌표) - (엔티티 월드 좌표) 로 로컬 변환해서 ODE에 넣는다.
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

            // 1) 블록의 마인크래프트 "월드" 좌표 (블록 중심)
            double wx = pos.getX() + 0.5;
            double wy = pos.getY() + 0.5;
            double wz = pos.getZ() + 0.5;

            // 2) ★ 엔티티 "로컬" 좌표계로 변환:
            //    로컬 = 월드 - 엔티티월드
            //    (ODE World = 엔티티 로컬)
            double lx = wx - lastEntityX;
            double ly = wy - lastEntityY;
            double lz = wz - lastEntityZ;

            // 3) ODE 지오메트리 위치를 "엔티티 로컬" 좌표로 설정
            physics.setGeomPosition(geom, lx, ly, lz);

            // 정적 geom으로 등록 (self-collision 제어용)
            physics.registerStaticGeom(geom);

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

    // ========== 정리 ==========

    public void cleanup() {
        logger.info("Cleaning up BlockCollisionManager ({} active geoms)", blockGeoms.size());

        for (Object geom : blockGeoms.values()) {
            removeBlockGeom(geom);
        }
        blockGeoms.clear();
        cachedSolidBlocks.clear();

        logger.info("BlockCollisionManager cleaned up (created: {}, removed: {})",
                totalBlocksCreated, totalBlocksRemoved);
    }

    // ========== 설정 ==========

    public void setScanRadius(int radius) {
        this.scanRadius = Math.max(1, Math.min(radius, 16));
    }

    public void setUpdateInterval(int ticks) {
        this.updateInterval = Math.max(1, Math.min(ticks, 20));
    }

    public void forceUpdate(Level level, double entityX, double entityY, double entityZ) {
        // 강제 업데이트 시에도 엔티티 월드 좌표를 제대로 쓰기 위해
        // updateCollisionArea 안에서 lastEntityX/Y/Z가 갱신된다.
        tickCounter = updateInterval;
        lastCenterPos = null; // 캐시 무효화
        updateCollisionArea(level, entityX, entityY, entityZ);
    }

    // ========== 정보 조회 ==========

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

    /**
     * 디버그 정보 반환
     */
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
