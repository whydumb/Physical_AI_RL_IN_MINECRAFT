package com.kAIS.KAIMyEntity.urdf;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.util.*;

/**
 * URDF 로봇 모델 컨테이너
 * - 링크/조인트 저장
 * - 이름→객체 인덱스
 * - parent→child 조인트 인덱스
 * - 루트 링크 계산
 */
public class URDFModel {
    private static final Logger logger = LogManager.getLogger();

    public final String name;

    public final List<URDFLink> links = new ArrayList<>();
    public final List<URDFJoint> joints = new ArrayList<>();

    private final Map<String, URDFLink> linkByName = new HashMap<>();
    private final Map<String, URDFJoint> jointByName = new HashMap<>();

    /** parentLinkName -> child joints */
    private final Map<String, List<URDFJoint>> childrenByLink = new HashMap<>();

    /** childLinkName -> parent joint (역추적용, 필요시) */
    private final Map<String, URDFJoint> parentJointByChildLink = new HashMap<>();

    /** 계산된 루트 링크 이름 */
    public String rootLinkName;

    public URDFModel(String name) {
        this.name = name;
    }

    // ========= 조작 유틸 =========

    public void addLink(URDFLink link) {
        if (link == null || link.name == null) return;
        links.add(link);
        linkByName.put(link.name, link);
    }

    public void addJoint(URDFJoint joint) {
        if (joint == null || joint.name == null) return;
        joints.add(joint);
        jointByName.put(joint.name, joint);
    }

    public URDFLink getLink(String name) {
        return linkByName.get(name);
    }

    public URDFJoint getJoint(String name) {
        return jointByName.get(name);
    }

    public int getLinkCount() {
        return links.size();
    }

    public int getJointCount() {
        return joints.size();
    }

    public int getMovableJointCount() {
        int c = 0;
        for (URDFJoint j : joints) if (j.isMovable()) c++;
        return c;
    }

    /** 이름→링크 맵 (읽기 전용 뷰) */
    public Map<String, URDFLink> getLinksByName() {
        return Collections.unmodifiableMap(linkByName);
    }

    /** 이름→조인트 맵 (필요하면) */
    public Map<String, URDFJoint> getJointsByName() {
        return Collections.unmodifiableMap(jointByName);
    }

    /** parent 링크의 자식 조인트 목록(없으면 빈 리스트) */
    public List<URDFJoint> getChildJoints(String parentLink) {
        List<URDFJoint> list = childrenByLink.get(parentLink);
        return (list != null) ? list : Collections.emptyList();
    }

    /** child 링크의 부모 조인트(없으면 null) */
    public URDFJoint getParentJointOf(String childLink) {
        return parentJointByChildLink.get(childLink);
    }

    // ========= 핵심: 트리 구성 =========

    /**
     * - 이름 인덱스 재작성
     * - parent→child 조인트 인덱스 구성
     * - 루트 링크 계산(부모로만 등장하고 자식으로 한 번도 등장하지 않은 링크)
     * - 진단 로그
     */
    public void buildHierarchy() {
        linkByName.clear();
        for (URDFLink l : links) {
            if (l != null && l.name != null) {
                linkByName.put(l.name, l);
            }
        }

        jointByName.clear();
        for (URDFJoint j : joints) {
            if (j != null && j.name != null) {
                jointByName.put(j.name, j);
            }
        }

        childrenByLink.clear();
        parentJointByChildLink.clear();

        // 조인트 검사 및 매핑
        Set<String> allParents = new HashSet<>();
        Set<String> allChildren = new HashSet<>();

        for (URDFJoint j : joints) {
            if (j == null) continue;

            if (isEmpty(j.parentLinkName) || isEmpty(j.childLinkName)) {
                logger.warn("Joint '{}' missing parent/child: parent='{}' child='{}'",
                        j != null ? j.name : "?", j.parentLinkName, j.childLinkName);
                continue;
            }

            if (!linkByName.containsKey(j.parentLinkName)) {
                logger.warn("Joint '{}' parent link '{}' not found in links.", j.name, j.parentLinkName);
                continue;
            }
            if (!linkByName.containsKey(j.childLinkName)) {
                logger.warn("Joint '{}' child link '{}' not found in links.", j.name, j.childLinkName);
                continue;
            }

            childrenByLink
                    .computeIfAbsent(j.parentLinkName, k -> new ArrayList<>())
                    .add(j);
            parentJointByChildLink.put(j.childLinkName, j);

            allParents.add(j.parentLinkName);
            allChildren.add(j.childLinkName);
        }

        // 루트: 부모로 등장했지만 자식으로는 한 번도 등장하지 않은 링크 후보
        String root = null;
        for (String p : allParents) {
            if (!allChildren.contains(p)) {
                root = p;
                break;
            }
        }
        // 모든 링크가 child로만 등장하거나 조인트가 0개인 경우 대비
        if (root == null && !links.isEmpty()) {
            // 최후 수단: 첫 링크를 루트로
            root = links.get(0).name;
            logger.warn("Root link not found by parent/child set difference. Falling back to first link '{}'.", root);
        }

        rootLinkName = root;

        // ====== 진단 로그 ======
        logger.info("=== Hierarchy Built ===");
        logger.info(" links={} joints={} movable={}", links.size(), joints.size(), getMovableJointCount());
        logger.info(" rootLink={}", rootLinkName);

        // 각 링크 자식 수
        int emptyChildLists = 0;
        for (URDFLink l : links) {
            List<URDFJoint> cs = childrenByLink.get(l.name);
            int n = (cs == null ? 0 : cs.size());
            if (n == 0) emptyChildLists++;
            logger.debug(" [children] {} -> {} joint(s)", l.name, n);
        }
        if (emptyChildLists == links.size()) {
            logger.warn("All links report 0 child joints. Check joint parent/child names and parser.");
        }
    }

    private static boolean isEmpty(String s) {
        return s == null || s.isEmpty();
    }
}
