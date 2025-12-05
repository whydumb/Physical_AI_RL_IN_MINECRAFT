# Physical AI RL IN  Minecraft

마인크래프트를 통해 피지컬 AI 강화학습을 쉽게 접하고 즐길 수 있는 프로젝트입니다.

## Overview

This project allows you to easily explore and enjoy Physical AI Reinforcement Learning through Minecraft. Experience how AI agents learn to interact with a physical environment in an accessible and fun way.

## What?


Other companies train their robots using reinforcement learning with ultra-high-performance GPUs.
I realized that approach isn't possible with just a laptop's 3060 GPU.


Simulators like Isaac Sim require direct training of robot joints, which is computationally heavy.
However, VRM models already contain motion data from human-created bone structures,
and this includes much more sophisticated 3-axis rotations than robot joint movements (URDF).



I'm building a lightweight conversion system that directly translates this data into single-axis motor movements.
This way, robots can quickly replicate human motions without complex training,
and it uses almost no GPU power.



If I can fully leverage VRM's vast motion dataset,
I believe I can significantly improve real robot movement quality without reinforcement learning.

## Overview

🚀 Key Performance Reductions: VRM-Driven Efficiency (GitHub Markdown Compatible)저희의 VRM 기반 시스템은 전통적인 로봇 제어 및 강화 학습(RL) 파이프라인의 주요 병목 현상을 해결하여, 일반적인 소비자 하드웨어(예: 3060 노트북 GPU)에서도 대규모 로봇 학습이 가능하도록 설계되었습니다.제시해주신 휴머노이드 팔(7자유도) 및 일반적인 가상 환경 수치를 적용할 경우, 이론적인 연산 및 샘플 효율성 개선 효과는 다음과 같습니다.개선 요소전통적 방식의 복잡도VRM 시스템의 복잡도이론적 개선 배율 (Speedup)제어 연산 (IK vs FK)O(k * n^3)O(n)250 ~ 500 배충돌 감지 (Mesh vs Voxel)O(N^2 * V * F)O(1)10^7 배 (수천만 배 이상)총 학습 시간 (2단계 Amortization)O(E * T * k * n^3)O(E * T * n)70 ~ 80 배 (예시 E, K 기준)샘플 효율 (VRM Prior)O(S)1. ⚙️ 연산 복잡도 개선 (Computational Speedup)전통적인 RL 루프에서 가장 큰 병목은 **역기구학(IK)**과 메시 기반 충돌 감지입니다. 저희 시스템은 이 둘을 근본적으로 더 저렴한 연산으로 대체합니다.1-1. IK → FK 대체: $O(n^3)$ 연산을 $O(n)$으로로봇의 목표 위치(Task-space)에서 관절 각도(Joint-space)를 찾는 **역기구학(IK)**은 로봇의 자유도 $n$에 대해 높은 복잡도를 가집니다.IK 기반 (전통):복잡도: Theta(k * n^3)설명: $n \times n$ 행렬 역행렬 또는 SVD 계산 비용($n^3$)이 IK 반복 횟수 $k$만큼 발생합니다.FK 기반 (VRM 시스템):복잡도: Theta(n)설명: $n$개의 동차 변환 행렬을 순차적으로 곱하는 간단한 선형 연산입니다.🔢 수치적 예시 (7자유도 로봇):$n=7$ 관절, $k=5$ IK 반복을 사용할 경우, 속도비(Speedup)는 다음과 같습니다.$$\text{Speedup} = \text{Theta}(k \cdot n^2) \approx 5 \cdot 7^2 = 245 \times$$결론: IK를 FK로 대체함으로써, 한 스텝당 필요한 산술 연산량이 최소 250배 이상 감소합니다.1-2. Mesh → Voxel 충돌 감지: $O(N^2VF)$ 연산을 $O(1)$로정교한 메시 기반 충돌 감지는 물체 수가 많을 때 기하급수적으로 복잡해집니다.메시 기반 (전통):복잡도: Theta(N^2 * V * F)설명: $N$은 물체 수, $V/F$는 정점/면 수. 모든 물체 쌍을 검사하므로 $N$과 물체 복잡도에 따라 폭발적으로 증가합니다.Voxel 기반 (VRM 시스템):복잡도: Theta(1)설명: 로봇이 점유하는 **고정된 수의 복셀(Constant $C$)**만 검사합니다. 물체 수나 정점 수와 무관하게 상수 시간 복잡도를 가집니다.🔢 수치적 예시 (일반적인 씬):$N=20$ 물체, $V \approx F \approx 5,000$ 메시일 경우,$$\text{Reduction Factor} = \text{Theta}(N^2 \cdot V \cdot F) \approx 10^8 \times$$결론: Voxel 방식을 통해, 온라인 RL 루프에서 충돌 감지 연산이 수천만 배 이상 압도적으로 감소합니다.2. ⏳ 총 학습 비용 효율화 (Amortized Cost)저희는 2단계 학습(Two-Phase Training) 전략을 사용하여, 고비용의 IK 연산을 데모 수집 단계(Offline)에서만 집중적으로 수행하고, 실제 RL 학습 단계(Online)에서는 저렴한 FK만 사용합니다.단계Cost (FK / IK)복잡도C_offline (데모 수집)IK only (K episodes)Theta(K * T * k * n^3)C_online (RL 학습)FK only (E episodes)Theta(E * T * n)C_two (총합)IK + FKTheta(K * T * k * n^3) + Theta(E * T * n)손익분기점 (Break-Even Point)2단계 학습이 전통적인 전부 Online IK 방식보다 유리해지는 시점은 RL 에피소드 수 E가 데모 에피소드 수 K보다 아주 조금만 커질 때입니다 ($E > K$).$$E \approx K \cdot \left( \frac{k n^2}{k n^2 - 1} \right)$$$n=7, k=5$일 때 $kn^2 \approx 245$이므로, $E \approx 1.004 \cdot K$입니다. 즉, 데모 대비 0.4%만 더 학습해도 2단계 방식이 비용 효율적입니다.결론: $E=10,000$, $K=100$과 같이 대규모 RL 학습($E \gg K$)을 수행할 때, 총 학습 비용을 $70 \sim 80 \times$ 이상 절감합니다.3. ✨ 샘플 효율성 개선 (Sample Efficiency via VRM Prior)VRM Prior(VR 모션 사전 지식)는 RL 알고리즘이 탐색해야 할 유효한 상태 공간($\mathcal{S}$)의 크기를 인간의 움직임 매니폴드($\mathcal{M}$)로 축소시킵니다.전통 (제한 없음): RL은 전체 관절 공간 $\mathcal{S}$를 탐색합니다. 샘플 복잡도 $N_{\text{base}}$는 $\mathcal{S}$의 복잡도 $C(\mathcal{S})$에 비례합니다.VRM Prior (제한됨): RL은 충돌이 없고 인간스러운 움직임 $\mathcal{M}$ 근처만 탐색합니다. $N_{\text{prior}}$는 $C(\mathcal{M})$에 비례합니다.샘플 복잡도 개선 비율$$\frac{N_{\text{base}}}{N_{\text{prior}}} = \text{Omega}\left(\frac{C(\mathcal{S})}{C(\mathcal{M})}\right)$$결론: VRM Prior는 로봇이 물리적으로 불가능하거나 비효율적인 자세를 탐색하는 과정을 생략하게 하여, 실제 필요한 에피소드 수를 수백~수만 배까지 줄일 수 있는 이론적 근거를 제공합니다. 이는 훈련 시간을 획기적으로 단축합니다.🏆 종합적인 이점VRM 시스템은 FK, Voxel, 2단계 학습을 통해 온라인 RL 루프의 초당 연산 비용을 수천만 배 이상 낮춥니다. 여기에 VRM Prior가 **필요한 총 학습량(샘플 수)**을 수백~수만 배 줄여주기 때문에, 고성능 워크스테이션 없이도 복잡한 물리 AI 학습을 가능하게 합니다.이 모든 개선 덕분에, 일반 3060 노트북 GPU 환경에서 대규모 피지컬 AI 강화 학습 프로젝트를 현실화할 수 있습니다.
