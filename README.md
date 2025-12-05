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

1. IK → FK: 실제 숫자 박아서 비교
파라미터 고정:

관절 수: 
n
=
7
n=7
IK 반복 횟수: 
k
=
5
∼
10
k=5∼10
에피소드 수: 
E
=
10,000
E=10,000
한 에피소드 길이: 
T
=
1,000
T=1,000
전체 스텝 수: 
E
T
=
10
4
×
10
3
=
10
7
ET=10 
4
 ×10 
3
 =10 
7
  스텝
1-1. 한 스텝당 계산량
IK 기반 (전통)

한 스텝당 IK 연산량 ~ 
k
⋅
n
3
k⋅n 
3
 
n
3
=
7
3
=
343
n 
3
 =7 
3
 =343
따라서
k
=
5
k=5: 
5
×
343
=
1,715
5×343=1,715 (단위 연산 수 비례)
k
=
10
k=10: 
10
×
343
=
3,430
10×343=3,430
FK 기반 (네 방식)

한 스텝당 FK 연산량 ~ 
n
=
7
n=7
한 스텝 속도비

속도비
=
k
n
3
n
=
k
n
2
=
k
⋅
49
속도비= 
n
kn 
3
 
​
 =kn 
2
 =k⋅49
k
=
5
k=5: 
49
×
5
=
245
49×5=245배
k
=
10
k=10: 
49
×
10
=
490
49×10=490배
즉, “관절 7개 + IK 5~10회 반복”이면
한 스텝당 IK 대신 FK 쓰면 대략 245~490배 정도 연산 줄어든다는 계산이 됨.

1-2. 전체 학습 (E,T 다 넣고 계산)
전통 IK 기반 전체 비용

C
IK
≈
E
T
⋅
k
n
3
=
10
7
⋅
k
⋅
343
=
3.43
×
10
9
⋅
k
C 
IK
​
 ≈ET⋅kn 
3
 =10 
7
 ⋅k⋅343=3.43×10 
9
 ⋅k
k
=
5
k=5: 
3.43
×
10
9
×
5
=
1.715
×
10
10
3.43×10 
9
 ×5=1.715×10 
10
 
k
=
10
k=10: 
3.43
×
10
9
×
10
=
3.43
×
10
10
3.43×10 
9
 ×10=3.43×10 
10
 
FK 기반 전체 비용

C
FK
≈
E
T
⋅
n
=
10
7
×
7
=
7
×
10
7
C 
FK
​
 ≈ET⋅n=10 
7
 ×7=7×10 
7
 
전체 학습 속도비

C
IK
C
FK
=
3.43
×
10
9
⋅
k
7
×
10
7
=
3.43
7
⋅
10
2
⋅
k
≈
0.49
⋅
100
⋅
k
=
49
k
C 
FK
​
 
C 
IK
​
 
​
 = 
7×10 
7
 
3.43×10 
9
 ⋅k
​
 = 
7
3.43
​
 ⋅10 
2
 ⋅k≈0.49⋅100⋅k=49k
k
=
5
k=5: 약 
245
245배
k
=
10
k=10: 약 
490
490배
=> 전체 학습 기준으로도 IK vs FK의 비율은 그대로 
49
k
49k배, 즉 250~500배 정도.

1-3. 메모리 (대충 스케일 감만)
IK:

야코비안 
J
J (대략 
m
×
n
m×n),
J
J
T
JJ 
T
  같은 
n
×
n
n×n 행렬들,
그 외 임시 버퍼들
⇒ 메모리 스케일: 
O
(
n
2
)
O(n 
2
 )
FK:

관절각 벡터 
θ
∈
R
n
θ∈R 
n
 ,
소수의 
4
×
4
4×4 변환행렬(상수 수준)
⇒ 메모리 스케일: 
O
(
n
)
O(n)
관절 7개면:

IK: 대략 “행렬 몇 개” 단위 → 7×7 여러 개 → 상수×49
FK: 벡터 1개 + 작은 매트릭스 몇 개 → 상수×7
스케일 상 IK 보조 메모리 ≈ FK의 ~7배 수준(이론상)
실제 구현에선 라이브러리 오버헤드까지 포함하면 차이는 더 벌어질 수도 있음.

2. Mesh vs Voxel: 현실적인 수치로
파라미터:

물체 개수: 
N
=
10
∼
50
N=10∼50
한 물체당 정점: 
V
≈
1,000
∼
10,000
V≈1,000∼10,000
면(Face): 대충 
F
≈
V
F≈V 정도라고 보자.
2-1. Mesh 충돌 (나이브하게 잡음)
물체 쌍 개수: 
(
N
2
)
≈
N
2
2
=
O
(
N
2
)
( 
2
N
​
 )≈ 
2
N 
2
 
​
 =O(N 
2
 )
물체 한 쌍 충돌 체크: 최악 기준 
O
(
V
F
)
≈
O
(
V
2
)
O(VF)≈O(V 
2
 )
따라서

C
mesh
∼
O
(
N
2
V
F
)
≈
O
(
N
2
V
2
)
C 
mesh
​
 ∼O(N 
2
 VF)≈O(N 
2
 V 
2
 )
2-2. Voxel (Minecraft 스타일)
가정:

공간을 해상도 고정된 격자(혹은 hash grid)로 쪼갬
로봇이 한 타임스텝에 차지하는 voxel 개수 = 상수 
C
C (로봇 크기 고정이니까)
각 voxel에 대해 “비었냐 / occupancy 있냐” 조회 = hash lookup = 평균 
O
(
1
)
O(1)
그러면:

C
voxel
∼
O
(
C
⋅
1
)
=
O
(
1
)
C 
voxel
​
 ∼O(C⋅1)=O(1)
즉, 물체 개수 N, 정점 수 V, 면 수 F 다 상관없고,
한 스텝당 충돌 체크 복잡도는 상수

2-3. 대충 숫자 박기
중간값 하나 잡자:

N
=
20
N=20
V
=
F
=
5,000
V=F=5,000
그럼:

물체 쌍 개수 ≈ 
20
⋅
19
2
≈
190
2
20⋅19
​
 ≈190
한 쌍당 primitive check upper bound ≈ 
V
F
=
25
×
10
6
VF=25×10 
6
 
총 primitive check:

190
×
25
×
10
6
=
4,750
×
10
6
≈
4.75
×
10
9
190×25×10 
6
 =4,750×10 
6
 ≈4.75×10 
9
 
→ 한 스텝에 대충 50억 번 수준의 primitive 비교(상수는 무시해도 느낌 오지?)

반면 voxel은:

로봇이 차지하는 voxel 수를 100개 정도라 치면 (팔/몸 여러 링크)
100번 hash lookup = 그냥 100회 수준의 O(1) 연산
속도비(매우 거칠게):

C
mesh
C
voxel
≈
4.75
×
10
9
10
2
≈
4.75
×
10
7
C 
voxel
​
 
C 
mesh
​
 
​
 ≈ 
10 
2
 
4.75×10 
9
 
​
 ≈4.75×10 
7
 
→ 약 5천만 배 정도 차이 (물론 실제 엔진은 broad-phase 최적화 하니까 이보다 작지만,
빅오 기준으론 
O
(
N
2
V
F
)
O(N 
2
 VF) vs 
O
(
1
)
O(1)이라 “차원이 다르다”는 건 맞음)

3. 2단계(Offline IK + Online FK) 학습: 손익분기점 숫자
파라미터 다시:

데모 에피소드: 
K
=
100
K=100
학습 에피소드: 
E
=
10,000
E=10,000
에피소드 길이: 
T
=
1,000
T=1,000
관절 수: 
n
=
7
n=7
IK 반복: 
k
=
5
∼
10
k=5∼10
3-1. 전통: 전부 Online IK
C
trad
=
E
T
⋅
(
k
n
3
)
=
10
7
⋅
k
⋅
343
=
3.43
×
10
9
⋅
k
C 
trad
​
 =ET⋅(kn 
3
 )=10 
7
 ⋅k⋅343=3.43×10 
9
 ⋅k
3-2. 2단계 방식
Offline (데모 + IK):
C
offline
=
K
T
⋅
(
k
n
3
)
=
10
5
⋅
k
⋅
343
=
3.43
×
10
7
⋅
k
C 
offline
​
 =KT⋅(kn 
3
 )=10 
5
 ⋅k⋅343=3.43×10 
7
 ⋅k
Online (RL + FK):
C
online
=
E
T
⋅
n
=
10
7
⋅
7
=
7
×
10
7
C 
online
​
 =ET⋅n=10 
7
 ⋅7=7×10 
7
 
합치면:

C
two
=
3.43
×
10
7
⋅
k
+
7
×
10
7
C 
two
​
 =3.43×10 
7
 ⋅k+7×10 
7
 
k
=
5
k=5:

(C_{\text{two}} = 3.43 \times 10^7 \times 5 + 7 \times 10^7 = 1.715 \times 10^8 + 7 \times 10^7 \approx 2.415 \times 10^8)

(C_{\text{trad}} = 3.43 \times 10^9 \times 5 = 1.715 \times 10^{10})

속도비:

C
trad
C
two
≈
1.715
×
10
10
2.415
×
10
8
≈
71
C 
two
​
 
C 
trad
​
 
​
 ≈ 
2.415×10 
8
 
1.715×10 
10
 
​
 ≈71
k
=
10
k=10:

(C_{\text{two}} = 3.43 \times 10^7 \times 10 + 7 \times 10^7 = 3.43 \times 10^8 + 7 \times 10^7 \approx 4.13 \times 10^8)

(C_{\text{trad}} = 3.43 \times 10^9 \times 10 = 3.43 \times 10^{10})

속도비:

C
trad
C
two
≈
3.43
×
10
10
4.13
×
10
8
≈
83
C 
two
​
 
C 
trad
​
 
​
 ≈ 
4.13×10 
8
 
3.43×10 
10
 
​
 ≈83
정리:

네가 설정한 숫자 기준으로

“전부 온라인 IK” vs “데모에서만 IK + 나머지 FK” 비교하면
전체 학습에서 약 70~80배 정도 연산 감소.
이 70~80배는

한 스텝 기준 이론 속도비(245~490배)보다 작은데,
그 이유는: FK도 비용이 0은 아니고, offline IK도 한 번은 해야 해서.
3-3. 손익분기점 
E
E vs 
K
K (감으로 보기)
이론식(상수 무시):

전통: 
C
trad
∼
E
T
k
n
3
C 
trad
​
 ∼ETkn 
3
 
2단계: 
C
two
∼
K
T
k
n
3
+
E
T
n
C 
two
​
 ∼KTkn 
3
 +ETn
두 개가 같은 지점:

K
T
k
n
3
+
E
T
n
=
E
T
k
n
3
KTkn 
3
 +ETn=ETkn 
3
 
T
T로 나누고 정리하면:

K
k
n
2
+
E
=
E
k
n
2
Kkn 
2
 +E=Ekn 
2
 
E
(
k
n
2
−
1
)
=
K
k
n
2
E(kn 
2
 −1)=Kkn 
2
 
E
=
K
k
n
2
k
n
2
−
1
≈
K
(
1
+
1
k
n
2
−
1
)
≈
K
(
1
+
아주 작은 값
)
E= 
kn 
2
 −1
Kkn 
2
 
​
 ≈K(1+ 
kn 
2
 −1
1
​
 )≈K(1+아주 작은 값)
여기서 
n
=
7
,
k
=
5
n=7,k=5면 
k
n
2
=
5
⋅
49
=
245
kn 
2
 =5⋅49=245.

E
≈
245
K
244
≈
1.004
K
E≈ 
244
245K
​
 ≈1.004K
=> E가 K보다 “조금만” 커져도 2단계 방식이 유리해진다는 뜻.
실제로는 
E
=
10,000
E=10,000, 
K
=
100
K=100이라서 (E = 100K)
완전히 2단계 쪽이 이득인 영역 깊숙이 들어와 있음.

4. VRM 모션 프라이어 → 샘플 효율(대충 감)
수학적으로 복잡하게 안 가고, “상태 공간 줄이기” 관점으로만 보자.

전체 상태공간: 
S
S
인간이 VR로 조종하면서 실제로 가는 “그럴듯한” 상태들: 
M
⊂
S
M⊂S
가정:

좋은 정책은 거의 항상 
M
M 안에서만 돌아다닌다.

VRM 프라이어 + 약간의 노이즈로 탐색하면, 실제로도 대부분 
M
M 안을 탐색한다.

많은 RL 이론에서 샘플 복잡도는 대략 “상태 개수에 비례”:

N
need
∝
∣
탐색하는 상태 공간
∣
N 
need
​
 ∝∣탐색하는 상태 공간∣
그럼 두 경우 비교:

아무 프라이어 없음:

N
base
∼
∣
S
∣
N 
base
​
 ∼∣S∣
VRM 기반, 대부분 
M
M만 탐색:

N
prior
∼
∣
M
∣
N 
prior
​
 ∼∣M∣
비율:

N
base
N
prior
∼
∣
S
∣
∣
M
∣
N 
prior
​
 
N 
base
​
 
​
 ∼ 
∣M∣
∣S∣
​
 
만약:

실제로 유효한 “인간스러운” 상태가 전체의 0.1%라 치면

∣
M
∣
/
∣
S
∣
≈
10
−
3
∣M∣/∣S∣≈10 
−3
 
→ 샘플 복잡도 이론상 1000배 감소
0.01%라 치면

→ 10,000배 감소
정확한 숫자는 환경마다 다르지만, 포인트는:

VRM으로 “말이 되는 자세” 근처에서만 탐색하도록 만들면
탐색해야 할 상태 공간의 크기가 orders-of-magnitude 줄어든다
이건 곧
같은 성능을 내기 위한 에피소드/스텝 수가 그만큼 줄어든다는 얘기.
5. 한 줄 정리
연산 복잡도
IK → FK: 한 스텝 기준 대략 250~500배 감소 (
O
(
k
n
3
)
→
O
(
n
)
O(kn 
3
 )→O(n))
Mesh → Voxel: 현실적인 수치 넣으면 수천만 배 이상 차이도 나옴 (
O
(
N
2
V
F
)
→
O
(
1
)
O(N 
2
 VF)→O(1))
2단계 학습(offline IK + online FK):
네가 준 숫자 기준 전체 학습에서 약 70~80배 감소
메모리
IK: 
O
(
n
2
)
O(n 
2
 ), FK: 
O
(
n
)
O(n) → 관절 7개면 대략 7배 스케일 차이
샘플 효율
VRM 모션 프라이어로 “인간스러운 상태공간”만 주로 탐색하면
이론적으로 수백~수만 배 샘플 절약도 가능 (상태 공간 축소 비율에 비례)
