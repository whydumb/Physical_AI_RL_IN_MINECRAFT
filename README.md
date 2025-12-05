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
