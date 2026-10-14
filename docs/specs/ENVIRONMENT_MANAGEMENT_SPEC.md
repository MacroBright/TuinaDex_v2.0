# ENVIRONMENT_MANAGEMENT_SPEC

> 本文件是 TuinaDex v2.0 多端异构环境管理架构设计规范的索引入口。  
> 完整规范原文位于：[`docs/superpowers/specs/2026-10-14-environment-management-architecture-design.md`](../superpowers/specs/2026-10-14-environment-management-architecture-design.md)

---

### 核心结论速查

1. **环境与节点分工**：
   - **云端 GPU**：`ENV-CLOUD` (Docker + CUDA 12.x + PyTorch)，运行 VLA 大模型 Action Chunking 推流服务与 LeRobot 策略离线训练。
   - **香橙派 AI Pro 20T**：`ENV-EDGE` (Ubuntu 22.04 aarch64 + CANN 8.x + ROS 2 Humble)，运行 30fps NPU 穴位识别、SocketCAN + USB-TTL 底层控制中枢与 300ms 安全看门狗。
   - **本机 PC (Legion Y9000P)**：`ENV-DEV-HW` (Ubuntu 26.04 + ROS 2 Lyrical + Conda `arm_robot`/`leap_hand`)，专职遥操作专家示教采集、MuJoCo 仿真与单元测试；`ENV-DATA` (Docker) 进行无污染数据集转换。
2. **跨端通信核心基线**：
   - 全网统一使用 **Eclipse CycloneDDS**（`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`）；
   - 统一域 `ROS_DOMAIN_ID=42`；
   - 跨版本仅传输 ROS 2 标准消息（`sensor_msgs`、`geometry_msgs`、`std_msgs`）。
3. **安全看门狗阈值**：
   - 动作推流超时：`> 300ms` 触发阻尼减速回退态 (`Relax-to-OPEN`)；
   - 灵巧手电流上限：`150 mA`；
   - 机械臂安全工作空间：$\le 70\%$ 臂展。
