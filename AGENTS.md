# AGENTS.md — TuinaDex v2.0 (中医推拿机械臂-灵巧手具身智能系统)

> 本文件是 TuinaDex v2.0 工程专属规则层。
> 全局通用规则位于 `~/.codex/AGENTS.md`，本项目专属的构建、测试、接口契约与硬件红线放这里。

---

## 1. 项目目的
TuinaDex 是面向中医推拿/按摩场景构建的具身智能机器人系统。
当前阶段（MVP）专注于 **16-DOF LEAP Hand 灵巧手** 的最小闭环系统：
`ROS2 运行时 -> 遥操作示教 -> rosbag2 数据记录 -> Dataset Adapter -> LeRobot 策略学习 -> ROS2 真机推理部署`。
后续再无缝平滑扩展至 6-DOF 机械臂与视觉感知系统。

---

## 2. 技术栈与运行环境
- **中间件与框架**: ROS2 Lyrical Luth Desktop / Python 3.14+ / Hugging Face LeRobot
  - 当前宿主机为 Ubuntu 26.04 (Resolute)，已通过官方 ROS apt 源安装 `ros-lyrical-desktop` 和 `ros-dev-tools`。
  - `~/.bashrc` 已固化 `source /opt/ros/lyrical/setup.bash`。
  - `scripts/ros2_docker.sh` 仅作为官方 `ros:lyrical` 镜像的隔离回退环境，不再作为默认运行入口。
- **硬件通信协议**: 
  - LEAP Hand: Dynamixel Protocol 2.0 (USB-TTL, 波特率 4,000,000 bps, 位置-电流模式)
  - 机械臂 (后续): Linux SocketCAN (500 kbps)
- **构建系统**: `colcon build --symlink-install`
- **单测框架**: `pytest tests/`

---

## 3. 核心接口与架构不变量 (Invariants)
1. **坐标与单位契约**:
   - ROS2 接口与 LeRobot 数据集全链路强制 **SI 国际标准单位：弧度 (rad)**；
   - 以实测标定后的 `OPEN_POSE`（全开平伸）为 **0 rad 基准零位**，手指各关节向内弯曲屈曲为正 ($q > 0$)。
2. **状态与动作严格分离**:
   - 反馈状态: `/hand/joint_states` (`sensor_msgs/JointState`)，反映真实物理关节位置；
   - 目标指令: `/hand/joint_command` (`sensor_msgs/JointState`)，反映上层目标位置期望；
   - 动作空间 (Action Space): 严格使用 **16 维 absolute joint position**，绝对禁止使用增量 delta。
3. **双域架构解耦**:
   - **ROS2** 负责机器人运行时（Runtime）：通信、控制循环、硬件抽象、安全看门狗；
   - **LeRobot** 负责策略学习（Learning）：数据集处理、离线训练、策略评测；
   - 严禁 LeRobot 绕过 ROS2 直接读写硬件驱动。
4. **数据事实源契约**:
   - `rosbag2` 是实验原始数据的唯一事实源 (Source of Truth)；
   - `LeRobotDataset` 是下游学习数据产品，由独立离线工具 `tuinadex_dataset` 转换生成。

---

## 4. 关键路径约束与硬件安全红线 (Safety Guardrails)
- **高危操作防范**:
  - 严禁无物理限位保护向电机发送越界绝对弧度；
  - 串口断开或主端手势丢失超过 300ms 时，灵巧手必须触发安全看门狗，平滑阻尼回退至全开平伸姿态 (`Relax-to-OPEN`)；
  - 电机工作电流上限锁定为 `150mA`，杜绝堵转过热或暴力夹捏；
  - 关键控制循环内部禁止执行重型阻塞 I/O 或高耗时复杂计算。
- **硬件需人工确认的操作清单**:
  - 重新烧录/刷写舵机 ID 或固件；
  - 覆盖修改物理绝对零位标定文件 (`poses.json`)；
  - 机械臂上电使能或解除急停。

---

## 5. 完成标准与验证 (Completion Criteria)
声称功能完成前必须提供充分实证：
- ROS2 节点能成功通过 `colcon build` 编译；
- 单元测试覆盖率充足且全绿 (`pytest tests/`)；
- 无物理硬件连接时，提供完整的虚拟仿真/Mock 回环测试或离线转换自检。

### 验证命令 (固化)
- 宿主机纯逻辑与遥操/驱动单测：
  `~/win_office/conda/envs/leap_hand/bin/python -m pytest tests/ -v`
- ROS2 集成测试（宿主机 Lyrical Desktop）：
  `source /opt/ros/lyrical/setup.bash && python3 -m pytest tests/test_leap_hand_node_integration.py tests/test_recorder_node.py -v`
- LeRobot 适配器测试：
  `~/win_office/conda/envs/smolvla/bin/python -m pytest tests/test_dataset_adapter.py -v`
- 构建：
  `source /opt/ros/lyrical/setup.bash && colcon build --symlink-install --packages-select tuinadex_hand tuinadex_teleop tuinadex_recorder`
