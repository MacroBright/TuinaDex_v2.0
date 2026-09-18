# TuinaDex v2.0 — LEAP Hand MVP 接口规范与数据契约规范 (LEAP_HAND_MVP_INTERFACE_SPEC)

> **版本**: v1.0.0 (架构冻结版)  
> **生效日期**: 2026-09-18  
> **适用范围**: `tuinadex_hand` (运行时), `tuinadex_teleop` (遥操), `tuinadex_recorder` (采集), `tuinadex_dataset` (适配器)

---

## 1. 16-DOF 关节空间与 Canonical Ordering

系统严格固定 16 维关节顺序（Index 0 ~ 15）：

| 索引 (Index) | Canonical 命名 | 舵机 ID | 手指与关节结构 | 运动方向符号 (`JOINT_DIR`) | 关节限位区间 (rad, 相对全开位 0) |
| :---: | :--- | :---: | :--- | :---: | :---: |
| **0** | `index_yaw` | 0 | 食指 MCP 侧摆 (Abduction) | -1 | $[-0.35, +0.35]$ |
| **1** | `index_mcp` | 1 | 食指 MCP 俯仰屈曲 (Flexion) | -1 | $[0.0, +2.00]$ |
| **2** | `index_pip` | 2 | 食指 PIP 近端指间屈曲 | -1 | $[0.0, +1.85]$ |
| **3** | `index_dip` | 3 | 食指 DIP 远端指间屈曲 | -1 | $[0.0, +1.80]$ |
| **4** | `middle_yaw` | 4 | 中指 MCP 侧摆 (Abduction) | -1 | $[-0.35, +0.35]$ |
| **5** | `middle_mcp` | 5 | 中指 MCP 俯仰屈曲 (Flexion) | -1 | $[0.0, +2.00]$ |
| **6** | `middle_pip` | 6 | 中指 PIP 近端指间屈曲 | -1 | $[0.0, +1.85]$ |
| **7** | `middle_dip` | 7 | 中指 DIP 远端指间屈曲 | -1 | $[0.0, +1.80]$ |
| **8** | `ring_yaw` | 8 | 无名指 MCP 侧摆 (Abduction) | -1 | $[-0.35, +0.35]$ |
| **9** | `ring_mcp` | 9 | 无名指 MCP 俯仰屈曲 (Flexion) | -1 | $[0.0, +2.00]$ |
| **10** | `ring_pip` | 10 | 无名指 PIP 近端指间屈曲 | -1 | $[0.0, +1.85]$ |
| **11** | `ring_dip` | 11 | 无名指 DIP 远端指间屈曲 | -1 | $[0.0, +1.80]$ |
| **12** | `thumb_yaw` | 12 | 拇指 根部回转 (CMC Abduction) | **+1** | $[-0.30, +1.80]$ |
| **13** | `thumb_cmc` | 13 | 拇指 CMC 俯仰屈曲 | -1 | $[0.0, +1.60]$ |
| **14** | `thumb_mcp` | 14 | 拇指 MCP 屈曲 | -1 | $[0.0, +1.80]$ |
| **15** | `thumb_ip` | 15 | 拇指 IP 指尖屈曲 | -1 | $[0.0, +1.80]$ |

---

## 2. 坐标系统与双向数学转换契约

定义：
- $\boldsymbol{q} \in \mathbb{R}^{16}$：机器人关节空间弧度。$q_i = 0$ 表示全开平伸基准姿势；$q_i > 0$ 表示指关节屈曲弯曲；单位：$\text{rad}$。
- $\boldsymbol{\theta}_{\text{dxl}} \in \mathbb{R}^{16}$：Dynamixel 舵机底层绝对弧度（取值范围大致在 $2.8 \sim 4.8\text{ rad}$）。
- $\boldsymbol{\theta}_{\text{OPEN\_POSE}} \in \mathbb{R}^{16}$：真实硬件实测标定的全开平伸绝对弧度。
- $\mathbf{J}_{\text{dir}} = \text{diag}(\mathbf{JOINT\_DIR}) \in \mathbb{R}^{16 \times 16}$：方向符号对角阵。

### 2.1 指令下发转换 (Robot Command $\to$ DXL Hardware)
$$\boldsymbol{\theta}_{\text{cmd\_dxl}} = \text{clip}\Big(\boldsymbol{\theta}_{\text{OPEN\_POSE}} + \mathbf{J}_{\text{dir}} \cdot \boldsymbol{q}_{\text{cmd}}, \;\boldsymbol{\theta}_{\text{limit\_min}}, \;\boldsymbol{\theta}_{\text{limit\_max}}\Big)$$

### 2.2 状态回读转换 (DXL Hardware $\to$ Robot State)
$$\boldsymbol{q}_{\text{actual}} = \mathbf{J}_{\text{dir}} \cdot (\boldsymbol{\theta}_{\text{actual\_dxl}} - \boldsymbol{\theta}_{\text{OPEN\_POSE}})$$

---

## 3. ROS2 通信话题与数据契约

### 3.1 状态反馈话题 (`/hand/joint_states`)
- **发布频率**: `50 Hz`
- **消息类型**: `sensor_msgs/JointState`
- **字段语义**:
  - `header.stamp`: 硬件回读时刻的 ROS2 系统时钟
  - `name`: 16 维 Canonical Joint 命名数组
  - `position`: 16 维实测关节弧度 $\boldsymbol{q}_{\text{actual}}$ (rad)
  - `velocity`: 16 维估算角速度 (rad/s)
  - `effort`: 16 维电机电流 (mA)

### 3.2 目标控制话题 (`/hand/joint_command`)
- **接收频率**: 最多 `50 Hz`（由遥控/策略节点发布）
- **消息类型**: `sensor_msgs/JointState` (MVP 极简标准格式)
- **字段语义**:
  - `header.stamp`: 指令生成时间戳
  - `position`: 16 维目标绝对关节弧度 $\boldsymbol{q}_{\text{target}}$ (rad，0 为全开)
  - 严禁使用相对增量 delta。

---

## 4. LeRobot 学习特征空间规范 (Schema Contract)

| 特征键名 (`features` key) | 维度 / 类型 | 说明 |
| :--- | :--- | :--- |
| `observation.state` | `(16,) float32` | 对应 `/hand/joint_states.position`（单位：$\text{rad}$） |
| `action` | `(16,) float32` | 对应 `/hand/joint_command.position`（单位：$\text{rad}$） |

* **数据同步对齐基准**: 以 50Hz 为基准时间网格，通过最近邻/线性插值完成 State 与 Action 的严格时间戳对齐。
* **原始数据保护**: 示教过程中只记录 ROS2 原始 `rosbag2` 文件，由离线转换工具 `tuinadex_dataset` 生成 Parquet 格式的 `LeRobotDataset`。
