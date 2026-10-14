# TuinaDex v2.0 多端异构环境管理实施方案 (Implementation Plan)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 落地 TuinaDex v2.0 五大环境（`TD_v2.0-ENV-*`）的工程脚手架、一键自动化初始化脚本、DDS 跨端网络配置、Docker 容器构建文件以及自检验证套件。

**Architecture:** 依据 [`docs/superpowers/specs/2026-10-14-environment-management-architecture-design.md`](../specs/2026-10-14-environment-management-architecture-design.md)，采用三层异构节点拓扑（云端 GPU + 香橙派 AI Pro + 本机 PC），建立统一 CycloneDDS 网络配置（`ROS_DOMAIN_ID=42`），通过 udev 规则与一键脚本固化外设与总线，通过 Dockerfiles 隔离复杂 AI 数据处理与 VLA 推理环境。

**Tech Stack:** Bash, ROS 2 (Lyrical / Humble), Eclipse CycloneDDS, Docker, Linux udev / systemd, Python 3.10 / 3.14, pytest.

**Spec:** [`docs/superpowers/specs/2026-10-14-environment-management-architecture-design.md`](../specs/2026-10-14-environment-management-architecture-design.md)

## Global Constraints

- 全网统一使用 `rmw_cyclonedds_cpp`，`ROS_DOMAIN_ID=42`。
- 灵巧手与机械臂全链路强制 SI 国际标准单位：弧度 (rad)。
- 灵巧手安全限流 150mA，机械臂运动限制在 70% 臂展内。
- 物理总线：reBot-DevArm RS 走 SocketCAN `can0` @ 1Mbps；LEAP Hand 走 USB-TTL @ 4Mbps。
- 跨端仅允许传输 ROS 2 标准消息类型 (`sensor_msgs`, `geometry_msgs`, `std_msgs`)。

---

### Task 1: CycloneDDS 跨端网络配置与环境检测工具

**Files:**
- Create: `scripts/env/setup_cyclonedds.sh`
- Create: `scripts/env/check_env.sh`
- Create: `tests/test_env_management.py`

**Interfaces:**
- Produces: `scripts/env/cyclonedds.xml`, 环境变量 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `ROS_DOMAIN_ID=42`.

- [x] **Step 1: 编写测试用例 `tests/test_env_management.py` (验证脚本存在与执行权限)**
- [x] **Step 2: 运行测试验证失败**
- [x] **Step 3: 实现 `scripts/env/setup_cyclonedds.sh` 与 `scripts/env/check_env.sh`**
- [x] **Step 4: 运行测试验证通过**
- [x] **Step 5: 提交 Task 1 代码**

---

### Task 2: 本机开发机 (`TD_v2.0-ENV-DEV-HW & SIM`) 初始化脚本

**Files:**
- Create: `scripts/env/99-tuinadex-devices.rules`
- Create: `scripts/env/init_dev_pc.sh`

**Interfaces:**
- Produces: udev 别名 `/dev/tuinadex_hand`，SocketCAN 用户组权限，Conda/ROS 基础依赖检查。

- [x] **Step 1: 在 `tests/test_env_management.py` 中增加 udev 与 init_dev_pc 验证用例**
- [x] **Step 2: 运行测试验证失败**
- [x] **Step 3: 编写 `99-tuinadex-devices.rules` 与 `init_dev_pc.sh`**
- [x] **Step 4: 运行测试验证通过**
- [x] **Step 5: 提交 Task 2 代码**

---

### Task 3: 香橙派边缘中枢 (`TD_v2.0-ENV-EDGE`) 初始化脚本

**Files:**
- Create: `scripts/env/tuinadex-can.service`
- Create: `scripts/env/init_orangepi.sh`

**Interfaces:**
- Produces: 开机自启 systemd CAN 服务 `tuinadex-can.service`，香橙派 ROS 2 Humble 与 CANN 依赖检测。

- [x] **Step 1: 在 `tests/test_env_management.py` 中增加 systemd 服务与 init_orangepi 验证用例**
- [x] **Step 2: 运行测试验证失败**
- [x] **Step 3: 编写 `tuinadex-can.service` 与 `init_orangepi.sh`**
- [x] **Step 4: 运行测试验证通过**
- [x] **Step 5: 提交 Task 3 代码**

---

### Task 4: 数据清洗与转换 Docker 容器构建 (`TD_v2.0-ENV-DATA`)

**Files:**
- Create: `docker/data.Dockerfile`
- Create: `scripts/run_data_convert.sh`

**Interfaces:**
- Produces: 镜像 `tuinadex-data:v2.0`，一键运行脚本 `scripts/run_data_convert.sh`。

- [x] **Step 1: 在 `tests/test_env_management.py` 中增加 data.Dockerfile 语法校验测试**
- [x] **Step 2: 运行测试验证失败**
- [x] **Step 3: 编写 `docker/data.Dockerfile` 与 `scripts/run_data_convert.sh`**
- [x] **Step 4: 运行测试验证通过**
- [x] **Step 5: 提交 Task 4 代码**

---

### Task 5: 云端 VLA 推理与训练容器脚手架 (`TD_v2.0-ENV-CLOUD`)

**Files:**
- Create: `docker/vla_cloud.Dockerfile`
- Create: `scripts/launch_vla_cloud.sh`

**Interfaces:**
- Produces: 镜像 `tuinadex-vla:latest`，启动命令 `scripts/launch_vla_cloud.sh`。

- [x] **Step 1: 在 `tests/test_env_management.py` 中增加 vla_cloud.Dockerfile 语法校验测试**
- [x] **Step 2: 运行测试验证失败**
- [x] **Step 3: 编写 `docker/vla_cloud.Dockerfile` 与 `scripts/launch_vla_cloud.sh`**
- [x] **Step 4: 运行测试验证通过**
- [x] **Step 5: 提交 Task 5 代码**

---

### Task 6: 环境自动化健康检查与回归测试套件

**Files:**
- Modify: `tests/test_env_management.py`
- Create: `scripts/verify_all_envs.sh`

**Interfaces:**
- Produces: 一键运行完整健康检查报告与自动化回归。

- [x] **Step 1: 完善 `tests/test_env_management.py` 覆盖所有脚本和配置文件**
- [x] **Step 2: 编写 `scripts/verify_all_envs.sh` 并赋予可执行权限**
- [x] **Step 3: 运行完整测试套件验证全绿**
- [x] **Step 4: 提交 Task 6 代码与全景验证**
