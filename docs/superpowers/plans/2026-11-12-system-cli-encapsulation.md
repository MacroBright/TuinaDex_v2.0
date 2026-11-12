# TuinaDex v2.0 系统级统一 CLI (`tuina`) 全域封装架构与实施计划

> **日期**: 2026-11-12
> **工单状态**: Phase 1 侦察建模完成 → Phase 2 规划冻结 → Phase 3 执行落地
> **协调/规划**: Antigravity (Seat 1 / Phase 1 & 2)

---

## 1. 背景与目标 (Goal & Scope)

TuinaDex v2.0 已完成仿真与真机端遥操作、三机位 Web Hub 数据采集与基础启动编排联调。此前存在大量分散的启动入口（`ros2 launch ...`、`python scripts/*.py`、`bash scripts/*.sh`、`leap-*`、`rebot_cli`），且跨越三个互斥运行环境（`tuinadex-control` Python 3.10、`tuinadex-vla` Python 3.10 + CUDA 12.8、宿主机 ROS 2 Lyrical Python 3.14）。

本计划将全系统运维与操作封装为单一系统级 CLI `tuina`（全局软链 `/home/bright/.local/bin/tuina -> <repo>/bin/tuina`，支持 Bash/Zsh 自动补全），同时完全向后兼容既有 `leap-control`、`leap-calibrate`、`leap-diagnose`、`leap-teleop`、`rebot_cli` 与 `scripts/*.sh`。

### 确认架构决策 (User-Confirmed Decisions)
- **A1 (兼容性与入口)**: 全局命令 `tuina` 统一调度；保留所有 `leap-*`、`rebot_cli` console scripts 与 `scripts/*.sh`。
- **A2 (八大语义域)**: `launch` / `teleop` / `collect` / `calibrate` / `arm` / `hand` / `data` / `sys`。
- **A3 (硬件安全门禁)**: 所有涉及真机执行/总线写电/电机解锁或带载运动的子命令强制执行 **硬件前置自检 + 交互式安全确认**，提供全局 `-y` / `--yes` 快速放行；`--sim` / `--mock` / `--mock-teleop` / `--dry-run` 自动跳过确认。
- **A4 (协作流)**: 写入 `.agent/SESSION_STATE.md` 黑板并严格执行排他性文件领空与分票验证（`[W1]`~`[W5]`）。

---

## 2. Phase 1 侦察发现与接口漂移修复清单

1. **三运行时污染风险 (`env_detector.py` & `bin/tuina`)**:
   - `tuinadex-control` (Python 3.10) 需要前置 `cmeel.prefix/lib` 至 `LD_LIBRARY_PATH` 以防止加载宿主机 Boost Python 3.14（`PyObject_DelAttr` 未定义崩溃），并加入 `src/*` 到 `PYTHONPATH`。
   - `tuinadex-vla` (Python 3.10 + CUDA 12.8 + LeRobot 0.4.4) 运行 `tuinadex_dataset.cli` 与数据集检视/测试，应以干净环境变量切换至 `tuinadex-vla` 的 Python，严禁混入 ROS 2 Python 3.14 路径或 `cmeel`。
   - 原生 ROS 2 Lyrical (`/opt/ros/lyrical/setup.bash` + `install/setup.bash`) 使用系统 `/usr/bin/python3` (Python 3.14)，**严禁** 注入 Python 3.10 的 `PYTHONPATH` 或 `cmeel.prefix/lib`。
   - 现状问题：`bin/tuina` 当前在启动入口处无条件将 `tuinadex-control` 的 `cmeel.prefix/lib` 与全量 `PYTHONPATH` 写入并导出，导致随后派生的 ROS 2 命令继承 3.10 路径。修复方案：在 `bin/tuina` 保持干净轻量引导 `tuinadex_bringup.cli.main`，由 `env_detector.py` 按目标工作负载的 `runtime` 配置文件（`"control"` | `"vla"` | `"ros2"` | `"bash"`）精准构造隔离环境。
2. **`tuina teleop calibrate` 参数漂移 (`cli/teleop.py`)**:
   - `--imu-port` 默认值误设为 `/dev/ttyUSB0`（会与 LEAP Hand 串口冲突），而 `scripts/calibrate_imu.py` 与 `scripts/00_preflight_calibration.py` 均使用 `/dev/myimu`。
   - `--camera-id` 强制传 `0`，破坏了 `00_preflight_calibration.py` 的自动 RealSense / V4L2 探测逻辑（`default=None`）。
   - `--imu-only --output <path>` 向 `scripts/calibrate_imu.py` 传递了不支持的 `--output` 参数（后者仅接受 `--output-session` / `--output-imu`）。
3. **`tuina collect terminal` 真机参数缺失 (`cli/collect.py`)**:
   - 当前硬编码追加 `--standalone`，在 `--env real` 时不应强制 `--standalone`；同时缺失真机三相机必需的 `--camera-config`、`--camera-map`、`--max-staleness` 参数。
4. **`tuina launch` 目标覆盖不全 (`cli/launch.py`)**:
   - 仅支持 `robot.launch.py`，缺少 `hand.launch.py`、`arm.launch.py`、`sim.launch.py` 快捷选择及 `use_scene_world` / `physics_substeps` 透传与真机安全门禁。

---

## 3. 八大命令域全景树 (`tuina <domain> <subcmd>`)

| 顶层域 | 子命令 | 目标脚本 / 模块 / Launch | 运行时隔离 | 安全门禁 |
| :--- | :--- | :--- | :--- | :--- |
| **`launch`** | `(root)` (`--target {robot,hand,arm,sim}`) | `robot.launch.py` / `hand.launch.py` / `arm.launch.py` / `sim.launch.py` | `ros2` | `--real` (非 mock/sim) |
| **`teleop`** | `sim`, `hand`, `arm`, `composite`/`real`, `calibrate`, `node` | `run_sim_teleop.py`, `tuinadex_hand.cli.teleop`, `teleop_*.launch.py`, `00_preflight_calibration.py`, `calibrate_imu.py` | `control` / `ros2` | 真机非 `--mock` |
| **`collect`** | `hub`, `terminal`, `check` | `collect_teleop_data.py`, `e2e_hub_collection_check.sh` / `e2e_collection_check.sh` | `control` / `bash` | `--env real` 非 `--mock-teleop` |
| **`calibrate`** | `all`/`preflight`, `imu`, `hand`, `hand-limits`, `arm-ready`, `arm-goto` | `00_preflight_calibration.py`, `calibrate_imu.py`, `tuinadex_hand.cli.calibrate`, `tuinadex_hand.cli.diagnostics`, `teach_arm_ready_pose.py`, `test_goto_ready_pose.py` | `control` / `ros2` | 真机非 `--mock` 及机械臂上/卸力命令 |
| **`arm`** | `cli`, `status`, `watch`/`drag`, `zero`, `move`, `teach`, `goto-ready`, `can` | `rebot.cli`, `teach_arm_ready_pose.py`, `test_goto_ready_pose.py`, `ip link` SocketCAN | `control` / `bash` | `zero`, `move`, `teach`, `goto-ready`, `can up/down/vcan` |
| **`hand`** | `control`, `calibrate`, `diagnose`, `teleop` | `tuinadex_hand.cli.{control,calibrate,diagnostics,teleop}` | `control` | 非 `--mock` |
| **`data`** | `convert`, `convert-docker`, `vla-cloud`, `inspect` | `tuinadex_dataset.cli`, `run_data_convert.sh`, `launch_vla_cloud.sh`, 数据集只读统计 | `vla` / `bash` | 无 (离线处理) |
| **`sys`** | `env`, `verify`, `init-pc`, `init-edge`, `dds`, `build`, `test`, `e2e`, `rosbridge`, `teleop-bridge`, `pub`, `probe`, `docker`, `completion` | `scripts/env/*.sh`, `verify_all_envs.sh`, `colcon`, `pytest`, `e2e_*.sh`, `start_rosbridge.sh`, `ros2_teleop_bridge.py`, `mock_command_publisher.py`, `sim_loop_probe.py`, `ros2_docker.sh`, Shell 补全 | 按需匹配 (`bash`/`ros2`/`control`/`vla`) | `init-pc --install-rules` 等高危系统配置提示 |

---

## 4. 分票实施路径 (Atomic Work Tickets)

### `[W1]` 运行时隔离路由器与硬件安全门禁
- **独占文件**:
  - `src/tuinadex_bringup/tuinadex_bringup/cli/env_detector.py`
  - `src/tuinadex_bringup/tuinadex_bringup/cli/safety.py` (新增)
  - `src/tuinadex_bringup/tuinadex_bringup/cli/main.py`
  - `bin/tuina`
- **关键契约**:
  1. `build_runtime_env(profile="control"|"vla"|"ros2"|"bash")`:
     - `"control"`: 清除 `/opt/ros` 相关 Python 3.14 混淆路径，注入 `tuinadex-control` 的 `cmeel.prefix/lib` 到 `LD_LIBRARY_PATH` 最前，添加 `src/*` 与 `dataset` 到 `PYTHONPATH`。
     - `"vla"`: 清除 `/opt/ros` Python 3.14 与 `cmeel` 路径，添加 `dataset` 到 `PYTHONPATH`，使用 `tuinadex-vla` Python。
     - `"ros2"`: 清除 conda Python `PYTHONPATH` 与 `cmeel.prefix/lib`，保持系统 `/opt/ros/lyrical` 与 `install/setup.bash` 语义与 CycloneDDS 环境，使系统 `/usr/bin/python3` (3.14) 无冲突启动 ROS 2 节点与 launch。
     - `"bash"`: 纯净系统环境外加 CycloneDDS (`ROS_DOMAIN_ID`/`RMW_IMPLEMENTATION`)。
  2. `main.py` 全局支持 `-y` / `--yes` 并向子命令/后端位置无关透传；新增 `--env-info` 展示 `control`、`vla`、ROS2 Python、`can0`、`myimu`、`ttyUSB*` 等状态。
  3. `safety.py`: 提供 `confirm_hardware_action(action_desc, checks=..., skip_confirm=False, dry_run=False)`：
     - 前置自检：检查 SocketCAN 接口（`/sys/class/net/<channel>/operstate` 与 `carrier`）、串口设备存在性与 `/dev/myimu` 碰撞警告；以 `[PREFLIGHT]` 明确报告状态。
     - 风险提示：要求操作员确认 E-Stop 急停就位及安全空间清空（交互输入 `[y/N]`，非 TTY 时需 `--yes` 或安全中止）；`skip_confirm=True` (`--yes`) 或 `dry_run=True` 时放行（`--dry-run` 仅打印自检摘要与命令）。

### `[W2]` 既有三域修复与补全 (`launch` / `teleop` / `collect`)
- **独占文件**:
  - `src/tuinadex_bringup/tuinadex_bringup/cli/launch.py`
  - `src/tuinadex_bringup/tuinadex_bringup/cli/teleop.py`
  - `src/tuinadex_bringup/tuinadex_bringup/cli/collect.py`
- **关键修改**:
  - `launch.py`: 新增 `--target {robot,hand,arm,sim}`、`--use-scene-world`/`--no-scene-world`、`--physics-substeps`；保留 `robot.launch.py` 为默认目标并保持既有测试参数兼容；`--real` 非 `--mock` 接入安全门禁；统一以 `runtime="ros2"` 执行 `ros2 launch`。
  - `teleop.py`: 修复 `calibrate`（默认 `--imu-port /dev/myimu`、`--camera-id None`、`--imu-only` 下将 `--output` 映射至 `--output-imu` 并支持 `--output-session`/`--mock`/`--auto`/`--skip-*`）；`hand`/`composite`/`real`/`sim`/`node` 精准路由 `control` 与 `ros2` 运行时，真机非 `--mock` 接入安全门禁；保留 `test_tuina_cli.py` 与 `test_tuina_cli_teleop.py` 全部既有契约与输出格式。
  - `collect.py`: 修复 `terminal` 子命令支持 `--camera-config`/`--camera-map`/`--max-staleness`，仅在 `--env sim` 或显式传 `--standalone` 时加 `--standalone`；`check` 支持 `--legacy`；`--env real` 非 `--mock-teleop` 接入安全门禁；以 `runtime="control"` 执行 `collect_teleop_data.py`。

### `[W3]` 新增三大控制域 (`calibrate` / `arm` / `hand`)
- **独占文件**:
  - `src/tuinadex_bringup/tuinadex_bringup/cli/calibrate.py` (新增)
  - `src/tuinadex_bringup/tuinadex_bringup/cli/arm.py` (新增)
  - `src/tuinadex_bringup/tuinadex_bringup/cli/hand.py` (新增)
- **关键契约**:
  - 所有转发参数严格对齐底层脚本 `argparse` 定义（严禁幻觉传参）。
  - `tuina arm can` 支持 `up`/`down`/`status`/`vcan`，默认 bitrate `1000000` (1 Mbps，对齐 `tuinadex-can.service` 与 `arm_params.yaml`)。
  - 复用 `teleop.resolve_hand_port()`，避免 `/dev/ttyUSB*` 与 `/dev/myimu` 串口冲突。

### `[W4]` 新增数据与系统运维域 (`data` / `sys`)
- **独占文件**:
  - `src/tuinadex_bringup/tuinadex_bringup/cli/data.py` (新增)
  - `src/tuinadex_bringup/tuinadex_bringup/cli/sys_tools.py` (新增)
- **关键契约**:
  - `data`: 全部 LeRobot v3.0 转换与检视统一走 `runtime="vla"`，Docker 工具走 `runtime="bash"`。
  - `sys`: 覆盖环境检查/初始化/CycloneDDS、`colcon build`、分层 `pytest`（自动路由 `control`/`vla`/`ros2`）、5 套 E2E、`rosbridge`、`teleop-bridge`、`pub`、`probe`、`docker` 与 `completion`。

### `[W5]` Shell 自动补全、全量 TDD 测试、包重编与文档闭环
- **独占文件**:
  - `bin/tuina-completion.bash` (新增)
  - `tests/test_tuina_cli_system.py` (新增)
  - `src/tuinadex_bringup/README.md`
  - `scripts/README.md`
- **关键验证**:
  - `tuina-completion.bash` 同时兼容 Bash 与 Zsh (`bashcompinit`) 补全一级/二级域及高频 flag。
  - 在 `tests/test_tuina_cli_system.py` 覆盖运行时环境隔离、安全门禁（TTY 拒绝/确认、`--yes`、`--sim`/`--mock`、`--dry-run` 免确认）、全 8 域 `--dry-run` 路由与零漂移参数断言。
  - 重建 `tuinadex_bringup` (`colcon build --symlink-install --packages-select tuinadex_bringup`)，并在 `python3` (3.14) 与 `tuinadex-control` (3.10) 双环境实测 CLI 与 pytest 全绿。
