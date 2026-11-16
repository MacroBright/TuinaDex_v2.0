# tests: 全局质量防线与自动化测试套件

`tests/` 目录构成了 TuinaDex v2.0 系统的 **全量质量防护网与测试驱动开发 (TDD) 门禁**。

包含 **50+** 个测试文件、百余项单元测试与端到端集成测试，严格遵循 Anti-Mock（核心物理与动力学严禁虚假打桩）与 Anti-Skip（严禁无故跳过测试）准则。

---

## 快速测试指令矩阵 (Test Command Matrix)

系统提供 `./bin/tuina sys test --suite <suite>` 自动路由至对应的隔离运行时（`ros2` Lyrical Python 3.14 / `tuinadex-control` / `tuinadex-vla`）：

| 测试类型 | 运行环境 / `--suite` | 推荐执行命令 | 实测指标 |
| :--- | :--- | :--- | :--- |
| **系统 CLI 全域控制台** | `ros2` / `--suite cli` | `./bin/tuina sys test --suite cli` | **37 passed** (~0.6s) |
| **核心安全与限位算法** | `control` / `--suite core` | `./bin/tuina sys test --suite core` | **23 passed** (~0.2s) |
| **遥操运动学与视-惯感知** | `control` / `--suite teleop` | `./bin/tuina sys test --suite teleop` | **107 passed** |
| **数据采集 Hub 与 API** | `control` / `--suite hub` | `./bin/tuina sys test --suite hub` | **34 passed** |
| **真机 UVC 多相机 / 驱动 / 仿真 / 环境** | `tuinadex-control` | `source scripts/activate_tuinadex.sh control && pytest tests/test_real_camera_rig.py tests/test_env_management.py tests/test_leap_cli_and_pose_manager.py tests/test_leap_teleop_cli.py tests/test_leap_driver.py tests/test_arm_driver.py tests/test_sim_bridge.py tests/test_sim_core.py -q` | **53 passed** |
| **LeRobot v3.0 数据集导出** | `vla` / `--suite dataset` | `./bin/tuina sys test --suite dataset` | `3 passed, 2 skipped` *(vla 跳过 rosbag2 用例)* |
| **ROS 2 节点与启动集成** | `ros2` / `--suite ros2` | `ROS_DOMAIN_ID=99 ./bin/tuina sys test --suite ros2` | **8 passed** *(CycloneDDS 隔离域)* |
| **22D 复合遥操节点单测** | ROS 2 (`lyrical`) | `ROS_DOMAIN_ID=99 bash -c 'source /opt/ros/lyrical/setup.bash && source install/setup.bash && python3 -m pytest tests/test_teleop_sim_composite_node.py -q'` | 通过（覆盖离合、HandHold 与姿态配置） |

---

## 1. 测试分层与目录拓扑

```text
tests/
├── conftest.py                       # 全局 pytest Fixtures (ROS 2 上下文隔离, Mock 相机数据)
├── test_tuina_cli.py                 # tuina launch / 基础 CLI 参数解析与 dry-run 单测
├── test_tuina_cli_teleop.py          # tuina teleop hand / composite / real / calibrate 路由单测
├── test_tuina_cli_system.py          # tuina calibrate / arm / hand / data / sys 与安全门禁单测
│
├── teleop/                           # 遥操作系统专项算法测试集 (零 ROS 纯 Python 极速单测)
│   ├── test_arm_ik_solver.py         # Pink QP-IK 机械臂逆运动学解算与阻尼最小二乘
│   ├── test_hand_retargeter.py       # 16-DOF 灵巧手几何映射、拇指对捏与优化重定向
│   ├── test_clutch.py                # 空间相对增量离合器平滑接管与脱离测试
│   ├── test_safety.py                # 4 级安全门限守护 (感知丢失, 位姿跳变, 力过载, 奇异点)
│   ├── test_teleop_hotkeys.py        # OpenCV HUD 快捷键状态机与 I40 门禁防护
│   ├── test_imu_stream.py            # 6 轴 IMU 姿态解算与坐标系映射矩阵
│   ├── test_calibrate_imu.py         # 手腕 IMU 3 姿态正交对齐与零位校准测试
│   └── test_preflight_calibration.py # 起飞前三合一交互向导流程单测
│
├── test_command_processing.py        # 16D/22D 弧度规范重排与逐关节物理限位截断门 (Spec §1)
├── test_watchdog_fsm.py              # 300ms 安全看门狗状态机 (FOLLOWING -> RELAXING)
├── test_leap_cli_and_pose_manager.py # 16-DOF LEAP Hand CLI 与 poses.json 姿态持久化校验
├── test_leap_teleop_cli.py           # leap-teleop 视觉直驱终端参数与 HUD 单测
├── test_leap_driver.py               # 16-DOF LEAP Hand Dynamixel 驱动层与 150mA 电流硬锁
├── test_arm_driver.py                # 6-DOF reBot 机械臂 SocketCAN 驱动层与限位测试
├── test_teleop_sim_composite_node.py # 22-DOF 复合遥操 ROS 2 节点 (HandHold / ReadyPose / 退出归零)
│
├── test_camera_rig.py                # 仿真三机位 EGL 离屏渲染阵列独立生产线程与时间戳同步
├── test_real_camera_rig.py           # 真机 UVC 多相机配置解析、旋转翻转与延迟保护单测
├── test_data_hub_controller.py       # Web UI Hub 采集控制器状态机与遥测推送
├── test_collection_hub_api.py        # 零依赖 HTTP REST API (/api/action/*, /events/telemetry)
│
├── test_dataset_adapter.py           # 原生 rosbag2 数据反序列化与 50Hz 插值
├── test_dataset_sync_validator.py    # 50Hz 时间网格对齐与限位有效性校验器
├── test_lerobot_v3_exporter.py       # Hugging Face LeRobotDataset v3.0 多模态 Parquet 与 H.264 MP4
└── test_env_management.py            # 双 Conda 环境隔离、DDS 配置与依赖锁定门禁测试
```

---

## 2. 编写与运行单测规范

1. **测试隔离原则 (Invariant 19)**:
   - 涉及 ROS 2 节点通信的集成测试，**强制声明独立的 `ROS_DOMAIN_ID=99`**，防止被局域网内其他运行中的节点（如后台 teleop 视窗）串流干扰；
2. **纯粹性下沉**:
   - 涉及运动学、重定向、滤波器与看门狗的算法测试，一律编写为脱机纯 Python 用例（置于 `tests/teleop/` 或直接针对 `tuinadex_core`），单测执行时间控制在 $<0.1\text{s}$ 内；
3. **真实性原则 (Anti-Mock)**:
   - 严禁为了通过 CI 对核心物理动力学（MuJoCo 接触、刚体积分）使用假数据桩；
   - 严禁在测试中断言假结果（空断言）或使用 `@pytest.mark.skip` 掩盖真实缺陷。

