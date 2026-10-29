# reBot-CLI 命令行控制工具使用指南

`rebot-cli` 是专为 reBot B601-RS 机械臂定制的命令行交互与调试工具。支持**交互式控制台（REPL 逐关节微调）**、**单次脚本化命令行控制**、以及**手动拖动示教/角度实时监控（watch/drag）**。

---

## 1. 安装与命令入口

在 Conda 虚拟环境（如 `rebot_rs`）中，在 `rebot_control` 根目录下执行可编辑安装即可生成全局 `rebot-cli` 命令：

```bash
cd rebot_control
pip install -e .
```

也可以直接通过 Python 模块调用：
```bash
python -m rebot.cli
```

---

## 2. 模式一：交互式控制台 (Interactive REPL)

直接运行命令进入控制台：
```bash
rebot-cli
# 或指定配置文件/默认限速:
rebot-cli --config config/rebotarm_rs.yaml --speed 15.0
```

启动后机械臂会自动建立 CAN 连接、使能电机并进入驻留锁定状态。终端提示符为 `[rebot-arm]>`。

### 交互指令速查表

| 指令 | 语法 | 说明 | 示例 |
|---|---|---|---|
| **状态查询** | `status` (或 `st`) | 查询当前 6 轴角度、平滑目标与 MOS 驱动温度 | `status` |
| **单关节绝对位置** | `set <joint> <angle>` | 将指定关节移动到指定绝对角度（度） | `set 1 25.0` (将 J1 移动到 25°) |
| **单关节相对微调** | `step <joint> <delta>` | 在当前目标角度基础上相对增减步长 | `step 2 5` (增 5°), `step 2 -5` (减 5°) |
| **全轴同步到位** | `all <a1> <a2> <a3> <a4> <a5> <a6>` | 同时下发全部 6 个关节目标角度 | `all 0 10 20 0 0 0` |
| **平滑回零** | `zero` (或 `z`) | 所有关节平滑回归至 0° 机械基准 | `zero` |
| **动态限速** | `speed [joint] <deg_s>` | 调整指定关节或所有关节的速度上限 (°/s) | `speed 30` (全轴 30°/s)<br>`speed 1 45` (J1 设为 45°/s) |
| **手动拖动/示教** | `watch [joint]` (别名 `drag`) | **失能电机**并以 ~20Hz 高频单行刷新当前机械角度，按 `Ctrl+C` 退出并自动同步锁定 | `watch` (监控全轴)<br>`watch 2` (聚焦 J2 并显示偏移量) |
| **电机失能** | `disable` | 手动失能所有电机（力矩卸除，请托住机械臂） | `disable` |
| **电机使能** | `enable` | 重新使能并锁定电机（自动读取当前位置防突跳） | `enable` |
| **查看帮助** | `help [cmd]` (或 `?`) | 查看指令列表或具体指令说明 | `help set` |
| **安全退出** | `exit` / `quit` / `q` | 平滑回零、失能电机并关闭 CAN 退出 | `exit` |

---

## 3. 模式二：单次命令行调用 (One-Shot Commands)

适合在 Shell 脚本、CI 或日常一键测试中直接下发动作：

### 3.1 状态只读查看 (`status`)
读取机械臂当前实际机械角度与电机温度并退出（不移动机械臂）：
```bash
rebot-cli status
```

### 3.2 手动拖动与角度监控 (`watch`)
失能电机进入手动拖动示教，终端实时高频输出关节机械角度：
```bash
# 监控所有 6 个关节
rebot-cli watch

# 聚焦监控特定关节（如 J1），输出绝对角度与拖动位移偏移量
rebot-cli watch --joint 1

# 自定义刷新率（如 30 Hz）
rebot-cli watch -j 2 --hz 30
```
> 按 `Ctrl + C` 退出监控。

### 3.3 单关节单次移动 (`move --joint --angle`)
```bash
# 将 J1 移动到 25°，限制速度 15°/s，移动完成后按 Ctrl+C 安全回零
rebot-cli move --joint 1 --angle 25.0 --speed 15.0

# 移动完成后直接退出，不自动回零（保持在目标姿态）
rebot-cli move --joint 2 --angle -10.0 --no-zero
```

### 3.4 全关节单次移动 (`move --angles`)
```bash
rebot-cli move --angles 10 15 20 -10 -20 30 --speed 20.0
```

### 3.5 一键安全平滑回零 (`zero`)
```bash
rebot-cli zero --speed 25.0
```

---

## 4. 电机控制参数配置与修改

CLI 读取的参数来自配置文件：[`config/rebotarm_rs.yaml`](file:///home/bright/win_office/ubantu_files/project/reRot_DevArm/rebot_control/config/rebotarm_rs.yaml)。

### 4.1 CAN 与控制频率参数
```yaml
can:
  channel: "can0"       # CAN 通道名称
  host_id: 0xFD         # 主机 CAN ID

control:
  control_hz: 200.0          # MIT 控制指令发送频率 (Hz)
  telemetry_hz: 2.0          # 电机状态/温度轮询频率 (Hz)
  default_speed_deg_s: 30.0  # 各关节默认运动限速 (°/s)
```

### 4.2 电机刚度与阻尼参数 (`kp` / `kd`)
机械臂采用 MIT 混合阻抗控制，位置环控制方程为：
$$\tau = K_p (q_{des} - q) + K_d (\dot{q}_{des} - \dot{q}) + \tau_{ff}$$

在 `config/rebotarm_rs.yaml` 的 `motors` 列表中针对 J1~J6 逐电机配置：
```yaml
motors:
  - id: 1
    type: "rs-06"
    kp: 50.0            # 位置比例刚度 (越大越硬，过大易振荡)
    kd: 1.5             # 速度阻尼 (抑制超调)
  - id: 2
    type: "rs-06"
    kp: 50.0
    kd: 1.5
  # ...
  - id: 6
    type: "rs-00"
    kp: 30.0            # 腕部 RS00 较小，刚度相应较低
    kd: 1.0
```

### 4.3 运动与回零速度限制
```yaml
return_zero:
  max_speed_deg_s: 30.0         # 退出时回零最高角速度 (°/s)
  thermal_max_speed_deg_s: 30.0 # 过温保护回零速度 (°/s)
  min_time_s: 3.0               # 最短平滑回零时间 (秒)
```

---

## 5. 安全防护要点

1. **工作空间限制**：SDK 不做软件限位，目标角度直接下发电机。请保持在约 70% 工作空间内，避免靠近机械止档与奇异点。
2. **失能与拖动防护**：在执行 `disable` 或 `watch`（拖动示教）指令前，电机阻抗将归零，**务必先用手托扶大臂（J2/J3）**，防止机械臂在重力作用下突然下垂磕碰桌面。
3. **急停与退出**：
   - 交互模式下按 `Ctrl+C` 触发平滑回零；
   - 再次按 `Ctrl+C` 立即紧急失能并退出。

