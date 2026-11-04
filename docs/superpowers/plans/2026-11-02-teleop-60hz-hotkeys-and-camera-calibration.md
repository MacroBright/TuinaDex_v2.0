# TuinaDex v2.0 仿真遥操与采集端优化实施规划与工单规范

> **执行事实源**: `docs/superpowers/plans/2026-11-02-teleop-60hz-hotkeys-and-camera-calibration.md`  
> **发起时间**: 2026-11-02  
> **担当智能体**: Antigravity (Seat 1: Recon & Lead) -> 交付 Claude Code (Seat 2: Lead) & Codex (Seat 3: Coder)  
> **工程完整性模式**: `development`  

---

## 一、 用户测试反馈与现状根因分析 (Evidence-Based Diagnosis)

### 问题 1: 遥操端人手关键点检测帧率低 (在 30 帧左右跳变)，需优化稳定至 60 帧
- **实测根因**:
  1. **USB 物理层降级**: 当前 RealSense D455 插入了 USB 2.1 端口 (`usb_type_descriptor: 2.1`)。受 480Mbps 带宽限制，固件硬件限制 RGB+Depth 最高为 30fps；若插入 USB 3.2 端口方可硬件级输出 60fps；
  2. **代码硬编码与默认参数截断**:
     - `teleop_sim_composite_node.py` L180 硬编码 `RealSenseStream(..., fps=30)`；
     - `teleop_sim_composite_node.py` L126 与 `teleop_sim.launch.py` L25 默认参数写死 `publish_rate_hz=30.0`；
  3. **串行阻塞架构**: 控制定时器循环 `_loop_step` 中同步调用 `self.stream.read_frames()`，内部 `wait_for_frames(timeout_ms)` 同步阻塞等帧（~33ms）+ MediaPipe 推理（~12ms）+ 动力学解算（~8ms）+ GUI 绘图，单线程总延时累积达 50ms+，导致帧率严重跳变。
- **解决方案**:
  1. 重构 `RealSenseStream` 为**异步多线程环形缓冲生产者**（采集线程独占 `wait_for_frames`，控制主线程以 60Hz 轮询非阻塞读取最新对齐帧）；
  2. 支持相机自动探测：若为 USB 3.0 则硬件开启 60fps；若为 USB 2.1 则以 30fps 采集并在 60Hz 控制主循环中通过 1€-Filter 空间时序外推/插值平滑输出，消除控制端跳变；
  3. 将 `publish_rate_hz` 默认值提升为 `60.0`，并在 launch 参数中暴露 `camera_fps`（默认 60，降级 30）。

---

### 问题 2: 遥操端 HUD 快捷键 (a, i, o, m 等) 不起作用，IMU 轴 x/y 无法对调
- **实测根因**:
  1. `ui/teleop_hud.py` L207 底部绘制了提示：`[C] Clutch | [S] Safety | [Z] Zero Hand | [I] Zero IMU | [M] Mode | [A] Pos | [O] Orient`；
  2. 但在 `teleop_sim_composite_node.py` L528-547 的按键响应事件中，**仅实现了 `[C]` 与 `[S]`**；
  3. `[I]` 仅在 `imu_reading.is_valid` 为 True 时处理，且未提供反馈；
  4. **按键 `[A]` (Axis Mapping 切换/对调)、`[M]` (姿态模式切换)、`[Z]` (零位标定)、`[O]` (方向归零) 在代码中完全缺失任何逻辑分支**，属于虚标未实现。
- **解决方案**:
  在 `teleop_sim_composite_node.py` 的按键事件分发中补齐完整状态机映射：
  - **按键 `[A]` / `[a]`**: 动态切换人手到机械臂位置轴映射，支持循环切换 `"-y,+x,+z"` (标准对调)、`"+x,+y,+z"` (直通)、`"-x,+y,+z"`、`"+y,+x,+z"`，并在 HUD 和终端输出当前 Mapping；
  - **按键 `[M]` / `[m]`**: 循环切换机械臂姿态跟踪模式：`FIXED_DOWNWARD` -> `TRACK_IMU` -> `TRACK_WRIST`；
  - **按键 `[I]` / `[i]`**: 强制重新校准当前 IMU 姿态为基准零位（解除 `is_valid` 死锁判断，刷新 `wrist_imu_tracker.zero()`）；
  - **按键 `[X]` / `[x]` (专用轴交换)**: 直接交换 IMU 的 X 轴与 Y 轴映射分量，快速纠正横纵反向；
  - **按键 `[Z]` / `[z]`**: 将当前手掌各指节弧度设定为临时零位偏置 (`hand_retargeter.zero()`)。

---

### 问题 3: 采集端三机位光学与空间拓扑偏差
- **实测根因**:
  - 场景床面中心位于 `[0, 0, 0.68]`，人体模特中心位于 `[+0.473, 0.000, +0.828]`，机械臂基座位于 `[+0.200, -0.550, +0.655]`；
  - 当前 `overhead_cam` 位于 `[0.20, -0.85, 1.45]`，斜向偏外侧，无法垂直俯视背部与机械臂；
  - 当前 `wrist_cam` 位于 `[0, -0.03, 0.04]`，贴紧手心，无法拍到手背及 4 指展开全貌；
  - 当前 `side_cam` 位于 `[0.20, -1.35, 0.78]`，位于床侧外沿向内看，与机械臂伸展方向平行，而非垂直于前后/上下运动平面。
- **解决方案**:
  重新标定 `src/tuinadex_sim/worlds/tuina_scene.xml` 与 `src/tuinadex_teleop/configs/mjcf/tuina_sim_scene.xml`：
  1. **`overhead_cam` (正中心正上方广角俯视)**:
     - 空间位置：`pos="0.35 -0.15 1.85"`（人体中心与机械臂作业区上方）
     - 镜头朝向：`xyaxes="1 0 0 0 1 0"` (光轴垂直朝下 -Z，X 沿人体轴向，Y 沿床宽)
     - 视场角：`fovy="68"` (完整覆盖人体背部全貌及机械臂全行程)
  2. **`wrist_cam` (机械臂末端腕部上方后方俯视灵巧手全貌)**:
     - 挂载父体：保持在 `leap_hand_palm` 或 `link6`
     - 空间位置：`pos="0.05 0.02 0.16"`（手掌后上方外拉）
     - 镜头朝向：下倾 45° 俯视灵巧手四指与手背
     - 视场角：`fovy="80"` (超广角视野无畸变)
  3. **`side_cam` (人体模特头部一侧，垂直于前后与上下运动方向)**:
     - 空间位置：`pos="-0.65 -0.20 0.95"`（头部外侧，高于床面）
     - 镜头朝向：`xyaxes="0 -1 0 0 0 1"` (光轴沿 +X 直视脚部方向，垂直于 Y 轴伸缩与 Z 轴升降)
     - 视场角：`fovy="52"` (完整拍摄机械臂大臂、小臂、腕部的侧面运动轮廓)
  4. **不变量守护**: 两份 XML 必须保持逐字节一致。

---

### 问题 4: 浏览器可视化数据采集端刷新率低且频闪 (Flicker)
- **实测根因**:
  1. `server.py` L248 中 MJPEG 的 Content-Type 为 `multipart/x-mixed-replace; boundary=frame`，而 boundary 输出分块缺少标准的双换行闭合符，导致现代 Webkit/Blink 引擎解码交替时触发 `<img>` 标签闪白（清空重绘）；
  2. `server.py` 单连接在每个 wait_frame 循环中调用阻塞写入，若客户端或网络存在微小抖动，TCP 堆积导致丢帧和连接抖动；
  3. 采集端主循环在 `collect_teleop_data.py` 中受控制循环 tick 同步制约，UI 刷新率被 `ui_every` 强行降频至 25Hz；
  4. 前端使用单个裸 `<img>` 标签直接展示长连接 MJPEG 流，缺乏重连与双缓冲平滑。
- **解决方案**:
  1. **MJPEG 协议规范化**: 标准化 boundary 分块结构，输出规范的 `\r\n--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ...\r\n\r\n<payload>\r\n`；
  2. **提高推流上限与自适应跳帧**: 在 `StreamHub.wait_frame()` 中引入最新帧快速提取与陈旧帧跳过（Drop-Stale-Frames），防止 TCP 拥塞产生积压延时；
  3. **前端防频闪机制**: 在 `app.js` 与 `index.html` 中优化图像加载机制，添加 CSS 硬件加速 `backface-visibility: hidden; transform: translateZ(0);`，杜绝图层清屏重绘闪烁；
  4. 将 UI SSE 遥测推流频率与视频流同步提升至稳定 50Hz/60Hz。

---

## 二、 黄金不变量与架构约束 (Golden Invariants)

- **I37 (双机位文件单一事实源)**: `src/tuinadex_sim/worlds/tuina_scene.xml` 与 `src/tuinadex_teleop/configs/mjcf/tuina_sim_scene.xml` 必须逐字节一致，由单元测试守护；
- **I38 (感知与控制非阻塞分离)**: 相机物理读取与模型推理严禁阻塞 ROS 2 控制定时器线程，相机采集延迟超过阈值必须降级平滑，禁止硬阻塞；
- **I39 (零外部依赖原则)**: Web UI Hub 严格保持原生 Python `http.server` 与原生 HTML5/CSS/JS，禁止引入 npm/node 构建栈；
- **I40 (热键交互安全性)**: 快捷键切换坐标轴映射 (`[A]`) 或归零 (`[I]`) 时，必须先核验机械臂速度或离合状态，禁止在剧烈运动中突变目标点引发速度越界。

---

## 三、 细分实施原子工单 (Atomic Work Orders W8 ~ W11)

### 工单 W8: 遥操感知管线异步解耦与 60Hz 稳定化
- **排他文件领空**: `[FILES: src/tuinadex_teleop/tuinadex_teleop/perception/realsense_stream.py, src/tuinadex_teleop/tuinadex_teleop/teleop_sim_composite_node.py, src/tuinadex_teleop/launch/teleop_sim.launch.py, tests/teleop/test_realsense_stream.py]`
- **交付内容**:
  1. 在 `realsense_stream.py` 中引入后台帧捕获线程与双缓冲，`read_frames()` 改为非阻塞零拷贝读取；
  2. 增加自动探测：USB 3.x 启用 60fps，USB 2.x 启用 30fps 并由 1€ 滤波器在节点端插值平滑至 60Hz；
  3. `teleop_sim_composite_node.py` 定时器周期默认升级至 60Hz，修复硬编码 `fps=30`；
  4. 编写针对异步采集与平滑插值的单元测试。

### 工单 W9: 遥操 OpenCV HUD 快捷键全功能交互闭环
- **排他文件领空**: `[FILES: src/tuinadex_teleop/tuinadex_teleop/teleop_sim_composite_node.py, src/tuinadex_teleop/tuinadex_teleop/core/clutch.py, src/tuinadex_teleop/tuinadex_teleop/perception/imu_stream.py, src/tuinadex_teleop/tuinadex_teleop/ui/teleop_hud.py, tests/teleop/test_teleop_hotkeys.py]`
- **交付内容**:
  1. 补齐 `teleop_sim_composite_node.py` 的按键事件：`[A]`(轴循环切换)、`[X]`(IMU X/Y 轴快速对调)、`[M]`(姿态模式切换)、`[I]`(IMU 零位刷新)、`[Z]`(手部零位校准)；
  2. 在 `WristImuTracker` 中增加动态轴映射更新接口 `swap_xy_axes()`；
  3. 在 `teleop_hud.py` 顶部或侧边面板实时高亮当前处于激活状态的 Axis Mapping 与 Orientation Mode，并在按键触发时显示 1 秒 Toast 提示；
  4. 编写针对快捷键状态机响应的自动化测试。

### 工单 W10: 场景三机位光学与几何位姿标定重构
- **排他文件领空**: `[FILES: src/tuinadex_sim/worlds/tuina_scene.xml, src/tuinadex_teleop/configs/mjcf/tuina_sim_scene.xml, src/tuinadex_teleop/tuinadex_teleop/cameras/camera_specs.py, tests/test_camera_rig.py]`
- **交付内容**:
  1. 将 `overhead_cam` 重构为 `pos="0.35 -0.15 1.85" xyaxes="1 0 0 0 1 0" fovy="68"`，实现人体背部与机械臂垂直俯视全景；
  2. 将 `wrist_cam` 重构为腕部上方俯视手背与四指，实现广角 `fovy="80"`；
  3. 将 `side_cam` 重构为人体头部一侧 `pos="-0.65 -0.20 0.95" xyaxes="0 -1 0 0 0 1" fovy="52"`，实现垂直侧视机械臂全貌；
  4. 同步更新 `camera_specs.py` 中对应的标称参数与测试断言；
  5. 确保两份 XML 逐字节完全一致并通过 `test_camera_rig.py` 验证。

### 工单 W11: Web UI Hub MJPEG 推流协议加固与防频闪优化
- **排他文件领空**: `[FILES: src/tuinadex_teleop/tuinadex_teleop/hub/server.py, src/tuinadex_teleop/tuinadex_teleop/hub/static/index.html, src/tuinadex_teleop/tuinadex_teleop/hub/static/app.js, src/tuinadex_teleop/tuinadex_teleop/hub/static/style.css, tests/test_collection_hub_api.py]`
- **交付内容**:
  1. 规范 `server.py` 的 `_mjpeg` 边界输出格式，修复分块协议引起的浏览器重绘闪烁；
  2. 在 `StreamHub` 中增加快速跳帧与非阻塞缓冲，防止多客户端并发拉流时的 TCP 拥塞；
  3. 在 `index.html` 与 `style.css` 中为视频容器增加 GPU 硬件合成渲染层，杜绝清屏频闪；
  4. 将 UI 遥测与流刷新率提升至最高 50Hz/60Hz；
  5. 运行 Hub API 自动化测试验证。

---

## 四、 验收判定标准 (Acceptance Criteria)

1. **帧率指标**: RealSense 遥操端在具备 USB 3.0 或插值模式下稳定运行于 60Hz，HUD 仪表盘 FPS 显示维持在 58~60fps，不再跳变至 25~30fps；
2. **按键交互**: 按键 `[A]` 或 `[X]` 可立即对调 IMU 与位置映射轴，按键 `[M]` 可切换姿态模式，按键 `[I]` 立即重置零位，HUD 实时呈现 Toast 提示；
3. **相机视野**:
   - 上帝视角完整呈现人体模特背部全貌及机械臂全行程；
   - 腕部相机完整呈现灵巧手手背与 4 根手指展开运动；
   - 侧向视角从头部方向清晰垂直呈现机械臂侧面全景；
4. **采集界面**: 浏览器访问 `http://127.0.0.1:8080` 时，三路画面平滑流畅、无黑屏/白屏跳闪，遥测曲线 50Hz 实时响应；
5. **回归测试**: 8 包全量编译通过，全量单元测试与 E2E 终验脚本全绿。

