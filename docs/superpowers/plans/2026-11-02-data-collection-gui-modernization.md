# TuinaDex v2.0 数据采集 Web Hub 现代具身工效学升级方案与实施计划

> **执行事实源**: `docs/superpowers/plans/2026-11-02-data-collection-gui-modernization.md`  
> **发起时间**: 2026-11-02  
> **设计参考**: Stanford ALOHA 2 (Audio & Hotkeys), DROID (Phases & Dropped Alarms), DexCap (Anatomical Grid & Force Bubbles), LeRobot LeLab (Focus Layout & Scrubber)  
> **定位层次**: Layer 4 Middleware (tuinadex_teleop / hub)  

---

## 一、 架构动机与设计目标

TuinaDex v2.0 拥有 22 自由度（6-DOF 臂 + 16-DOF 手）联合控制空间与推拿接触力感知。目前内置的 Web Hub 虽然实现了基础的多相机 MJPEG 流与 22 维数据柱状图，但在高通量、长时间的真实采集场景中存在以下人机工效学痛点：
1. **视网膜割裂**：3 相机平铺导致主镜头太小，且视频与下方遥测数值割裂，操作员需频繁移开视线；
2. **高维关节认知负荷重**：22 个关节平铺为列表，无法直观分辨各手指弯曲与卡顿；
3. **缺乏推拿力学感知**：仅有数字数值，缺乏直观的指尖受力同心圆气泡雷达图；
4. **示教心流易被打断**：双手被遥操设备占用时，缺少全键盘盲操支持与即时音频反馈 (Audio Cues)；
5. **任务阶段粒度粗**：推拿手法复合度高（按、揉、推、拿），缺乏一键阶段切分标记（Phase Annotation）；
6. **当日进度无感知**：缺乏试次历史列表（Episode Drawer）与目标配额进度条。

**目标**：在保持**零 npm/前端框架依赖**（纯原生 HTML5/CSS3/Vanilla JS）与**100% 兼容 LeRobotDataset v3.0 契约**的前提下，将当前 Web Hub 升级为工业级沉浸式具身采集工作台。

---

## 二、 变更文件清单与职责映射

| 文件路径 | 模块性质 | 核心改动点 |
| :--- | :--- | :--- |
| `src/tuinadex_teleop/tuinadex_teleop/hub/static/index.html` | 页面骨架 | 重构为“一大两小”相机主副网格、主视窗悬浮 OSD 挂件层、22D 生理学分组网格、试次历史抽屉、底部快捷键 Dock |
| `src/tuinadex_teleop/tuinadex_teleop/hub/static/style.css` | 视觉样式 | 现代暗黑具身主题、主副视窗动态缩放、OSD 悬浮层、指尖压力气泡、全屏禅道模式 (`.zen-mode`) |
| `src/tuinadex_teleop/tuinadex_teleop/hub/static/app.js` | 交互逻辑 | Web Audio 蜂鸣音、全键盘盲操与数字键阶段标记、Tab 键交换主副相机、解剖学关节柱渲染、试次列表管理 |
| `tests/test_collection_hub_api.py` | 质量防线 | 确保静态资源路由与 REST API / SSE 契约不变且全绿 |
| `src/tuinadex_teleop/tuinadex_teleop/collection/README.md` | 技术文档 | 同步更新快捷键、主副视角切换与阶段标注使用说明 |

---

## 三、 实施任务细化 (Bite-Sized Tasks)

### Task 1: 键盘盲操与 Web Audio 音频反馈系统 (ALOHA Style)
- **目标**: 实现无鼠标依赖的全键盘盲操（Space、S、D、C、Tab、F、1-4）与纯原生 Web Audio 合成音。
- **文件**: `src/tuinadex_teleop/tuinadex_teleop/hub/static/app.js`
- **步骤**:
  1. 封装 `WebAudioCues` 类：利用浏览器的 `AudioContext` 实时生成振荡器音效（开始录制 880Hz->1175Hz 升调双音、停止录制降调双音、保存成功三和弦、舍弃低沉音、受力超限 220Hz 警示音）；
  2. 扩展全局 `keydown` 监听器：
     - `Space`: 智能状态机推进（IDLE 进 RECORDING；RECORDING 进 STOP；REVIEWING 进 SAVE）；
     - `S`: 保存试次；`D`: 舍弃重来；
     - `1-4`: 阶段快捷键切分，自动调用 `/api/action/label` 写入对应阶段（`1: 准备/接近`, `2: 定位下压`, `3: 循环推揉`, `4: 阻尼抬手`），并伴随轻微确认音；
     - `Tab`: 触发主副相机交换；
     - `F`: 切换全屏禅道模式。

### Task 2: “一大两小”相机主副布局与 OSD 悬浮挂件 (Focus + Thumbnails)
- **目标**: 给予操作者大画幅主视窗，关键遥测悬浮于主视窗四角，支持一键/Tab 交换。
- **文件**: `index.html`, `style.css`, `app.js`
- **步骤**:
  1. 重构 `.camera-grid` 为 `.camera-stage`：分为主视窗 `.primary-viewport` 与副视窗组 `.secondary-strip`；
  2. 主视窗叠加 `.osd-layer`（CSS 半透明黑底胶囊）：
     - 左上角：任务语言指令与阶段指示胶囊（支持动态高亮）；
     - 右上角：运行状态丸与 FPS / 丢帧延迟指示；
     - 左下角：实时推拿总接触力读数（大字牌 `24.5 N`）；
     - 右下角：相机视角名称与 `[点击/Tab 互换]` 提示；
  3. `app.js` 中维护 `currentPrimaryCam` 状态，点击任意副视窗或按 `Tab` 时无刷新对调 `src` 与标签；
  4. 增加相机丢帧检测与红色呼吸边框告警（若延迟 > 100ms 或丢帧）。

### Task 3: 22 维生理学解剖分组卡片与指尖受力同心圆雷达
- **目标**: 将 22 个扁平柱状图重构为“臂 6D + 手指 4×4D”生理学生动卡片，指尖受力以同心圆雷达直观展示。
- **文件**: `index.html`, `style.css`, `app.js`
- **步骤**:
  1. 在 HTML 中划分为 5 个卡片组：
     - `reBot 机械臂 (6-DOF)`：基座、大臂、肘、腕 R/P/Y
     - `灵巧手 拇指 (Thumb 4-DOF)`
     - `灵巧手 食指 (Index 4-DOF)`
     - `灵巧手 中指 (Middle 4-DOF)`
     - `灵巧手 无名指 (Ring 4-DOF)`
  2. 柱状图采用双向对称中位零点渲染（向左伸展平伸，向右弯曲屈曲）；
  3. 绘制指尖受力雷达卡片：4 个独立圆形指示气泡，受力越大半径与明度越大，绿色 (<10N) -> 黄色 (10-30N) -> 红色 (>35N 警示)。

### Task 4: 试次历史抽屉、目标配额与底部功能坞 (Hotkey Ribbon)
- **目标**: 实时反馈当前会话的采集成果与当日配额，底部常驻快捷键指南。
- **文件**: `index.html`, `style.css`, `app.js`
- **步骤**:
  1. 在界面右侧增加 `.episode-drawer`（试次抽屉与配额面板）：
     - 显示采集配额进度条（如 `今日进度: 12 / 50 条 (24%)`）；
     - 维护本地会话试次历史列表（显示试次编号、时长、总帧数、平均受力、保存/舍弃状态）；
  2. 底部增加常驻黑底灰字的功能坞 `.hotkey-ribbon`（显示所有键位图例）；
  3. 实现全屏禅道模式（`.zen-mode`）：隐藏非关键面板，主相机最大化沉浸示教。

### Task 5: 自动化回归测试与端到端质检
- **目标**: 保证原有控制循环、API 接口、SSE 流与 LeRobot 导出器 100% 兼容。
- **步骤**:
  1. 运行 `pytest tests/test_collection_hub_api.py -q`；
  2. 运行 `bash scripts/e2e_hub_collection_check.sh` 验证端到端真实录制与 H.264 MP4 视频封装；
  3. 同步更新文档并向用户汇报。

---

## 四、 验证标准与成功准则 (Definition of Done)
1. 浏览器打开 `http://127.0.0.1:8080`，呈现现代化具身大视窗布局；
2. 按 `Space` 启动录制时听到升调蜂鸣，主视窗显示红色 REC 标牌；
3. 按 `1-4` 键能实时切换手法阶段并在主视窗 OSD 显示；
4. 按 `Tab` 或点击副机位能无缝对调主副相机；
5. 22 维关节按臂/手生理学生理学分组对齐呈现；
6. 试次完成后点击保存，右侧试次列表实时增加一条记录，后台正确写入 `LeRobotDataset v3.0`；
7. 全量单测与 E2E 脚本通过。
