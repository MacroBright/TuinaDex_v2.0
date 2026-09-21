#!/usr/bin/env bash
# ==============================================================================
# TuinaDex v2.0 - ROS2 WebSocket Bridge Launcher for ROS MCP Server
# ==============================================================================
# 作用: 启动 rosbridge_server 提供 WebSocket (默认 ws://0.0.0.0:9090)，
#       供 ros-mcp-server (FastMCP) 与多智能体 (Antigravity/Claude Code/Codex)
#       实时互联、感知 ROS 2 节点状态与发送指令。
# ==============================================================================

set -e

# 加载 ROS 2 环境
if [ -f "/opt/ros/lyrical/setup.bash" ]; then
    # shellcheck disable=SC1091
    source /opt/ros/lyrical/setup.bash
else
    echo "[ERROR] 未检测到 /opt/ros/lyrical/setup.bash，请确认 ROS2 Lyrical 已安装！" >&2
    exit 1
fi

PORT="${1:-9090}"
ADDRESS="${2:-0.0.0.0}"

echo "============================================================"
echo " [TuinaDex ROS2] 正在启动 rosbridge_websocket..."
echo " 监听地址: ws://${ADDRESS}:${PORT}"
echo " MCP 接入点: 127.0.0.1:${PORT}"
echo " 按 Ctrl+C 终止服务"
echo "============================================================"

exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:="${ADDRESS}" port:="${PORT}"
