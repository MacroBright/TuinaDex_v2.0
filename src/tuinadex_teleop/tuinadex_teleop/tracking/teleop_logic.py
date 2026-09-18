"""teleop_logic.py — 遥操节点可单测的纯判定逻辑 (不依赖 rclpy/cv2)."""


def handedness_ok(handedness: str, required: str) -> bool:
    """required 为 "any"/"" 时接受 Left/Right 任意利手; 否则要求精确匹配."""
    if required in ("any", ""):
        return handedness in ("Left", "Right")
    return handedness == required


def regained_after_gap(prev_seen: bool, now_seen: bool) -> bool:
    """丢手 -> 重捕获 的单帧转移判定 (触发滤波器 reset 用)."""
    return (not prev_seen) and now_seen
