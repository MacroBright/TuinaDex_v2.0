"""cli.py — python -m tuinadex_dataset.cli --bag <dir> --out <repo_id> [--fps 50]."""
import argparse

from .adapter import convert_bag_to_episode, save_to_lerobot


def main():
    ap = argparse.ArgumentParser(description="rosbag2 -> LeRobotDataset 离线转换")
    ap.add_argument("--bag", required=True, help="rosbag2 目录路径")
    ap.add_argument("--out", required=True, help="LeRobotDataset repo_id / 输出路径")
    ap.add_argument("--fps", type=float, default=50.0)
    ap.add_argument("--allow-invalid", action="store_true",
                    help="校验失败时仍导出 (默认拒绝)")
    args = ap.parse_args()

    grid, states, actions, ok, errors = convert_bag_to_episode(args.bag, fps=args.fps)
    if not ok and not args.allow_invalid:
        raise SystemExit("校验失败, 拒绝导出:\n  " + "\n  ".join(errors))
    if not ok:
        print("[警告] 校验失败但按 --allow-invalid 继续:\n  " + "\n  ".join(errors))
    out = save_to_lerobot(grid, states, actions, repo_id=args.out, fps=args.fps)
    print(f"完成: {len(grid)} 帧 @ {args.fps}Hz -> {out}")


if __name__ == "__main__":
    main()
