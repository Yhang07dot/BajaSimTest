#!/usr/bin/env python3
"""View the atomic JSON score report without ROS setup or extra dependencies."""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/baja_perception_sim'))
from baja_perception_sim.score_display import format_score


def main():
    parser = argparse.ArgumentParser(description='实时显示感知算法评分，Ctrl+C 退出')
    parser.add_argument('--run-dir', type=Path, default=ROOT / 'generated/current')
    parser.add_argument('--once', action='store_true', help='只显示一次')
    args = parser.parse_args()
    report = args.run_dir.resolve() / 'results/summary.json'
    try:
        while True:
            if sys.stdout.isatty() and not args.once:
                print('\033[2J\033[H', end='')
            print(f'评分文件：{report}')
            try:
                result = json.loads(report.read_text())
                age = time.time() - report.stat().st_mtime
                if age > 3:
                    print(f'离线报告：文件已 {age:.0f} 秒未更新。运行仿真后会自动刷新。')
                print(format_score(result))
            except FileNotFoundError:
                print('尚未生成评分文件。请先启动 ./run.sh；评分器每秒更新一次。')
            except (json.JSONDecodeError, OSError) as error:
                print(f'暂时无法读取报告：{error}')
            if args.once:
                return
            print('\nCtrl+C 退出评分查看，不会停止仿真。', flush=True)
            time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
