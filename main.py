# -*- coding: utf-8 -*-
"""
main.py —— 一键启动入口
========================

按依赖顺序拉起三大件：
    1. Modbus/TCP 从站线程（模拟 4 个站台呼叫盒，127.0.0.1:5020）；
    2. 仿真引擎线程（地图 + 车队 + 调度 + 交通管制，按真实时间节拍推进）；
    3. Flask Web 看板（http://127.0.0.1:5000，前台阻塞运行）。

常用命令：
    python main.py                 # 一键启动（推荐）
    python main.py --cars 8        # 8 台车演示
    python main.py --lam 0.6       # 调大任务到达强度
    python main.py --no-modbus     # 不启动 Modbus 从站（纯看板演示）
    python main.py --headless      # 无界面跑引擎（配合日志观察）
Ctrl+C 依次优雅停机：看板 -> 引擎 -> 从站。
"""

import argparse
import sys
import threading

import config
from dashboard.app import create_app
from dispatch.dispatcher import SimulationEngine
from plc_link.modbus_server import ModbusBridge


def parse_args():
    p = argparse.ArgumentParser(description="多AGV调度与交通管制仿真 - 一键启动")
    p.add_argument("--cars", type=int, default=config.FLEET_SIZE,
                   help=f"车队规模（默认 {config.FLEET_SIZE}）")
    p.add_argument("--lam", type=float, default=config.TASK_LAMBDA,
                   help=f"任务泊松到达强度 λ（默认 {config.TASK_LAMBDA}/s）")
    p.add_argument("--no-modbus", action="store_true",
                   help="不启动 Modbus 从站")
    p.add_argument("--headless", action="store_true",
                   help="无 Web 看板，仅后台跑引擎（Ctrl+C 退出）")
    return p.parse_args()


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8")   # Windows 控制台中文防乱码
    except Exception:
        pass
    args = parse_args()

    # ---------- 1. 仿真引擎 ----------
    engine = SimulationEngine(fleet_size=args.cars, lam=args.lam)
    worker = threading.Thread(target=engine.run, daemon=True,
                              name="SimEngineThread")

    # ---------- 2. Modbus 从站 ----------
    bridge = None
    if not args.no_modbus:
        bridge = ModbusBridge()
        bridge.start()
        engine.modbus = bridge        # 注入引擎：呼叫->任务、完成->寄存器

    # ---------- 3. 启动 ----------
    worker.start()
    print(f"[main] 引擎已启动：{args.cars} 台 AGV，λ={args.lam}/s，"
          f"{'含' if bridge else '不含'} Modbus 从站")

    if args.headless:
        try:
            while worker.is_alive():
                threading.Event().wait(1.0)
        except KeyboardInterrupt:
            print("\n[main] 收到 Ctrl+C，正在停机…")
        finally:
            engine.stop()
            if bridge:
                bridge.stop()
        return

    # ---------- 4. Web 看板（前台阻塞） ----------
    from flask import Flask  # noqa: F401 仅为提示依赖
    app = create_app(engine)
    print(f"[main] 看板地址：http://{config.WEB_HOST}:{config.WEB_PORT}"
          f"  （Ctrl+C 退出）")
    try:
        app.run(host=config.WEB_HOST, port=config.WEB_PORT,
                debug=False, use_reloader=False)
    finally:
        engine.stop()                 # Flask 退出后联动停机
        if bridge:
            bridge.stop()
        print("[main] 已优雅停机")


if __name__ == "__main__":
    main()
