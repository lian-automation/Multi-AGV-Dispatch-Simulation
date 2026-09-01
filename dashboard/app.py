# -*- coding: utf-8 -*-
"""
app.py —— Flask Web 看板后端（规格 6）
========================================

路由一览：
    GET  /                      看板页面（templates/index.html）
    GET  /api/state             全量状态快照：地图/车辆/任务/曲线/事件（前端 500ms 轮询）
    POST /api/pause             暂停 / 继续仿真
    POST /api/agv/<id>/reset    人工复位指定故障车（对应"待人工复位"规格）
    POST /api/call/<k>          模拟站台 k 呼叫（写 HRk=1，走真实 Modbus 链路）

两种运行方式：
    1) 集成运行（推荐）：python main.py —— 引擎+Modbus+看板一起启动；
    2) 单独调试看板：python -m dashboard.app —— 内部同样会拉起引擎与从站。
"""

import threading

from flask import Flask, jsonify, render_template

import config
from dispatch.dispatcher import SimulationEngine


def create_app(engine):
    """工厂函数：把仿真引擎注入 Flask 应用。"""

    app = Flask(__name__)

    @app.route("/")
    def index():
        return render_template("index.html")

    @app.route("/api/state")
    def api_state():
        """全量快照。引擎加锁拷贝，避免读到半更新的数据。"""
        return jsonify(engine.snapshot())

    @app.route("/api/pause", methods=["POST"])
    def api_pause():
        # 线程纪律（P2-4）：与引擎 tick 互斥，外部线程不绕锁直改状态
        with engine.lock:
            engine.paused = not engine.paused
            state = "暂停" if engine.paused else "继续"
            engine.events.add("info", f"仿真已{state}（看板操作）")
            paused = engine.paused
        return jsonify({"paused": paused})

    @app.route("/api/agv/<int:agv_id>/reset", methods=["POST"])
    def api_reset(agv_id):
        # 线程纪律（P2-4）：复位会改 agv 的 state/path/task/next_cell，
        # 必须持 engine.lock 与 tick 互斥
        with engine.lock:
            for agv in engine.agvs:
                if agv.id == agv_id:
                    if agv.reset_fault():
                        agv.fault_reset_at = None
                        engine.events.add("fault", f"AGV{agv.id} 已人工复位（看板按钮），恢复待命")
                        return jsonify({"ok": True})
                    return jsonify({"ok": False, "msg": "该车不在故障态"}), 400
            return jsonify({"ok": False, "msg": "车辆不存在"}), 404

    @app.route("/api/call/<int:k>", methods=["POST"])
    def api_call(k):
        """模拟站台呼叫按钮：优先走 Modbus 寄存器链路（更贴近真实）。"""
        if not 0 <= k <= 3:
            return jsonify({"ok": False, "msg": "站台序号 0~3"}), 400
        # 线程纪律（P2-4）：任务创建（含直连退化路径的 Task id 分配与
        # dispatcher.tasks.append）统一持 engine.lock，与 assign_pending 互斥
        with engine.lock:
            bridge = engine.modbus
            if bridge is not None:
                addr = bridge.trigger_call(k)
                via = f"Modbus HR{addr}=1"
            else:
                # 未启用 Modbus 时退化为直接生成任务（保证功能不缺失）
                engine.dispatcher.create_task("outbound", now=engine.sim_time,
                                              station_index=k)
                via = "直连调度器（未启用Modbus）"
        return jsonify({"ok": True, "via": via})

    return app


def main():
    """单独调试入口：python -m dashboard.app"""
    # ---- 启动 Modbus 从站（站台呼叫盒）----
    from plc_link.modbus_server import ModbusBridge
    bridge = ModbusBridge()
    bridge.start()

    # ---- 启动仿真引擎线程 ----
    engine = SimulationEngine()
    engine.modbus = bridge
    worker = threading.Thread(target=engine.run, daemon=True, name="SimEngineThread")
    worker.start()

    # ---- 前台运行 Flask ----
    app = create_app(engine)
    print(f"[Web] 看板地址 http://{config.WEB_HOST}:{config.WEB_PORT}")
    app.run(host=config.WEB_HOST, port=config.WEB_PORT,
            debug=False, use_reloader=False)


if __name__ == "__main__":
    main()
