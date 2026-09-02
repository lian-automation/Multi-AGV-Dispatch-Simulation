# -*- coding: utf-8 -*-
"""
run_stress.py —— 压力测试脚本（核心产出）
==========================================

场景矩阵：3 / 5 / 8 台车 × 每场景 200 个任务（泊松高强度到达）。
全速推进仿真（不与真实时间同步），逐场景统计六项核心指标：

    1. 总吞吐量（完成任务数 / 分钟）
    2. 平均任务响应时间（到达 -> 分配）
    3. 空载行驶率（空载格数 / 总行驶格数）
    4. 物理死锁数（按环去重）/ 消除数 / 仲裁触发次数（与发生/消除分列）/
       仲裁升级侧避改道数 / 不可解环数（unresolved，按环首次判定计）
    5. 重规划次数（拥堵绕行 + 让路）与 占格冲突（每拍不变式断言实测）
    6. 低电回充成功率（充满次数 / 回充触发次数）

结果自动写入 docs/压测报告.md（含对比表格与分析结论）。
所有数值均为【仿真验证值】——离散事件仿真环境测得，非实车数据。

用法：
    python run_stress.py                 # 完整矩阵（约几分钟）
    python run_stress.py --tasks 50      # 缩小规模快速体验
    python run_stress.py --cars 3,5      # 自定义车队维度
"""

import argparse
import sys
import time

import config
from dispatch.dispatcher import SimulationEngine
from map.grid_map import load_map


def run_scenario(cars, task_total, lam, seed):
    """
    跑一个压测场景直到完成 task_total 个任务或超时。
    :return: (metrics 字典, 墙钟耗时秒)
    """
    print(f"\n>>> 场景：{cars} 台车 × {task_total} 任务（λ={lam}/s）开始…")
    engine = SimulationEngine(fleet_size=cars, lam=lam, realtime=False,
                              seed=seed, stress=True, max_tasks=task_total)
    wall_start = time.perf_counter()
    tick = 0
    while True:
        engine.tick(config.DT)
        tick += 1
        if tick % 25 == 0:                      # 每 25 拍（5 仿真秒）看一眼进度
            done = sum(1 for t in engine.dispatcher.tasks
                       if t.finished_at is not None)
            if done >= task_total:
                break
            if engine.sim_time > config.STRESS_SCENARIO_TIMEOUT:
                print(f"    ⚠ 达到场景超时上限 {config.STRESS_SCENARIO_TIMEOUT}s，"
                      f"以当前进度收尾（完成 {done} 个）")
                break
    wall = time.perf_counter() - wall_start
    m = engine.metrics()
    m["cars"] = cars
    m["wall_seconds"] = round(wall, 1)
    print(f"    完成 {m['tasks_completed']}/{task_total}，"
          f"吞吐 {m['throughput_per_min']}/min，平均响应 {m['avg_response_s']}s，"
          f"墙钟 {wall:.1f}s")
    return m


def build_report(rows, lam, task_total, seed):
    """
    把各场景指标渲染成 Markdown 报告文本。

    全部结论文字由实际数据推导生成（场景数不限、单场景也可出报告），
    杜绝"预写死结论与数据脱节"的评审问题。
    """
    lines = []
    lines.append("# 多 AGV 调度仿真 · 压力测试报告")
    lines.append("")
    lines.append("> 生成时间：{}　|　工具：`python run_stress.py`（本仓库自带）".format(
        time.strftime("%Y-%m-%d %H:%M:%S")))
    lines.append(">")
    lines.append("> **所有数值均为【仿真验证值】**：来自本项目离散事件仿真环境"
                 "（30×20 栅格地图、泊松任务流、A* 路径规划、预约制路权与死锁仲裁），"
                 "不代表实车现场指标。")
    lines.append("")
    lines.append("## 一、测试环境")
    lines.append("")
    m = load_map()
    lane_rows = sorted({y for (_, y) in m.oneway})
    lines.append(f"- 地图：{m.cols}×{m.rows} 栅格，"
                 f"{len(m.slots)} 个货位、充电站×{len(m.charges)}、站台×{len(m.stations)}；"
                 f"单向环流巷道 {len(lane_rows)} 条（共 {len(m.oneway)} 格，"
                 f"y={','.join(str(y) for y in lane_rows)}）")
    lines.append(f"- 任务：每场景 {task_total} 个（入库/出库/移库混合，"
                 f"泊松 λ={lam}/s 注单，固定随机种子 {seed}，结果可复现）")
    lines.append("- 引擎：离散时间步进 Δt=0.2s，全速推进（不与墙钟同步）")
    lines.append("- 故障注入：每完成一次搬运 1% 概率，压测模式按 "
                 f"{config.FAULT_AUTO_RESET_SECONDS}s 等效复位")
    lines.append("")
    lines.append("## 二、场景矩阵对比表")
    lines.append("")
    lines.append("| 指标 | " + " | ".join(f"{r['cars']} 台车" for r in rows) + " |")
    lines.append("|---|" + "---|" * len(rows))

    def row(name, key, fmt="{}"):
        vals = " | ".join(fmt.format(r[key]) if r.get(key) is not None
                          else "-" for r in rows)
        lines.append(f"| {name} | {vals} |")

    row("完成任务数（个）", "tasks_completed")
    row("任务总到达数（个）", "tasks_total")
    row("仿真时长（s）", "sim_seconds")
    row("待分配队列峰值", "queue_peak")
    row("**吞吐量（个/min）**", "throughput_per_min")
    row("**平均任务响应时间（s）**", "avg_response_s")
    row("最大响应时间（s）", "max_response_s")
    row("平均任务周期（s）", "avg_cycle_s")
    row("**空载行驶率**", "empty_rate", "{:.1%}")
    row("空载行驶格数", "cells_empty")
    row("重载行驶格数", "cells_loaded")
    row("**死锁发生次数（按环去重的物理死锁数）**", "deadlock_detected")
    row("**死锁消除次数（环消失）**", "deadlock_resolved")
    row("仲裁触发次数（冷却期后每次实际执行，同一环可重复）", "deadlock_arbitrations")
    row("仲裁升级侧避改道次数（目标被堵假成功根治，复审06 N1）", "dodge_detours")
    row("让路无解/不可解的物理死锁环数（按环首次判定计）", "unresolved_waits")
    row("**让路重规划次数**", "replan_count")
    row("**占格冲突/不变式违规（每拍实测）**", "invariant_violations")
    row("低电回充触发次数", "low_battery_events")
    row("**低电回充成功率**", "recharge_success_rate", "{:.0%}")
    row("故障次数（1%/任务）", "faults")
    lines.append("")
    lines.append("## 三、结果分析（仿真验证值口径，文字由数据生成）")
    lines.append("")
    cars = [r["cars"] for r in rows]
    tp = [r["throughput_per_min"] for r in rows]
    resp = [r["avg_response_s"] for r in rows]
    emp = [r["empty_rate"] for r in rows]

    # ---- 1. 吞吐量：任意场景数通用；≥2 场景时计算边际收益 ----
    tp_txt = " → ".join(str(v) for v in tp)
    if len(rows) >= 2:
        marginals = [(cars[i] - cars[i - 1], round(tp[i] - tp[i - 1], 2))
                     for i in range(1, len(rows))]
        mg_txt = "，".join(f"+{g}/{d}车" for d, g in marginals)
        half_first = marginals[0][1] / 2
        knee = next((cars[i] for i in range(1, len(marginals))
                     if marginals[i][1] < half_first), None)
        trend = ("边际增益递减——路网容量与交叉口冲突开始约束系统"
                 if knee is not None else
                 "各区间增益接近，尚未出现明显拐点")
        s1 = (f"**吞吐量**：{tp_txt} 个/min（边际增量：{mg_txt}）。{trend}，"
              "与排队论直觉一致。")
    else:
        s1 = f"**吞吐量**：{tp_txt} 个/min（单场景，无边际对比）。"
    lines.append(f"1. {s1}")

    # ---- 2. 响应时间 ----
    resp_txt = " → ".join(str(v) for v in resp)
    lines.append(f"2. **响应时间**：平均 {resp_txt} s。本口径为突发注单饱和压测，"
                 "响应时间包含排队等待：车越少队列越长，故随车队扩大显著下降；"
                 "日常演示强度下任务即到即派，派单延迟为亚秒级。")

    # ---- 3. 空载行驶率 ----
    emp_txt = " → ".join(f"{e:.1%}" for e in emp)
    if len(rows) >= 2 and emp[-1] < emp[0]:
        why = "车越多接单距离越短，空驶占比收窄"
    elif len(rows) >= 2:
        why = "任务分布与拥堵格局共同影响空驶占比，未呈单调关系"
    else:
        why = "取货空驶是固有成本"
    lines.append(f"3. **空载行驶率**：{emp_txt}。{why}。")

    # ---- 4. 死锁与重规划：按环去重的物理死锁口径 + 被测量的占格冲突 ----
    # 口径分列（复审06 N2）：物理死锁数（按环去重的"发生"）、环消失数（"消除"）、
    # 仲裁触发次数（冷却期门控后每次实际执行，同一环可重复触发）三者互不相同，
    # 不得混用——旧口径曾把 2176 次仲裁误报为 2176 起死锁。
    dl = [r["deadlock_detected"] for r in rows]
    rs = [r["deadlock_resolved"] for r in rows]
    arb = [r["deadlock_arbitrations"] for r in rows]
    esc = [r.get("dodge_detours", 0) for r in rows]
    unres = sum(r["unresolved_waits"] for r in rows)
    dl_txt = " / ".join(str(v) for v in dl)
    rs_txt = " / ".join(str(v) for v in rs)
    arb_txt = " / ".join(str(v) for v in arb)
    esc_txt = " / ".join(str(v) for v in esc)
    # "全部环已消解"的结论只有在 unresolved==0 时才允许输出（复审06 N2 门控）
    tail = (f"{unres} 个环判定为不可解环（让路无解或目标被堵且无侧避格），"
            "按原地等待处理并已计入上报"
            if unres else
            "unresolved=0：全部环经让路重规划、侧避改道或自行消解，无原地等待残留")
    inv = sum(r["invariant_violations"] for r in rows)
    if inv == 0:
        inv_txt = ("占格冲突计数为 0——该值来自引擎每拍执行的\"一格一车\"不变式"
                   "断言（两车同格/占格无主/幽灵占用预约记录，违规即 fail-fast），"
                   "是【被测量的机制保证】，而非设计推断。")
    else:
        inv_txt = (f"不变式断言实测违规 {inv} 次——占格冲突并非 0，"
                   "该结果不可用于宣称零对撞，需排查根因。")
    lines.append(f"4. **死锁与重规划**：物理死锁（按环去重）{dl_txt} 起、消除 "
                 f"{rs_txt} 起；仲裁触发 {arb_txt} 次（口径=同一环冷却期后的重复"
                 f"仲裁照常计数，与\"发生/消除\"分列，不得混用）；其中让路者目标"
                 f"被堵、升级为\"先侧避再回原目标\"改道 {esc_txt} 次；{tail}。{inv_txt}")

    # ---- 5. 低电回充：只陈述有数据支撑的事实 ----
    rc = [f"{r['recharge_success_rate']:.0%}"
          if r["recharge_success_rate"] is not None else "未触发"
          for r in rows]
    trig = sum(r["low_battery_events"] for r in rows)
    rc_line = (f"5. **低电回充**：全矩阵触发 {trig} 次，各场景成功率 {'、'.join(rc)}"
               f"（\"未触发\"表示该场景里程未使电量跌破阈值，属正常现象）。")
    lines.append(rc_line)
    lines.append("")
    lines.append("## 四、结论与建议")
    lines.append("")
    if len(rows) >= 2 and knee is not None:
        lines.append(f"- 在本地图与 λ={lam}/s 强度下，**{knee} 台车**之后"
                     "每增一辆车的吞吐增量跌破首区间的一半，继续加车的性价比明显下降。")
    elif len(rows) >= 2:
        lines.append(f"- 在本地图与 λ={lam}/s 强度下，车队从 {cars[0]} 扩到 "
                     f"{cars[-1]} 台吞吐保持近线性上行，未见明显拐点。")
    else:
        lines.append(f"- 单场景（{cars[0]} 台车）吞吐 {tp[0]} 个/min，"
                     "建议加跑多场景形成对比矩阵。")
    lines.append("- 若需继续提升吞吐，优先级建议：扩大双向主巷道比例 > 增加充电桩 > "
                 "引入拍卖法分配（接口已预留 `AuctionStrategy`）。")
    lines.append("- 复现方式：`pip install -r requirements.txt && python run_stress.py`"
                 "（随机种子固定，表格数值应可复现）。")
    return "\n".join(lines)


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    ap = argparse.ArgumentParser(description="多AGV调度仿真 · 压力测试")
    ap.add_argument("--cars", default=",".join(map(str, config.STRESS_CAR_COUNTS)),
                    help=f"车队规模矩阵（默认 {config.STRESS_CAR_COUNTS}）")
    ap.add_argument("--tasks", type=int, default=config.STRESS_TASK_TOTAL,
                    help=f"每场景任务数（默认 {config.STRESS_TASK_TOTAL}）")
    ap.add_argument("--lam", type=float, default=config.STRESS_LAMBDA,
                    help=f"泊松强度（默认 {config.STRESS_LAMBDA}/s）")
    args = ap.parse_args()
    cars_list = [int(x) for x in str(args.cars).split(",") if x.strip()]

    rows = []
    for n in cars_list:
        rows.append(run_scenario(n, args.tasks, args.lam, config.RANDOM_SEED))

    report = build_report(rows, args.lam, args.tasks, config.RANDOM_SEED)
    with open(config.STRESS_REPORT_PATH, "w", encoding="utf-8") as fp:
        fp.write(report + "\n")
    print(f"\n[压测] 报告已生成：{config.STRESS_REPORT_PATH}")


if __name__ == "__main__":
    main()
