# -*- coding: utf-8 -*-
"""
modbus_client_test.py —— Modbus 链路自测脚本（规格 5 附带）
============================================================

对从站做一轮完整的功能验证，全部通过则打印 PASS 并以退出码 0 结束：

    步骤1  读 HR20 心跳两次（间隔约 1.2s），验证心跳在自增、链路存活；
    步骤2  依次向 HR0~HR3 写 1（模拟工人按下 4 个站台的呼叫按钮），
           轮询等待对应完成码 HR10+k 变为 1；
           （独立从站演示模式下完成码约 5 秒点亮；
             挂接仿真引擎时 = 真实派车送货到站后才点亮）
    步骤3  把读到的完成码回写 0 应答（握手闭环）。

用法：
    python -m plc_link.modbus_client_test          # 默认连 127.0.0.1:5020
"""

import sys
import time

from pymodbus.client import ModbusTcpClient

import config


def read_hr(client, address):
    """读单个保持寄存器；失败抛异常终止测试。"""
    rr = client.read_holding_registers(address, count=1,
                                       slave=config.MODBUS_UNIT_ID)
    if rr.isError():
        raise RuntimeError(f"读 HR{address} 失败：{rr}")
    return rr.registers[0]


def write_hr(client, address, value):
    """写单个保持寄存器；失败抛异常终止测试。"""
    rq = client.write_register(address, value, slave=config.MODBUS_UNIT_ID)
    if rq.isError():
        raise RuntimeError(f"写 HR{address}={value} 失败：{rq}")


def wait_completion(client, station_index, timeout_s=30.0):
    """轮询等待站台 k 的完成码变 1，返回实际耗时秒数。"""
    addr = config.HR_DONE_BASE + station_index
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if read_hr(client, addr) == 1:
            return time.time() - (deadline - timeout_s)
        time.sleep(0.2)
    raise TimeoutError(f"等待完成码 HR{addr} 超时（{timeout_s:.0f}s）")


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

    host, port = config.MODBUS_HOST, config.MODBUS_PORT
    print(f"=== Modbus 自测开始：连接 {host}:{port} ===")
    client = ModbusTcpClient(host=host, port=port)
    if not client.connect():
        print("FAIL：无法连接从站！请先启动 python -m plc_link.modbus_server 或 python main.py")
        sys.exit(1)
    print("[OK] TCP 连接建立")

    passed = True
    try:
        # ---------- 步骤 1：心跳检测 ----------
        hb1 = read_hr(client, config.HR_HEARTBEAT)
        time.sleep(1.2)
        hb2 = read_hr(client, config.HR_HEARTBEAT)
        ok = hb2 != hb1
        print(f"[{'OK' if ok else 'FAIL'}] 心跳 HR20：{hb1} -> {hb2}"
              f"（{'心跳正常' if ok else '心跳未变化！'}）")
        passed &= ok

        # ---------- 步骤 2 + 3：四站台呼叫 -> 完成码 -> 应答 ----------
        for k in range(4):
            call_addr = config.HR_CALL_BASE + k
            t0 = time.time()
            write_hr(client, call_addr, 1)               # 模拟按下呼叫盒按钮
            print(f"[.. ] 已写入呼叫 HR{call_addr}=1（站台{k+1}），等待完成码…")
            wait_completion(client, k)                   # 阻塞等 HR10+k 变 1
            dt = time.time() - t0
            write_hr(client, config.HR_DONE_BASE + k, 0)  # 回写 0 应答（握手）
            ack = read_hr(client, config.HR_DONE_BASE + k)
            ok = ack == 0
            print(f"[{'OK' if ok else 'FAIL'}] 站台{k+1}：完成码点亮耗时 {dt:.1f}s，"
                  f"应答回写后 HR{config.HR_DONE_BASE + k}={ack}")
            passed &= ok
    except Exception as exc:                             # noqa: BLE001 测试脚本要打印任何异常
        print(f"FAIL：{exc}")
        passed = False
    finally:
        client.close()

    print("=" * 46)
    print("自测结论：", "PASS —— 全部通过 ✔" if passed else "FAIL —— 存在失败项 ✘")
    sys.exit(0 if passed else 2)


if __name__ == "__main__":
    main()
