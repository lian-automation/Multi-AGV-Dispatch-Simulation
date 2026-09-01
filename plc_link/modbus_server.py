# -*- coding: utf-8 -*-
"""
modbus_server.py —— pymodbus 从站：模拟 4 个站台呼叫盒（规格 5）
================================================================

寄存器映射（保持寄存器 HR，单元号 config.MODBUS_UNIT_ID）：
    HR0 ~HR3  ：站台 1~4 呼叫。外部写 1 = 工人按下呼叫按钮生成搬运任务，
                从站消费后自动清零；
    HR10~HR13 ：站台 1~4 完成码。对应站台的搬运任务送达后置 1，
                客户端读走后应回写 0 应答（握手闭环）；
    HR20      ：心跳，每秒自增 1（mod 65536），用于监控从站存活。

两种运行方式：
    1) 独立演示（无仿真引擎）：python -m plc_link.modbus_server
       收到呼叫后约 5 秒自动点亮完成码，便于单独调试协议链路；
    2) 集成运行：由 main.py 创建 ModbusBridge 并注入引擎（engine.modbus），
       呼叫会真实转化为出库任务，任务送达站台后点亮完成码。

线程模型：
    pymodbus 3.6 的服务端是 asyncio 实现 —— 本模块在独立子线程里启动事件循环；
    仿真引擎线程通过线程安全的 deque / call_soon_threadsafe 与之交互，
    业务代码完全不必关心异步细节。
"""

import asyncio
import sys
import threading
import time
from collections import deque

from pymodbus.datastore import (ModbusSequentialDataBlock,
                                ModbusServerContext, ModbusSlaveContext)
from pymodbus.server import StartAsyncTcpServer

import config

# 功能码 3 = 读保持寄存器（datastore 按功能码分区存取）
FC_HOLDING = 3


class ModbusBridge:
    """
    引擎 <-> Modbus 从站的桥接器。
    仿真引擎只认识三个方法：
        start()                 启动从站后台线程
        poll_calls(engine)      引擎每拍调用：取出新呼叫并转成出库任务
        set_completion(k)       任务送达站台 k 时点亮完成码
    """

    def __init__(self, host=None, port=None, standalone=False):
        self.host = host or config.MODBUS_HOST
        self.port = port or config.MODBUS_PORT
        self.standalone = standalone     # True=独立演示模式（自动点亮完成码）

        self._calls = deque()            # 待处理的呼叫站台序号（线程安全足够）
        self._lock = threading.Lock()
        self._loop = None                # 从站 asyncio 事件循环（子线程内）
        self._block = None               # 保持寄存器数据块（仅循环线程访问！）
        self._thread = None              # 从站守护线程

    # ==================================================================
    # 启动 / 停止
    # ==================================================================
    def start(self):
        """在守护线程中启动 asyncio 事件循环与 Modbus/TCP 从站。"""
        try:
            sys.stdout.reconfigure(encoding="utf-8")
        except Exception:
            pass
        self._thread = threading.Thread(target=self._run_loop, daemon=True,
                                        name="ModbusSlaveThread")
        self._thread.start()

    def stop(self):
        """尽力而为地关闭从站（进程退出时守护线程也会自然结束）。"""
        try:
            from pymodbus.server import ServerAsyncStop
            if self._loop is not None:
                self._loop.call_soon_threadsafe(ServerAsyncStop.set)
        except Exception:
            pass  # 进程即将退出，忽略关闭异常

    def _run_loop(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._loop = loop
        loop.run_until_complete(self._serve())

    async def _serve(self):
        # ---- 构造数据区：32 个保持寄存器，初值全 0 ----
        self._block = ModbusSequentialDataBlock(0, [0] * config.HR_BLOCK_SIZE)
        try:
            slave = ModbusSlaveContext(hr=self._block, zero_mode=True)  # 0 基地址
        except TypeError:            # 兼容不同小版本的参数名
            slave = ModbusSlaveContext(hr=self._block)
        context = ModbusServerContext(slaves=slave, single=True)

        print(f"[Modbus] 从站启动：{self.host}:{self.port} unit="
              f"{config.MODBUS_UNIT_ID}"
              f"{'（独立演示模式：呼叫后5s自动完成）' if self.standalone else '（挂接仿真引擎）'}")
        # 心跳、呼叫扫描与 TCP 服务三者并行：
        # pymodbus 3.6 的 StartAsyncTcpServer 是阻塞协程，负责监听并处理请求
        await asyncio.gather(
            StartAsyncTcpServer(context=context,
                                address=(self.host, self.port)),
            self._heartbeat_loop(),
            self._call_watch_loop(),
        )

    # ==================================================================
    # 从站内部协程（均在从站事件循环线程内执行，可安全读写数据块）
    # ==================================================================
    async def _heartbeat_loop(self):
        """HR20 心跳每秒自增（mod 65536）。"""
        while True:
            await asyncio.sleep(config.MODBUS_HEARTBEAT_PERIOD)
            cur = self._get_reg(config.HR_HEARTBEAT)
            self._set_reg(config.HR_HEARTBEAT, (cur + 1) % 65536)

    async def _call_watch_loop(self):
        """轮询 HR0~3：发现呼叫(值=1)立即清零消费，按模式分发。"""
        while True:
            await asyncio.sleep(config.MODBUS_CALL_POLL)
            for k in range(self.map_station_count()):
                addr = config.HR_CALL_BASE + k
                if self._get_reg(addr) == 1:
                    self._set_reg(addr, 0)              # 消费掉呼叫信号
                    if self.standalone:
                        # 协程内应使用 get_running_loop()（P3-7）：
                        # get_event_loop() 在无运行循环场景会触发弃用告警
                        asyncio.get_running_loop().create_task(
                            self._auto_complete(k))
                        print(f"[Modbus] 站台{k+1} 呼叫 -> 5 秒后模拟完成")
                    else:
                        with self._lock:
                            self._calls.append(k)       # 交给仿真引擎处理
                        print(f"[Modbus] 站台{k+1} 呼叫 -> 已转发调度系统")

    @staticmethod
    def map_station_count():
        """"4 个站台"的来源：与地图站台数一致（读 config 校验过的地图为 4）。"""
        return 4

    async def _auto_complete(self, k):
        """独立演示模式：5 秒后点亮完成码，模拟一次完整搬运。"""
        await asyncio.sleep(5.0)
        self._set_reg(config.HR_DONE_BASE + k, 1)
        print(f"[Modbus] 站台{k+1} 任务完成 -> HR{config.HR_DONE_BASE + k}=1")

    # ==================================================================
    # 数据块读写（仅从站循环线程内调用）
    # 注意：pymodbus 3.6 中数据块层的方法签名是 (address, ...)，
    #       功能码参数只存在于 ModbusSlaveContext 层，两者不能混用。
    # ==================================================================
    def _get_reg(self, addr):
        return self._block.getValues(addr, count=1)[0]

    def _set_reg(self, addr, value):
        self._block.setValues(addr, [int(value)])

    # ==================================================================
    # 仿真引擎侧接口（引擎线程调用）
    # ==================================================================
    def poll_calls(self, engine):
        """取走全部待处理呼叫，逐个转成"货位 -> 站台k"的出库任务。"""
        while True:
            with self._lock:
                if not self._calls:
                    break
                k = self._calls.popleft()
            engine.dispatcher.create_task(
                "outbound", now=engine.sim_time, station_index=k)

    def set_completion(self, station_index):
        """任务送达站台 station_index：跨线程投递点亮完成码的指令。"""
        if self._loop is None:
            return
        addr = config.HR_DONE_BASE + station_index
        self._loop.call_soon_threadsafe(self._set_reg, addr, 1)

    def trigger_call(self, station_index):
        """
        看板"模拟站台呼叫"按钮：向 HR_CALL_BASE+k 写 1，
        与外部呼叫盒/自测客户端按键完全等效（走同一条寄存器链路）。
        :return: 写入的寄存器地址；从站未启动时返回 None
        """
        if self._loop is None:
            return None
        addr = config.HR_CALL_BASE + station_index
        self._loop.call_soon_threadsafe(self._set_reg, addr, 1)
        return addr


# ----------------------------------------------------------------------
# 独立运行入口：python -m plc_link.modbus_server
# ----------------------------------------------------------------------
def main():
    bridge = ModbusBridge(standalone=True)
    bridge.start()
    print("[Modbus] 按 Ctrl+C 退出。另开终端运行 "
          "python -m plc_link.modbus_client_test 可自测链路。")
    try:
        while True:
            time.sleep(1.0)          # 主线程挂起保活；从站跑在守护线程里
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()


if __name__ == "__main__":
    main()
