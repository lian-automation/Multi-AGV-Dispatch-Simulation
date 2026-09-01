# -*- coding: utf-8 -*-
"""traffic 包：交通管制（路权预约 + 死锁检测与解除）。对外导出 TrafficController。"""

from .controller import TrafficController

__all__ = ["TrafficController"]
