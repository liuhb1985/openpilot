#!/usr/bin/env python3
"""
fake_pandad — PC 模式下的占位进程

在 PC 端口没有真实 Panda 设备时，manager.py 仍会订阅 deviceState 和
pandaStates；如果这两个 endpoint 不存在，SubMaster 构造时会直接报
IpcError ("No such file or directory")。

真正运行的 pandad 是 C++ 二进制 openpilot/selfdrive/pandad/pandad，
它检测到 Panda 硬件后才会 bind publisher；PC 上没有硬件，binder
永远不建立，于是 manager 卡在订阅阶段。

这里提供一个最小替代：每秒发布一次空的 pandaStates 与 offroad
deviceState。它并不模拟 can/ignition/电压等信号——只让 manager
与上游受它做的事能够跑起来。PC 调试 UI / 模型路径用。
"""
import os
import sys
import time
import signal

import cereal.messaging as messaging

from openpilot.common.swaglog import cloudlog

running = True

def _stop(*_):
  global running
  running = False

signal.signal(signal.SIGINT, _stop)
signal.signal(signal.SIGTERM, _stop)


def main():
  pm = messaging.PubMaster(['deviceState', 'pandaStates'])

  cloudlog.info("fake_pandad: starting on PC; publishing synthetic deviceState + pandaStates")

  interval = float(os.getenv('FAKE_PANDAD_INTERVAL', '1.0'))
  next_log = time.monotonic() + 60

  while running:
    now = time.monotonic()
    msg = messaging.new_message('deviceState')
    msg.deviceState.started = False
    pm.send('deviceState', msg)

    ps_msg = messaging.new_message('pandaStates', 0)
    pm.send('pandaStates', ps_msg)

    if now > next_log:
      cloudlog.debug(f"fake_pandad: heartbeat (interval={interval}s)")
      next_log = now + 60

    time.sleep(interval)

  cloudlog.info("fake_pandad: exiting")


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    sys.exit(0)