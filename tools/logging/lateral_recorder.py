#!/usr/bin/env python3
"""Record only the messages needed for lateral-control tuning.

This is a lightweight alternative to a full route log. It subscribes to the
live cereal bus and writes a replayable capnp log (bz2 compressed).

Usage:
  python3 tools/logging/lateral_recorder.py --duration 300 --output lateral_run1.bz2
  python3 tools/logging/lateral_recorder.py --addr 192.168.1.5   # remote ZMQ bridge
"""
import argparse
import os
import time

from openpilot.tools.lib.live_logreader import live_logreader
from openpilot.tools.lib.logreader import save_log

SERVICES = [
  "controlsState",
  "carState",
  "carControl",
  "carOutput",
  "modelV2",
  "lateralPlan",
  "liveParameters",
  "liveTorqueParameters",
  "liveDelay",
  "livePose",
  "gyroscope",
  "accelerometer",
]


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--duration", type=float, default=300.0, help="seconds to record")
  parser.add_argument("--output", type=str, default="", help="output .bz2 path")
  parser.add_argument("--addr", type=str, default="127.0.0.1", help="cereal bus address")
  args = parser.parse_args()

  output = args.output or f"lateral_{int(time.time())}.bz2"
  if not output.endswith(".bz2"):
    output += ".bz2"

  messages = []
  start = time.monotonic()
  print(f"[lateral_recorder] recording {args.duration:.0f}s -> {output}", flush=True)
  try:
    for event in live_logreader(SERVICES, addr=args.addr):
      messages.append(event)
      if time.monotonic() - start >= args.duration:
        break
  except KeyboardInterrupt:
    print("[lateral_recorder] interrupted, saving what was captured", flush=True)
  finally:
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    save_log(output, messages, compress=True)
    print(f"[lateral_recorder] saved {len(messages)} events to {output}", flush=True)


if __name__ == "__main__":
  main()
