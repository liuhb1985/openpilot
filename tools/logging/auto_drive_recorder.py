#!/usr/bin/env python3
"""Automatically record a replayable lateral/full-system log while the car is moving.

The service only subscribes to the cereal bus. It starts recording when the car
moves or lateral control is active, keeps a short pre-roll, and saves the log a
few seconds after the car stops. A JSON status file is written continuously.
"""
from __future__ import annotations

import fcntl
import glob
import json
import os
import signal
import sys
import time
from collections import deque

from openpilot.system.hardware.hw import Paths
from openpilot.tools.lib.live_logreader import live_logreader
from openpilot.tools.lib.logreader import save_log


SERVICES = [
  "selfdriveState", "carState", "carControl", "carOutput", "controlsState",
  "pandaStates", "modelV2", "drivingModelData", "lateralPlan", "longitudinalPlan",
  "liveParameters", "liveTorqueParameters", "liveDelay", "livePose", "liveCalibration",
  "gyroscope", "accelerometer", "cameraOdometry", "roadCameraState",
  "wideRoadCameraState", "driverCameraState", "driverStateV2",
  "driverMonitoringState", "onroadEvents", "deviceState", "peripheralState",
  "managerState", "procLog", "errorLogMessage", "radarState", "liveTracks",
]

ROOT_DIR = os.path.dirname(Paths.log_root())
LOG_DIR = os.path.join(ROOT_DIR, "auto_drive")
STATUS_FILE = "/tmp/auto_drive_status.json"
LOCK_FILE = "/tmp/auto_drive_recorder.lock"

PRE_ROLL_EVENTS = 12000
SEGMENT_SECONDS = float(os.getenv("AUTO_LOG_SEGMENT_SEC", "60"))
MAX_SEGMENT_EVENTS = int(os.getenv("AUTO_LOG_MAX_EVENTS", "150000"))
STOP_DELAY_SEC = float(os.getenv("AUTO_LOG_STOP_DELAY_SEC", "10"))
MOVE_SPEED_MS = float(os.getenv("AUTO_LOG_MOVE_SPEED_MS", "0.3"))
MAX_FILES = int(os.getenv("AUTO_LOG_MAX_FILES", "40"))
MAX_BYTES = int(float(os.getenv("AUTO_LOG_MAX_GB", "4")) * 1024**3)


class DriveRecorder:
  def __init__(self) -> None:
    os.makedirs(LOG_DIR, exist_ok=True)
    self.preroll: deque = deque(maxlen=PRE_ROLL_EVENTS)
    self.events: list = []
    self.recording = False
    self.drive_id = ""
    self.segment = 0
    self.segment_start = 0.0
    self.last_moving = 0.0
    self.last_status = 0.0
    self.saved_files = 0
    self.saved_events = 0
    self.last_saved = ""
    self.state = "unknown"
    self.v_ego = 0.0
    self.cruise_enabled = False
    self.lat_active = False
    self.last_seen = {s: 0.0 for s in SERVICES}
    self.current_file = ""

  def _fresh(self, service: str, seconds: float) -> bool:
    return time.time() - self.last_seen[service] < seconds

  def _health(self) -> dict:
    return {
      "control": self._fresh("controlsState", 3.0),
      "model": self._fresh("modelV2", 3.0),
      "camera": self._fresh("roadCameraState", 3.0) or self._fresh("wideRoadCameraState", 3.0),
      "imu": self._fresh("livePose", 3.0),
      "panda": self._fresh("pandaStates", 4.0),
      "manager": self._fresh("managerState", 5.0),
    }

  def write_status(self, force: bool = False) -> None:
    now = time.time()
    if not force and now - self.last_status < 1.0:
      return
    self.last_status = now
    status = {
      "time": int(now),
      "state": str(self.state),
      "vEgo": round(self.v_ego, 3),
      "cruiseEnabled": bool(self.cruise_enabled),
      "latActive": bool(self.lat_active),
      "recording": bool(self.recording),
      "currentFile": self.current_file,
      "bufferedEvents": len(self.events),
      "prerollEvents": len(self.preroll),
      "savedFiles": self.saved_files,
      "savedEvents": self.saved_events,
      "lastSaved": self.last_saved,
      "driveId": self.drive_id,
      "segment": self.segment,
      "logDir": LOG_DIR,
      "health": self._health(),
    }
    tmp = STATUS_FILE + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
      json.dump(status, f, ensure_ascii=False, indent=2)
    os.replace(tmp, STATUS_FILE)

  def _new_segment(self) -> None:
    self.segment += 1
    stamp = time.strftime("%Y%m%d_%H%M%S")
    self.current_file = os.path.join(LOG_DIR, f"auto_drive_{stamp}_{self.drive_id}_s{self.segment:02d}.bz2")
    self.segment_start = time.time()

  def save_segment(self, reason: str = "") -> None:
    if not self.events:
      return
    if not self.current_file:
      self._new_segment()

    tmp = self.current_file[:-4] + ".tmp.bz2"
    save_log(tmp, self.events, compress=True)
    os.replace(tmp, self.current_file)

    self.saved_files += 1
    self.saved_events += len(self.events)
    self.last_saved = self.current_file
    print(f"[auto_drive] saved {len(self.events)} events -> {self.current_file} ({reason})", flush=True)
    self.events = []
    self.current_file = ""
    self.cleanup()
    self.write_status(force=True)

  def cleanup(self) -> None:
    files = sorted(glob.glob(os.path.join(LOG_DIR, "auto_drive_*.bz2")), key=os.path.getmtime)
    total = sum(os.path.getsize(f) for f in files)
    for path in files:
      if len(files) <= MAX_FILES and total <= MAX_BYTES:
        break
      try:
        size = os.path.getsize(path)
        os.remove(path)
        files.remove(path)
        total -= size
        print(f"[auto_drive] removed old log {path}", flush=True)
      except OSError:
        break

  def start_drive(self, now: float) -> None:
    self.recording = True
    self.drive_id = time.strftime("%Y%m%d_%H%M%S")
    self.segment = 0
    self.events = list(self.preroll)
    self._new_segment()
    print(f"[auto_drive] start at v={self.v_ego:.2f} state={self.state} latActive={self.lat_active}", flush=True)
    self.write_status(force=True)

  def stop_drive(self, reason: str) -> None:
    print(f"[auto_drive] stop: {reason}", flush=True)
    self.save_segment(reason)
    self.recording = False
    self.current_file = ""
    self.write_status(force=True)

  def process(self, evt) -> None:
    now = time.time()
    msg = evt.which()
    if msg in self.last_seen:
      self.last_seen[msg] = now

    if msg == "carState":
      self.v_ego = float(evt.carState.vEgo)
      self.cruise_enabled = bool(evt.carState.cruiseState.enabled)
    elif msg == "selfdriveState":
      self.state = str(evt.selfdriveState.state)
    elif msg == "carControl":
      self.lat_active = bool(evt.carControl.latActive)

    state_active = self.cruise_enabled or self.state.lower() not in ("disabled", "unknown")
    driving = (self.v_ego > MOVE_SPEED_MS) or (state_active and self.v_ego > 0.1)
    if driving:
      self.last_moving = now

    if self.recording:
      self.events.append(evt)
      split = (now - self.segment_start >= SEGMENT_SECONDS) or (len(self.events) >= MAX_SEGMENT_EVENTS)
      if split:
        self.save_segment("segment")
        self._new_segment()
      if not driving and now - self.last_moving >= STOP_DELAY_SEC:
        self.stop_drive("stopped")
    else:
      self.preroll.append(evt)
      if driving:
        self.start_drive(now)

    self.write_status()

  def handle_signal(self, signum, frame) -> None:
    print(f"[auto_drive] received signal {signum}, saving buffer", flush=True)
    if self.events:
      self.save_segment("signal")
    self.write_status(force=True)
    sys.exit(0)


def main() -> None:
  lock = open(LOCK_FILE, "w")
  try:
    fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
  except BlockingIOError:
    print("[auto_drive] already running", flush=True)
    sys.exit(0)

  recorder = DriveRecorder()
  signal.signal(signal.SIGTERM, recorder.handle_signal)
  signal.signal(signal.SIGINT, recorder.handle_signal)
  print(f"[auto_drive] started, log_dir={LOG_DIR}, segment={SEGMENT_SECONDS:.0f}s", flush=True)
  recorder.write_status(force=True)

  while True:
    try:
      for event in live_logreader(SERVICES):
        recorder.process(event)
    except Exception as e:
      print(f"[auto_drive] bus error: {e!r}, retrying", flush=True)
      time.sleep(2)
      recorder.write_status(force=True)


if __name__ == "__main__":
  main()
