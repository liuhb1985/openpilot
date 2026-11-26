#!/usr/bin/env python3
"""Upload finished openpilot route logs to a configurable remote HTTP(S) endpoint.

The process is always started on PC. Without LOG_UPLOAD_URL it stays idle, so it
is safe to enable by default.

Environment variables:
  LOG_UPLOAD_URL       Required. Target URL or template, e.g.
                       https://example.com/upload/{route}/{segment}/{filename}
                       Placeholders: route, segment, filename, name.
  LOG_UPLOAD_TOKEN     Optional bearer token sent as Authorization header.
  LOG_UPLOAD_METHOD    PUT (default) or POST.
  LOG_UPLOAD_INTERVAL  Poll interval in seconds (default 10).
  LOG_UPLOAD_MIN_AGE   Minimum file age in seconds before upload (default 60).
  LOG_UPLOAD_INCLUDE   Comma-separated filenames (default rlog.zst,qlog.zst).
"""
import glob
import json
import os
import time
import urllib.error
import urllib.request

from openpilot.system.hardware.hw import Paths

UPLOAD_URL = os.getenv("LOG_UPLOAD_URL", "").strip()
UPLOAD_TOKEN = os.getenv("LOG_UPLOAD_TOKEN", "").strip()
UPLOAD_METHOD = os.getenv("LOG_UPLOAD_METHOD", "PUT").upper()
UPLOAD_INTERVAL = max(1.0, float(os.getenv("LOG_UPLOAD_INTERVAL", "10")))
UPLOAD_MIN_AGE = max(0.0, float(os.getenv("LOG_UPLOAD_MIN_AGE", "60")))
UPLOAD_INCLUDE = [x.strip() for x in os.getenv("LOG_UPLOAD_INCLUDE", "rlog.zst,qlog.zst").split(",") if x.strip()]
STATE_FILE = os.path.join(Paths.swaglog_root(), "log_upload_state.json")


def load_state() -> dict:
  try:
    with open(STATE_FILE, "r", encoding="utf-8") as f:
      return json.load(f)
  except (OSError, ValueError):
    return {}


def save_state(state: dict) -> None:
  tmp = STATE_FILE + ".tmp"
  with open(tmp, "w", encoding="utf-8") as f:
    json.dump(state, f)
  os.replace(tmp, STATE_FILE)


def split_segment(segment_dir: str) -> tuple[str, str]:
  name = os.path.basename(segment_dir.rstrip(os.sep))
  route, sep, segment = name.rpartition("--")
  return (route, segment) if sep else (name, "0")


def build_url(route: str, segment: str, filename: str) -> str:
  try:
    return UPLOAD_URL.format(route=route, segment=segment, filename=filename, name=filename)
  except (KeyError, IndexError, ValueError):
    return UPLOAD_URL


def upload_file(path: str, route: str, segment: str, filename: str) -> bool:
  url = build_url(route, segment, filename)
  headers = {"Content-Type": "application/octet-stream"}
  if UPLOAD_TOKEN:
    headers["Authorization"] = f"Bearer {UPLOAD_TOKEN}"

  with open(path, "rb") as f:
    req = urllib.request.Request(url, data=f.read(), headers=headers, method=UPLOAD_METHOD)
  with urllib.request.urlopen(req, timeout=300) as resp:
    return 200 <= resp.status < 300


def main() -> None:
  os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)
  state = load_state()

  while True:
    if not UPLOAD_URL:
      time.sleep(UPLOAD_INTERVAL)
      continue

    now = time.time()
    log_root = Paths.log_root()
    for segment_dir in glob.glob(os.path.join(log_root, "*--*")):
      # rlog.lock exists while the segment is still being written
      if os.path.exists(os.path.join(segment_dir, "rlog.lock")):
        continue
      route, segment = split_segment(segment_dir)
      for filename in UPLOAD_INCLUDE:
        path = os.path.join(segment_dir, filename)
        if not os.path.isfile(path):
          continue
        if now - os.path.getmtime(path) < UPLOAD_MIN_AGE:
          continue
        key = os.path.abspath(path)
        if key in state:
          continue
        try:
          if upload_file(path, route, segment, filename):
            state[key] = now
            save_state(state)
            print(f"[LOG_UPLOAD] uploaded {path} -> {build_url(route, segment, filename)}", flush=True)
          else:
            print(f"[LOG_UPLOAD] failed {path}", flush=True)
        except (OSError, urllib.error.URLError, ValueError) as e:
          print(f"[LOG_UPLOAD] retry {path}: {e}", flush=True)

    if len(state) > 10000:
      state = dict(list(state.items())[-5000:])
      save_state(state)

    time.sleep(UPLOAD_INTERVAL)


if __name__ == "__main__":
  main()
