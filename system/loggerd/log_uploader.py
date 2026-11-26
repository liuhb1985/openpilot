#!/usr/bin/env python3
"""Rotate, cap and upload openpilot route logs.

Started on PC by default. Two upload modes are supported:
  git  (default): commit finished logs into this repo and push them.
  http: PUT/POST each finished file to an arbitrary URL.

Only finished segments are touched (rlog.lock must be gone) and files must be
at least LOG_UPLOAD_MIN_AGE seconds old. Total log size is capped at
LOG_MAX_BYTES (default 1000 MiB); the oldest finished segments are removed
until the cap is met.

Environment variables:
  LOG_UPLOAD_MODE          git (default) or http.
  LOG_UPLOAD_GIT_REMOTE    git remote (default origin).
  LOG_UPLOAD_GIT_BRANCH    git branch (default current branch).
  LOG_MAX_BYTES            max total log size in bytes (default 1048576000).
  LOG_UPLOAD_INTERVAL      poll interval seconds (default 10).
  LOG_UPLOAD_MIN_AGE       file age before upload seconds (default 60).
  LOG_UPLOAD_INCLUDE       comma-separated filenames (default rlog.zst,qlog.zst).
  LOG_UPLOAD_GIT_MAX_FILE_BYTES  split files larger than this before git push
                           (default 20 MiB); parts are named <file>.partNN.
  LOG_UPLOAD_URL           http mode target URL/template.
  LOG_UPLOAD_TOKEN         http mode bearer token.
  LOG_UPLOAD_METHOD        http mode PUT (default) or POST.
"""
import glob
import json
import os
import shutil
import subprocess
import time
import urllib.error
import urllib.request

from openpilot.system.hardware.hw import Paths

MODE = os.getenv("LOG_UPLOAD_MODE", "git").strip().lower()
UPLOAD_URL = os.getenv("LOG_UPLOAD_URL", "").strip()
UPLOAD_TOKEN = os.getenv("LOG_UPLOAD_TOKEN", "").strip()
UPLOAD_METHOD = os.getenv("LOG_UPLOAD_METHOD", "PUT").upper()
UPLOAD_INTERVAL = max(1.0, float(os.getenv("LOG_UPLOAD_INTERVAL", "10")))
UPLOAD_MIN_AGE = max(0.0, float(os.getenv("LOG_UPLOAD_MIN_AGE", "60")))
UPLOAD_INCLUDE = [x.strip() for x in os.getenv("LOG_UPLOAD_INCLUDE", "rlog.zst,qlog.zst").split(",") if x.strip()]
MAX_LOG_BYTES = int(os.getenv("LOG_MAX_BYTES", str(1000 * 1024 * 1024)))
MAX_GIT_FILE_BYTES = int(os.getenv("LOG_UPLOAD_GIT_MAX_FILE_BYTES", str(20 * 1024 * 1024)))
GIT_REMOTE = os.getenv("LOG_UPLOAD_GIT_REMOTE", "origin").strip()
GIT_BRANCH = os.getenv("LOG_UPLOAD_GIT_BRANCH", "").strip()
GIT_ROOT = Paths.repo_root()
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


def git(args: list[str]) -> subprocess.CompletedProcess:
  return subprocess.run(["git", "-C", GIT_ROOT] + args, check=True, capture_output=True, text=True)


def git_current_branch() -> str:
  if GIT_BRANCH:
    return GIT_BRANCH
  try:
    return git(["rev-parse", "--abbrev-ref", "HEAD"]).stdout.strip()
  except subprocess.CalledProcessError:
    return "master"


def split_for_git(path: str) -> list[str]:
  size = os.path.getsize(path)
  if size <= MAX_GIT_FILE_BYTES:
    return [path]
  parts = []
  with open(path, "rb") as f:
    idx = 1
    while True:
      chunk = f.read(MAX_GIT_FILE_BYTES)
      if not chunk:
        break
      part = f"{path}.part{idx:02d}"
      with open(part, "wb") as pf:
        pf.write(chunk)
      parts.append(part)
      idx += 1
  print(f"[LOG_UPLOAD] split {path} ({size} bytes) into {len(parts)} parts for git", flush=True)
  return parts


def git_upload_files(paths: list[str]) -> bool:
  if not paths:
    return False
  upload_paths = []
  split_map = {}
  for path in paths:
    parts = split_for_git(path)
    split_map[path] = parts
    upload_paths.extend(parts)
  rel = [os.path.relpath(p, GIT_ROOT).replace(os.sep, "/") for p in upload_paths]
  try:
    git(["add", "--"] + rel)
    git(["commit", "--only", "-m", f"chore(logs): upload drive logs {int(time.time())}", "--"] + rel)
    git(["push", GIT_REMOTE, git_current_branch()])
    for original, parts in split_map.items():
      if len(parts) > 1:
        try:
          os.remove(original)
        except OSError:
          pass
    return True
  except subprocess.CalledProcessError as e:
    print(f"[LOG_UPLOAD] git failed: {(e.stderr or e.stdout or '').strip()}", flush=True)
    return False


def build_url(route: str, segment: str, filename: str) -> str:
  try:
    return UPLOAD_URL.format(route=route, segment=segment, filename=filename, name=filename)
  except (KeyError, IndexError, ValueError):
    return UPLOAD_URL


def http_upload_file(path: str, route: str, segment: str, filename: str) -> bool:
  url = build_url(route, segment, filename)
  headers = {"Content-Type": "application/octet-stream"}
  if UPLOAD_TOKEN:
    headers["Authorization"] = f"Bearer {UPLOAD_TOKEN}"
  with open(path, "rb") as f:
    req = urllib.request.Request(url, data=f.read(), headers=headers, method=UPLOAD_METHOD)
  with urllib.request.urlopen(req, timeout=300) as resp:
    return 200 <= resp.status < 300


def enforce_size_limit(log_root: str, state: dict) -> dict:
  total = 0
  for root, _, files in os.walk(log_root):
    for name in files:
      try:
        total += os.path.getsize(os.path.join(root, name))
      except OSError:
        pass
  if total <= MAX_LOG_BYTES:
    return state

  segments = []
  for d in glob.glob(os.path.join(log_root, "*--*")):
    if os.path.exists(os.path.join(d, "rlog.lock")):
      continue
    try:
      mtime = max(os.path.getmtime(os.path.join(d, f)) for f in os.listdir(d) if os.path.isfile(os.path.join(d, f)))
    except (OSError, ValueError):
      mtime = os.path.getmtime(d)
    segments.append((mtime, d))
  segments.sort()

  for _, segment_dir in segments:
    if total <= MAX_LOG_BYTES * 0.95:
      break
    size = 0
    for root, _, files in os.walk(segment_dir):
      for name in files:
        try:
          size += os.path.getsize(os.path.join(root, name))
        except OSError:
          pass
    try:
      shutil.rmtree(segment_dir)
      total -= size
      for key in [k for k in state if os.path.abspath(k).startswith(os.path.abspath(segment_dir) + os.sep)]:
        del state[key]
      print(f"[LOG_UPLOAD] removed old segment {segment_dir} ({size} bytes)", flush=True)
    except OSError as e:
      print(f"[LOG_UPLOAD] cannot remove {segment_dir}: {e}", flush=True)
  return state


def main() -> None:
  os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)
  log_root = Paths.log_root()
  state = load_state()

  while True:
    state = enforce_size_limit(log_root, state)
    save_state(state)

    if MODE == "http" and not UPLOAD_URL:
      time.sleep(UPLOAD_INTERVAL)
      continue
    if MODE not in ("git", "http"):
      print(f"[LOG_UPLOAD] unknown LOG_UPLOAD_MODE={MODE}", flush=True)
      time.sleep(UPLOAD_INTERVAL)
      continue

    now = time.time()
    pending = []
    for segment_dir in glob.glob(os.path.join(log_root, "*--*")):
      if os.path.exists(os.path.join(segment_dir, "rlog.lock")):
        continue
      for filename in UPLOAD_INCLUDE:
        path = os.path.join(segment_dir, filename)
        if not os.path.isfile(path):
          continue
        if now - os.path.getmtime(path) < UPLOAD_MIN_AGE:
          continue
        key = os.path.abspath(path)
        if key in state:
          continue
        if MODE == "http":
          route, segment = split_segment(segment_dir)
          try:
            if http_upload_file(path, route, segment, filename):
              state[key] = now
              save_state(state)
              print(f"[LOG_UPLOAD] uploaded {path} -> {build_url(route, segment, filename)}", flush=True)
            else:
              print(f"[LOG_UPLOAD] failed {path}", flush=True)
          except (OSError, urllib.error.URLError, ValueError) as e:
            print(f"[LOG_UPLOAD] retry {path}: {e}", flush=True)
        else:
          pending.append(path)

    if MODE == "git" and pending:
      if git_upload_files(pending):
        for path in pending:
          state[os.path.abspath(path)] = now
        save_state(state)
        print(f"[LOG_UPLOAD] pushed {len(pending)} log file(s) to git", flush=True)

    if len(state) > 10000:
      state = dict(list(state.items())[-5000:])
      save_state(state)

    time.sleep(UPLOAD_INTERVAL)


if __name__ == "__main__":
  main()
