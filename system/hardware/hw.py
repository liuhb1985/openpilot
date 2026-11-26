import os
import platform
from pathlib import Path

from openpilot.system.hardware import PC

DEFAULT_DOWNLOAD_CACHE_ROOT = "/tmp/comma_download_cache"

class Paths:
  @staticmethod
  def comma_home() -> str:
    return os.path.join(str(Path.home()), ".comma" + os.environ.get("OPENPILOT_PREFIX", ""))

  @staticmethod
  def repo_root() -> str:
    if os.environ.get("OPENPILOT_ROOT"):
      return os.environ["OPENPILOT_ROOT"]
    # 从当前文件向上查找项目根（含 .git 或 SConstruct）
    cur = Path(__file__).resolve()
    for parent in [cur.parent] + list(cur.parents):
      if (parent / ".git").exists() or (parent / "SConstruct").exists():
        return str(parent)
    return str(Path.cwd())

  @staticmethod
  def log_root() -> str:
    if os.environ.get('LOG_ROOT', False):
      root = os.environ['LOG_ROOT']
    elif PC:
      root = str(Path(Paths.repo_root()) / "logs" / "realdata")
    else:
      root = '/data/media/0/realdata/'
    if PC:
      os.makedirs(root, exist_ok=True)
    return root

  @staticmethod
  def swaglog_root() -> str:
    if PC:
      return os.path.join(Paths.comma_home(), "log")
    else:
      return "/data/log/"

  @staticmethod
  def swaglog_ipc() -> str:
    return "ipc:///tmp/logmessage" + os.environ.get("OPENPILOT_PREFIX", "")

  @staticmethod
  def download_cache_root() -> str:
    if os.environ.get('COMMA_CACHE', False):
      return os.environ['COMMA_CACHE'] + "/"
    return DEFAULT_DOWNLOAD_CACHE_ROOT + os.environ.get("OPENPILOT_PREFIX", "") + "/"

  @staticmethod
  def persist_root() -> str:
    if PC:
      return os.path.join(Paths.comma_home(), "persist")
    else:
      return "/persist/"

  @staticmethod
  def stats_root() -> str:
    if PC:
      return str(Path(Paths.comma_home()) / "stats")
    else:
      return "/data/stats/"

  @staticmethod
  def config_root() -> str:
    if PC:
      return Paths.comma_home()
    else:
      return "/tmp/.comma"

  @staticmethod
  def shm_path() -> str:
    if PC and platform.system() == "Darwin":
      return "/tmp"  # This is not really shared memory on macOS, but it's the closest we can get
    return "/dev/shm"
