#pragma once

#include <string>
#include <unistd.h>

#include "system/hardware/base.h"
#include "common/util.h"

#if QCOM2
#include "system/hardware/tici/hardware.h"
#define Hardware HardwareTici
#else
#include "system/hardware/pc/hardware.h"
#define Hardware HardwarePC
#endif

namespace Path {
  inline std::string openpilot_prefix() {
    return util::getenv("OPENPILOT_PREFIX", "");
  }

  inline std::string comma_home() {
    return util::getenv("HOME") + "/.comma" + Path::openpilot_prefix();
  }

  inline std::string repo_root() {
    if (const char *env = getenv("OPENPILOT_ROOT")) {
      return env;
    }
    char cwd[4096];
    if (getcwd(cwd, sizeof(cwd)) != nullptr) {
      std::string p = cwd;
      for (int i = 0; i < 8; ++i) {
        if (util::file_exists(p + "/.git") || util::file_exists(p + "/SConstruct")) {
          return p;
        }
        size_t pos = p.rfind('/');
        if (pos == std::string::npos || pos == 0) break;
        p = p.substr(0, pos);
      }
    }
    return comma_home();
  }

  inline std::string log_root() {
    if (const char *env = getenv("LOG_ROOT")) {
      return env;
    }
    if (Hardware::PC()) {
      std::string root = Path::repo_root() + "/logs/realdata";
      util::create_directories(root, 0775);
      return root;
    }
    return "/data/media/0/realdata";
  }

  inline std::string params() {
    return util::getenv("PARAMS_ROOT", Hardware::PC() ? (Path::comma_home() + "/params") : "/data/params");
  }

  inline std::string rsa_file() {
    return Hardware::PC() ? Path::comma_home() + "/persist/comma/id_rsa" : "/persist/comma/id_rsa";
  }

  inline std::string swaglog_ipc() {
    return "ipc:///tmp/logmessage" + Path::openpilot_prefix();
  }

  inline std::string download_cache_root() {
    if (const char *env = getenv("COMMA_CACHE")) {
      return env;
    }
    return "/tmp/comma_download_cache" + Path::openpilot_prefix() + "/";
  }

 inline std::string shm_path() {
    #ifdef __APPLE__
     return"/tmp";
    #else
     return "/dev/shm";
    #endif
 }
}  // namespace Path
