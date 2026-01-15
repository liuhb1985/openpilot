#ifndef CAMERAD_HHH_
#define CAMERAD_HHH_

#include "common/util.h"
#include <list>

class camera;

class camerad {
protected:
  void camera_runner();

public:
  void run();

private:
  std::list<camera *> m_cameras;
  ExitHandler m_do_exit;          // 退出标志
};

#endif