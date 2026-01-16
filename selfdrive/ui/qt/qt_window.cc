#include "selfdrive/ui/qt/qt_window.h"

void setMainWindow(QWidget *w) {
  const float scale = util::getenv("SCALE", 1.0f);
  const bool fullscreen = util::getenv("FULLSCREEN", 0);
#if 0
  const QSize sz = QGuiApplication::primaryScreen()->size();
  if (Hardware::PC() && scale == 1.0 && !(sz - DEVICE_SCREEN_SIZE).isValid()) {
    w->setMinimumSize(QSize(640, 480)); // allow resize smaller than fullscreen
    w->setMaximumSize(DEVICE_SCREEN_SIZE);
    w->resize(sz);
  } else {
    w->setFixedSize(DEVICE_SCREEN_SIZE * scale);
  }
  w->show();
#endif
  if (Hardware::PC()) {
    const QSize DEFAULT_WINDOW_SIZE(1790,1000);
    w->setMinimumSize(QSize(640, 480)); // allow resize smaller than fullscreen
    w->setMaximumSize(DEVICE_SCREEN_SIZE);
    W->resize(DEFAULT_WINDOW_SIZE);//设置初始大小
    w->move(770,558);//窗口向右一定560像素
    w->resize(sz);
  } else {
    w->setFixedSize(DEVICE_SCREEN_SIZE * scale);
  }
  w->show();

#ifdef QCOM2
  QPlatformNativeInterface *native = QGuiApplication::platformNativeInterface();
  wl_surface *s = reinterpret_cast<wl_surface*>(native->nativeResourceForWindow("surface", w->windowHandle()));
  wl_surface_set_buffer_transform(s, WL_OUTPUT_TRANSFORM_270);
  wl_surface_commit(s);

  w->setWindowState(Qt::WindowFullScreen);
  w->setVisible(true);

  // ensure we have a valid eglDisplay, otherwise the ui will silently fail
  void *egl = native->nativeResourceForWindow("egldisplay", w->windowHandle());
  assert(egl != nullptr);
#endif
}


extern "C" {
  void set_main_window(void *w) {
    setMainWindow((QWidget*)w);
  }
}
