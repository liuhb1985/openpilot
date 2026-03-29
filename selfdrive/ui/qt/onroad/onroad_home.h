#pragma once

#include "selfdrive/ui/qt/onroad/alerts.h"
#include "selfdrive/ui/qt/onroad/annotated_camera.h"

#include <QLabel>
#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>

class BlindSpotWidget : public QWidget {
  Q_OBJECT
public:
  BlindSpotWidget(QWidget* parent = 0);
  void updateImage();
private:
  void paintEvent(QPaintEvent* event) override;
  QImage bsm_img;
  QTimer* timer;
  uint64_t last_frame_id = 0;
};

class OnroadWindow : public QOpenGLWidget, protected QOpenGLFunctions {
  Q_OBJECT

public:
  OnroadWindow(QWidget* parent = 0);
  bool isMapVisible() const { return map && map->isVisible(); }
  void showMapPanel(bool show) { if (map) map->setVisible(show); }

signals:
  void mapPanelRequested();

private:
  void paintEvent(QPaintEvent *event);
  void mousePressEvent(QMouseEvent* e) override;
  OnroadAlerts *alerts;
  AnnotatedCameraWidget *nvg;
  BlindSpotWidget *bsm_widget;
  QColor bg = bg_colors[STATUS_DISENGAGED];
  QColor bg_long = bg_colors[STATUS_DISENGAGED];
  QWidget *map = nullptr;
  QHBoxLayout* split;

private slots:
  void offroadTransition(bool offroad);
  void updateState(const UIState &s);

protected:
    void initializeGL() override; // OpenGL 초기화
};
