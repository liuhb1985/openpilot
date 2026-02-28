#include <cassert>
#include <cmath>
#include <string>
#include <tuple>
#include <vector>
#include <thread> //차선캘리

#include <QDebug>
#include <QProcess>

#include "common/watchdog.h"
#include "common/util.h"
#include "selfdrive/ui/qt/network/networking.h"
#include "selfdrive/ui/qt/offroad/settings.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/widgets/prime.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"
#include "selfdrive/ui/qt/offroad/developer_panel.h"
#include "selfdrive/ui/qt/offroad/firehose.h"

TogglesPanel::TogglesPanel(SettingsWindow *parent) : ListWidget(parent) {
  // param, title, desc, icon
  std::vector<std::tuple<QString, QString, QString, QString>> toggle_defs{
    {
      "OpenpilotEnabledToggle",
      tr("Enable openpilot"),
      tr("Use the openpilot system for adaptive cruise control and lane keep driver assistance. Your attention is required at all times to use this feature. Changing this setting takes effect when the car is powered off."),
      "../assets/img_chffr_wheel.png",
    },
    {
      "ExperimentalMode",
      tr("Experimental Mode"),
      "",
      "../assets/img_experimental_white.svg",
    },
    {
      "DisengageOnAccelerator",
      tr("Disengage on Accelerator Pedal"),
      tr("When enabled, pressing the accelerator pedal will disengage openpilot."),
      "../assets/offroad/icon_disengage_on_accelerator.svg",
    },
    {
      "IsLdwEnabled",
      tr("Enable Lane Departure Warnings"),
      tr("Receive alerts to steer back into the lane when your vehicle drifts over a detected lane line without a turn signal activated while driving over 31 mph (50 km/h)."),
      "../assets/offroad/icon_warning.png",
    },
    {
      "AlwaysOnDM",
      tr("Always-On Driver Monitoring"),
      tr("Enable driver monitoring even when openpilot is not engaged."),
      "../assets/offroad/icon_monitoring.png",
    },
    {
      "RecordFront",
      tr("Record and Upload Driver Camera"),
      tr("Upload data from the driver facing camera and help improve the driver monitoring algorithm."),
      "../assets/offroad/icon_monitoring.png",
    },
    {
      "RecordAudio",
      tr("Record and Upload Microphone Audio"),
      tr("Record and store microphone audio while driving. The audio will be included in the dashcam video in comma connect."),
      "../assets/offroad/microphone.png",
    },
    {
      "IsMetric",
      tr("Use Metric System"),
      tr("Display speed in km/h instead of mph."),
      "../assets/offroad/icon_metric.png",
    },
  };


  std::vector<QString> longi_button_texts{tr("Aggressive"), tr("Standard"), tr("Relaxed") , tr("MoreRelaxed") };
  long_personality_setting = new ButtonParamControl("LongitudinalPersonality", tr("Driving Personality"),
                                          tr("Standard is recommended. In aggressive mode, openpilot will follow lead cars closer and be more aggressive with the gas and brake. "
                                             "In relaxed mode openpilot will stay further away from lead cars. On supported cars, you can cycle through these personalities with "
                                             "your steering wheel distance button."),
                                          "../assets/offroad/icon_speed_limit.png",
                                          longi_button_texts);

  // set up uiState update for personality setting
  QObject::connect(uiState(), &UIState::uiUpdate, this, &TogglesPanel::updateState);

  for (auto &[param, title, desc, icon] : toggle_defs) {
    auto toggle = new ParamControl(param, title, desc, icon, this);

    bool locked = params.getBool((param + "Lock").toStdString());
    toggle->setEnabled(!locked);

    addItem(toggle);
    toggles[param.toStdString()] = toggle;

    // insert longitudinal personality after NDOG toggle
    if (param == "DisengageOnAccelerator") {
      addItem(long_personality_setting);
    }
  }

  // Toggles with confirmation dialogs
  toggles["ExperimentalMode"]->setActiveIcon("../assets/img_experimental.svg");
  toggles["ExperimentalMode"]->setConfirmation(true, true);
}

void TogglesPanel::updateState(const UIState &s) {
  const SubMaster &sm = *(s.sm);

  if (sm.updated("selfdriveState")) {
    auto personality = sm["selfdriveState"].getSelfdriveState().getPersonality();
    if (personality != s.scene.personality && s.scene.started && isVisible()) {
      long_personality_setting->setCheckedButton(static_cast<int>(personality));
    }
    uiState()->scene.personality = personality;
  }
}

void TogglesPanel::expandToggleDescription(const QString &param) {
  toggles[param.toStdString()]->showDescription();
}

void TogglesPanel::showEvent(QShowEvent *event) {
  updateToggles();
}

void TogglesPanel::updateToggles() {
  auto experimental_mode_toggle = toggles["ExperimentalMode"];
  const QString e2e_description = QString("%1<br>"
                                          "<h4>%2</h4><br>"
                                          "%3<br>"
                                          "<h4>%4</h4><br>"
                                          "%5<br>")
                                  .arg(tr("openpilot defaults to driving in <b>chill mode</b>. Experimental mode enables <b>alpha-level features</b> that aren't ready for chill mode. Experimental features are listed below:"))
                                  .arg(tr("End-to-End Longitudinal Control"))
                                  .arg(tr("Let the driving model control the gas and brakes. openpilot will drive as it thinks a human would, including stopping for red lights and stop signs. "
                                          "Since the driving model decides the speed to drive, the set speed will only act as an upper bound. This is an alpha quality feature; "
                                          "mistakes should be expected."))
                                  .arg(tr("New Driving Visualization"))
                                  .arg(tr("The driving visualization will transition to the road-facing wide-angle camera at low speeds to better show some turns. The Experimental mode logo will also be shown in the top right corner."));

  const bool is_release = params.getBool("IsReleaseBranch");
  auto cp_bytes = params.get("CarParamsPersistent");
  if (!cp_bytes.empty()) {
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(cp_bytes.data(), cp_bytes.size()));
    cereal::CarParams::Reader CP = cmsg.getRoot<cereal::CarParams>();

    if (hasLongitudinalControl(CP)) {
      // normal description and toggle
      experimental_mode_toggle->setEnabled(true);
      experimental_mode_toggle->setDescription(e2e_description);
      long_personality_setting->setEnabled(true);
    } else {
      // no long for now
      experimental_mode_toggle->setEnabled(false);
      long_personality_setting->setEnabled(false);
      params.remove("ExperimentalMode");

      const QString unavailable = tr("Experimental mode is currently unavailable on this car since the car's stock ACC is used for longitudinal control.");

      QString long_desc = unavailable + " " + \
                          tr("openpilot longitudinal control may come in a future update.");
      if (CP.getAlphaLongitudinalAvailable()) {
        if (is_release) {
          long_desc = unavailable + " " + tr("An alpha version of openpilot longitudinal control can be tested, along with Experimental mode, on non-release branches.");
        } else {
          long_desc = tr("Enable the openpilot longitudinal control (alpha) toggle to allow Experimental mode.");
        }
      }
      experimental_mode_toggle->setDescription("<b>" + long_desc + "</b><br><br>" + e2e_description);
    }

    experimental_mode_toggle->refresh();
  } else {
    experimental_mode_toggle->setDescription(e2e_description);
  }
}

DevicePanel::DevicePanel(SettingsWindow *parent) : ListWidget(parent) {
  setSpacing(50);
  addItem(new LabelControl(tr("Dongle ID"), getDongleId().value_or(tr("N/A"))));
  addItem(new LabelControl(tr("Serial"), params.get("HardwareSerial").c_str()));

  // power buttons
  QHBoxLayout* power_layout = new QHBoxLayout();
  power_layout->setSpacing(30);

  QPushButton* reboot_btn = new QPushButton(tr("Reboot"));
  reboot_btn->setObjectName("reboot_btn");
  power_layout->addWidget(reboot_btn);
  QObject::connect(reboot_btn, &QPushButton::clicked, this, &DevicePanel::reboot);
  //차선캘리
  QPushButton *reset_CalibBtn = new QPushButton(tr("ReCalibration"));
  reset_CalibBtn->setObjectName("reset_CalibBtn");
  power_layout->addWidget(reset_CalibBtn);
  QObject::connect(reset_CalibBtn, &QPushButton::clicked, this, &DevicePanel::calibration);

  QPushButton* poweroff_btn = new QPushButton(tr("Power Off"));
  poweroff_btn->setObjectName("poweroff_btn");
  power_layout->addWidget(poweroff_btn);
  QObject::connect(poweroff_btn, &QPushButton::clicked, this, &DevicePanel::poweroff);

  if (false && !Hardware::PC()) {
      connect(uiState(), &UIState::offroadTransition, poweroff_btn, &QPushButton::setVisible);
  }

  addItem(power_layout);

  QHBoxLayout* init_layout = new QHBoxLayout();
  init_layout->setSpacing(30);

  QPushButton* init_btn = new QPushButton(tr("Git Pull & Reboot"));
  init_btn->setObjectName("init_btn");
  init_layout->addWidget(init_btn);
  //QObject::connect(init_btn, &QPushButton::clicked, this, &DevicePanel::reboot);
  QObject::connect(init_btn, &QPushButton::clicked, [&]() {
    if (ConfirmationDialog::confirm(tr("Git pull & Reboot?"), tr("Yes"), this)) {
      QString pullscript = "cd /data/openpilot && "
        "git fetch origin && "
        "LOCAL=$(git rev-parse HEAD) && "
        "BRANCH=$(git branch --show-current) && "
        "REMOTE=$(git rev-parse origin/$BRANCH) && "
        "if [ $LOCAL != $REMOTE ]; then "
        "echo 'Local is behind. Pulling updates...' && "
        "git pull --ff-only && "
        "sudo reboot; "
        "else "
        "echo 'Already up to date.'; "
        "fi'";

      bool success = QProcess::startDetached("/bin/sh", QStringList() << "-c" << pullscript);

      if (!success) {
        ConfirmationDialog::alert(tr("Failed to start update process."), this);
      } else {
        ConfirmationDialog::alert(tr("Update process started. Device will reboot if updates are applied."), this);
      }
    }
    });

  QPushButton* default_btn = new QPushButton(tr("Set default"));
  default_btn->setObjectName("default_btn");
  init_layout->addWidget(default_btn);
  //QObject::connect(default_btn, &QPushButton::clicked, this, &DevicePanel::poweroff);
  QObject::connect(default_btn, &QPushButton::clicked, [&]() {
    if (ConfirmationDialog::confirm(tr("Set to default?"), tr("Yes"), this)) {
      //emit parent->closeSettings();
      QTimer::singleShot(1000, []() {
        printf("Set to default\n");
        Params().putInt("SoftRestartTriggered", 2);
        printf("Set to default2\n");
        });
    }
    });

  QPushButton* remove_mapbox_key_btn = new QPushButton(tr("Remove MapboxKey"));
  remove_mapbox_key_btn->setObjectName("remove_mapbox_key_btn");
  init_layout->addWidget(remove_mapbox_key_btn);
  QObject::connect(remove_mapbox_key_btn, &QPushButton::clicked, [&]() {
    if (ConfirmationDialog::confirm(tr("Remove Mapbox key?"), tr("Yes"), this)) {
      QTimer::singleShot(1000, []() {
        Params().put("MapboxPublicKey", "");
        Params().put("MapboxSecretKey", "");
        });
    }
    });

  setStyleSheet(R"(
    #reboot_btn { height: 120px; border-radius: 15px; background-color: #2CE22C; }
    #reboot_btn:pressed { background-color: #24FF24; }
    #reset_CalibBtn { height: 120px; border-radius: 15px; background-color: #FFBB00; }
    #reset_CalibBtn:pressed { background-color: #FF2424; }
    #poweroff_btn { height: 120px; border-radius: 15px; background-color: #E22C2C; }
    #poweroff_btn:pressed { background-color: #FF2424; }
    #init_btn { height: 120px; border-radius: 15px; background-color: #2C2CE2; }
    #init_btn:pressed { background-color: #2424FF; }
    #default_btn { height: 120px; border-radius: 15px; background-color: #BDBDBD; }
    #default_btn:pressed { background-color: #A9A9A9; }
    #remove_mapbox_key_btn { height: 120px; border-radius: 15px; background-color: #BDBDBD; }
    #remove_mapbox_key_btn:pressed { background-color: #A9A9A9; }
  )");
  addItem(init_layout);

  pair_device = new ButtonControl(tr("Pair Device"), tr("PAIR"),
                                  tr("Pair your device with comma connect (connect.comma.ai) and claim your comma prime offer."));
  connect(pair_device, &ButtonControl::clicked, [=]() {
    PairingPopup popup(this);
    popup.exec();
  });
  addItem(pair_device);

  // offroad-only buttons

  // 断开连接自动关机
  addItem(new CValueControl("EnableDisconnectShutdown", tr("Auto Shutdown on Disconnect"), tr("Auto poweroff when Panda disconnects"), 0, 1, 1));
  addItem(new CValueControl("DisconnectShutdownDelay", tr("Disconnect Delay (sec)"), "", 1, 60, 1));

  auto dcamBtn = new ButtonControl(tr("Driver Camera"), tr("PREVIEW"),
                                   tr("Preview the driver facing camera to ensure that driver monitoring has good visibility. (vehicle must be off)"));
  connect(dcamBtn, &ButtonControl::clicked, [=]() { emit showDriverView(); });
  addItem(dcamBtn);

  auto retrainingBtn = new ButtonControl(tr("Review Training Guide"), tr("REVIEW"), tr("Review the rules, features, and limitations of openpilot"));
  connect(retrainingBtn, &ButtonControl::clicked, [=]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to review the training guide?"), tr("Review"), this)) {
      emit reviewTrainingGuide();
    }
  });
  addItem(retrainingBtn);

  auto statusCalibBtn = new ButtonControl(tr("Calibration Status"), tr("SHOW"), "");
  connect(statusCalibBtn, &ButtonControl::showDescriptionEvent, this, &DevicePanel::updateCalibDescription);
  addItem(statusCalibBtn);

  std::string calib_bytes = params.get("CalibrationParams");
  if (!calib_bytes.empty()) {
    try {
      AlignedBuffer aligned_buf;
      capnp::FlatArrayMessageReader cmsg(aligned_buf.align(calib_bytes.data(), calib_bytes.size()));
      auto calib = cmsg.getRoot<cereal::Event>().getLiveCalibration();
      if (calib.getCalStatus() != cereal::LiveCalibrationData::Status::UNCALIBRATED) {
        double pitch = calib.getRpyCalib()[1] * (180 / M_PI);
        double yaw = calib.getRpyCalib()[2] * (180 / M_PI);
        QString position = QString("%2 %1° %4 %3°")
                           .arg(QString::number(std::abs(pitch), 'g', 1), pitch > 0 ? "↓" : "↑",
                                QString::number(std::abs(yaw), 'g', 1), yaw > 0 ? "←" : "→");
        params.put("DevicePosition", position.toStdString());
      }
    } catch (kj::Exception) {
      qInfo() << "invalid CalibrationParams";
    }
  }

  if (Hardware::TICI()) {
    auto regulatoryBtn = new ButtonControl(tr("Regulatory"), tr("VIEW"), "");
    connect(regulatoryBtn, &ButtonControl::clicked, [=]() {
      const std::string txt = util::read_file("../assets/offroad/fcc.html");
      ConfirmationDialog::rich(QString::fromStdString(txt), this);
    });
    addItem(regulatoryBtn);
  }

  auto translateBtn = new ButtonControl(tr("Change Language"), tr("CHANGE"), "");
  connect(translateBtn, &ButtonControl::clicked, [=]() {
    QMap<QString, QString> langs = getSupportedLanguages();
    QString selection = MultiOptionDialog::getSelection(tr("Select a language"), langs.keys(), langs.key(uiState()->language), this);
    if (!selection.isEmpty()) {
      // put language setting, exit Qt UI, and trigger fast restart
      params.put("LanguageSetting", langs[selection].toStdString());
      qApp->exit(18);
      watchdog_kick(0);
    }
  });
  addItem(translateBtn);

  QObject::connect(uiState()->prime_state, &PrimeState::changed, [this] (PrimeState::Type type) {
    pair_device->setVisible(type == PrimeState::PRIME_TYPE_UNPAIRED);
  });
  QObject::connect(uiState(), &UIState::offroadTransition, [=](bool offroad) {
    for (auto btn : findChildren<ButtonControl *>()) {
      if (btn != pair_device) {
        btn->setEnabled(offroad);
      }
    }
    translateBtn->setEnabled(true);
    statusCalibBtn->setEnabled(true);
  });

}

void DevicePanel::updateCalibDescription() {
  QString desc =
      tr("openpilot requires the device to be mounted within 4° left or right and "
         "within 5° up or 9° down. openpilot is continuously calibrating, resetting is rarely required.");
  std::string calib_bytes = params.get("CalibrationParams");
  if (!calib_bytes.empty()) {
    try {
      AlignedBuffer aligned_buf;
      capnp::FlatArrayMessageReader cmsg(aligned_buf.align(calib_bytes.data(), calib_bytes.size()));
      auto calib = cmsg.getRoot<cereal::Event>().getLiveCalibration();
      if (calib.getCalStatus() != cereal::LiveCalibrationData::Status::UNCALIBRATED) {
        double pitch = calib.getRpyCalib()[1] * (180 / M_PI);
        double yaw = calib.getRpyCalib()[2] * (180 / M_PI);
        desc += tr(" Your device is pointed %1° %2 and %3° %4.")
                    .arg(QString::number(std::abs(pitch), 'g', 1), pitch > 0 ? tr("down") : tr("up"),
                         QString::number(std::abs(yaw), 'g', 1), yaw > 0 ? tr("left") : tr("right"));
      }
    } catch (kj::Exception) {
      qInfo() << "invalid CalibrationParams";
    }
  }
  qobject_cast<ButtonControl *>(sender())->setDescription(desc);
}

void DevicePanel::reboot() {
  if (!uiState()->engaged()) {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reboot?"), tr("Reboot"), this)) {
      // Check engaged again in case it changed while the dialog was open
      if (!uiState()->engaged()) {
        params.putBool("DoReboot", true);
      }
    }
  } else {
    ConfirmationDialog::alert(tr("Disengage to Reboot"), this);
  }
}

//차선캘리
void execAndReboot(const std::string& cmd) {
    system(cmd.c_str());
    Params().putBool("DoReboot", true);
}

void DevicePanel::calibration() {
  if (!uiState()->engaged()) {
    QStringList calibOptions;
    calibOptions << tr("AllCalibParams")
                 << tr("CalibrationParams")
                 << tr("LiveDelay")
                 << tr("LiveTorqueParameters")
                 << tr("LiveParameters")
                 << tr("LiveParametersV2");
    QString selectedParam = MultiOptionDialog::getSelection(
      tr("Select calibration parameter to reset"),
      calibOptions,
      "",
      this
    );

if (selectedParam.isEmpty()) return;
    QString confirmMsg = tr("Are you sure you want to reset %1?").arg(selectedParam);
    if (!ConfirmationDialog::confirm(confirmMsg, tr("ReCalibration"), this)) return;
    if (uiState()->engaged()) {
      ConfirmationDialog::alert(tr("Reboot & Disengage to Calibration"), this);
      return;
    }
    std::thread worker([selectedParam]() {
      if (selectedParam == "AllCalibParams") {
        std::string cmd = "rm -f " + Params().getParamPath("CalibrationParams") + " " +
                          Params().getParamPath("LiveParameters") + " " +
                          Params().getParamPath("LiveParametersV2") + " " +
                          Params().getParamPath("LiveTorqueParameters") + " " +
                          Params().getParamPath("LiveDelay");
        execAndReboot(cmd);
      } else {
        // 修复校准参数删除路径
        std::string cmd = "rm -f " + Params().getParamPath(selectedParam.toStdString());
        execAndReboot(cmd);
      }
    });
    worker.detach();
  } else {
    ConfirmationDialog::alert(tr("Reboot & Disengage to Calibration"), this);
  }
}

void DevicePanel::poweroff() {
  if (!uiState()->engaged()) {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to power off?"), tr("Power Off"), this)) {
      // Check engaged again in case it changed while the dialog was open
      if (!uiState()->engaged()) {
        params.putBool("DoShutdown", true);
      }
    }
  } else {
    ConfirmationDialog::alert(tr("Disengage to Power Off"), this);
  }
}

void SettingsWindow::showEvent(QShowEvent *event) {
  setCurrentPanel(0);
}

void SettingsWindow::setCurrentPanel(int index, const QString &param) {
  if (!param.isEmpty()) {
    // Check if param ends with "Panel" to determine if it's a panel name
    if (param.endsWith("Panel")) {
      QString panelName = param;
      panelName.chop(5); // Remove "Panel" suffix

      // Find the panel by name
      for (int i = 0; i < nav_btns->buttons().size(); i++) {
        if (nav_btns->buttons()[i]->text() == tr(panelName.toStdString().c_str())) {
          index = i;
          break;
        }
      }
    } else {
      emit expandToggleDescription(param);
    }
  }

  panel_widget->setCurrentIndex(index);
  nav_btns->buttons()[index]->setChecked(true);
}

SettingsWindow::SettingsWindow(QWidget *parent) : QFrame(parent) {

  // setup two main layouts
  sidebar_widget = new QWidget;
  QVBoxLayout *sidebar_layout = new QVBoxLayout(sidebar_widget);
  panel_widget = new QStackedWidget();

  // close button
  QPushButton *close_btn = new QPushButton(tr("×"));
  close_btn->setStyleSheet(R"(
    QPushButton {
      font-size: 140px;
      padding-bottom: 20px;
      border-radius: 100px;
      background-color: #292929;
      font-weight: 400;
    }
    QPushButton:pressed {
      background-color: #3B3B3B;
    }
  )");
  close_btn->setFixedSize(200, 200);
  sidebar_layout->addSpacing(45);
  sidebar_layout->addWidget(close_btn, 0, Qt::AlignCenter);
  QObject::connect(close_btn, &QPushButton::clicked, this, &SettingsWindow::closeSettings);

  // setup panels
  DevicePanel *device = new DevicePanel(this);
  QObject::connect(device, &DevicePanel::reviewTrainingGuide, this, &SettingsWindow::reviewTrainingGuide);
  QObject::connect(device, &DevicePanel::showDriverView, this, &SettingsWindow::showDriverView);

  TogglesPanel *toggles = new TogglesPanel(this);
  QObject::connect(this, &SettingsWindow::expandToggleDescription, toggles, &TogglesPanel::expandToggleDescription);

  auto networking = new Networking(this);
  QObject::connect(uiState()->prime_state, &PrimeState::changed, networking, &Networking::setPrimeType);

  QList<QPair<QString, QWidget *>> panels = {
    {tr("Device"), device},
    {tr("Network"), networking},
    {tr("Toggles"), toggles},
  };
  if(Params().getBool("SoftwareMenu")) {
    panels.append({tr("Software"), new SoftwarePanel(this)});
  }
  if(false) {
    panels.append({tr("Firehose"), new FirehosePanel(this)});
  }
  panels.append({ tr("胡萝卜"), new CarrotPanel(this) });
  panels.append({ tr("Developer"), new DeveloperPanel(this) });

  nav_btns = new QButtonGroup(this);
  for (auto &[name, panel] : panels) {
    QPushButton *btn = new QPushButton(name);
    btn->setCheckable(true);
    btn->setChecked(nav_btns->buttons().size() == 0);
    btn->setStyleSheet(R"(
      QPushButton {
        color: grey;
        border: none;
        background: none;
        font-size: 65px;
        font-weight: 500;
      }
      QPushButton:checked {
        color: white;
      }
      QPushButton:pressed {
        color: #ADADAD;
      }
    )");
    btn->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    nav_btns->addButton(btn);
    sidebar_layout->addWidget(btn, 0, Qt::AlignRight);

    const int lr_margin = name != tr("Network") ? 50 : 0;  // Network panel handles its own margins
    panel->setContentsMargins(lr_margin, 25, lr_margin, 25);

    ScrollView *panel_frame = new ScrollView(panel, this);
    panel_widget->addWidget(panel_frame);

    QObject::connect(btn, &QPushButton::clicked, [=, w = panel_frame]() {
      btn->setChecked(true);
      panel_widget->setCurrentWidget(w);
    });
  }
  sidebar_layout->setContentsMargins(50, 50, 100, 50);

  // main settings layout, sidebar + main panel
  QHBoxLayout *main_layout = new QHBoxLayout(this);

  sidebar_widget->setFixedWidth(500);
  main_layout->addWidget(sidebar_widget);
  main_layout->addWidget(panel_widget);

  setStyleSheet(R"(
    * {
      color: white;
      font-size: 50px;
    }
    SettingsWindow {
      background-color: black;
    }
    QStackedWidget, ScrollView {
      background-color: #292929;
      border-radius: 30px;
    }
  )");
}


#include <QScroller>
#include <QListWidget>

static QStringList get_list(const char* path) {
  QStringList stringList;
  QFile textFile(path);
  if (textFile.open(QIODevice::ReadOnly)) {
    QTextStream textStream(&textFile);
    while (true) {
      QString line = textStream.readLine();
      if (line.isNull()) {
        break;
      } else {
        stringList.append(line);
      }
    }
  }
  return stringList;
}

CarrotPanel::CarrotPanel(QWidget* parent) : QWidget(parent) {
  main_layout = new QStackedLayout(this);
  homeScreen = new QWidget(this);
  carrotLayout = new QVBoxLayout(homeScreen);
  carrotLayout->setMargin(10);

  QHBoxLayout* select_layout = new QHBoxLayout();
  select_layout->setSpacing(10);


  QPushButton* start_btn = new QPushButton(tr("Start"));
  start_btn->setObjectName("start_btn");
  QObject::connect(start_btn, &QPushButton::clicked, this, [this]() {
    this->currentCarrotIndex = 0;
    this->togglesCarrot(0);
    updateButtonStyles();
  });

  QPushButton* cruise_btn = new QPushButton(tr("Cruise"));
  cruise_btn->setObjectName("cruise_btn");
  QObject::connect(cruise_btn, &QPushButton::clicked, this, [this]() {
    this->currentCarrotIndex = 1;
    this->togglesCarrot(1);
    updateButtonStyles();
  });

  QPushButton* speed_btn = new QPushButton(tr("Speed"));
  speed_btn->setObjectName("speed_btn");
  QObject::connect(speed_btn, &QPushButton::clicked, this, [this]() {
    this->currentCarrotIndex = 2;
    this->togglesCarrot(2);
    updateButtonStyles();
  });

  QPushButton* latLong_btn = new QPushButton(tr("Tuning"));
  latLong_btn->setObjectName("latLong_btn");
  QObject::connect(latLong_btn, &QPushButton::clicked, this, [this]() {
    this->currentCarrotIndex = 3;
    this->togglesCarrot(3);
    updateButtonStyles();
  });

  QPushButton* disp_btn = new QPushButton(tr("Disp"));
  disp_btn->setObjectName("disp_btn");
  QObject::connect(disp_btn, &QPushButton::clicked, this, [this]() {
    this->currentCarrotIndex = 4;
    this->togglesCarrot(4);
    updateButtonStyles();
  });

  QPushButton* path_btn = new QPushButton(tr("Path"));
  path_btn->setObjectName("path_btn");
  QObject::connect(path_btn, &QPushButton::clicked, this, [this]() {
    this->currentCarrotIndex = 5;
    this->togglesCarrot(5);
    updateButtonStyles();
  });


  updateButtonStyles();

  select_layout->addWidget(start_btn);
  select_layout->addWidget(cruise_btn);
  select_layout->addWidget(speed_btn);
  select_layout->addWidget(latLong_btn);
  select_layout->addWidget(disp_btn);
  select_layout->addWidget(path_btn);
  carrotLayout->addLayout(select_layout, 0);

  QWidget* toggles = new QWidget();
  QVBoxLayout* toggles_layout = new QVBoxLayout(toggles);

  cruiseToggles = new ListWidget(this);
  cruiseToggles->addItem(new CValueControl("CruiseButtonMode", tr("按钮：巡航按钮模式"), tr("0:普通,1:用户1,2:用户2"), 0, 2, 1));
  cruiseToggles->addItem(new CValueControl("CancelButtonMode", tr("按钮：取消按钮模式"), tr("0:纵向,1:纵向+横向"), 0, 1, 1));
  cruiseToggles->addItem(new CValueControl("LfaButtonMode", tr("按钮：LFA按钮模式"), tr("0:普通,1:减速&停止&前车就绪"), 0, 1, 1));
  cruiseToggles->addItem(new CValueControl("CruiseSpeedUnitBasic", tr("按钮：巡航速度单位(基础)"), tr("调整每次按键改变的速度"), 1, 20, 1));
  cruiseToggles->addItem(new CValueControl("CruiseSpeedUnit", tr("按钮：巡航速度单位(扩展)"), tr("调整每次按键改变的速度"), 1, 20, 1));
  cruiseToggles->addItem(new CValueControl("CruiseEcoControl", tr("巡航：经济控制(4km/h)"), tr("临时提高设定速度以改善燃油经济性。"), 0, 10, 1));
  cruiseToggles->addItem(new CValueControl("AutoSpeedUptoRoadSpeedLimit", tr("巡航：自动加速(0%)"), tr("根据前车自动加速至道路限速。"), 0, 200, 10));
  cruiseToggles->addItem(new CValueControl("TFollowGap1", tr("GAP1：应用时间间隔(110)x0.01s"), tr("设置与前车的跟车间隔时间"), 70, 300, 5));
  cruiseToggles->addItem(new CValueControl("TFollowGap2", tr("GAP2：应用时间间隔(120)x0.01s"), tr("设置与前车的跟车间隔时间"), 70, 300, 5));
  cruiseToggles->addItem(new CValueControl("TFollowGap3", tr("GAP3：应用时间间隔(160)x0.01s"), tr("设置与前车的跟车间隔时间"), 70, 300, 5));
  cruiseToggles->addItem(new CValueControl("TFollowGap4", tr("GAP4：应用时间间隔(180)x0.01s"), tr("设置与前车的跟车间隔时间"), 70, 300, 5));
  cruiseToggles->addItem(new CValueControl("DynamicTFollow", tr("动态跟车间隔控制"), tr("根据情况动态调整跟车间隔"), 0, 100, 5));
  cruiseToggles->addItem(new CValueControl("DynamicTFollowLC", tr("动态跟车间隔控制(变道)"), tr("变道时的动态跟车间隔控制"), 0, 100, 5));
  cruiseToggles->addItem(new CValueControl("MyDrivingMode", tr("驾驶模式：选择"), tr("1:经济,2:安全,3:普通,4:运动"), 1, 4, 1));
  cruiseToggles->addItem(new CValueControl("MyDrivingModeAuto", tr("驾驶模式：自动"), tr("仅普通模式"), 0, 1, 1));
  cruiseToggles->addItem(new CValueControl("TrafficLightDetectMode", tr("交通灯检测模式"), tr("0:无, 1:仅停止, 2: 停走模式"), 0, 2, 1));
  cruiseToggles->addItem(new CValueControl("AChangeCostStarting", tr("变道成本起始值"), tr("变道成本的起始值，影响变道决策"), 0, 200, 10));
  cruiseToggles->addItem(new CValueControl("TrafficStopDistanceAdjust", tr("交通停止距离调整"), tr("调整交通停止时的距离，正值表示增加距离，负值表示减少距离"), -600, 600, 50));
  //cruiseToggles->addItem(new CValueControl("CruiseSpeedMin", tr("巡航：最低速度(10)"), tr("巡航控制的最低速度限制"), 5, 50, 1));
  //cruiseToggles->addItem(new CValueControl("AutoResumeFromGas", tr("油门自动巡航：使用"), tr("松开油门踏板时自动开启巡航，60%油门自动开启巡航"), 0, 3, 1));
  //cruiseToggles->addItem(new CValueControl("AutoResumeFromGasSpeed", tr("油门自动巡航：速度(30)"), tr("驾驶速度超过设定值时自动开启巡航"), 20, 140, 5));
  //cruiseToggles->addItem(new CValueControl("TFollowSpeedAddM", tr("GAP：额外跟车间隔40km/h(0)x0.01s"), tr("速度相关的额外跟车间隔，最大100km/h"), -100, 200, 5));
  //cruiseToggles->addItem(new CValueControl("TFollowSpeedAdd", tr("GAP：额外跟车间隔100km/h(0)x0.01s"), tr("速度相关的额外跟车间隔，最大100km/h"), -100, 200, 5));
  //cruiseToggles->addItem(new CValueControl("MyEcoModeFactor", tr("驾驶模式：经济模式加速比(80%)"), tr("经济模式下的加速比例"), 10, 95, 5));
  //cruiseToggles->addItem(new CValueControl("MySafeModeFactor", tr("驾驶模式：安全模式比例(60%)"), tr("安全模式下的加速/停止距离/减速比/跟车间隔控制比例"), 10, 90, 10));
  //cruiseToggles->addItem(new CValueControl("MyHighModeFactor", tr("驾驶模式：运动模式比例(100%)"), tr("运动模式下的加速比控制比例"), 100, 300, 10));

  latLongToggles = new ListWidget(this);
  latLongToggles->addItem(new CValueControl("UseLaneLineSpeed", tr("车道线模式速度(0)"), tr("车道线模式下使用横向控制"), 0, 200, 5));
  latLongToggles->addItem(new CValueControl("UseLaneLineCurveSpeed", tr("车道线模式弯道速度(0)"), tr("车道线模式，仅限高速"), 0, 200, 5));
  latLongToggles->addItem(new CValueControl("AdjustLaneOffset", tr("调整车道偏移(0)cm"), tr("调整车道居中偏移量"), 0, 500, 5));
  latLongToggles->addItem(new CValueControl("LaneChangeNeedTorque", tr("变道需要扭矩"), tr("-1:禁用变道, 0: 不需要扭矩, 1:需要扭矩"), -1, 1, 1));
  latLongToggles->addItem(new CValueControl("LaneChangeDelay", tr("变道延迟"), tr("变道延迟时间(x0.1秒)"), 0, 100, 5));
  latLongToggles->addItem(new CValueControl("LaneChangeBsd", tr("变道盲区检测"), tr("-1:忽略BSD, 0:BSD检测, 1: 阻止转向扭矩"), -1, 1, 1));
  latLongToggles->addItem(new CValueControl("CustomSR", tr("横向：转向比x0.1(0)"), tr("自定义转向比"), 0, 300, 1));
  latLongToggles->addItem(new CValueControl("SteerRatioRate", tr("横向：转向比变化率x0.01(100)"), tr("转向比应用速率"), 30, 170, 1));
  latLongToggles->addItem(new CValueControl("PathOffset", tr("横向：路径偏移"), tr("(-)向左, (+)向右"), -150, 150, 1));
  latLongToggles->addItem(new CValueControl("SteerActuatorDelay", tr("横向：转向执行器延迟(30)"), tr("x0.01, 0:使用延迟参数"), 0, 100, 1));
  latLongToggles->addItem(new CValueControl("LatSmoothSec", tr("横向：平滑时间(13)"), tr("横向平滑系数(x0.01)"), 1, 30, 1));
  latLongToggles->addItem(new CValueControl("LateralTorqueCustom", tr("横向：自定义扭矩(0)"), tr("自定义横向扭矩控制"), 0, 2, 1));
  latLongToggles->addItem(new CValueControl("LateralTorqueAccelFactor", tr("横向：扭矩加速度因子(2500)"), tr("扭矩与加速度的关系因子"), 1000, 6000, 10));
  latLongToggles->addItem(new CValueControl("LateralTorqueFriction", tr("横向：扭矩摩擦(100)"), tr("转向摩擦补偿系数"), 0, 1000, 10));
  latLongToggles->addItem(new CValueControl("CustomSteerMax", tr("横向：自定义最大转向(0)"), tr("自定义最大转向角度/扭矩"), 0, 30000, 5));
  latLongToggles->addItem(new CValueControl("CustomSteerDeltaUp", tr("横向：转向增量上限(0)"), tr("转向速度上限"), 0, 50, 1));
  latLongToggles->addItem(new CValueControl("CustomSteerDeltaDown", tr("横向：转向增量下限(0)"), tr("转向速度下限"), 0, 50, 1));
  latLongToggles->addItem(new CValueControl("LongTuningKpV", tr("纵向：比例增益(100)"), tr("PID控制器比例系数"), 0, 150, 5));
  latLongToggles->addItem(new CValueControl("LongTuningKiV", tr("纵向：积分增益(0)"), tr("PID控制器积分系数"), 0, 2000, 5));
  latLongToggles->addItem(new CValueControl("LongTuningKf", tr("纵向：前馈增益(100)"), tr("PID控制器前馈系数"), 0, 200, 5));
  latLongToggles->addItem(new CValueControl("LongActuatorDelay", tr("纵向：执行器延迟(20)"), tr("纵向控制执行器延迟"), 0, 200, 5));
  latLongToggles->addItem(new CValueControl("VEgoStopping", tr("纵向：停止速度因子(50)"), tr("停止过程中的速度因子"), 1, 100, 5));
  latLongToggles->addItem(new CValueControl("RadarReactionFactor", tr("纵向：雷达反应因子(100)"), tr("雷达数据反应速度因子"), 0, 200, 10));
  latLongToggles->addItem(new CValueControl("StoppingAccel", tr("纵向：停止起始加速度x0.01(-40)"), tr("开始停止时的加速度值"), -100, 0, 5));
  latLongToggles->addItem(new CValueControl("StopDistanceCarrot", tr("纵向：停止距离(600)cm"), tr("停止时的目标距离"), 300, 1000, 10));
  latLongToggles->addItem(new CValueControl("JLeadFactor3", tr("纵向：前车加加速度因子(0)"), tr("前车加速度影响因子(x0.01)"), 0, 100, 5));
  latLongToggles->addItem(new CValueControl("CruiseMaxVals0", tr("加速度：0km/h(160)"), tr("指定速度下的所需加速度(x0.01m/s²)"), 1, 250, 5));
  latLongToggles->addItem(new CValueControl("CruiseMaxVals1", tr("加速度：10km/h(160)"), tr("指定速度下的所需加速度(x0.01m/s²)"), 1, 250, 5));
  latLongToggles->addItem(new CValueControl("CruiseMaxVals2", tr("加速度：40km/h(120)"), tr("指定速度下的所需加速度(x0.01m/s²)"), 1, 250, 5));
  latLongToggles->addItem(new CValueControl("CruiseMaxVals3", tr("加速度：60km/h(100)"), tr("指定速度下的所需加速度(x0.01m/s²)"), 1, 250, 5));
  latLongToggles->addItem(new CValueControl("CruiseMaxVals4", tr("加速度：80km/h(80)"), tr("指定速度下的所需加速度(x0.01m/s²)"), 1, 250, 5));
  latLongToggles->addItem(new CValueControl("CruiseMaxVals5", tr("加速度：110km/h(70)"), tr("指定速度下的所需加速度(x0.01m/s²)"), 1, 250, 5));
  latLongToggles->addItem(new CValueControl("CruiseMaxVals6", tr("加速度：140km/h(60)"), tr("指定速度下的所需加速度(x0.01m/s²)"), 1, 250, 5));
  latLongToggles->addItem(new CValueControl("MaxAngleFrames", tr("最大角度帧数(89)"), tr("89:基本模式, 85~87:转向仪表盘报错"), 80, 100, 1));
  //latLongToggles->addItem(new CValueControl("AutoLaneChangeSpeed", tr("自动变道速度(20)"), tr("自动变道的最小速度要求"), 1, 100, 5));
  //latLongToggles->addItem(new CValueControl("JerkStartLimit", tr("纵向：启动加加速度(10)x0.1"), tr("启动时的加加速度限制"), 1, 50, 1));
  //latLongToggles->addItem(new CValueControl("LongitudinalTuningApi", tr("纵向：控制类型"), tr("0:速度PID, 1:加速度PID, 2:加速度PID(逗号标准)"), 0, 2, 1));
  //latLongToggles->addItem(new CValueControl("StartAccelApply", tr("纵向：启动加速2.0x(0)%"), tr("停止到启动时的加速度加速率，0:不使用"), 0, 100, 10));
  //latLongToggles->addItem(new CValueControl("StopAccelApply", tr("纵向：停止加速度-2.0x(0)%"), tr("停止保持时的刹车压力调整，0:不使用"), 0, 100, 10));
  //latLongToggles->addItem(new CValueControl("TraffStopDistanceAdjust", tr("纵向：交通停止距离调整(150)cm"), tr("调整交通停止时的距离"), -1000, 1000, 10));
  //latLongToggles->addItem(new CValueControl("CruiseMinVals", tr("纵向：减速率(120)"), tr("设置减速率(x0.01m/s²)"), 50, 250, 5));

  dispToggles = new ListWidget(this);
  dispToggles->addItem(new CValueControl("ShowDebugUI", tr("调试信息"), tr("显示调试相关信息"), 0, 2, 1));
  dispToggles->addItem(new CValueControl("ShowTpms", tr("胎压信息"), tr("显示胎压监测信息"), 0, 3, 1));
  dispToggles->addItem(new CValueControl("ShowDateTime", tr("时间信息"), tr("0:无,1:时间/日期,2:仅时间,3:仅日期"), 0, 3, 1));
  dispToggles->addItem(new CValueControl("ShowPathEnd", tr("路径终点"), tr("0:不显示,1:显示"), 0, 1, 1));
  dispToggles->addItem(new CValueControl("ShowDeviceState", tr("设备状态"), tr("0:不显示,1:显示"), 0, 1, 1));
  dispToggles->addItem(new CValueControl("ShowLaneInfo", tr("车道信息"), tr("-1:无, 0:路径, 1:路径+车道线, 2: 路径+车道线+路边"), -1, 2, 1));
  dispToggles->addItem(new CValueControl("ShowRadarInfo", tr("雷达信息"), tr("0:无,1:显示,2:相对位置,3:停止车辆"), 0, 3, 1));
  dispToggles->addItem(new CValueControl("ShowRouteInfo", tr("路线信息"), tr("0:无,1:显示"), 0, 1, 1));
  dispToggles->addItem(new CValueControl("ShowPlotMode", tr("调试图表"), tr("显示调试绘图模式"), 0, 10, 1));
  dispToggles->addItem(new CValueControl("ShowCustomBrightness", tr("亮度比例"), tr("自定义屏幕亮度比例"), 0, 100, 10));
  //dispToggles->addItem(new CValueControl("ShowHudMode", tr("显示模式"), tr("0:青蛙,1:自动驾驶,2:底部,3:顶部,4:左侧,5:左下"), 0, 5, 1));
  //dispToggles->addItem(new CValueControl("ShowSteerRotate", tr("方向盘旋转"), tr("0:无,1:旋转显示"), 0, 1, 1));
  //dispToggles->addItem(new CValueControl("ShowAccelRpm", tr("加速表"), tr("0:无,1:显示,2:加速+转速"), 0, 2, 1));
  //dispToggles->addItem(new CValueControl("ShowTpms", tr("胎压监测"), tr("0:无,1:显示"), 0, 1, 1));
  //dispToggles->addItem(new CValueControl("ShowSteerMode", tr("方向盘显示模式"), tr("0:黑色,1:彩色,2:无"), 0, 2, 1));
  //dispToggles->addItem(new CValueControl("ShowConnInfo", tr("APM连接状态"), tr("0:无,1:显示"), 0, 1, 1));
  //dispToggles->addItem(new CValueControl("ShowBlindSpot", tr("盲区监测信息"), tr("0:无,1:显示"), 0, 1, 1));
  //dispToggles->addItem(new CValueControl("ShowGapInfo", tr("跟车间隔信息"), tr("0:无,1:显示"), -1, 1, 1));
  //dispToggles->addItem(new CValueControl("ShowDmInfo", tr("驾驶员监控信息"), tr("0:无,1:显示,-1:禁用(需重启)"), -1, 1, 1));

  pathToggles = new ListWidget(this);
  pathToggles->addItem(new CValueControl("ShowPathColorCruiseOff", tr("路径颜色：巡航关闭"), tr("(+10:描边)0:红,1:橙,2:黄,3:绿,4:蓝,5:靛蓝,6:紫罗兰,7:棕,8:白,9:黑"), 0, 19, 1));
  pathToggles->addItem(new CValueControl("ShowPathMode", tr("路径模式：无车道"), tr("0:普通,1,2:矩形,3,4:^^,5,6:矩形,7,8:^^,9,10,11,12:平滑^^"), 0, 15, 1));
  pathToggles->addItem(new CValueControl("ShowPathColor", tr("路径颜色：无车道"), tr("(+10:描边)0:红,1:橙,2:黄,3:绿,4:蓝,5:靛蓝,6:紫罗兰,7:棕,8:白,9:黑"), 0, 19, 1));
  pathToggles->addItem(new CValueControl("ShowPathModeLane", tr("路径模式：车道模式"), tr("0:普通,1,2:矩形,3,4:^^,5,6:矩形,7,8:^^,9,10,11,12:平滑^^"), 0, 15, 1));
  pathToggles->addItem(new CValueControl("ShowPathColorLane", tr("路径颜色：车道模式"), tr("(+10:描边)0:红,1:橙,2:黄,3:绿,4:蓝,5:靛蓝,6:紫罗兰,7:棕,8:白,9:黑"), 0, 19, 1));
  pathToggles->addItem(new CValueControl("ShowPathWidth", tr("路径宽度比例(100%)"), tr("路径显示宽度比例"), 10, 200, 10));

  startToggles = new ListWidget(this);
  QString selected = QString::fromStdString(Params().get("CarSelected3"));
  QPushButton* selectCarBtn = new QPushButton(selected.length() > 1 ? selected : tr("选择您的车辆"));
  selectCarBtn->setObjectName("selectCarBtn");
  selectCarBtn->setStyleSheet(R"(
    QPushButton {
      margin-top: 20px; margin-bottom: 20px; padding: 10px; height: 120px; border-radius: 15px;
      color: #FFFFFF; background-color: #2C2CE2;
    }
    QPushButton:pressed {
      background-color: #2424FF;
    }
  )");
  //selectCarBtn->setFixedSize(350, 100);
  connect(selectCarBtn, &QPushButton::clicked, [=]() {
    QString selected = QString::fromStdString(Params().get("CarSelected3"));

    QStringList all_items = get_list((QString::fromStdString(Params().getParamPath()) + "/SupportedCars").toStdString().c_str());
    all_items.append(get_list((QString::fromStdString(Params().getParamPath()) + "/SupportedCars_gm").toStdString().c_str()));
    all_items.append(get_list((QString::fromStdString(Params().getParamPath()) + "/SupportedCars_toyota").toStdString().c_str()));
    all_items.append(get_list((QString::fromStdString(Params().getParamPath()) + "/SupportedCars_mazda").toStdString().c_str()));
    all_items.append(get_list((QString::fromStdString(Params().getParamPath()) + "/SupportedCars_honda").toStdString().c_str()));
    all_items.append(get_list((QString::fromStdString(Params().getParamPath()) + "/SupportedCars_ford").toStdString().c_str()));
    all_items.append(get_list((QString::fromStdString(Params().getParamPath()) + "/SupportedCars_tesla").toStdString().c_str()));
    all_items.append(get_list((QString::fromStdString(Params().getParamPath()) + "/SupportedCars_volkswagen").toStdString().c_str()));

    QMap<QString, QStringList> car_groups;
    for (const QString& car : all_items) {
      QStringList parts = car.split(" ", QString::SkipEmptyParts);
      if (!parts.isEmpty()) {
        QString manufacturer = parts.first();
        car_groups[manufacturer].append(car);
      }
    }

    QStringList manufacturers = car_groups.keys();
    QString selectedManufacturer = MultiOptionDialog::getSelection(tr("Select Manufacturer"), manufacturers, manufacturers.isEmpty() ? "" : manufacturers.first(), this);

    if (!selectedManufacturer.isEmpty()) {
      QStringList cars = car_groups[selectedManufacturer];
      QString selectedCar = MultiOptionDialog::getSelection(tr("Select your car"), cars, selected, this);

      if (!selectedCar.isEmpty()) {
        if (selectedCar == "[ Not Selected ]") {
          Params().remove("CarSelected3");
        } else {
          printf("Selected Car: %s\n", selectedCar.toStdString().c_str());
          Params().put("CarSelected3", selectedCar.toStdString());
          QTimer::singleShot(1000, []() {
            Params().putInt("SoftRestartTriggered", 1);
          });
          ConfirmationDialog::alert(selectedCar, this);
        }
        selected = QString::fromStdString(Params().get("CarSelected3"));
        selectCarBtn->setText((selected.isEmpty() || selected == "[ Not Selected ]") ? tr("SELECT YOUR CAR") : selected);
      }
    }
  });

  startToggles->addItem(selectCarBtn);
  startToggles->addItem(new CValueControl("HyundaiCameraSCC", tr("现代：摄像头SCC"), tr("1:连接SCC的CAN线到CAM, 2:同步巡航状态, 3:原厂纵向"), 0, 3, 1));
  startToggles->addItem(new CValueControl("CanfdHDA2", tr("CANFD：HDA2模式"), tr("1:HDA2,2:HDA2+BSM盲区监测"), 0, 2, 1));
  startToggles->addItem(new CValueControl("EnableRadarTracks", tr("启用雷达跟踪"), tr("1:启用雷达跟踪, -1,2:完全禁用HKG SCC雷达"), -1, 3, 1));
  startToggles->addItem(new CValueControl("EnableEscc", "Enable Hyundai ESCC(1)", "1:Enable ESCC, 0:Disable Escc. Reboot device after change the setting", 0, 1, 1));
  startToggles->addItem(new CValueControl("AutoCruiseControl", tr("自动巡航控制"), tr("软保持，自动巡航开关控制"), 0, 3, 1));
  startToggles->addItem(new CValueControl("CruiseOnDist", tr("巡航：自动开启距离(0cm)"), tr("当油门/刹车关闭时，前车接近时自动开启巡航"), 0, 2500, 50));
  startToggles->addItem(new CValueControl("AutoEngage", tr("启动时自动启用"), tr("1:启用转向, 2:启用转向/巡航"), 0, 2, 1));
  startToggles->addItem(new CValueControl("AutoGasTokSpeed", tr("自动油门启用速度"), tr("油门(加速)启用速度"), 0, 200, 5));
  startToggles->addItem(new CValueControl("SpeedFromPCM", tr("从PCM读取巡航速度"), tr("丰田必须设置为1, 本田设置为3"), 0, 3, 1));
  startToggles->addItem(new CValueControl("SoundVolumeAdjust", tr("音量(100%)"), tr("调整提示音量"), 5, 200, 5));
  startToggles->addItem(new CValueControl("SoundVolumeAdjustEngage", tr("启用时音量(10%)"), tr("启用时的提示音量"), 5, 200, 5));
  startToggles->addItem(new CValueControl("MaxTimeOffroadMin", tr("自动关机时间(分钟)"), tr("离线后自动关机时间"), 1, 600, 10));
  startToggles->addItem(new CValueControl("EnableConnect", tr("启用连接"), tr("您的设备可能会被Comma封禁"), 0, 2, 1));
  startToggles->addItem(new CValueControl("MapboxStyle", tr("地图样式(0)"), tr("选择地图显示样式"), 0, 2, 1));
  startToggles->addItem(new CValueControl("RecordRoadCam", tr("记录道路摄像头(0)"), tr("1:前摄像头, 2:前摄像头+广角摄像头"), 0, 2, 1));
  startToggles->addItem(new CValueControl("HDPuse", tr("使用HDP(CCNC)(0)"), tr("1:使用APN时, 2:始终使用"), 0, 2, 1));
  startToggles->addItem(new CValueControl("NNFF", tr("神经网络NNFF"), tr("Twilsonco的NNFF(需重启)"), 0, 1, 1));
  startToggles->addItem(new CValueControl("NNFFLite", tr("神经网络轻量版NNFFLite"), tr("Twilsonco的NNFF-Lite(需重启)"), 0, 1, 1));
  startToggles->addItem(new CValueControl("AutoGasSyncSpeed", tr("自动更新巡航速度"), tr("自动同步当前速度到巡航设定"), 0, 1, 1));
  startToggles->addItem(new CValueControl("DisableMinSteerSpeed", tr("禁用最小转向速度"), tr("移除转向最小速度限制"), 0, 1, 1));
  startToggles->addItem(new CValueControl("DisableDM", tr("禁用驾驶员监控"), tr("关闭驾驶员监控系统"), 0, 1, 1));
  startToggles->addItem(new CValueControl("HotspotOnBoot", tr("启动时开启热点"), tr("设备启动时自动开启热点"), 0, 1, 1));
  startToggles->addItem(new CValueControl("SoftwareMenu", tr("启用软件菜单"), tr("显示软件更新菜单"), 0, 1, 1));
  startToggles->addItem(new CValueControl("IsLdwsCar", tr("车道偏离预警车辆"), tr("启用车道偏离预警功能"), 0, 1, 1));
  startToggles->addItem(new CValueControl("HardwareC3xLite", tr("硬件是否为C3x Lite"), tr("识别硬件版本"), 0, 1, 1));
  startToggles->addItem(new CValueControl("ShareData", tr("共享数据"), tr("0:无, 1:TCP JSON数据(需重启)"), 0, 1, 1));
  //startToggles->addItem(new CValueControl("CarrotCountDownSpeed", tr("导航倒计时速度(10)"), tr("导航倒计时显示的速度阈值"), 0, 200, 5));
  //startToggles->addItem(new ParamControl("NoLogging", tr("禁用日志记录"), tr("关闭数据记录功能"), this));
  //startToggles->addItem(new ParamControl("LaneChangeNeedTorque", tr("变道：需要扭矩"), tr("设置变道是否需要方向盘扭矩输入"), this));
  //startToggles->addItem(new CValueControl("LaneChangeLaneCheck", tr("变道：检查车道存在"), tr("0:不检查,1:检查车道,2:检查车道+路边"), 0, 2, 1));

  speedToggles = new ListWidget(this);
  speedToggles->addItem(new CValueControl("AutoCurveSpeedLowerLimit", tr("弯道：最低速度(30)"), tr("接近弯道时降低速度，设置最低速度"), 30, 200, 5));
  speedToggles->addItem(new CValueControl("AutoCurveSpeedFactor", tr("弯道：自动控制比例(100%)"), tr("弯道速度控制比例"), 50, 300, 1));
  speedToggles->addItem(new CValueControl("AutoCurveSpeedAggressiveness", tr("弯道：激进程度(100%)"), tr("弯道减速的激进程度"), 50, 300, 1));
  speedToggles->addItem(new CValueControl("AutoRoadSpeedLimitOffset", tr("道路限速偏移(-1)"), tr("-1:不使用,道路限速+偏移"), -1, 100, 1));
  speedToggles->addItem(new CValueControl("AutoRoadSpeedAdjust", tr("道路限速自动调整(50%)"), tr("根据道路限速调整比例"), -1, 100, 5));
  speedToggles->addItem(new CValueControl("AutoNaviSpeedCtrlEnd", tr("测速摄像头减速结束点(6s)"), tr("设置减速完成点，值越大距离摄像头越远完成减速"), 3, 20, 1));
  speedToggles->addItem(new CValueControl("AutoNaviSpeedCtrlMode", tr("导航速度控制模式(2)"), tr("0:不减速, 1: 仅测速摄像头, 2: +防减速带, 3: +移动测速"), 0, 3, 1));
  speedToggles->addItem(new CValueControl("AutoNaviSpeedDecelRate", tr("测速摄像头减速率x0.01m/s²(80)"), tr("值越小，从更远距离开始减速"), 10, 200, 10));
  speedToggles->addItem(new CValueControl("AutoNaviSpeedSafetyFactor", tr("测速摄像头安全系数(105%)"), tr("速度控制的安全裕度"), 80, 120, 1));
  speedToggles->addItem(new CValueControl("AutoNaviSpeedBumpTime", tr("减速带时间距离(1s)"), tr("减速带提前时间"), 1, 50, 1));
  speedToggles->addItem(new CValueControl("AutoNaviSpeedBumpSpeed", tr("减速带速度(35Km/h)"), tr("通过减速带时的速度"), 10, 100, 5));
  speedToggles->addItem(new CValueControl("AutoNaviCountDownMode", tr("导航倒计时模式(2)"), tr("0: 关闭, 1:导航+摄像头, 2:导航+摄像头+减速带"), 0, 2, 1));
  speedToggles->addItem(new CValueControl("TurnSpeedControlMode", tr("转向速度控制模式(1)"), tr("0: 关闭, 1:视觉, 2:视觉+导航, 3: 仅导航"), 0, 3, 1));
  speedToggles->addItem(new CValueControl("CarrotSmartSpeedControl", tr("智能速度控制(0)"), tr("0: 关闭, 1:加速智能, 2:减速智能, 3: 全部"), 0, 3, 1));
  speedToggles->addItem(new CValueControl("MapTurnSpeedFactor", tr("地图转向速度系数(100)"), tr("基于地图数据的转向速度系数"), 50, 300, 5));
  speedToggles->addItem(new CValueControl("ModelTurnSpeedFactor", tr("模型转向速度系数(0)"), tr("基于模型的转向速度系数"), 0, 80, 10));
  speedToggles->addItem(new CValueControl("AutoTurnControl", tr("自动转向控制(0)"), tr("0:无, 1: 变道, 2: 变道+速度, 3: 速度"), 0, 3, 1));
  speedToggles->addItem(new CValueControl("AutoTurnControlSpeedTurn", tr("自动转向：转向速度(20)"), tr("0:无, 转向速度"), 0, 100, 5));
  speedToggles->addItem(new CValueControl("AutoTurnControlTurnEnd", tr("自动转向：转向距离时间(6)"), tr("距离=速度*时间"), 0, 30, 1));
  speedToggles->addItem(new CValueControl("AutoTurnMapChange", tr("自动转向：自动地图切换(0)"), tr("根据转向自动切换地图"), 0, 1, 1));

  toggles_layout->addWidget(cruiseToggles);
  toggles_layout->addWidget(latLongToggles);
  toggles_layout->addWidget(dispToggles);
  toggles_layout->addWidget(pathToggles);
  toggles_layout->addWidget(startToggles);
  toggles_layout->addWidget(speedToggles);
  ScrollView* toggles_view = new ScrollView(toggles, this);
  carrotLayout->addWidget(toggles_view, 1);

  homeScreen->setLayout(carrotLayout);
  main_layout->addWidget(homeScreen);
  main_layout->setCurrentWidget(homeScreen);

  togglesCarrot(0);
}

void CarrotPanel::togglesCarrot(int widgetIndex) {
  startToggles->setVisible(widgetIndex == 0);
  cruiseToggles->setVisible(widgetIndex == 1);
  speedToggles->setVisible(widgetIndex == 2);
  latLongToggles->setVisible(widgetIndex == 3);
  dispToggles->setVisible(widgetIndex == 4);
  pathToggles->setVisible(widgetIndex == 5);
}

void CarrotPanel::updateButtonStyles() {
  QString styleSheet = R"(
      #start_btn, #cruise_btn, #speed_btn, #latLong_btn ,#disp_btn, #path_btn {
        height: 120px; border-radius: 15px; background-color: #393939;
      }
      #start_btn:pressed, #cruise_btn:pressed, #speed_btn:pressed, #latLong_btn:pressed, #disp_btn:pressed, #path_btn:pressed {
        background-color: #4a4a4a;
      }
  )";

  switch (currentCarrotIndex) {
  case 0:
    styleSheet += "#start_btn { background-color: #33ab4c; }";
    break;
  case 1:
    styleSheet += "#cruise_btn { background-color: #33ab4c; }";
    break;
  case 2:
    styleSheet += "#speed_btn { background-color: #33ab4c; }";
    break;
  case 3:
    styleSheet += "#latLong_btn { background-color: #33ab4c; }";
    break;
  case 4:
    styleSheet += "#disp_btn { background-color: #33ab4c; }";
    break;
  case 5:
    styleSheet += "#path_btn { background-color: #33ab4c; }";
    break;
  }

  setStyleSheet(styleSheet);
}


CValueControl::CValueControl(const QString& params, const QString& title, const QString& desc, int min, int max, int unit)
  : AbstractControl(title, desc), m_params(params), m_min(min), m_max(max), m_unit(unit) {

  label.setAlignment(Qt::AlignVCenter | Qt::AlignRight);
  label.setStyleSheet("color: #e0e879");
  hlayout->addWidget(&label);

  QString btnStyle = R"(
    QPushButton {
      padding: 0;
      border-radius: 50px;
      font-size: 20px;
      font-weight: 300;
      color: #E4E4E4;
      background-color: #393939;
    }
    QPushButton:pressed {
      background-color: #4a4a4a;
    }
  )";

  btnminus.setStyleSheet(btnStyle);
  btnplus.setStyleSheet(btnStyle);
  btnminus.setFixedSize(100, 100);
  btnplus.setFixedSize(100, 100);
  btnminus.setText("－");
  btnplus.setText("＋");
  hlayout->addWidget(&btnminus);
  hlayout->addWidget(&btnplus);

  connect(&btnminus, &QPushButton::released, this, &CValueControl::decreaseValue);
  connect(&btnplus, &QPushButton::released, this, &CValueControl::increaseValue);

  refresh();
}

void CValueControl::showEvent(QShowEvent* event) {
  AbstractControl::showEvent(event);
  refresh();
}

void CValueControl::refresh() {
  label.setText(QString::fromStdString(Params().get(m_params.toStdString())));
}

void CValueControl::adjustValue(int delta) {
  int value = QString::fromStdString(Params().get(m_params.toStdString())).toInt();
  value = qBound(m_min, value + delta, m_max);
  Params().putInt(m_params.toStdString(), value);
  refresh();
}

void CValueControl::increaseValue() {
  adjustValue(m_unit);
}

void CValueControl::decreaseValue() {
  adjustValue(-m_unit);
}
