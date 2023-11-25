#include <filesystem>

#include "selfdrive/ui/qt/offroad/frogpilot_settings.h"

FrogPilotControlsPanel::FrogPilotControlsPanel(QWidget *parent) : FrogPilotPanel(parent) {
  setParams();

  mainLayout = new QVBoxLayout(this);

  QLabel *const descriptionLabel = new QLabel("Click on the toggle names to see a detailed toggle description", this);
  mainLayout->addWidget(descriptionLabel);
  mainLayout->addSpacing(25);
  mainLayout->addWidget(whiteHorizontalLine());

  static const std::vector<std::tuple<QString, QString, QString, QString>> toggles = {
    {"AdjustablePersonalities", "Adjustable Personalities", "Switch personalities using the 'Distance' button on the steering wheel (GM/Lexus/Toyota Only) or via the onroad UI for other makes.\n\n1 bar = Aggressive\n2 bars = Standard\n3 bars = Relaxed", "../assets/offroad/icon_distance.png"},
    {"AlwaysOnLateral", "Always on Lateral / No disengage on Brake Pedal", "Keep openpilot lateral control when using either the brake or gas pedals. openpilot is only disengaged by deactivating the 'Cruise Control' button.", "../assets/offroad/icon_always_on_lateral.png"},
    {"ConditionalExperimental", "Conditional Experimental Mode", "Automatically activate 'Experimental Mode' based on specified conditions.", "../assets/offroad/icon_conditional.png"},
    {"CustomPersonalities", "Custom Driving Personalities", "Customize the driving personality profiles to your liking.", "../assets/offroad/icon_custom.png"},
    {"DeviceShutdown", "Device Shutdown Timer", "Set the timer for when the device turns off after being offroad to reduce energy waste and prevent battery drain.", "../assets/offroad/icon_time.png"},
    {"ExperimentalModeViaPress", "Experimental Mode Via Steering Wheel / Screen", "Enable or disable Experimental Mode by double-clicking the 'Lane Departure'/LKAS button on the steering wheel (Toyota/Lexus Only) or double tapping the screen for other makes.\n\nOverrides 'Conditional Experimental Mode'. ", "../assets/img_experimental_white.svg"},
    {"FireTheBabysitter", "Fire the Babysitter", "Disable some of openpilot's 'Babysitter Protocols'.", "../assets/offroad/icon_babysitter.png"},
    {"LateralTune", "Lateral Tuning", "Change the way openpilot steers.", "../assets/offroad/icon_lateral_tune.png"},
    {"LongitudinalTune", "Longitudinal Tuning", "Change the way openpilot accelerates and brakes.", "../assets/offroad/icon_longitudinal_tune.png"},
    {"Model", "Model Selector (Requires Reboot)", "Select your preferred openpilot model.\n\nFV = Farmville(Default)\nNLP = New Lemon Pie\nBD = Blue Diamond", "../assets/offroad/icon_calibration.png"},
    {"NudgelessLaneChange", "Nudgeless Lane Change", "Switch lanes without having to nudge the steering wheel.", "../assets/offroad/icon_lane.png"},
    {"PauseLateralOnSignal", "Pause Lateral On Turn Signal", "Pauses lateral control when a turn signal is active.", "../assets/offroad/icon_pause_lane.png"},
    {"SpeedLimitController", "Speed Limit Controller", "Use Open Street Maps, Navigate On openpilot, and your car's dashboard (Toyota only) to set the vehicle's speed to the current speed limit.", "../assets/offroad/icon_speed_limit.png"},
    {"TurnDesires", "Turn Desires", "Use turn desires when below the minimum lane change speed for more precise turns.", "../assets/navigation/direction_continue_right.png"},
    {"VisionTurnControl", "Vision Turn Speed Control", "Automatically adjusts the vehicle's speed in accordance of the curvature of the road for smoother turning.", "../assets/offroad/icon_vtc.png"}
  };

  for (const auto &[key, label, desc, icon] : toggles) {
    ParamControl *control = createParamControl(key, label, desc, icon, this);
    if (key == "AdjustablePersonalities") {
      mainLayout->addWidget(new AdjustablePersonalities());
      mainLayout->addWidget(horizontalLine());
    } else if (key == "AlwaysOnLateral") {
      createSubControl(key, label, desc, icon, {}, {
        {"AlwaysOnLateralMain", "Enable AOL On Cruise Main", "Enables Always On Lateral by simply turning on cruise control as opposed to requiring openpilot to be enabled first."}
      });
    } else if (key == "ConditionalExperimental") {
      createSubControl(key, label, desc, icon, {
        createDualParamControl(new CESpeed(), new CESpeedLead()),
      });
      createSubButtonControl(key, {
        {"CECurves", "Curves"},
        {"CECurvesLead", "Curves With Lead"},
        {"CENavigation", "Navigation Based"}
      }, mainLayout);
      createSubButtonControl(key, {
        {"CESlowerLead", "Slower Lead Ahead"},
        {"CEStopLights", "Stop Lights and Stop Signs"},
        {"CESignal", "Turn Signal < " + QString(isMetric ? "90kph" : "55mph")}
      }, mainLayout);
    } else if (key == "CustomPersonalities") {
      createSubControl(key, label, desc, icon, {
        createDualParamControl(new AggressiveFollow(), new AggressiveJerk()),
        createDualParamControl(new StandardFollow(), new StandardJerk()),
        createDualParamControl(new RelaxedFollow(), new RelaxedJerk()),
      });
    } else if (key == "DeviceShutdown") {
      mainLayout->addWidget(new DeviceShutdown());
      mainLayout->addWidget(horizontalLine());
    } else if (key == "FireTheBabysitter") {
      createSubControl(key, label, desc, icon, {}, {
        {"NoLogging", "Disable Logging", "Prevent all data tracking by comma to go completely incognitio or to even just reduce thermals.\n\nWARNING: This will prevent any drives from being recorded and they WILL NOT be recoverable!"}
      });
      createSubButtonControl(key, {
        {"MuteDM", "Mute DM"},
        {"MuteDoor", "Mute Door Open"},
        {"MuteOverheated", "Mute Overheat"},
        {"MuteSeatbelt", "Mute Seatbelt"}
      }, mainLayout);
    } else if (key == "LateralTune") {
      createSubControl(key, label, desc, icon, {}, {
        {"AverageCurvature", "Average Desired Curvature", "Use Pfeiferj's distance based curvature adjustment for smoother handling of curves."},
        {"NNFF", "NNFF - Neural Network Feedforward", "Use Twilsonco's Neural Network Feedforward torque system for more precise lateral control."}
      });
    } else if (key == "LongitudinalTune") {
      createSubControl(key, label, desc, icon, {
        new AccelerationProfile(),
        new StoppingDistance(),
      }, {
        {"AggressiveAcceleration", "Aggressive Acceleration With Lead", "Accelerate more aggressively behind a lead when starting from a stop."},
        {"SmoothBraking", "Smoother Braking Behind Lead", "More natural braking behavior when coming up to a slower vehicle."}
      });
    } else if (key == "Model") {
      mainLayout->addWidget(new Model());
      mainLayout->addWidget(horizontalLine());
    } else if (key == "NudgelessLaneChange") {
      createSubControl(key, label, desc, icon, {
        new LaneChangeTime(),
      });
      createSubButtonControl(key, {
        {"LaneDetection", "Lane Detection"},
        {"OneLaneChange", "One Lane Change Per Signal"},
      }, mainLayout);
    } else if (key == "SpeedLimitController") {
      std::vector<QWidget*> widgets;
      widgets.push_back(new SLCFallback());
      widgets.push_back(new SLCPriority());

      if (isMetric) {
        widgets.push_back(createDualParamControl(new Offset1Metric(), new Offset2Metric()));
        widgets.push_back(createDualParamControl(new Offset3Metric(), new Offset4Metric()));
      } else {
        widgets.push_back(createDualParamControl(new Offset1(), new Offset2()));
        widgets.push_back(createDualParamControl(new Offset3(), new Offset4()));
      }
      createSubControl(key, label, desc, icon, widgets);
    } else if (key == "VisionTurnControl") {
      createSubControl(key, label, desc, icon, {
        new CurveSensitivity(),
        new TurnAggressiveness(),
      });
    } else {
      mainLayout->addWidget(control);
      if (key != std::get<0>(toggles.back())) mainLayout->addWidget(horizontalLine());
    }
  }
  setInitialToggleStates();
}

FrogPilotVehiclesPanel::FrogPilotVehiclesPanel(QWidget *parent) : FrogPilotPanel(parent) {
  mainLayout = new QVBoxLayout(this);

  QHBoxLayout *forceFingerprintLayout = new QHBoxLayout();
  forceFingerprintLayout->setSpacing(25);
  forceFingerprintLayout->setContentsMargins(0, 0, 0, 0);

  QLabel *forceFingerprintLabel = new QLabel(tr("Force Fingerprint"));
  forceFingerprintLayout->addWidget(forceFingerprintLabel);

  forceFingerprintLayout->addStretch(1);

  QString currentCarModel = QString::fromStdString(params.get("CarModel"));
  QLabel *carModelLabel = new QLabel(currentCarModel);
  forceFingerprintLayout->addWidget(carModelLabel);

  ButtonControl *forceFingerprintButton = new ButtonControl(tr(""), tr("SET"));
  forceFingerprintLayout->addWidget(forceFingerprintButton);

  connect(forceFingerprintButton, &ButtonControl::clicked, this, [=]() {
    std::system("python3 /data/openpilot/scripts/set_fingerprints.py");
    std::string carModels = params.get("CarModels");

    QStringList cars = QString::fromStdString(carModels).split(',');
    QString selection = MultiOptionDialog::getSelection(tr("Select Your Car"), cars, currentCarModel, this);

    if (!selection.isEmpty()) {
      params.put("CarModel", selection.toStdString());
      carModelLabel->setText(selection);
    }
  });

  mainLayout->addLayout(forceFingerprintLayout);
  mainLayout->addWidget(whiteHorizontalLine());

  QHBoxLayout *gmLayout = new QHBoxLayout();
  gmLayout->setSpacing(25);
  gmLayout->setContentsMargins(0, 0, 0, 0);

  QLabel *gmIconLabel = new QLabel(this);
  gmIconLabel->setPixmap(QPixmap("../assets/offroad/icon_gm.png").scaledToWidth(80, Qt::SmoothTransformation));

  QLabel *gmTextLabel = new QLabel("GM", this);

  gmLayout->addWidget(gmIconLabel);
  gmLayout->addWidget(gmTextLabel);
  gmLayout->addStretch(1);
  mainLayout->addLayout(gmLayout);
  mainLayout->addWidget(whiteHorizontalLine());

  static const std::vector<std::tuple<QString, QString, QString, QString>> gmToggles = {
    {"EVTable", "EV Lookup Tables", "Smoothens out the gas and brake controls for EV vehicles.", "../assets/offroad/icon_blank.png"},
    {"LowerVolt", "Lower Volt Enable Speed", "Lowers the Volt's minimum enable speed in order to enable openpilot at any speed.", "../assets/offroad/icon_blank.png"}
  };

  for (const auto &[key, label, desc, icon] : gmToggles) {
    ParamControl *control = createParamControl(key, label, desc, icon, this);
    mainLayout->addWidget(control);
    if (key != std::get<0>(gmToggles.back())) mainLayout->addWidget(horizontalLine());
  }

  mainLayout->addWidget(whiteHorizontalLine());
  mainLayout->setSpacing(25);
  QHBoxLayout *toyotaLayout = new QHBoxLayout();
  toyotaLayout->addWidget(whiteHorizontalLine());
  toyotaLayout->setSpacing(25);
  toyotaLayout->setContentsMargins(0, 0, 0, 0);

  QLabel *toyotaIconLabel = new QLabel(this);
  toyotaIconLabel->setPixmap(QPixmap("../assets/offroad/icon_toyota.png").scaledToWidth(80, Qt::SmoothTransformation));

  QLabel *toyotaTextLabel = new QLabel("Toyota", this);

  toyotaLayout->addWidget(toyotaIconLabel);
  toyotaLayout->addWidget(toyotaTextLabel);
  toyotaLayout->addStretch(1);
  mainLayout->addLayout(toyotaLayout);
  mainLayout->addWidget(whiteHorizontalLine());

  static const std::vector<std::tuple<QString, QString, QString, QString>> toyotaToggles = {
    {"LockDoors", "Lock Doors In Drive", "Automatically locks the doors when in drive and unlocks when in park.", "../assets/offroad/icon_blank.png"},
    {"SNGHack", "SNG Hack", "Enable the SNG Hack for vehicles without stock stop and go.", "../assets/offroad/icon_blank.png"},
    {"TSS2Tune", "TSS2 Tune", "Tuning profile for TSS2 vehicles. Based on the tuning profile from DragonPilot.", "../assets/offroad/icon_blank.png"}
  };

  for (const auto &[key, label, desc, icon] : toyotaToggles) {
    ParamControl *control = createParamControl(key, label, desc, icon, this);
    mainLayout->addWidget(control);
    if (key != std::get<0>(toyotaToggles.back())) mainLayout->addWidget(horizontalLine());
  }

  setInitialToggleStates();
}

FrogPilotVisualsPanel::FrogPilotVisualsPanel(QWidget *parent) : FrogPilotPanel(parent) {
  mainLayout = new QVBoxLayout(this);

  QLabel *const descriptionLabel = new QLabel("Click on the toggle names to see a detailed toggle description", this);
  mainLayout->addWidget(descriptionLabel);
  mainLayout->addSpacing(25);
  mainLayout->addWidget(whiteHorizontalLine());

  static const std::vector<std::tuple<QString, QString, QString, QString>> toggles = {
    {"CustomTheme", "Custom Theme", "Enable the ability to use custom themes.", "../assets/frog.png"},
    {"CameraView", "Camera View (Cosmetic Only)", "Set your preferred camera view for the onroad UI. This toggle is purely cosmetic and will not affect openpilot's use of the other cameras.", "../assets/offroad/icon_camera.png"},
    {"Compass", "Compass", "Add a compass to the onroad UI that indicates your current driving direction.", "../assets/offroad/icon_compass.png"},
    {"CustomUI", "Custom UI", "Customize the UI to your liking.", "../assets/offroad/icon_road.png"},
    {"DriverCamera", "Driver Camera On Reverse", "Displays the driver camera when in reverse.", "../assets/img_driver_face_static.png"},
    {"GreenLightAlert", "Green Light Alert", "Displays an alert when a light turns from red to green.", "../assets/offroad/icon_green_light.png"},
    {"RotatingWheel", "Rotating Steering Wheel", "The steering wheel in top right corner of the onroad UI rotates alongside your physical steering wheel.", "../assets/offroad/icon_rotate.png"},
    {"ScreenBrightness", "Screen Brightness", "Choose a custom screen brightness level or use the default 'Auto' brightness setting.", "../assets/offroad/icon_light.png"},
    {"SilentMode", "Silent Mode", "Disables all openpilot sounds for a completely silent experience.", "../assets/offroad/icon_mute.png"},
    {"WheelIcon", "Steering Wheel Icon", "Replace the stock openpilot steering wheel icon with a custom icon.\n\nWant to submit your own steering wheel? Message me on Discord\n@FrogsGoMoo!", "../assets/offroad/icon_openpilot.png"},
  };

  for (const auto &[key, label, desc, icon] : toggles) {
    ParamControl *control = createParamControl(key, label, desc, icon, this);
    if (key == "CameraView") {
      mainLayout->addWidget(new CameraView());
      mainLayout->addWidget(horizontalLine());
    } else if (key == "CustomUI") {
      createSubControl(key, label, desc, icon, {
        createDualParamControl(new LaneLinesWidth(), new RoadEdgesWidth()),
        createDualParamControl(new PathWidth(), new PathEdgeWidth())
      });
      createSubButtonControl(key, {
        {"AccelerationPath", "Acceleration Path"},
        {"AdjacentPath", "Adjacent Paths"},
        {"BlindSpotPath", "Blind Spot Path"},
      }, mainLayout);
      createSubButtonControl(key, {
        {"ShowFPS", "FPS Counter"},
        {"LeadInfo", "Lead Info and Logics"},
        {"RoadNameUI", "Road Name"},
      }, mainLayout);
      createSubButtonControl(key, {
        {"UnlimitedLength", "'Unlimited' Road UI Length"},
      }, mainLayout);
    } else if (key == "CustomTheme") {
      createSubControl(key, label, desc, icon, {
        createDualParamControl(new CustomColors(), new CustomIcons()),
        createDualParamControl(new CustomSignals(), new CustomSounds()),
      });
    } else if (key == "ScreenBrightness") {
      mainLayout->addWidget(new ScreenBrightness());
      mainLayout->addWidget(horizontalLine());
    } else if (key == "WheelIcon") {
      mainLayout->addWidget(new WheelIcon());
      mainLayout->addWidget(horizontalLine());
    } else {
      mainLayout->addWidget(control);
      if (key != std::get<0>(toggles.back())) mainLayout->addWidget(horizontalLine());
    }
  }
  setInitialToggleStates();
}

ParamControl *FrogPilotPanel::createParamControl(const QString &key, const QString &label, const QString &desc, const QString &icon, QWidget *parent) {
  ParamControl *control = new ParamControl(key, label, desc, icon);
  connect(control, &ParamControl::toggleFlipped, [=](bool state) {
    paramsMemory.putBoolNonBlocking("FrogPilotTogglesUpdated", true);

    if (key == "NNFF") {
      if (params.getBool("NNFF")) {
        const bool addSSH = ConfirmationDialog::yesorno("Would you like to grant 'twilsonco' SSH access to improve NNFF? This won't affect any added SSH keys.", parent);
        params.putBoolNonBlocking("TwilsoncoSSH", addSSH);
        if (addSSH) {
          ConfirmationDialog::toggleAlert("Message 'twilsonco' on Discord to get your device properly configured.", "Acknowledge", parent);
        }
      }
    }

    static const QMap<QString, QString> parameterWarnings = {
      {"AggressiveAcceleration", "This will make openpilot driving more aggressively behind lead vehicles!"},
      {"AlwaysOnLateralMain", "This is very experimental and isn't guaranteed to work. If you run into any issues please report it in the FrogPilot Discord!"},
      {"SmoothBraking", "This will modify openpilot's braking behavior!"},
      {"TSS2Tune", "This will modify openpilot's acceleration and braking behavior!"}
    };
    if (parameterWarnings.contains(key) && params.getBool(key.toStdString())) {
      ConfirmationDialog::toggleAlert("WARNING: " + parameterWarnings[key], "I understand the risks.", parent);
    }

    static const QSet<QString> parameterReboots = {
      "AlwaysOnLateral", "AlwaysOnLateralMain", "FireTheBabysitter", "NoLogging", "NNFF", "TSS2Tune",
    };
    if (parameterReboots.contains(key)) {
      if (ConfirmationDialog::toggle("Reboot required to take effect.", "Reboot Now", parent)) {
        Hardware::reboot();
      }
    }

    auto it = childControls.find(key.toStdString());
    if (it != childControls.end()) {
      for (QWidget *widget : it->second) {
        widget->setVisible(state);
      }
    }

    if (key == "ConditionalExperimental") {
      params.putBoolNonBlocking("ExperimentalMode", state);
    }
  });
  return control;
}

QFrame *FrogPilotPanel::horizontalLine(QWidget *parent) const {
  QFrame *line = new QFrame(parent);

  line->setFrameShape(QFrame::StyledPanel);
  line->setStyleSheet(R"(
    border-width: 1px;
    border-bottom-style: solid;
    border-color: gray;
  )");
  line->setFixedHeight(2);

  return line;
}

QFrame *FrogPilotPanel::whiteHorizontalLine(QWidget *parent) const {
  QFrame *line = new QFrame(parent);

  line->setFrameShape(QFrame::StyledPanel);
  line->setStyleSheet(R"(
    border-width: 1px;
    border-bottom-style: solid;
    border-color: white;
  )");
  line->setFixedHeight(2);

  return line;
}

QWidget *FrogPilotPanel::createDualParamControl(ParamValueControl *control1, ParamValueControl *control2) {
  QWidget *mainControl = new QWidget(this);
  QHBoxLayout *layout = new QHBoxLayout();

  layout->addWidget(control1);
  layout->addStretch();
  layout->addWidget(control2);
  mainControl->setLayout(layout);

  return mainControl;
}

QWidget *FrogPilotPanel::addSubControls(const QString &parentKey, QVBoxLayout *layout, const std::vector<std::tuple<QString, QString, QString>> &controls) {
  QWidget *mainControl = new QWidget(this);

  mainControl->setLayout(layout);
  mainLayout->addWidget(mainControl);
  mainControl->setVisible(params.getBool(parentKey.toStdString()));

  for (const auto &[key, label, desc] : controls) addControl(key, "   " + label, desc, layout);

  return mainControl;
}

void FrogPilotPanel::addControl(const QString &key, const QString &label, const QString &desc, QVBoxLayout *layout, const QString &icon) {
  layout->addWidget(createParamControl(key, label, desc, icon, this));
  layout->addWidget(horizontalLine());
}

void FrogPilotPanel::createSubControl(const QString &key, const QString &label, const QString &desc, const QString &icon, const std::vector<QWidget*> &subControls, const std::vector<std::tuple<QString, QString, QString>> &additionalControls) {
  ParamControl *control = createParamControl(key, label, desc, icon, this);

  mainLayout->addWidget(control);
  mainLayout->addWidget(horizontalLine());

  QVBoxLayout *subControlLayout = new QVBoxLayout();

  for (QWidget *subControl : subControls) {
    subControlLayout->addWidget(subControl);
    subControlLayout->addWidget(horizontalLine());
  }

  QWidget *mainControl = addSubControls(key, subControlLayout, additionalControls);

  connect(control, &ParamControl::toggleFlipped, [=](bool state) { mainControl->setVisible(state); });
}

void FrogPilotPanel::createSubButtonControl(const QString &parentKey, const std::vector<QPair<QString, QString>> &buttonKeys, QVBoxLayout *subControlLayout) {
  QHBoxLayout *buttonsLayout = new QHBoxLayout();
  QWidget *line = horizontalLine();

  buttonsLayout->addStretch();

  for (const auto &[key, label] : buttonKeys) {
    FrogPilotButtonParamControl* button = new FrogPilotButtonParamControl(key, label);
    mainLayout->addWidget(button);
    buttonsLayout->addWidget(button);
    buttonsLayout->addStretch();
    button->setVisible(params.getBool(parentKey.toStdString()));
    childControls[parentKey.toStdString()].push_back(button);
  }

  subControlLayout->addLayout(buttonsLayout);

  line = horizontalLine();
  mainLayout->addWidget(line);

  childControls[parentKey.toStdString()].push_back(line);
}

void FrogPilotPanel::setInitialToggleStates() {
  for (const auto& [key, controlSet] : childControls) {
    bool state = params.getBool(key);
    for (QWidget *widget : controlSet) {
      widget->setVisible(state);
    }
  }
}

void FrogPilotPanel::setParams() {
  if (!std::filesystem::exists("/data/openpilot/selfdrive/modeld/models/supercombo.thneed")) {
    params.putBool("DoReboot", true);
  }

  if (params.getBool("DisableOnroadUploads")) {
    paramsMemory.putBool("DisableOnroadUploads", true);
  }
  if (params.getBool("FireTheBabysitter") and params.getBool("MuteDM")) {
    paramsMemory.putBool("MuteDM", true);
  }
  if (params.getBool("FireTheBabysitter") and params.getBool("NoLogging")) {
    paramsMemory.putBool("NoLogging", true);
  }
  if (params.getBool("RoadNameUI") || params.getBool("SpeedLimitController")) {
    paramsMemory.putBool("OSM", true);
  }

  const bool FrogsGoMoo = params.get("DongleId").substr(0, 3) == "be6";

  const std::map<std::string, std::string> default_values {
    {"AccelerationPath", "1"},
    {"AccelerationProfile", "3"},
    {"AdjacentPath", "1"},
    {"AdjustablePersonalities", "1"},
    {"AggressiveAcceleration", "1"},
    {"AggressiveFollow", FrogsGoMoo ? "10" : "12"},
    {"AggressiveJerk", FrogsGoMoo ? "6" : "5"},
    {"AlwaysOnLateral", "1"},
    {"AlwaysOnLateralMain", FrogsGoMoo ? "1" : "0"},
    {"AverageCurvature", FrogsGoMoo ? "1" : "0"},
    {"BlindSpotPath", "1"},
    {"CameraView", FrogsGoMoo ? "1" : "0"},
    {"CECurves", "1"},
    {"CECurvesLead", "0"},
    {"CENavigation", "1"},
    {"CESignal", "1"},
    {"CESlowerLead", "0"},
    {"CESpeed", "0"},
    {"CESpeedLead", "0"},
    {"CEStopLights", "1"},
    {"Compass", "1"},
    {"ConditionalExperimental", "1"},
    {"CurveSensitivity", FrogsGoMoo ? "125" : "100"},
    {"CustomColors", "1"},
    {"CustomIcons", "1"},
    {"CustomPersonalities", "1"},
    {"CustomSignals", "1"},
    {"CustomSounds", "1"},
    {"CustomTheme", "1"},
    {"CustomUI", "1"},
    {"DeviceShutdown", "9"},
    {"DriverCamera", "0"},
    {"EVTable", "0"},
    {"ExperimentalModeViaPress", "1"},
    {"FireTheBabysitter", FrogsGoMoo ? "1" : "0"},
    {"GreenLightAlert", "0"},
    {"LaneChangeTime", "0"},
    {"LaneDetection", "1"},
    {"LaneLinesWidth", "4"},
    {"LateralTune", "1"},
    {"LeadInfo", "1"},
    {"LockDoors", "0"},
    {"LongitudinalTune", "1"},
    {"LowerVolt", "0"},
    {"Model", "0"},
    {"MuteDM", "1"},
    {"MuteDoor", "1"},
    {"MuteOverheated", "1"},
    {"MuteSeatbelt", "1"},
    {"NNFF", FrogsGoMoo ? "1" : "0"},
    {"NudgelessLaneChange", "1"},
    {"NumericalTemp", "1"},
    {"Offset1", "5"},
    {"Offset2", FrogsGoMoo ? "7" : "5"},
    {"Offset3", "10"},
    {"Offset4", FrogsGoMoo ? "20" : "10"},
    {"OneLaneChange", "1"},
    {"PathEdgeWidth", "20"},
    {"PathWidth", "61"},
    {"PauseLateralOnSignal", "0"},
    {"RelaxedFollow", "30"},
    {"RelaxedJerk", "50"},
    {"RoadEdgesWidth", "2"},
    {"RoadNameUI", "1"},
    {"RotatingWheel", "1"},
    {"SLCFallback", "2"},
    {"SLCPriority", "1"},
    {"SNGHack", "0"},
    {"ScreenBrightness", "101"},
    {"ShowCPU", FrogsGoMoo ? "1" : "0"},
    {"ShowMemoryUsage", FrogsGoMoo ? "1" : "0"},
    {"ShowFPS", FrogsGoMoo ? "1" : "0"},
    {"Sidebar", "1"},
    {"SilentMode", "0"},
    {"SmoothBraking", "1"},
    {"SpeedLimitController", "1"},
    {"StandardFollow", "15"},
    {"StandardJerk", "10"},
    {"StoppingDistance", FrogsGoMoo ? "6" : "3"},
    {"TSS2Tune", "1"},
    {"TurnAggressiveness", FrogsGoMoo ? "85" : "100"},
    {"TurnDesires", "1"},
    {"UnlimitedLength", "1"},
    {"VisionTurnControl", "1"},
    {"WheelIcon", "1"}
  };

  bool rebootRequired = false;
  for (const auto& [key, value] : default_values) {
    if (params.get(key).empty()) {
      params.put(key, value);
      rebootRequired = true;
    }
  }

  if (rebootRequired) {
    while (!std::filesystem::exists("/data/openpilot/prebuilt")) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    Hardware::reboot();
  }
}
