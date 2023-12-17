#include <filesystem>

#include "selfdrive/frogpilot/ui/frogpilot_settings.h"

FrogPilotControlsPanel::FrogPilotControlsPanel(QWidget *parent) : FrogPilotPanel(parent) {
  setParams();

  mainLayout = new QVBoxLayout(this);

  QLabel *const descriptionLabel = new QLabel("Click on the toggle names to see a detailed toggle description", this);
  mainLayout->addWidget(descriptionLabel);
  mainLayout->addSpacing(25);
  mainLayout->addWidget(whiteHorizontalLine());

  static const std::vector<std::tuple<QString, QString, QString, QString>> toggles = {
    {"LateralTune", "Lateral Tuning", "Change the way openpilot steers.", "../frogpilot/assets/toggle_icons/icon_lateral_tune.png"},
    {"LongitudinalTune", "Longitudinal Tuning", "Change the way openpilot accelerates and brakes.", "../frogpilot/assets/toggle_icons/icon_longitudinal_tune.png"},
  };

  for (const auto &[key, label, desc, icon] : toggles) {
    ParamControl *control = createParamControl(key, label, desc, icon, this);
    if (key == "") {
    } else if (key == "LateralTune") {
    } else if (key == "LongitudinalTune") {
      createSubControl(key, label, desc, icon, {
        new AccelerationProfile(),
      });
    } else {
      mainLayout->addWidget(control);
      if (key != std::get<0>(toggles.back())) mainLayout->addWidget(horizontalLine());
    }
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
  };

  for (const auto &[key, label, desc, icon] : toggles) {
    ParamControl *control = createParamControl(key, label, desc, icon, this);
    if (key == "") {
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

    static const QMap<QString, QString> parameterWarnings = {
    };
    if (parameterWarnings.contains(key) && params.getBool(key.toStdString())) {
      ConfirmationDialog::toggleAlert("WARNING: " + parameterWarnings[key], "I understand the risks.", parent);
    }

    static const QSet<QString> parameterReboots = {
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
