#include <filesystem>

#include "selfdrive/frogpilot/ui/frogpilot_functions.h"
#include "selfdrive/ui/ui.h"

bool FrogPilotConfirmationDialog::toggle(const QString &prompt_text, const QString &confirm_text, QWidget *parent) {
  ConfirmationDialog d = ConfirmationDialog(prompt_text, confirm_text, tr("Reboot Later"), false, parent);
  return d.exec();
}

bool FrogPilotConfirmationDialog::toggleAlert(const QString &prompt_text, const QString &button_text, QWidget *parent) {
  ConfirmationDialog d = ConfirmationDialog(prompt_text, button_text, "", false, parent);
  return d.exec();
}

bool FrogPilotConfirmationDialog::yesorno(const QString &prompt_text, QWidget *parent) {
  ConfirmationDialog d = ConfirmationDialog(prompt_text, tr("Yes"), tr("No"), false, parent);
  return d.exec();
}

FrogPilotButtonIconControl::FrogPilotButtonIconControl(const QString &title, const QString &text, const QString &desc, const QString &icon, QWidget *parent) : AbstractControl(title, desc, icon, parent) {
  btn.setText(text);
  btn.setStyleSheet(R"(
    QPushButton {
      padding: 0;
      border-radius: 50px;
      font-size: 35px;
      font-weight: 500;
      color: #E4E4E4;
      background-color: #393939;
    }
    QPushButton:pressed {
      background-color: #4a4a4a;
    }
    QPushButton:disabled {
      color: #33E4E4E4;
    }
  )");
  btn.setFixedSize(250, 100);
  QObject::connect(&btn, &QPushButton::clicked, this, &FrogPilotButtonIconControl::clicked);
  hlayout->addWidget(&btn);
}

void setDefaultParams() {
  Params params = Params();
  bool FrogsGoMoo = params.get("DongleId").substr(0, 3) == "be6";

  std::map<std::string, std::string> defaultValues {
    {"AccelerationPath", FrogsGoMoo ? "1" : "0"},
    {"AccelerationProfile", "2"},
    {"AdjacentPath", FrogsGoMoo ? "1" : "0"},
    {"AdjustablePersonalities", "3"},
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
    {"CEStopLightsLead", FrogsGoMoo ? "0" : "1"},
    {"Compass", FrogsGoMoo ? "1" : "0"},
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
    {"EVTable", FrogsGoMoo ? "0" : "1"},
    {"ExperimentalModeViaPress", "1"},
    {"FireTheBabysitter", FrogsGoMoo ? "1" : "0"},
    {"GreenLightAlert", "0"},
    {"LaneChangeTime", "0"},
    {"LaneDetection", "1"},
    {"LaneLinesWidth", "4"},
    {"LateralTune", "1"},
    {"LeadInfo", FrogsGoMoo ? "1" : "0"},
    {"LockDoors", "0"},
    {"LongitudinalTune", "1"},
    {"LongPitch", FrogsGoMoo ? "0" : "1"},
    {"LowerVolt", FrogsGoMoo ? "0" : "1"},
    {"ModelUI", "1"},
    {"MTSCEnabled", "1"},
    {"MuteDM", FrogsGoMoo ? "1" : "0"},
    {"MuteDoor", FrogsGoMoo ? "1" : "0"},
    {"MuteOverheated", FrogsGoMoo ? "1" : "0"},
    {"MuteSeatbelt", FrogsGoMoo ? "1" : "0"},
    {"NNFF", FrogsGoMoo ? "1" : "0"},
    {"NudgelessLaneChange", "1"},
    {"NumericalTemp", FrogsGoMoo ? "1" : "0"},
    {"Offset1", "5"},
    {"Offset2", FrogsGoMoo ? "7" : "5"},
    {"Offset3", "5"},
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
    {"ScreenBrightness", "101"},
    {"ShowCPU", FrogsGoMoo ? "1" : "0"},
    {"ShowFPS", FrogsGoMoo ? "1" : "0"},
    {"ShowMemoryUsage", FrogsGoMoo ? "1" : "0"},
    {"Sidebar", FrogsGoMoo ? "1" : "0"},
    {"SilentMode", "0"},
    {"SLCFallback", "2"},
    {"SLCOverride", FrogsGoMoo ? "2" : "1"},
    {"SLCPriority", "1"},
    {"SmoothBraking", "1"},
    {"SNGHack", FrogsGoMoo ? "0" : "1"},
    {"SpeedLimitController", "1"},
    {"StandardFollow", "15"},
    {"StandardJerk", "10"},
    {"StoppingDistance", FrogsGoMoo ? "6" : "3"},
    {"TSS2Tune", "1"},
    {"TurnAggressiveness", FrogsGoMoo ? "150" : "100"},
    {"TurnDesires", "1"},
    {"UnlimitedLength", "1"},
    {"UseSI", FrogsGoMoo ? "1" : "0"},
    {"VisionTurnControl", "1"},
    {"WheelIcon", FrogsGoMoo ? "1" : "0"}
  };

  bool rebootRequired = false;
  for (const auto &[key, value] : defaultValues) {
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
