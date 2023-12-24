#include <cmath>
#include <filesystem>
#include <unordered_set>

#include "selfdrive/frogpilot/ui/control_settings.h"
#include "selfdrive/ui/ui.h"

FrogPilotControlsPanel::FrogPilotControlsPanel(SettingsWindow *parent) : ListWidget(parent) {
  backButton = new ButtonControl(tr(""), tr("BACK"));
  connect(backButton, &ButtonControl::clicked, [this]() {
    hideSubToggles();
  });
  addItem(backButton);

  const std::vector<std::tuple<QString, QString, QString, QString>> controlToggles {
    {"AdjustablePersonalities", "Adjustable Personalities", "Use the 'Distance' button on the steering wheel or the onroad UI to switch between openpilot's driving personalities.\n\n1 bar = Aggressive\n2 bars = Standard\n3 bars = Relaxed", "../frogpilot/assets/toggle_icons/icon_distance.png"},

    {"AlwaysOnLateral", "Always on Lateral", "Maintain openpilot lateral control when the brake or gas pedals are used.\n\n1Deactivation occurs only through the 'Cruise Control' button.", "../frogpilot/assets/toggle_icons/icon_always_on_lateral.png"},
    {"AlwaysOnLateralMain", "   Enable 'Always on Lateral' On Cruise Main", "Activate 'Always On Lateral' when cruise control is engaged without needing openpilot to be enabled first.", "../frogpilot/assets/toggle_icons/icon_blank.png"},

    {"ConditionalExperimental", "Conditional Experimental Mode", "Automatically switches to 'Experimental Mode' under predefined conditions.", "../frogpilot/assets/toggle_icons/icon_conditional.png"},
    {"CECurves", "Curve Detected Ahead", "Switch to 'Experimental Mode' when a curve is detected.", ""},
    {"CENavigation", "Navigation Based", "Switch to 'Experimental Mode' based on navigation data. (i.e. Intersections, stop signs, etc.)", ""},
    {"CESlowerLead", "Slower Lead Detected Ahead", "Switch to 'Experimental Mode' when a slower lead vehicle is detected ahead.", ""},
    {"CEStopLights", "Stop Lights and Stop Signs", "Switch to 'Experimental Mode' when a stop light or stop sign is detected.", ""},
    {"CESignal", "Turn Signal When Driving Below 55 mph", "Switch to 'Experimental Mode' when using turn signals below 55 mph to help assit with turns.", ""},

    {"CustomPersonalities", "Custom Driving Personalities", "Customize the driving personality profiles to your driving style.", "../frogpilot/assets/toggle_icons/icon_custom.png"},
    {"AggressiveFollow", "Follow Time", "Set the 'Aggressive' personality' following distance. Represents seconds to follow behind the lead vehicle.\n\nStock: 1.25 seconds.", "../frogpilot/assets/other_images/aggressive.png"},
    {"AggressiveJerk", "Jerk Value", "Configure brake/gas pedal responsiveness for the 'Aggressive' personality. Higher values yield a more 'relaxed' response.\n\nStock: 0.5.", "../frogpilot/assets/other_images/aggressive.png"},
    {"StandardFollow", "Follow Time", "Set the 'Standard' personality following distance. Represents seconds to follow behind the lead vehicle.\n\nStock: 1.45 seconds.", "../frogpilot/assets/other_images/standard.png"},
    {"StandardJerk", "Jerk Value", "Adjust brake/gas pedal responsiveness for the 'Standard' personality. Higher values yield a more 'relaxed' response.\n\nStock: 1.0.", "../frogpilot/assets/other_images/standard.png"},
    {"RelaxedFollow", "Follow Time", "Set the 'Relaxed' personality following distance. Represents seconds to follow behind the lead vehicle.\n\nStock: 1.75 seconds.", "../frogpilot/assets/other_images/relaxed.png"},
    {"RelaxedJerk", "Jerk Value", "Set brake/gas pedal responsiveness for the 'Relaxed' personality. Higher values yield a more 'relaxed' response.\n\nStock: 1.0.", "../frogpilot/assets/other_images/relaxed.png"},

    {"DeviceShutdown", "Device Shutdown Timer", "Configure the timer for automatic device shutdown when offroad conserving energy and preventing battery drain.", "../frogpilot/assets/toggle_icons/icon_time.png"},
    {"ExperimentalModeViaPress", "Experimental Mode Via 'LKAS' Button / Screen", "Toggle Experimental Mode by double-clicking the 'Lane Departure'/'LKAS' button or double tapping screen.\n\nOverrides 'Conditional Experimental Mode'.", "../assets/img_experimental_white.svg"},

    {"FireTheBabysitter", "Fire the Babysitter", "Deactivate some of openpilot's 'Babysitter' protocols for more user autonomy.", "../frogpilot/assets/toggle_icons/icon_babysitter.png"},
    {"NoLogging", "Disable All Logging", "Turn off all data tracking to enhance privacy or reduce thermal load.\n\nWARNING: This action will prevent drive recording and data cannot be recovered!", ""},
    {"MuteDM", "Mute Driver Monitoring", "Disable driver monitoring.", ""},
    {"MuteDoor", "Mute Door Open Alert", "Disable alerts for open doors.", ""},
    {"MuteOverheated", "Mute Overheated System Alert", "Disable alerts for the device being overheated.", ""},
    {"MuteSeatbelt", "Mute Seatbelt Unlatched Alert", "Disable alerts for unlatched seatbelts.", ""},

    {"LateralTune", "Lateral Tuning", "Modify openpilot's steering behavior.", "../frogpilot/assets/toggle_icons/icon_lateral_tune.png"},
    {"AverageCurvature", "Average Desired Curvature", "Use Pfeiferj's distance-based curvature adjustment for improved curve handling.", ""},
    {"NNFF", "NNFF - Neural Network Feedforward", "Use Twilsonco's Neural Network Feedforward for enhanced precision in lateral control.", ""},

    {"LongitudinalTune", "Longitudinal Tuning", "Modify openpilot's acceleration and braking behavior.", "../frogpilot/assets/toggle_icons/icon_longitudinal_tune.png"},
    {"AccelerationProfile", "Acceleration Profile", "Change the acceleration rate to be either sporty or eco-friendly.", ""},
    {"AggressiveAcceleration", "Aggressive Acceleration With Lead", "Increase acceleration aggressiveness when following a lead vehicle from a stop.", ""},
    {"SmoothBraking", "Smoother Braking Behind Lead", "Smoothen out the braking behavior when approaching slower vehicles.", ""},
    {"StoppingDistance", "Increased Stopping Distance", "Increase the stopping distance for a more comfortable stop.", ""},

    {"Model", "Model Selector", "Choose your preferred openpilot model.", "../assets/offroad/icon_calibration.png"},
    {"MTSCEnabled", "Map Turn Speed Control", "Slow down for anticipated curves detected by your downloaded maps.", "../frogpilot/assets/toggle_icons/icon_speed_map.png"},

    {"NudgelessLaneChange", "Nudgeless Lane Change", "Enable lane changes without manual steering input.", "../frogpilot/assets/toggle_icons/icon_lane.png"},
    {"LaneChangeTime", "Lane Change Timer", "Specify a delay before executing a nudgeless lane change.", ""},
    {"LaneDetection", "Lane Detection", "Block nudgeless lane changes when a lane isn't detected.", ""},
    {"OneLaneChange", "One Lane Change Per Signal", "Limit to one nudgeless lane change per turn signal activation.", ""},
    {"PauseLateralOnSignal", "Pause Lateral On Turn Signal", "Temporarily disable lateral control during turn signal use.", ""},

    {"SpeedLimitController", "Speed Limit Controller", "Automatically adjust vehicle speed to match speed limits using 'Open Street Map's, 'Navigate On openpilot', or your car's dashboard (TSS2 Toyotas only).", "../assets/offroad/icon_speed_limit.png"},
    {"Offset1", "Speed Limit Offset (0-34 mph)", "Speed limit offset for speed limits between 0-34 mph.", ""},
    {"Offset2", "Speed Limit Offset (35-54 mph)", "Speed limit offset for speed limits between 35-54 mph.", ""},
    {"Offset3", "Speed Limit Offset (55-64 mph)", "Speed limit offset for speed limits between 55-64 mph.", ""},
    {"Offset4", "Speed Limit Offset (65-99 mph)", "Speed limit offset for speed limits between 65-99 mph.", ""},
    {"SLCFallback", "Fallback Method", "Choose your fallback method for when there are no speed limits currently being obtained from Navigation, OSM, and the car's dashboard.", ""},
    {"SLCPriority", "Speed Limit Priority", "Determine the priority order for what speed limits to use.", ""},

    {"TurnDesires", "Use Turn Desires", "Use turn desires for enhanced precision in turns below the minimum lane change speed.", "../assets/navigation/direction_continue_right.png"},

    {"VisionTurnControl", "Vision Turn Speed Controller", "Slow down for detected road curvature for smoother curve handling.", "../frogpilot/assets/toggle_icons/icon_vtc.png"},
    {"CurveSensitivity", "Curve Detection Sensitivity", "Set curve detection sensitivity. Higher values prompt earlier responses, lower values lead to smoother but later reactions.", ""},
    {"TurnAggressiveness", "Turn Speed Aggressiveness", "Set turn speed aggressiveness. Higher values result in faster turns, lower values yield gentler turns.", ""},
  };

  for (const auto &[param, title, desc, icon] : controlToggles) {
    ParamControl *toggle;

    if (param == "AdjustablePersonalities") {
      toggle = new ParamValueControl(param, title, desc, icon, 0, 3, {{0, "None"}, {1, "Steering Wheel"}, {2, "Onroad UI Button"}, {3, "Wheel + Button"}}, this, true);

    } else if (param == "ConditionalExperimental") {
      ParamManageControl *conditionalExperimentalToggle = new ParamManageControl(param, title, desc, icon, this);
      connect(conditionalExperimentalToggle, &ParamManageControl::manageButtonClicked, this, [this]() {
        parentToggleClicked();
        conditionalSpeedsImperial->setVisible(!isMetric);
        conditionalSpeedsMetric->setVisible(isMetric);
        for (auto &[key, toggle] : toggles) {
          toggle->setVisible(conditionalExperimentalKeys.find(key.c_str()) != conditionalExperimentalKeys.end());
        }
      });
      toggle = conditionalExperimentalToggle;

    } else if (param == "CECurves") {
      ParamValueControl *CESpeedImperial = new ParamValueControl("CESpeed", "Below", "Switch to 'Experimental Mode' below this speed in absence of a lead vehicle.", "", 0, 99, std::map<int, QString>(), this, false, " mph");
      ParamValueControl *CESpeedLeadImperial = new ParamValueControl("CESpeedLead", "Lead", "Switch to 'Experimental Mode' below this speed when following a lead vehicle.", "", 0, 99, std::map<int, QString>(), this, false, " mph");
      conditionalSpeedsImperial = new DualParamValueControl(CESpeedImperial, CESpeedLeadImperial, this);
      addItem(conditionalSpeedsImperial);

      ParamValueControl *CESpeedMetric = new ParamValueControl("CESpeed", "Below", "Switch to 'Experimental Mode' below this speed in absence of a lead vehicle.", "", 0, 99, std::map<int, QString>(), this, false, " kph");
      ParamValueControl *CESpeedLeadMetric = new ParamValueControl("CESpeedLead", "W/ Lead", "Switch to 'Experimental Mode' below this speed when following a lead vehicle.", "", 0, 99, std::map<int, QString>(), this, false, " kph");
      conditionalSpeedsMetric = new DualParamValueControl(CESpeedMetric, CESpeedLeadMetric, this);
      addItem(conditionalSpeedsMetric);

      std::vector<QString> curveToggles{tr("CECurvesLead")};
      std::vector<QString> curveToggleNames{tr("With Lead")};
      toggle = new ParamToggleControl("CECurves", tr("Curve Detected Ahead"), tr("Switch to 'Experimental Mode' when a curve is detected."), "", curveToggles, curveToggleNames);
    } else if (param == "CEStopLights") {
      std::vector<QString> stopLightToggles{tr("CEStopLightsLead")};
      std::vector<QString> stopLightToggleNames{tr("With Lead")};
      toggle = new ParamToggleControl("CEStopLights", tr("Stop Lights and Stop Signs"), tr("Switch to 'Experimental Mode' when a stop light or stop sign is detected."), "", stopLightToggles, stopLightToggleNames);

    } else if (param == "CustomPersonalities") {
      ParamManageControl *customPersonalitiesToggle = new ParamManageControl(param, title, desc, icon, this);
      connect(customPersonalitiesToggle, &ParamManageControl::manageButtonClicked, this, [this]() {
        parentToggleClicked();
        for (auto &[key, toggle] : toggles) {
          toggle->setVisible(customPersonalitiesKeys.find(key.c_str()) != customPersonalitiesKeys.end());
        }
      });
      toggle = customPersonalitiesToggle;
    } else if (param == "AggressiveFollow" || param == "StandardFollow" || param == "RelaxedFollow") {
      toggle = new ParamValueControl(param, title, desc, icon, 10, 50, std::map<int, QString>(), this, false, " seconds", 10);
    } else if (param == "AggressiveJerk" || param == "StandardJerk" || param == "RelaxedJerk") {
      toggle = new ParamValueControl(param, title, desc, icon, 1, 50, std::map<int, QString>(), this, false, "", 10);

    } else if (param == "DeviceShutdown") {
      std::map<int, QString> shutdownLabels;
      for (int i = 0; i <= 33; ++i) {
        shutdownLabels[i] = i == 0 ? "Instant" : i <= 3 ? QString::number(i * 15) + " mins" : QString::number(i - 3) + (i == 4 ? " hour" : " hours");
      }
      toggle = new ParamValueControl(param, title, desc, icon, 0, 33, shutdownLabels, this, false);

    } else if (param == "FireTheBabysitter") {
      ParamManageControl *fireTheBabysitterToggle = new ParamManageControl(param, title, desc, icon, this);
      connect(fireTheBabysitterToggle, &ParamManageControl::manageButtonClicked, this, [this]() {
        parentToggleClicked();
        for (auto &[key, toggle] : toggles) {
          toggle->setVisible(fireTheBabysitterKeys.find(key.c_str()) != fireTheBabysitterKeys.end());
        }
      });
      toggle = fireTheBabysitterToggle;

    } else if (param == "LateralTune") {
      ParamManageControl *lateralTuneToggle = new ParamManageControl(param, title, desc, icon, this);
      connect(lateralTuneToggle, &ParamManageControl::manageButtonClicked, this, [this]() {
        parentToggleClicked();
        for (auto &[key, toggle] : toggles) {
          toggle->setVisible(lateralTuneKeys.find(key.c_str()) != lateralTuneKeys.end());
        }
      });
      toggle = lateralTuneToggle;

    } else if (param == "LongitudinalTune") {
      ParamManageControl *longitudinalTuneToggle = new ParamManageControl(param, title, desc, icon, this);
      connect(longitudinalTuneToggle, &ParamManageControl::manageButtonClicked, this, [this]() {
        parentToggleClicked();
        for (auto &[key, toggle] : toggles) {
          toggle->setVisible(longitudinalTuneKeys.find(key.c_str()) != longitudinalTuneKeys.end());
        }
      });
      toggle = longitudinalTuneToggle;
    } else if (param == "AccelerationProfile") {
      toggle = new ParamValueControl(param, title, desc, icon, 0, 2, {{0, "Standard"}, {1, "Eco"}, {2, "Sport"}}, this, true);
    } else if (param == "StoppingDistance") {
      toggle = new ParamValueControl(param, title, desc, icon, 0, 10, std::map<int, QString>(), this, false, " feet");

    } else if (param == "Model") {
      modelSelectorButton = new ButtonIconControl(tr("Model Selector"), tr("SELECT"), tr("Select your preferred openpilot model."), "../assets/offroad/icon_calibration.png");
      const QStringList models = {"Blue Diamond V2", "Blue Diamond V1", "Farmville", "New Lemon Pie", "New Delhi"};
      connect(modelSelectorButton, &ButtonIconControl::clicked, this, [this, models]() {
        const int currentModel = params.getInt("Model");
        const QString currentModelLabel = models[currentModel];

        const QString selection = MultiOptionDialog::getSelection(tr("Select a driving model"), models, currentModelLabel, this);
        if (!selection.isEmpty()) {
          const int selectedModel = models.indexOf(selection);
          params.putInt("Model", selectedModel);
          params.remove("CalibrationParams");
          params.remove("LiveTorqueParameters");
          modelSelectorButton->setValue(selection);
          if (ConfirmationDialog::toggle("Reboot required to take effect.", "Reboot Now", this)) {
            Hardware::reboot();
          }
        }
      });
      modelSelectorButton->setValue(models[params.getInt("Model")]);
      addItem(modelSelectorButton);

    } else if (param == "NudgelessLaneChange") {
      ParamManageControl *laneChangeToggle = new ParamManageControl(param, title, desc, icon, this);
      connect(laneChangeToggle, &ParamManageControl::manageButtonClicked, this, [this]() {
        parentToggleClicked();
        for (auto &[key, toggle] : toggles) {
          toggle->setVisible(laneChangeKeys.find(key.c_str()) != laneChangeKeys.end());
        }
      });
      toggle = laneChangeToggle;
    } else if (param == "LaneChangeTime") {
      std::map<int, QString> laneChangeTimeLabels;
      for (int i = 0; i <= 10; ++i) {
        laneChangeTimeLabels[i] = i == 0 ? "Instant" : QString::number(i / 2.0) + " seconds";
      }
      toggle = new ParamValueControl(param, title, desc, icon, 0, 10, laneChangeTimeLabels, this, false);

    } else if (param == "SpeedLimitController") {
      ParamManageControl *speedLimitControllerToggle = new ParamManageControl(param, title, desc, icon, this);
      connect(speedLimitControllerToggle, &ParamManageControl::manageButtonClicked, this, [this]() {
        parentToggleClicked();
        slscPriorityButton->setVisible(true);
        for (auto &[key, toggle] : toggles) {
          toggle->setVisible(speedLimitControllerKeys.find(key.c_str()) != speedLimitControllerKeys.end());
        }
      });
      toggle = speedLimitControllerToggle;
    } else if (param == "Offset1" || param == "Offset2" || param == "Offset3" || param == "Offset4") {
      toggle = new ParamValueControl(param, title, desc, icon, 0, 99, std::map<int, QString>(), this, false, " mph");
    } else if (param == "SLCFallback") {
      toggle = new ParamValueControl(param, title, desc, icon, 0, 2, {{0, "None"}, {1, "Experimental Mode"}, {2, "Previous Speed Limit"}}, this, true);
    } else if (param == "SLCPriority") {
      const QStringList priorities {
        "Navigation, Dashboard, Offline Maps",
        "Navigation, Offline Maps, Dashboard",
        "Navigation, Offline Maps",
        "Navigation, Dashboard",
        "Navigation",
        "Offline Maps, Dashboard, Navigation",
        "Offline Maps, Navigation, Dashboard",
        "Offline Maps, Navigation",
        "Offline Maps, Dashboard",
        "Offline Maps",
        "Dashboard, Navigation, Offline Maps",
        "Dashboard, Offline Maps, Navigation",
        "Dashboard, Offline Maps",
        "Dashboard, Navigation",
        "Dashboard",
        "Highest",
        "Lowest",
        "",
      };

      slscPriorityButton = new ButtonControl(tr("Priority Order"), tr("SELECT"), tr("Determine priority order for selecting speed limits with 'Speed Limit Controller'."));
      connect(slscPriorityButton, &ButtonControl::clicked, this, [this, priorities]() {
        QStringList availablePriorities = {"Dashboard", "Navigation", "Offline Maps", "Highest", "Lowest", "None"};
        QStringList selectedPriorities;
        int priorityValue = -1;

        const QStringList priorityPrompts = {tr("Select your primary priority"), tr("Select your secondary priority"), tr("Select your tertiary priority")};

        for (int i = 0; i < 3; ++i) {
          const QString selection = MultiOptionDialog::getSelection(priorityPrompts[i], availablePriorities, "", this);
          if (selection.isEmpty()) break;

          if (selection == "None") {
            priorityValue = 17;
            break;
          } else if (selection == "Highest") {
            priorityValue = 15;
            break;
          } else if (selection == "Lowest") {
            priorityValue = 16;
            break;
          } else {
            selectedPriorities.append(selection);
            availablePriorities.removeAll(selection);
            availablePriorities.removeAll("Highest");
            availablePriorities.removeAll("Lowest");
          }
        }

        if (priorityValue == -1 && !selectedPriorities.isEmpty()) {
          QString priorityString = selectedPriorities.join(", ");
          priorityValue = priorities.indexOf(priorityString);
        }

        if (priorityValue != -1) {
          params.putInt("SLCPriority", priorityValue);
          slscPriorityButton->setValue(priorities[priorityValue]);
        }
      });
      slscPriorityButton->setValue(priorities[params.getInt("SLCPriority")]);
      addItem(slscPriorityButton);

    } else if (param == "VisionTurnControl") {
      ParamManageControl *visionTurnControlToggle = new ParamManageControl(param, title, desc, icon, this);
      connect(visionTurnControlToggle, &ParamManageControl::manageButtonClicked, this, [this]() {
        parentToggleClicked();
        for (auto &[key, toggle] : toggles) {
          toggle->setVisible(visionTurnControlKeys.find(key.c_str()) != visionTurnControlKeys.end());
        }
      });
      toggle = visionTurnControlToggle;
    } else if (param == "CurveSensitivity" || param == "TurnAggressiveness") {
      toggle = new ParamValueControl(param, title, desc, icon, 1, 200, std::map<int, QString>(), this, false, "%");

    } else {
      toggle = new ParamControl(param, title, desc, icon, this);
    }

    addItem(toggle);
    toggles[param.toStdString()] = toggle;

    connect(toggles["AlwaysOnLateral"], &ToggleControl::toggleFlipped, [this](bool state) {
      if (toggles.find("AlwaysOnLateralMain") != toggles.end()) {
        toggles["AlwaysOnLateralMain"]->setVisible(state);
      }
    });

    connect(toggle, &ToggleControl::toggleFlipped, [this]() {
      paramsMemory.putBool("FrogPilotTogglesUpdated", true);
    });

    connect(dynamic_cast<ParamValueControl*>(toggle), &ParamValueControl::buttonPressed, [this]() {
      paramsMemory.putBool("FrogPilotTogglesUpdated", true);
    });
  }

  conditionalExperimentalKeys = {"CECurves", "CECurvesLead", "CESlowerLead", "CENavigation", "CEStopLights", "CESignal"};
  customPersonalitiesKeys = {"AggressiveFollow", "AggressiveJerk", "StandardFollow", "StandardJerk", "RelaxedFollow", "RelaxedJerk"};
  fireTheBabysitterKeys = {"NoLogging", "MuteDM", "MuteDoor", "MuteOverheated", "MuteSeatbelt"};
  laneChangeKeys = {"LaneChangeTime", "LaneDetection", "OneLaneChange", "PauseLateralOnSignal"};
  lateralTuneKeys = {"AverageCurvature", "NNFF"};
  longitudinalTuneKeys = {"AccelerationProfile", "AggressiveAcceleration", "SmoothBraking", "StoppingDistance"};
  speedLimitControllerKeys = {"Offset1", "Offset2", "Offset3", "Offset4", "SLCFallback", "SLCPriority"};
  visionTurnControlKeys = {"CurveSensitivity", "TurnAggressiveness"};

  QObject::connect(uiState(), &UIState::uiUpdate, this, &FrogPilotControlsPanel::updateMetric);

  hideSubToggles();
  setDefaults();
}

void FrogPilotControlsPanel::updateMetric() {
  std::thread([this] {
    static bool checkedOnBoot = false;

    bool previousIsMetric = isMetric;
    isMetric = params.getBool("IsMetric");

    if (checkedOnBoot) {
      if (previousIsMetric == isMetric) return;
    }
    checkedOnBoot = true;

    if (isMetric != previousIsMetric) {
      const double distanceConversion = isMetric ? FOOT_TO_METER : METER_TO_FOOT;
      const double speedConversion = isMetric ? MILE_TO_KM : KM_TO_MILE;
      params.putInt("CESpeed", std::nearbyint(params.getInt("CESpeed") * speedConversion));
      params.putInt("CESpeedLead", std::nearbyint(params.getInt("CESpeedLead") * speedConversion));
      params.putInt("Offset1", std::nearbyint(params.getInt("Offset1") * speedConversion));
      params.putInt("Offset2", std::nearbyint(params.getInt("Offset2") * speedConversion));
      params.putInt("Offset3", std::nearbyint(params.getInt("Offset3") * speedConversion));
      params.putInt("Offset4", std::nearbyint(params.getInt("Offset4") * speedConversion));
      params.putInt("StoppingDistance", std::nearbyint(params.getInt("StoppingDistance") * distanceConversion));
    }

    ParamControl *ceSignalToggle = toggles["CESignal"];
    ParamValueControl *offset1Toggle = dynamic_cast<ParamValueControl*>(toggles["Offset1"]);
    ParamValueControl *offset2Toggle = dynamic_cast<ParamValueControl*>(toggles["Offset2"]);
    ParamValueControl *offset3Toggle = dynamic_cast<ParamValueControl*>(toggles["Offset3"]);
    ParamValueControl *offset4Toggle = dynamic_cast<ParamValueControl*>(toggles["Offset4"]);
    ParamValueControl *stoppingDistanceToggle = dynamic_cast<ParamValueControl*>(toggles["StoppingDistance"]);

    if (isMetric) {
      ceSignalToggle->setTitle("Turn Signal When Driving Below 90 kph");
      offset1Toggle->setTitle("Speed Limit Offset (0-34 kph)");
      offset2Toggle->setTitle("Speed Limit Offset (35-54 kph)");
      offset3Toggle->setTitle("Speed Limit Offset (55-64 kph)");
      offset4Toggle->setTitle("Speed Limit Offset (65-99 kph)");

      ceSignalToggle->setDescription("Switch to 'Experimental Mode' when using turn signals below 90 kph to help assit with turns.");
      offset1Toggle->setDescription("Set speed limit offset for limits between 0-34 kph.");
      offset2Toggle->setDescription("Set speed limit offset for limits between 35-54 kph.");
      offset3Toggle->setDescription("Set speed limit offset for limits between 55-64 kph.");
      offset4Toggle->setDescription("Set speed limit offset for limits between 65-99 kph.");

      offset1Toggle->updateControl(0, 99, " kph");
      offset2Toggle->updateControl(0, 99, " kph");
      offset3Toggle->updateControl(0, 99, " kph");
      offset4Toggle->updateControl(0, 99, " kph");
      stoppingDistanceToggle->updateControl(0, 5, " meters");
    } else {
      ceSignalToggle->setTitle("Turn Signal When Driving Below 55 mph");
      offset1Toggle->setTitle("Speed Limit Offset (0-34 mph)");
      offset2Toggle->setTitle("Speed Limit Offset (35-54 mph)");
      offset3Toggle->setTitle("Speed Limit Offset (55-64 mph)");
      offset4Toggle->setTitle("Speed Limit Offset (65-99 mph)");

      ceSignalToggle->setDescription("Switch to 'Experimental Mode' when using turn signals below 55 mph to help assit with turns.");
      offset1Toggle->setDescription("Set speed limit offset for limits between 0-34 mph.");
      offset2Toggle->setDescription("Set speed limit offset for limits between 35-54 mph.");
      offset3Toggle->setDescription("Set speed limit offset for limits between 55-64 mph.");
      offset4Toggle->setDescription("Set speed limit offset for limits between 65-99 mph.");

      offset1Toggle->updateControl(0, 99, " mph");
      offset2Toggle->updateControl(0, 99, " mph");
      offset3Toggle->updateControl(0, 99, " mph");
      offset4Toggle->updateControl(0, 99, " mph");
      stoppingDistanceToggle->updateControl(0, 10, " feet");
    }

    ceSignalToggle->refresh();
    offset1Toggle->refresh();
    offset2Toggle->refresh();
    offset3Toggle->refresh();
    offset4Toggle->refresh();
    stoppingDistanceToggle->refresh();

    previousIsMetric = isMetric;
  }).detach();
}

void FrogPilotControlsPanel::parentToggleClicked() {
  backButton->setVisible(true);
  conditionalSpeedsImperial->setVisible(false);
  conditionalSpeedsMetric->setVisible(false);
  modelSelectorButton->setVisible(false);
  slscPriorityButton->setVisible(false);
}

void FrogPilotControlsPanel::hideSubToggles() {
  backButton->setVisible(false);
  conditionalSpeedsImperial->setVisible(false);
  conditionalSpeedsMetric->setVisible(false);
  modelSelectorButton->setVisible(true);
  slscPriorityButton->setVisible(false);

  for (auto &[key, toggle] : toggles) {
    const bool subToggles = conditionalExperimentalKeys.find(key.c_str()) != conditionalExperimentalKeys.end() ||
                            customPersonalitiesKeys.find(key.c_str()) != customPersonalitiesKeys.end() ||
                            fireTheBabysitterKeys.find(key.c_str()) != fireTheBabysitterKeys.end() ||
                            laneChangeKeys.find(key.c_str()) != laneChangeKeys.end() ||
                            lateralTuneKeys.find(key.c_str()) != lateralTuneKeys.end() ||
                            longitudinalTuneKeys.find(key.c_str()) != longitudinalTuneKeys.end() ||
                            speedLimitControllerKeys.find(key.c_str()) != speedLimitControllerKeys.end() ||
                            visionTurnControlKeys.find(key.c_str()) != visionTurnControlKeys.end();
    toggle->setVisible(!subToggles);
    if (key == "AlwaysOnLateralMain") {
      toggle->setVisible(params.getBool("AlwaysOnLateral"));
    }
  }
}

void FrogPilotControlsPanel::hideEvent(QHideEvent *event) {
  hideSubToggles();
}

void FrogPilotControlsPanel::setDefaults() {
  const bool FrogsGoMoo = params.get("DongleId").substr(0, 3) == "be6";

  const std::map<std::string, std::string> defaultValues {
    {"AccelerationProfile", "2"},
    {"AdjustablePersonalities", "3"},
    {"AggressiveAcceleration", "1"},
    {"AggressiveFollow", FrogsGoMoo ? "10" : "12"},
    {"AggressiveJerk", FrogsGoMoo ? "6" : "5"},
    {"AlwaysOnLateral", "1"},
    {"AlwaysOnLateralMain", FrogsGoMoo ? "1" : "0"},
    {"AverageCurvature", FrogsGoMoo ? "1" : "0"},
    {"CECurves", "1"},
    {"CECurvesLead", "0"},
    {"CENavigation", "1"},
    {"CESignal", "1"},
    {"CESlowerLead", "0"},
    {"CESpeed", "0"},
    {"CESpeedLead", "0"},
    {"CEStopLights", "1"},
    {"CEStopLightsLead", FrogsGoMoo ? "0" : "1"},
    {"ConditionalExperimental", "1"},
    {"CurveSensitivity", FrogsGoMoo ? "125" : "100"},
    {"DeviceShutdown", "9"},
    {"ExperimentalModeViaPress", "1"},
    {"FireTheBabysitter", FrogsGoMoo ? "1" : "0"},
    {"LaneChangeTime", "0"},
    {"LaneDetection", "1"},
    {"LateralTune", "1"},
    {"LongitudinalTune", "1"},
    {"MTSCEnabled", "1"},
    {"MuteDM", FrogsGoMoo ? "1" : "0"},
    {"MuteDoor", FrogsGoMoo ? "1" : "0"},
    {"MuteOverheated", FrogsGoMoo ? "1" : "0"},
    {"MuteSeatbelt", FrogsGoMoo ? "1" : "0"},
    {"NNFF", FrogsGoMoo ? "1" : "0"},
    {"NudgelessLaneChange", "1"},
    {"Offset1", "5"},
    {"Offset2", FrogsGoMoo ? "7" : "5"},
    {"Offset3", "5"},
    {"Offset4", FrogsGoMoo ? "20" : "10"},
    {"OneLaneChange", "1"},
    {"PauseLateralOnSignal", "0"},
    {"RelaxedFollow", "30"},
    {"RelaxedJerk", "50"},
    {"SLCFallback", "2"},
    {"SLCPriority", "1"},
    {"SmoothBraking", "1"},
    {"SpeedLimitController", "1"},
    {"StandardFollow", "15"},
    {"StandardJerk", "10"},
    {"StoppingDistance", FrogsGoMoo ? "6" : "3"},
    {"TurnAggressiveness", FrogsGoMoo ? "150" : "100"},
    {"TurnDesires", "1"},
    {"VisionTurnControl", "1"},
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
