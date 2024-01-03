#include "selfdrive/frogpilot/ui/visual_settings.h"
#include "selfdrive/ui/ui.h"

FrogPilotVisualsPanel::FrogPilotVisualsPanel(SettingsWindow *parent) : FrogPilotListWidget(parent) {
  const std::vector<std::tuple<QString, QString, QString, QString>> visualToggles {
    {"CustomTheme", "Custom Themes", "Enable the ability to use custom themes.", "../frogpilot/assets/wheel_images/frog.png"},
    {"CustomColors", "Custom Colors", "Switch out the standard openpilot color scheme with a custom color scheme.\n\nWant to submit your own color scheme? Post it in the 'feature-request' channel in the FrogPilot Discord!", ""},
    {"CustomIcons", "Custom Icons", "Switch out the standard openpilot icons with a set of custom icons.\n\nWant to submit your own icon pack? Post it in the 'feature-request' channel in the FrogPilot Discord!", ""},
    {"CustomSignals", "Custom Signals", "Add custom animations for your turn signals for a personal touch!\n\nWant to submit your own turn signal animation? Post it in the 'feature-request' channel in the FrogPilot Discord!", ""},
    {"CustomSounds", "Custom Sounds", "Switch out the standard openpilot sounds with a set of custom sounds.\n\nWant to submit your own sound pack? Post it in the 'feature-request' channel in the FrogPilot Discord!", ""},

    {"CameraView", "Camera View (Cosmetic Only)", "Choose your preferred camera view for the onroad UI. This is a visual change only and doesn't impact openpilot.", "../frogpilot/assets/toggle_icons/icon_camera.png"},
    {"Compass", "Compass", "Add a compass to your onroad UI.", "../frogpilot/assets/toggle_icons/icon_compass.png"},

    {"CustomUI", "Custom Onroad UI", "Customize the Onroad UI with some additional visual functions.", "../assets/offroad/icon_road.png"},
    {"AdjacentPath", "Adjacent Paths", "Display paths to the left and right of your car, visualizing where the model detects lanes.", ""},
    {"BlindSpotPath", "Blind Spot Path", "Visualize your blind spots with a red path when another vehicle is detected nearby.", ""},
    {"ShowFPS", "FPS Counter", "Display the Frames Per Second (FPS) of your onroad UI for monitoring system performance.", ""},
    {"LeadInfo", "Lead Info and Logics", "Get detailed information about the vehicle ahead, including speed and distance, and the logic behind your following distance.", ""},
    {"RoadNameUI", "Road Name", "See the name of the road you're on at the bottom of your screen. Sourced from OpenStreetMap.", ""},

    {"DriverCamera", "Driver Camera On Reverse", "Show the driver's camera feed when you shift to reverse.", "../assets/img_driver_face_static.png"},
    {"GreenLightAlert", "Green Light Alert", "Get an alert when a traffic light changes from red to green.", "../frogpilot/assets/toggle_icons/icon_green_light.png"},

    {"ModelUI", "Model UI", "Personalize how the model's visualizations appear on your screen.", "../assets/offroad/icon_calibration.png"},
    {"AccelerationPath", "Acceleration Path", "Visualize the car's intended acceleration or deceleration with a color-coded path.", ""},
    {"LaneLinesWidth", "Lane Lines", "Adjust the visual thickness of lane lines on your display.\n\nDefault matches the MUTCD average of 4 inches.", ""},
    {"PathEdgeWidth", "Path Edges", "Adjust the width of the path edges shown on your UI to represent different driving modes and statuses.\n\nDefault is 20% of the total path.\n\nBlue = Navigation\nLight Blue = Always On Lateral\nGreen = Default with 'FrogPilot Colors'\nLight Green = Default with stock colors\nOrange = Experimental Mode Active\nYellow = Conditional Overriden", ""},
    {"PathWidth", "Path Width", "Customize the width of the driving path shown on your UI.\n\nDefault matches the width of a 2019 Lexus ES 350.", ""},
    {"RoadEdgesWidth", "Road Edges", "Adjust the visual thickness of road edges on your display.\n\nDefault is 1/2 of the MUTCD average lane line width of 4 inches.", ""},
    {"UnlimitedLength", "'Unlimited' Road UI Length", "Extend the display of the path, lane lines, and road edges as far as the system can detect, providing a more expansive view of the road ahead.", ""},

    {"ScreenBrightness", "Screen Brightness", "Customize your screen brightness.", "../frogpilot/assets/toggle_icons/icon_light.png"},
    {"SilentMode", "Silent Mode", "Mute openpilot sounds for a quieter driving experience.", "../frogpilot/assets/toggle_icons/icon_mute.png"},
    {"WheelIcon", "Steering Wheel Icon", "Replace the default steering wheel icon with a custom design, adding a unique touch to your interface.", "../assets/offroad/icon_openpilot.png"},
  };

  for (const auto &[param, title, desc, icon] : visualToggles) {
    ParamControl *toggle;

    if (param == "CameraView") {
      toggle = new ParamValueControl(param, title, desc, icon, 0, 3, {{0, "Auto"}, {1, "Standard"}, {2, "Wide"}, {3, "Driver"}}, this, true);

    } else if (param == "CustomTheme") {
      ParamManageControl *customThemeToggle = new ParamManageControl(param, title, desc, icon, this);
      QObject::connect(customThemeToggle, &ParamManageControl::manageButtonClicked, this, [this]() {
        parentToggleClicked();
        for (auto &[key, toggle] : toggles) {
          toggle->setVisible(customThemeKeys.find(key.c_str()) != customThemeKeys.end());
        }
      });
      toggle = customThemeToggle;
    } else if (param == "CustomColors" || param == "CustomIcons" || param == "CustomSignals" || param == "CustomSounds") {
      std::map<int, QString> themeLabels = {{0, "Stock"}, {1, "Frog"}, {2, "Tesla"}, {3, "Stalin"}};
      toggle = new ParamValueControl(param, title, desc, icon, 0, 3, themeLabels, this, true);

    } else if (param == "CustomUI") {
      ParamManageControl *customUIToggle = new ParamManageControl(param, title, desc, icon, this);
      QObject::connect(customUIToggle, &ParamManageControl::manageButtonClicked, this, [this]() {
        parentToggleClicked();
        for (auto &[key, toggle] : toggles) {
          toggle->setVisible(customOnroadUIKeys.find(key.c_str()) != customOnroadUIKeys.end());
        }
      });
      toggle = customUIToggle;
    } else if (param == "LeadInfo") {
      std::vector<QString> leadInfoToggles{tr("UseSI")};
      std::vector<QString> leadInfoToggleNames{tr("Use SI Values")};

      toggle = new ParamToggleControl("LeadInfo", tr("Lead Info and Logics"), 
                                      tr("Get detailed information about the vehicle ahead, including speed and distance, and the logic behind your following distance."), 
                                      "", leadInfoToggles, leadInfoToggleNames);

    } else if (param == "ModelUI") {
      ParamManageControl *modelUIToggle = new ParamManageControl(param, title, desc, icon, this);
      QObject::connect(modelUIToggle, &ParamManageControl::manageButtonClicked, this, [this]() {
        parentToggleClicked();
        for (auto &[key, toggle] : toggles) {
          toggle->setVisible(modelUIKeys.find(key.c_str()) != modelUIKeys.end());
        }
      });
      toggle = modelUIToggle;
    } else if (param == "LaneLinesWidth" || param == "RoadEdgesWidth") {
      toggle = new ParamValueControl(param, title, desc, icon, 0, 24, std::map<int, QString>(), this, false, " inches");
    } else if (param == "PathEdgeWidth") {
      toggle = new ParamValueControl(param, title, desc, icon, 0, 100, std::map<int, QString>(), this, false, "%");
    } else if (param == "PathWidth") {
      toggle = new ParamValueControl(param, title, desc, icon, 0, 100, std::map<int, QString>(), this, false, " feet", 10);

    } else if (param == "ScreenBrightness") {
      std::map<int, QString> brightnessLabels;
      for (int i = 0; i <= 101; ++i) {
        brightnessLabels[i] = i == 0 ? "Screen Off" : i == 101 ? "Auto" : QString::number(i) + "%";
      }
      toggle = new ParamValueControl(param, title, desc, icon, 0, 101, brightnessLabels, this, false);

    } else if (param == "WheelIcon") {
      std::vector<QString> wheelToggles{tr("RotatingWheel")};
      std::vector<QString> wheelToggleNames{tr("Rotating")};
      std::map<int, QString> steeringWheelLabels = {{0, "Stock"}, {1, "Lexus"}, {2, "Toyota"}, {3, "Frog"}, {4, "Rocket"}, {5, "Hyundai"}, {6, "Stalin"}};
      toggle = new ParamValueToggleControl(param, title, desc, icon, 0, 6, steeringWheelLabels, this, true, "", 1, wheelToggles, wheelToggleNames);

    } else {
      toggle = new ParamControl(param, title, desc, icon, this);
    }

    addItem(toggle);
    toggles[param.toStdString()] = toggle;

    QObject::connect(toggle, &ToggleControl::toggleFlipped, [this]() {
      std::thread([this]() {
        paramsMemory.putBool("FrogPilotTogglesUpdated", true);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        paramsMemory.putBool("FrogPilotTogglesUpdated", false);
      }).detach();
    });

    QObject::connect(static_cast<ParamValueControl*>(toggles["ScreenBrightness"]), &ParamValueControl::valueChanged, [](int value) {
      uiState()->scene.screen_brightness = value;
    });

    QObject::connect(static_cast<ParamValueControl*>(toggle), &ParamValueControl::buttonPressed, [this]() {
      std::thread([this]() {
        paramsMemory.putBool("FrogPilotTogglesUpdated", true);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        paramsMemory.putBool("FrogPilotTogglesUpdated", false);
      }).detach();
    });
  }

  customOnroadUIKeys = {"AdjacentPath", "BlindSpotPath", "ShowFPS", "LeadInfo", "RoadNameUI"};
  customThemeKeys = {"CustomColors", "CustomIcons", "CustomSignals", "CustomSounds"};
  modelUIKeys = {"AccelerationPath", "LaneLinesWidth", "PathEdgeWidth", "PathWidth", "RoadEdgesWidth", "UnlimitedLength"};

  QObject::connect(uiState(), &UIState::uiUpdate, this, &FrogPilotVisualsPanel::updateState);

  hideSubToggles();
}

void FrogPilotVisualsPanel::updateState() {
  if (isVisible()) {
    if (paramsMemory.getInt("FrogPilotTogglesOpen") == 2) {
      hideSubToggles();
    }
  }

  std::thread([this] {
    static bool checkedOnBoot = false;

    bool previousIsMetric = isMetric;
    isMetric = params.getBool("IsMetric");

    if (checkedOnBoot) {
      if (previousIsMetric == isMetric) return;
    }
    checkedOnBoot = true;

    if (isMetric != previousIsMetric) {
      const double distanceConversion = isMetric ? INCH_TO_CM : CM_TO_INCH;
      const double speedConversion = isMetric ? FOOT_TO_METER : METER_TO_FOOT;
      params.putInt("LaneLinesWidth", std::nearbyint(params.getInt("LaneLinesWidth") * distanceConversion));
      params.putInt("RoadEdgesWidth", std::nearbyint(params.getInt("RoadEdgesWidth") * distanceConversion));
      params.putInt("PathWidth", std::nearbyint(params.getInt("PathWidth") * speedConversion));
    }

    ParamValueControl *laneLinesWidthToggle = static_cast<ParamValueControl*>(toggles["LaneLinesWidth"]);
    ParamValueControl *roadEdgesWidthToggle = static_cast<ParamValueControl*>(toggles["RoadEdgesWidth"]);
    ParamValueControl *pathWidthToggle = static_cast<ParamValueControl*>(toggles["PathWidth"]);

    if (isMetric) {
      laneLinesWidthToggle->setDescription("Customize the lane line width.\n\nDefault matches the Vienna average of 10 centimeters.");
      roadEdgesWidthToggle->setDescription("Customize the road edges width.\n\nDefault is 1/2 of the Vienna average lane line width of 10 centimeters.");

      laneLinesWidthToggle->updateControl(0, 60, " centimeters");
      roadEdgesWidthToggle->updateControl(0, 60, " centimeters");
      pathWidthToggle->updateControl(0, 30, " meters");
    } else {
      laneLinesWidthToggle->setDescription("Customize the lane line width.\n\nDefault matches the MUTCD average of 4 inches.");
      roadEdgesWidthToggle->setDescription("Customize the road edges width.\n\nDefault is 1/2 of the MUTCD average lane line width of 4 inches.");

      laneLinesWidthToggle->updateControl(0, 24, " inches");
      roadEdgesWidthToggle->updateControl(0, 24, " inches");
      pathWidthToggle->updateControl(0, 100, " feet");
    }

    laneLinesWidthToggle->refresh();
    roadEdgesWidthToggle->refresh();

    previousIsMetric = isMetric;
  }).detach();
}

void FrogPilotVisualsPanel::parentToggleClicked() {
  paramsMemory.putInt("FrogPilotTogglesOpen", 1);
}

void FrogPilotVisualsPanel::hideSubToggles() {
  for (auto &[key, toggle] : toggles) {
    const bool subToggles = modelUIKeys.find(key.c_str()) != modelUIKeys.end() ||
                            customOnroadUIKeys.find(key.c_str()) != customOnroadUIKeys.end() ||
                            customThemeKeys.find(key.c_str()) != customThemeKeys.end();
    toggle->setVisible(!subToggles);
  }
}

void FrogPilotVisualsPanel::hideEvent(QHideEvent *event) {
  paramsMemory.putInt("FrogPilotTogglesOpen", 0);

  hideSubToggles();
}
