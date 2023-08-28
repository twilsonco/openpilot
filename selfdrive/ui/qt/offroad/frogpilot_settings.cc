#include <QHBoxLayout>
#include <QWidget>
#include <tuple>
#include <vector>

#include "common/params.h"
#include "selfdrive/ui/qt/offroad/frogpilot_settings.h"
#include "selfdrive/ui/qt/widgets/input.h"
#include "selfdrive/ui/ui.h"

FrogPilotControlsPanel::FrogPilotControlsPanel(QWidget *parent) : FrogPilotPanel(parent) {
  mainLayout = new QVBoxLayout(this);

  QLabel *const descriptionLabel = new QLabel("Click on the toggle names to see a detailed toggle description", this);
  mainLayout->addWidget(descriptionLabel);
  mainLayout->addSpacing(25);
  mainLayout->addWidget(white_horizontal_line());

  static const std::vector<std::tuple<QString, QString, QString, QString>> toggles = {
    {"ConditionalExperimentalMode", "Conditional Experimental Mode", "Automatically activate 'Experimental Mode' based on specified conditions.", "../assets/offroad/icon_conditional.png"},
    {"CustomDrivingPersonalities", "Custom Driving Personalities", "Customize the driving personality profiles to your liking.", "../assets/offroad/icon_custom.png"},
    {"DeviceShutdownTimer", "Device Shutdown Timer", "Set the timer for when the device turns off after being offroad to reduce energy waste and prevent battery drain.", "../assets/offroad/icon_time.png"},
    {"DrivingPersonalitiesUIWheel", "Driving Personalities Via UI / Wheel", "Switch driving personalities using the 'Distance' button on the steering wheel (Toyota/Lexus Only) or via the onroad UI for other makes.\n\n1 bar = Aggressive\n2 bars = Standard\n3 bars = Relaxed", "../assets/offroad/icon_distance.png"},
    {"ExperimentalModeViaWheel", "Experimental Mode Via Steering Wheel / Screen", "Enable or disable Experimental Mode by double-clicking the 'Lane Departure'/LKAS button on the steering wheel (Toyota/Lexus Only) or double tapping the screen for other makes.\n\nOverrides 'Conditional Experimental Mode'. ", "../assets/img_experimental_white.svg"},
    {"FireTheBabysitter", "Fire the Babysitter", "Disable some of openpilot's 'Babysitter Protocols'.", "../assets/offroad/icon_babysitter.png"},
    {"LateralTuning", "Lateral Tuning", "Change the way openpilot steers.", "../assets/offroad/icon_lateral_tune.png"},
    {"LongitudinalTuning", "Longitudinal Tuning", "Change the way openpilot accelerates and brakes.", "../assets/offroad/icon_longitudinal_tune.png"},
    {"NudgelessLaneChange", "Nudgeless Lane Change", "Switch lanes without having to nudge the steering wheel.", "../assets/offroad/icon_lane.png"},
    {"TurnDesires", "Turn Desires", "Use turn desires when below the minimum lane change speed for more precise turns.", "../assets/navigation/direction_continue_right.png"}
  };

  for (const auto &[key, label, desc, icon] : toggles) {
    ParamControl *control = createParamControl(key, label, desc, icon, this);
    if (key == "ConditionalExperimentalMode") {
      createSubControl(key, label, desc, icon, {
        createDualParamControl(new ConditionalExperimentalModeSpeed(), new ConditionalExperimentalModeSpeedLead()),
      });
      createSubButtonControl(key, {
        {"ConditionalExperimentalModeCurves", "Curves"},
        {"ConditionalExperimentalModeCurvesLead", "Curves W/ Lead"},
        {"ConditionalExperimentalModeSlowerLead", "Slower Lead"},
        {"ConditionalExperimentalModeStopLights", "Stop Lights"},
        {"ConditionalExperimentalModeSignal", "Turn Signal"}
      }, mainLayout);
    } else if (key == "CustomDrivingPersonalities") {
      createSubControl(key, label, desc, icon, {
        createDualParamControl(new AggressivePersonalityValue(), new AggressiveJerkValue()),
        createDualParamControl(new StandardPersonalityValue(), new StandardJerkValue()),
        createDualParamControl(new RelaxedPersonalityValue(), new RelaxedJerkValue()),
      });
    } else if (key == "DeviceShutdownTimer") {
      mainLayout->addWidget(new DeviceShutdownTimer());
      mainLayout->addWidget(horizontal_line());
    } else if (key == "FireTheBabysitter") {
      createSubControl(key, label, desc, icon, {}, {
        {"DisableAllLogging", "Disable Logging", "Prevent all data tracking by comma to go completely incognitio or to even just reduce thermals.\n\nWARNING: This will prevent any drives from being recorded and they WILL NOT be recoverable!"}
      });
      createSubButtonControl(key, {
        {"MuteDM", "Mute DM"},
        {"MuteDoor", "Mute Door Open"},
        {"MuteSeatbelt", "Mute Seatbelt"},
        {"MuteSystemOverheat", "Mute Overheat"}
      }, mainLayout);
    } else if (key == "LateralTuning") {
      createSubControl(key, label, desc, icon, {}, {
        {"AverageDesiredCurvature", "Average Desired Curvature", "Use Pfeiferj's distance based curvature adjustment for smoother handling of curves."},
        {"NNFF", "NNFF - Neural Network Feedforward", "Use Twilsonco's Neural Network Feedforward torque system for more precise lateral control."}
      });
    } else if (key == "LongitudinalTuning") {
      createSubControl(key, label, desc, icon, {
        new AccelerationProfile(),
        new IncreasedStoppingDistance(),
      }, {
        {"AggressiveAcceleration", "Aggressive Acceleration With Lead", "Accelerate more aggressively behind a lead when starting from a stop."},
        {"SmootherBraking", "Smoother Braking Behind Lead", "More natural braking behavior when coming up to a slower vehicle."},
        {"TSS2Tune", "TSS2 Tune", "Tuning profile for TSS2 vehicles. Based on the tuning profile from DragonPilot."}
      });
    } else if (key == "NudgelessLaneChange") {
      createSubControl(key, label, desc, icon, {
        new LaneChangeTimer(),
      });
      createSubButtonControl(key, {
        {"LaneDetection", "Lane Detection"},
        {"OneLaneChange", "One Lane Change Per Signal"},
      }, mainLayout);
    } else {
      mainLayout->addWidget(control);
      if (key != std::get<0>(toggles.back())) mainLayout->addWidget(horizontal_line());
    }
  }
  setInitialToggleStates();
}

FrogPilotVisualsPanel::FrogPilotVisualsPanel(QWidget *parent) : FrogPilotPanel(parent) {
  mainLayout = new QVBoxLayout(this);

  QLabel *const descriptionLabel = new QLabel("Click on the toggle names to see a detailed toggle description", this);
  mainLayout->addWidget(descriptionLabel);
  mainLayout->addSpacing(25);
  mainLayout->addWidget(white_horizontal_line());

  static const std::vector<std::tuple<QString, QString, QString, QString>> toggles = {
    {"FrogTheme", "FrogPilot Theme", "Enable the beloved FrogPilot Theme! Disable toggle to revert back to the stock openpilot theme.", "../assets/images/frog_button_home.png"},
    {"Compass", "Compass", "Add a compass to the onroad UI that indicates your current driving direction.", "../assets/offroad/icon_compass.png"},
    {"CustomRoadUI", "Custom Road UI", "Customize the road UI to your liking.", "../assets/offroad/icon_road.png"},
    {"NumericalTemp", "Numerical Temperature Gauge", "Replace openpilot's 'GOOD', 'OK', and 'HIGH' temperature statuses with numerical values.\n\nTap the gauge to switch between Celsius and Fahrenheit.", "../assets/offroad/icon_temp.png"},
    {"RotatingWheel", "Rotating Steering Wheel", "The steering wheel in top right corner of the onroad UI rotates alongside your physical steering wheel.", "../assets/offroad/icon_rotate.png"},
    {"ScreenBrightness", "Screen Brightness", "Choose a custom screen brightness level or use the default 'Auto' brightness setting.", "../assets/offroad/icon_light.png"},
    {"Sidebar", "Sidebar Shown By Default", "Sidebar is shown by default while onroad as opposed to hidden.", "../assets/offroad/icon_metric.png"},
    {"SilentMode", "Silent Mode", "Disables all openpilot sounds for a completely silent experience.", "../assets/offroad/icon_mute.png"},
    {"SteeringWheel", "Steering Wheel Icon", "Replace the stock openpilot steering wheel icon with a custom icon.\n\nWant to submit your own steering wheel? Message me on Discord\n@FrogsGoMoo!", "../assets/offroad/icon_openpilot.png"},
    {"WideCameraDisable", "Wide Camera Disabled (Cosmetic Only)", "Disable the wide camera view. This toggle is purely cosmetic and will not affect openpilot's use of the wide camera.", "../assets/offroad/icon_camera.png"}
  };

  for (const auto &[key, label, desc, icon] : toggles) {
    ParamControl *control = createParamControl(key, label, desc, icon, this);
    if (key == "FrogTheme") {
      mainLayout->addWidget(control);
      mainLayout->addWidget(horizontal_line());
      createSubButtonControl(key, {
        {"FrogColors", "Colors"},
        {"FrogIcons", "Icons"},
        {"FrogSounds", "Sounds"},
        {"FrogSignals", "Turn Signals"}
      }, mainLayout);
    } else if (key == "CustomRoadUI") {
      createSubControl(key, label, desc, icon, {
        createDualParamControl(new LaneLinesWidth(), new RoadEdgesWidth()),
        createDualParamControl(new PathWidth(), new PathEdgeWidth()),
      });
      createSubButtonControl(key, {
        {"BlindSpotPath", "Blind Spot Path"},
        {"UnlimitedLength", "'Unlimited' Road UI Length"},
      }, mainLayout);
    } else if (key == "ScreenBrightness") {
      mainLayout->addWidget(new ScreenBrightness());
      mainLayout->addWidget(horizontal_line());
    } else if (key == "SteeringWheel") {
      mainLayout->addWidget(new SteeringWheel());
      mainLayout->addWidget(horizontal_line());
    } else {
      mainLayout->addWidget(control);
      if (key != std::get<0>(toggles.back())) mainLayout->addWidget(horizontal_line());
    }
  }
  setInitialToggleStates();
}

ParamControl *FrogPilotPanel::createParamControl(const QString &key, const QString &label, const QString &desc, const QString &icon, QWidget *parent) {
  ParamControl *control = new ParamControl(key, label, desc, icon);
  connect(control, &ParamControl::toggleFlipped, [=](bool state) {
    if (key == "NNFF") {
      if (params.getBool("NNFF")) {
        const bool addSSH = ConfirmationDialog::yesorno("Would you like to grant 'twilsonco' SSH access to improve NNFF? This won't affect any added SSH keys.", parent);
        params.putBool("TwilsoncoSSH", addSSH);
        if (addSSH) {
          ConfirmationDialog::toggleAlert("Message 'twilsonco' on Discord to get your device properly configured.", "Acknowledge", parent);
        }
      }
    }
    static const QMap<QString, QString> parameterWarnings = {
      {"AggressiveAcceleration", "This will make openpilot driving more aggressively!"},
      {"SmootherBraking", "This will modify openpilot's braking behavior!"},
      {"TSS2Tune", "This will modify openpilot's acceleration and braking behavior!"}
    };
    if (parameterWarnings.contains(key) && params.getBool(key.toStdString())) {
      ConfirmationDialog::toggleAlert("WARNING: " + parameterWarnings[key], "I understand the risks.", parent);
    }
    if (ConfirmationDialog::toggle("Reboot required to take effect.", "Reboot Now", parent)) {
      Hardware::reboot();
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

QFrame *FrogPilotPanel::horizontal_line(QWidget *parent) const {
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

QFrame *FrogPilotPanel::white_horizontal_line(QWidget *parent) const {
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
  layout->addWidget(horizontal_line());
}

void FrogPilotPanel::createSubControl(const QString &key, const QString &label, const QString &desc, const QString &icon, const std::vector<QWidget*> &subControls, const std::vector<std::tuple<QString, QString, QString>> &additionalControls) {
  ParamControl *control = createParamControl(key, label, desc, icon, this);
  mainLayout->addWidget(control);
  mainLayout->addWidget(horizontal_line());
  QVBoxLayout *subControlLayout = new QVBoxLayout();
  for (QWidget *subControl : subControls) {
    subControlLayout->addWidget(subControl);
    subControlLayout->addWidget(horizontal_line());
  }
  QWidget *mainControl = addSubControls(key, subControlLayout, additionalControls);
  connect(control, &ParamControl::toggleFlipped, [=](bool state) { mainControl->setVisible(state); });
}

void FrogPilotPanel::createSubButtonControl(const QString &parentKey, const std::vector<QPair<QString, QString>> &buttonKeys, QVBoxLayout *subControlLayout) {
  QHBoxLayout *buttonsLayout = new QHBoxLayout();
  QWidget *line = horizontal_line();
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
  line = horizontal_line();
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
