#pragma once

#include "common/params.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/input.h"
#include "selfdrive/ui/ui.h"

static const QString buttonStyle = R"(
  QPushButton {
    border-radius: 50px;
    font-size: 40px;
    font-weight: 500;
    height: 100px;
    padding: 0 20 0 20;
    margin: 15px;
    color: #E4E4E4;
    background-color: #393939;
  }
  QPushButton:pressed {
    background-color: #4a4a4a;
  }
  QPushButton:checked:enabled {
    background-color: #33Ab4C;
  }
  QPushButton:disabled {
    color: #33E4E4E4;
  }
)";

class FrogPilotButtonParamControl : public QPushButton {
  Q_OBJECT

public:
  FrogPilotButtonParamControl(const QString &param, const QString &label, const int minimumButtonWidth = 225)
    : QPushButton(), key(param.toStdString()), params(), 
      value(params.getBool(key)) {
    setCheckable(true);
    setChecked(value);
    setStyleSheet(buttonStyle);
    setMinimumWidth(minimumButtonWidth);
    setText(label);

    QObject::connect(this, &QPushButton::toggled, this, [this](bool checked) {
      params.putBoolNonBlocking(key, checked);
      paramsMemory.putBoolNonBlocking("FrogPilotTogglesUpdated", true);
      if (key == "MuteDM") {
        if (ConfirmationDialog::toggle("Reboot required to take effect.", "Reboot Now", this)) {
          Hardware::reboot();
        }
      }
    });
  }

private:
  const std::string key;
  Params params;
  Params paramsMemory{"/dev/shm/params"};
  bool value;
};

class ParamValueControl : public AbstractControl {
protected:
  ParamValueControl(const QString &name, const QString &description, const QString &iconPath)
    : AbstractControl(name, description, iconPath) {
    label.setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    label.setStyleSheet("color: #e0e879");
    label.setFixedWidth(170);

    setupButton(btnMinus, "-", -1);
    setupButton(btnPlus, "+", 1);

    hlayout->addWidget(&label);
    hlayout->addWidget(&btnMinus);
    hlayout->addWidget(&btnPlus);
  }

  void setupButton(QPushButton &btn, const QString &text, int delta) {
    btn.setStyleSheet(R"(
      QPushButton {
        background-color: #393939;
        color: #E4E4E4;
        border-radius: 50px;
        font-size: 50px;
        font-weight: 500;
        padding: 0;
      }
      QPushButton:pressed {
        background-color: #4a4a4a;
        color: #E4E4E4;
      }
    )");
    btn.setText(text);
    btn.setFixedSize(110, 100);
    btn.setAutoRepeat(true);
    btn.setAutoRepeatInterval(150);
    connect(&btn, &QPushButton::clicked, [this, delta]() { updateValue(delta); });
  }

  QPushButton btnMinus, btnPlus;
  QLabel label;
  Params params;
  Params paramsMemory{"/dev/shm/params"};
  bool isMetric = params.getBool("IsMetric");

  virtual void updateValue(int delta) = 0;
  virtual void refresh() = 0;
};

class FrogPilotPanel : public QWidget {
  Q_OBJECT

public:
  explicit FrogPilotPanel(QWidget *parent = nullptr) : QWidget(parent) {}
  QFrame *horizontalLine(QWidget *parent = nullptr) const;
  QFrame *whiteHorizontalLine(QWidget *parent = nullptr) const;

  Params params;
  Params paramsMemory{"/dev/shm/params"};
  bool isMetric = params.getBool("IsMetric");

protected:
  QVBoxLayout *mainLayout;
  std::map<std::string, std::vector<QWidget*>> childControls;

  ParamControl *createParamControl(const QString &key, const QString &label, const QString &desc, const QString &icon, QWidget *parent);
  QWidget *addSubControls(const QString &parentKey, QVBoxLayout *layout, const std::vector<std::tuple<QString, QString, QString>> &controls);
  QWidget *createDualParamControl(ParamValueControl *control1, ParamValueControl *control2);
  void addControl(const QString &key, const QString &label, const QString &desc, QVBoxLayout *layout, const QString &icon = "../assets/offroad/icon_blank.png");
  void createSubControl(const QString &key, const QString &label, const QString &desc, const QString &icon, const std::vector<QWidget*> &subControls, const std::vector<std::tuple<QString, QString, QString>> &additionalControls = {});
  void createSubButtonControl(const QString &parentKey, const std::vector<QPair<QString, QString>> &buttonKeys, QVBoxLayout *subControlLayout);
  void setInitialToggleStates();
  void setParams();
};

class FrogPilotControlsPanel : public FrogPilotPanel {
  Q_OBJECT

public:
  explicit FrogPilotControlsPanel(QWidget *parent = nullptr);
};

class FrogPilotVisualsPanel : public FrogPilotPanel {
  Q_OBJECT

public:
  explicit FrogPilotVisualsPanel(QWidget *parent = nullptr);
};

#define ParamController(className, paramName, labelText, descText, iconPath, getValueStrFunc, newValueFunc) \
class className : public ParamValueControl { \
  Q_OBJECT \
public: \
  className() : ParamValueControl(labelText, descText, iconPath) { \
    if (std::string(#className) == "CameraView" || std::string(#className) == "DeviceShutdown" || std::string(#className) == "WheelIcon") { \
      label.setFixedWidth(225); \
    } \
    if (std::string(#className) == "CESpeed" || std::string(#className) == "CESpeedLead") { \
      label.setFixedWidth(180); \
    } \
    refresh(); \
  } \
private: \
  void refresh() override { \
    label.setText(getValueStr()); \
  } \
  void updateValue(int delta) override { \
    int value = params.getInt(paramName); \
    value = newValue(value + delta); \
    params.putIntNonBlocking(paramName, value); \
    paramsMemory.putBoolNonBlocking("FrogPilotTogglesUpdated", true); \
    refresh(); \
  } \
  QString getValueStr() { getValueStrFunc; } \
  int newValue(int v) { newValueFunc; } \
};

ParamController(AccelerationProfile, "AccelerationProfile", "   Acceleration Profile", "Change the rate at which openpilot accelerates with either a sporty or more eco friendly profile.", "../assets/offroad/icon_blank.png",
  const int profile = params.getInt("AccelerationProfile");
  return profile == 1 ? "Eco" : profile == 2 ? "Normal" : "Sport";,
  return std::clamp(v, 1, 3);
)

ParamController(AggressiveJerk, "AggressiveJerk", "Jerk Value", "Set the jerk value for the 'Aggressive Personality'.\n\nValue represents the responsiveness of the brake/gas pedals.\n\nHigher value = Less responsive/more 'relaxed'\n\nStock has a value of 0.5.", "../assets/offroad/icon_blank.png",
  return QString::number(params.getInt("AggressiveJerk") / 10.0);,
  return std::clamp(v, 1, 50);
)

ParamController(AggressiveFollow, "AggressiveFollow", "Time", "Set the following distance for the 'Aggressive Personality'.\n\nValue represents the time (in seconds) to follow the lead vehicle.\n\nStock has a value of 1.25.", "../frogpilot/assets/other_images/aggressive.png",
  return QString::number(params.getInt("AggressiveFollow") / 10.0) + " sec";,
  return std::clamp(v, 10, 50);
)

ParamController(CameraView, "CameraView", "Camera View (Cosmetic Only)", "Set your preferred camera view for the onroad UI. This toggle is purely cosmetic and will not affect openpilot's use of the other cameras.", "../frogpilot/assets/toggle_icons/icon_camera.png",
  const int camera = params.getInt("CameraView");
  return camera == 0 ? "Auto" : camera == 1 ? "Standard" : camera == 2 ? "Wide" : "Driver";,
  return v >= 0 ? v % 4 : 3;
)

ParamController(CESpeed, "CESpeed", "Below", "Switch to 'Experimental Mode' below this speed when there is no lead vehicle.", "../assets/offroad/icon_blank.png",
  const int speed = params.getInt("CESpeed");
  return speed == 0 ? "Off" : QString::number(speed) + (isMetric ? " kph" : " mph");,
  return std::clamp(v, 0, isMetric ? 150 : 99);
)

ParamController(CESpeedLead, "CESpeedLead", "With Lead", "Switch to 'Experimental Mode' below this speed when there is a lead vehicle.", "../assets/offroad/icon_blank.png",
  const int speedLead = params.getInt("CESpeedLead");
  return speedLead == 0 ? "Off" : QString::number(speedLead) + (isMetric ? " kph" : " mph");,
  return std::clamp(v, 0, isMetric ? 150 : 99);
)

ParamController(CustomColors, "CustomColors", "Colors ", "Replace the stock openpilot colors with a custom color scheme.\n\nWant to submit your own color scheme? Post it in the 'feature-request' channel on the FrogPilot Discord!", "../assets/offroad/icon_blank.png",
  const int colors = params.getInt("CustomColors");
  return colors == 0 ? "Stock" : colors == 1 ? "Frog" : colors == 2 ? "Tesla" : "Stalin";,
  return v >= 0 ? v % 4 : 3;
)

ParamController(CustomIcons, "CustomIcons", "Icons", "Replace the stock openpilot icons with a custom icon pack.\n\nWant to submit your own icon pack? Post it in the 'feature-request' channel on the FrogPilot Discord!", "../assets/offroad/icon_blank.png",
  const int icons = params.getInt("CustomIcons");
  return icons == 0 ? "Stock" : icons == 1 ? "Frog" : icons == 2 ? "Tesla" : "Stalin";,
  return v >= 0 ? v % 4 : 3;
)

ParamController(CustomSignals, "CustomSignals", "Signals", "Enable a custom turn signal animation.\n\nWant to submit your own turn signal animation? Post it in the 'feature-request' channel on the FrogPilot Discord!", "../assets/offroad/icon_blank.png",
  const int turnSignals = params.getInt("CustomSignals");
  return turnSignals == 0 ? "Stock" : turnSignals == 1 ? "Frog" : "Stalin";,
  return v >= 0 ? v % 4 : 3;
)

ParamController(CustomSounds, "CustomSounds", "Sounds", "Replace the stock openpilot sounds with a custom sound pack.\n\nWant to submit your own custom sound pack? Post it in the 'feature-request' channel on the FrogPilot Discord!", "../assets/offroad/icon_blank.png",
  const int sounds = params.getInt("CustomSounds");
  return sounds == 0 ? "Stock" : sounds == 1 ? "Frog" : sounds == 2 ? "Tesla" : "Stalin";,
  return v >= 0 ? v % 4 : 3;
)

ParamController(DeviceShutdown, "DeviceShutdown", "Device Shutdown Timer", "Set the timer for when the device turns off after being offroad to reduce energy waste and prevent battery drain.", "../frogpilot/assets/toggle_icons/icon_time.png",
  const int time = params.getInt("DeviceShutdown");
  return time == 0 ? "Instant" : (time > 0 && time <= 3) ? QString::number(time * 15) + " mins" : QString::number(time - 3) + (time == 4 ? " hour" : " hours");,
  return std::clamp(v, 0, 33);
)

ParamController(LaneLinesWidth, "LaneLinesWidth", "Lanes", "Customize the lane line width.\n\nDefault matches the MUTCD average of 4 inches.", "../assets/offroad/icon_blank.png",
  return QString::number(params.getInt("LaneLinesWidth")) + (isMetric ? " cm" : " in");,
  return std::clamp(v, 0, isMetric ? 60 : 24);
)

ParamController(PathEdgeWidth, "PathEdgeWidth", "Path Edges", "Customize the path edge width that displays current driving statuses.\n\nDefault is 20% of the total path.\n\nBlue = Navigation\nLight Blue = Always On Lateral\nGreen = Default with 'FrogPilot Colors'\nLight Green = Default with stock colors\nOrange = Experimental Mode Active\nYellow = Conditional Overriden", "../assets/offroad/icon_blank.png",
  return QString::number(params.getInt("PathEdgeWidth")) + "%";,
  return std::clamp(v, 0, 100);
)

ParamController(PathWidth, "PathWidth", "Path ", "Customize the path width.\n\nDefault matches the width of a 2019 Lexus ES 350.", "../assets/offroad/icon_blank.png",
  return QString::number(params.getInt("PathWidth") / 10.0) + (isMetric ? " m" : " ft");,
  return std::clamp(v, 0, isMetric ? 30 : 100);
)

ParamController(RelaxedJerk, "RelaxedJerk", "Jerk Value", "Set the jerk value for the 'Relaxed Personality'.\n\nValue represents the responsiveness of the brake/gas pedals.\n\nHigher value = Less responsive/more 'relaxed'\n\nStock has a value of 1.0.", "../assets/offroad/icon_blank.png",
  return QString::number(params.getInt("RelaxedJerk") / 10.0);,
  return std::clamp(v, 1, 50);
)

ParamController(RelaxedFollow, "RelaxedFollow", "Time", "Set the following distance for the 'Relaxed Personality'.\n\nValue represents the time (in seconds) to follow the lead vehicle.\n\nStock has a value of 1.75.", "../frogpilot/assets/other_images/relaxed.png",
  return QString::number(params.getInt("RelaxedFollow") / 10.0) + " sec";,
  return std::clamp(v, 10, 50);
)

ParamController(RoadEdgesWidth, "RoadEdgesWidth", "Road Edges", "Customize the road edges width.\n\nDefault is 1/2 of the MUTCD average lane line width of 4 inches.", "../assets/offroad/icon_blank.png",
  return QString::number(params.getInt("RoadEdgesWidth")) + (isMetric ? " cm" : " in");,
  return std::clamp(v, 0, isMetric ? 60 : 24);
)

ParamController(ScreenBrightness, "ScreenBrightness", "Screen Brightness", "Set a custom screen brightness level or use the default 'Auto' brightness setting.", "../frogpilot/assets/toggle_icons/icon_light.png",
  const int brightness = params.getInt("ScreenBrightness");
  uiState()->scene.screen_brightness = brightness;
  return brightness == 101 ? "Auto" : brightness == 0 ? "Off" : QString::number(brightness) + "%";,
  return std::clamp(v, 0, 101);
)

ParamController(StandardJerk, "StandardJerk", "Jerk Value", "Set the jerk value for the 'Standard Personality'.\n\nValue represents the responsiveness of the brake/gas pedals.\n\nHigher value = Less responsive/more 'relaxed'\n\nStock has a value of 1.0.", "../assets/offroad/icon_blank.png",
  return QString::number(params.getInt("StandardJerk") / 10.0);,
  return std::clamp(v, 1, 50);
)

ParamController(StandardFollow, "StandardFollow", "Time", "Set the following distance for the 'Standard Personality'.\n\nValue represents the time (in seconds) to follow the lead vehicle.\n\nStock has a value of 1.45.", "../frogpilot/assets/other_images/standard.png",
  return QString::number(params.getInt("StandardFollow") / 10.0) + " sec";,
  return std::clamp(v, 10, 50);
)

ParamController(WheelIcon, "WheelIcon", "Steering Wheel Icon", "Replace the stock openpilot steering wheel icon with a custom icon.\n\nWant to submit your own steering wheel? Post it in the 'feature-request' channel on the FrogPilot Discord!", "../assets/offroad/icon_openpilot.png",
  const int wheel = params.getInt("WheelIcon");
  return wheel == 0 ? "Stock" : wheel == 1 ? "Lexus" : wheel == 2 ? "Toyota" : wheel == 3 ? "Frog" : wheel == 4 ? "Rocket" : wheel == 5 ? "Hyundai" : "Stalin";,
  return v >= 0 ? v % 7 : 6;
)
