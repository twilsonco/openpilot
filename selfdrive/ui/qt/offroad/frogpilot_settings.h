#pragma once

#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/input.h"
#include "selfdrive/ui/ui.h"

static const QString button_style = R"(
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
  FrogPilotButtonParamControl(const QString &param, const QString &label, const int minimum_button_width = 225)
    : QPushButton(), key(param.toStdString()), params(), 
      value(params.getBool(key)) {
    setCheckable(true);
    setChecked(value);
    setStyleSheet(button_style);
    setMinimumWidth(minimum_button_width);
    setText(label);

    QObject::connect(this, &QPushButton::toggled, this, [=](bool checked) {
      params.putBool(key, checked);
      if (ConfirmationDialog::toggle("Reboot required to take effect.", "Reboot Now", this)) {
        Hardware::reboot();
      }
    });
  }

private:
  const std::string key;
  Params params;
  bool value;
};

class ParamValueControl : public AbstractControl {
protected:
  ParamValueControl(const QString &name, const QString &description, const QString &iconPath) : AbstractControl(name, description, iconPath) {
    label.setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    label.setStyleSheet("color: #e0e879");
    label.setFixedWidth(170);
    setupButton(btnminus, "-", -1);
    setupButton(btnplus, "+", 1);
    hlayout->addWidget(&label);
    hlayout->addWidget(&btnminus);
    hlayout->addWidget(&btnplus);
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

  QPushButton btnminus, btnplus;
  QLabel label;
  Params params;
  Params params_memory{"/dev/shm/params"};

  virtual void updateValue(int delta) = 0;
  virtual void refresh() = 0;
};

class FrogPilotPanel : public QWidget {
  Q_OBJECT

public:
  explicit FrogPilotPanel(QWidget *parent = nullptr) : QWidget(parent) {}
  QFrame *horizontal_line(QWidget *parent = nullptr) const;
  QFrame *white_horizontal_line(QWidget *parent = nullptr) const;
  Params params;

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

#define ParamControllerInt(className, paramName, labelText, descText, iconPath, getValueStrFunc, newValueFunc) \
class className : public ParamValueControl { \
  Q_OBJECT \
public: \
  className() : ParamValueControl(labelText, descText, iconPath) { \
    if (std::string(#className) == "DeviceShutdownTimer" || std::string(#className) == "IncreasedStoppingDistance" || std::string(#className) == "SteeringWheel") { \
      label.setFixedWidth(225); \
    } \
    refresh(); \
  } \
private: \
  void refresh() override { \
    label.setText(getValueStr()); \
    params_memory.putBool("FrogPilotTogglesUpdated", true); \
  } \
  void updateValue(int delta) override { \
    int value = params.getInt(paramName); \
    value = newValue(value + delta); \
    params.putInt(paramName, value); \
    refresh(); \
  } \
  QString getValueStr() { getValueStrFunc } \
  int newValue(int v) { newValueFunc } \
};

ParamControllerInt(AccelerationProfile, "AccelerationProfile", "   Acceleration Profile", "Change the rate at which openpilot accelerates with either a sporty or more eco friendly profile.", "../assets/offroad/icon_blank.png",
  int value = params.getInt("AccelerationProfile");
  return value == 1 ? "Eco" : value == 2 ? "Normal" : "Sport";,
  return std::clamp(v, 1, 3);
)

ParamControllerInt(AggressiveJerkValue, "AggressiveJerkValue", "Jerk Value", "Set the jerk value for the 'Aggressive Personality'.\n\nValue represents the responsiveness of the brake/gas pedals.\n\nHigher value = Less responsive/more 'relaxed'\n\nStock has a value of 0.5.", "../assets/offroad/icon_blank.png",
  return QString::number(params.getInt("AggressiveJerkValue") / 10.0);,
  return std::clamp(v, 1, 50);
)

ParamControllerInt(AggressivePersonalityValue, "AggressivePersonalityValue", "Time", "Set the following distance for the 'Aggressive Personality'.\n\nValue represents the time (in seconds) to follow the lead vehicle.\n\nStock has a value of 1.25.", "../assets/aggressive.png",
  return QString::number(params.getInt("AggressivePersonalityValue") / 10.0) + " sec";,
  return std::clamp(v, 10, 50);
)

ParamControllerInt(ConditionalExperimentalModeSpeed, "ConditionalExperimentalModeSpeed", "Below", "Switch to 'Experimental Mode' below this speed when there is no lead vehicle.", "../assets/offroad/icon_blank.png",
  int value = params.getInt("ConditionalExperimentalModeSpeed");
  return value == 0 ? "Off" : QString::number(value) + "mph";,
  return std::clamp(v, 0, 99);
)

ParamControllerInt(ConditionalExperimentalModeSpeedLead, "ConditionalExperimentalModeSpeedLead", "With Lead", "Switch to 'Experimental Mode' below this speed when there is a lead vehicle.", "../assets/offroad/icon_blank.png",
  int value = params.getInt("ConditionalExperimentalModeSpeedLead");
  return value == 0 ? "Off" : QString::number(value) + "mph";,
  return std::clamp(v, 0, 99);
)

ParamControllerInt(DeviceShutdownTimer, "DeviceShutdownTimer", "Device Shutdown Timer", "Set the timer for when the device turns off after being offroad to reduce energy waste and prevent battery drain.", "../assets/offroad/icon_time.png",
  int value = params.getInt("DeviceShutdownTimer");
  return value == 0 ? "Instant" : (value > 0 && value <= 3) ? QString::number(value * 15) + " mins" : QString::number(value - 3) + (value == 4 ? " hour" : " hours");,
  return std::clamp(v, 0, 33);
)

ParamControllerInt(IncreasedStoppingDistance, "IncreasedStoppingDistance", "   Increase Stopping Distance", "Increase the stopping distance for a more comfortable stop.", "../assets/offroad/icon_blank.png",
  int value = params.getInt("IncreasedStoppingDistance");
  return value == 0 ? "Off" : QString::number(value) + " meters";,
  return std::clamp(v, 0, 5);
)

ParamControllerInt(LaneChangeTimer, "LaneChangeTimer", "   Lane Change Timer", "Set a time delay before openpilot conducts a nudgeless lane change.", "../assets/offroad/icon_blank.png",
  int delay = params.getInt("LaneChangeTimer");
  return delay == 0 ? "Instant" : QString::number(static_cast<double>(delay) / 2.0) + " sec";,
  return std::clamp(v, 0, 10);
)

ParamControllerInt(LaneLinesWidth, "LaneLinesWidth", "Lanes", "Customize the lane line width.\n\nDefault matches the MUTCD average of 4 inches.", "../assets/offroad/icon_blank.png",
  return QString::number(params.getInt("LaneLinesWidth")) + " in";,
  return std::clamp(v, 0, 24);
)

ParamControllerInt(PathEdgeWidth, "PathEdgeWidth", "Path Edges", "Customize the path edge width that displays current driving statuses.\n\nDefault is 20% of the total path.\n\nBlue = Navigation\nLight Blue = Always On Lateral\nGreen = Default with 'FrogPilot Colors'\nLight Green = Default with stock colors\nOrange = Experimental Mode Active\nYellow = Conditional Overriden", "../assets/offroad/icon_blank.png",
  return QString::number(params.getInt("PathEdgeWidth")) + "%";,
  return std::clamp(v, 0, 100);
)

ParamControllerInt(PathWidth, "PathWidth", "Path", "Customize the path width.\n\nDefault matches the width of a 2019 Lexus ES 350.", "../assets/offroad/icon_blank.png",
  return QString::number(params.getInt("PathWidth") / 10.0) + " ft";,
  return std::clamp(v, 0, 100);
)

ParamControllerInt(RelaxedJerkValue, "RelaxedJerkValue", "Jerk Value", "Set the jerk value for the 'Relaxed Personality'.\n\nValue represents the responsiveness of the brake/gas pedals.\n\nHigher value = Less responsive/more 'relaxed'\n\nStock has a value of 1.0.", "../assets/offroad/icon_blank.png",
  return QString::number(params.getInt("RelaxedJerkValue") / 10.0);,
  return std::clamp(v, 1, 50);
)

ParamControllerInt(RelaxedPersonalityValue, "RelaxedPersonalityValue", "Time", "Set the following distance for the 'Relaxed Personality'.\n\nValue represents the time (in seconds) to follow the lead vehicle.\n\nStock has a value of 1.75.", "../assets/relaxed.png",
  return QString::number(params.getInt("RelaxedPersonalityValue") / 10.0) + " sec";,
  return std::clamp(v, 10, 50);
)

ParamControllerInt(RoadEdgesWidth, "RoadEdgesWidth", "Road Edges", "Customize the road edges width.\n\nDefault is 1/2 of the MUTCD average lane line width of 4 inches.", "../assets/offroad/icon_blank.png",
  return QString::number(params.getInt("RoadEdgesWidth")) + " in";,
  return std::clamp(v, 0, 24);
)

ParamControllerInt(ScreenBrightness, "ScreenBrightness", "Screen Brightness", "Set a custom screen brightness level or use the default 'Auto' brightness setting.", "../assets/offroad/icon_light.png",
  int brightness = params.getInt("ScreenBrightness");
  return brightness == 101 ? "Auto" : brightness == 0 ? "Off" : QString::number(brightness) + "%";,
  return std::clamp(v, 0, 101);
)

ParamControllerInt(StandardJerkValue, "StandardJerkValue", "Jerk Value", "Set the jerk value for the 'Standard Personality'.\n\nValue represents the responsiveness of the brake/gas pedals.\n\nHigher value = Less responsive/more 'relaxed'\n\nStock has a value of 1.0.", "../assets/offroad/icon_blank.png",
  return QString::number(params.getInt("StandardJerkValue") / 10.0);,
  return std::clamp(v, 1, 50);
)

ParamControllerInt(StandardPersonalityValue, "StandardPersonalityValue", "Time", "Set the following distance for the 'Standard Personality'.\n\nValue represents the time (in seconds) to follow the lead vehicle.\n\nStock has a value of 1.45.", "../assets/standard.png",
  return QString::number(params.getInt("StandardPersonalityValue") / 10.0) + " sec";,
  return std::clamp(v, 10, 50);
)

ParamControllerInt(SteeringWheel, "SteeringWheel", "Steering Wheel Icon", "Replace the stock openpilot steering wheel icon with a custom icon.\n\nWant to submit your own steering wheel? Post it in the 'feature-request' channel on the FrogPilot Discord!", "../assets/offroad/icon_openpilot.png",
  int value = params.getInt("SteeringWheel");
  return value == 0 ? "Stock" : value == 1 ? "Lexus" : value == 2 ? "Toyota" : value == 3 ? "Frog" : value == 4 ? "Rocket" : "Hyundai";,
  return v >= 0 ? v % 6 : 5;
)
