#include "selfdrive/frogpilot/ui/visual_settings.h"
#include "selfdrive/ui/ui.h"

FrogPilotVisualsPanel::FrogPilotVisualsPanel(SettingsWindow *parent) : FrogPilotListWidget(parent) {
  const std::vector<std::tuple<QString, QString, QString, QString>> visualToggles {
    {"CustomUI", "Custom Onroad UI", "Customize the Onroad UI with some additional visual functions.", "../assets/offroad/icon_road.png"},

    {"QOLVisuals", "Quality of Life", "Miscellaneous quality of life changes to improve your overall openpilot experience.", "../frogpilot/assets/toggle_icons/quality_of_life.png"},
  };

  for (const auto &[param, title, desc, icon] : visualToggles) {
    ParamControl *toggle;

    if (param == "CustomUI") {
      FrogPilotParamManageControl *customUIToggle = new FrogPilotParamManageControl(param, title, desc, icon, this);
      QObject::connect(customUIToggle, &FrogPilotParamManageControl::manageButtonClicked, this, [this]() {
        parentToggleClicked();
        for (auto &[key, toggle] : toggles) {
          toggle->setVisible(customOnroadUIKeys.find(key.c_str()) != customOnroadUIKeys.end());
        }
      });
      toggle = customUIToggle;

    } else if (param == "QOLVisuals") {
      FrogPilotParamManageControl *qolToggle = new FrogPilotParamManageControl(param, title, desc, icon, this);
      QObject::connect(qolToggle, &FrogPilotParamManageControl::manageButtonClicked, this, [this]() {
        parentToggleClicked();
        for (auto &[key, toggle] : toggles) {
          toggle->setVisible(qolKeys.find(key.c_str()) != qolKeys.end());
        }
      });
      toggle = qolToggle;

    } else {
      toggle = new ParamControl(param, title, desc, icon, this);
    }

    addItem(toggle);
    toggles[param.toStdString()] = toggle;

    QObject::connect(toggle, &ToggleControl::toggleFlipped, [this]() {
      updateToggles();
    });

    QObject::connect(static_cast<FrogPilotParamValueControl*>(toggle), &FrogPilotParamValueControl::buttonPressed, [this]() {
      updateToggles();
    });

    QObject::connect(toggle, &AbstractControl::showDescriptionEvent, [this]() {
      update();
    });

    QObject::connect(static_cast<FrogPilotParamManageControl*>(toggle), &FrogPilotParamManageControl::manageButtonClicked, [this]() {
      update();
    });
  }

  std::set<std::string> rebootKeys = {""};
  for (const std::string &key : rebootKeys) {
    QObject::connect(toggles[key], &ToggleControl::toggleFlipped, [this]() {
      if (FrogPilotConfirmationDialog::toggle("Reboot required to take effect.", "Reboot Now", this)) {
        Hardware::reboot();
      }
    });
  }

  customOnroadUIKeys = {};
  customThemeKeys = {};
  modelUIKeys = {};
  qolKeys = {};

  QObject::connect(device(), &Device::interactiveTimeout, this, &FrogPilotVisualsPanel::hideSubToggles);
  QObject::connect(parent, &SettingsWindow::closeParentToggle, this, &FrogPilotVisualsPanel::hideSubToggles);
  QObject::connect(parent, &SettingsWindow::updateMetric, this, &FrogPilotVisualsPanel::updateMetric);

  hideSubToggles();
  updateMetric();
}

void FrogPilotVisualsPanel::updateToggles() {
  std::thread([this]() {
    paramsMemory.putBool("FrogPilotTogglesUpdated", true);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    paramsMemory.putBool("FrogPilotTogglesUpdated", false);
  }).detach();
}

void FrogPilotVisualsPanel::updateMetric() {
  bool previousIsMetric = isMetric;
  isMetric = params.getBool("IsMetric");

  if (isMetric != previousIsMetric) {
    double distanceConversion = isMetric ? INCH_TO_CM : CM_TO_INCH;
    double speedConversion = isMetric ? FOOT_TO_METER : METER_TO_FOOT;
  }

  if (isMetric) {
  } else {
  }

  previousIsMetric = isMetric;
}

void FrogPilotVisualsPanel::parentToggleClicked() {
  this->openParentToggle();
}

void FrogPilotVisualsPanel::hideSubToggles() {
  for (auto &[key, toggle] : toggles) {
    bool subToggles = modelUIKeys.find(key.c_str()) != modelUIKeys.end() ||
                      customOnroadUIKeys.find(key.c_str()) != customOnroadUIKeys.end() ||
                      customThemeKeys.find(key.c_str()) != customThemeKeys.end() ||
                      qolKeys.find(key.c_str()) != qolKeys.end();
    toggle->setVisible(!subToggles);
  }

  this->closeParentToggle();
}

void FrogPilotVisualsPanel::hideEvent(QHideEvent *event) {
  hideSubToggles();
}
