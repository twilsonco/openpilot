#include "selfdrive/frogpilot/ui/vehicle_settings.h"
#include "selfdrive/ui/ui.h"

FrogPilotVehiclesPanel::FrogPilotVehiclesPanel(SettingsWindow *parent) : FrogPilotListWidget(parent) {
  std::vector<std::tuple<QString, QString, QString, QString>> vehicleToggles {
  };

  for (auto &[param, title, desc, icon] : vehicleToggles) {
    ParamControl *toggle = new ParamControl(param, title, desc, icon, this);

    addItem(toggle);
    toggle->setVisible(false);
    toggles[param.toStdString()] = toggle;

    QObject::connect(toggle, &ToggleControl::toggleFlipped, [this]() {
      updateToggles();
    });
  }

  gmKeys = {};
  toyotaKeys = {};

  std::set<std::string> rebootKeys = {};
  for (const std::string &key : rebootKeys) {
    QObject::connect(toggles[key], &ToggleControl::toggleFlipped, [this]() {
      if (FrogPilotConfirmationDialog::toggle("Reboot required to take effect.", "Reboot Now", this)) {
        Hardware::reboot();
      }
    });
  }
}

void FrogPilotVehiclesPanel::updateToggles() {
  std::thread([this]() {
    paramsMemory.putBool("FrogPilotTogglesUpdated", true);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    paramsMemory.putBool("FrogPilotTogglesUpdated", false);
  }).detach();
}

void FrogPilotVehiclesPanel::setToggles() {
  bool gm = false;
  bool toyota = false;

  for (auto &[key, toggle] : toggles) {
    toggle->setVisible(false);

    if (gm) {
      toggle->setVisible(gmKeys.find(key.c_str()) != gmKeys.end());
    } else if (toyota) {
      toggle->setVisible(toyotaKeys.find(key.c_str()) != toyotaKeys.end());
    }
  }

  update();
}
