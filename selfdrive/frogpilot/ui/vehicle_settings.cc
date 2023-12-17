#include "selfdrive/frogpilot/ui/vehicle_settings.h"

FrogPilotVehiclesPanel::FrogPilotVehiclesPanel(SettingsWindow *parent) : ListWidget(parent) {
  noToggles = new QLabel(tr("No additional options available for the selected make."));
  noToggles->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  noToggles->setAlignment(Qt::AlignCenter);
  addItem(noToggles);

  std::vector<std::tuple<QString, QString, QString, QString>> vehicle_toggles {
  };

  for (auto &[param, title, desc, icon] : vehicle_toggles) {
    auto toggle = new ParamControl(param, title, desc, icon, this);
    addItem(toggle);
    toggles[param.toStdString()] = toggle;

    connect(toggle, &ToggleControl::toggleFlipped, [this]() {
      paramsMemory.putBool("FrogPilotTogglesUpdated", true);
    });
  }

  setToggles();
}

void FrogPilotVehiclesPanel::setToggles() {
  this->update();
}
