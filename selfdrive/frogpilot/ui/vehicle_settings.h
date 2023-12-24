#pragma once

#include <QStringList>
#include <set>

#include "selfdrive/ui/qt/offroad/settings.h"

class FrogPilotVehiclesPanel : public ListWidget {
  Q_OBJECT

public:
  explicit FrogPilotVehiclesPanel(SettingsWindow *parent);

private:
  void setDefaults();
  void setModels();
  void setToggles();

  ButtonControl *selectMakeButton;
  ButtonControl *selectModelButton;

  QLabel *noToggles;

  QString carMake;
  QString previousCarMake;

  QStringList models;

  std::map<std::string, ParamControl*> toggles;

  std::set<QString> gmKeys;
  std::set<QString> toyotaKeys;

  Params params;
  Params paramsMemory{"/dev/shm/params"};
};
