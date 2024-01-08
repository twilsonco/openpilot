#pragma once

#include <set>

#include "selfdrive/frogpilot/ui/frogpilot_functions.h"
#include "selfdrive/ui/qt/offroad/settings.h"

class FrogPilotVisualsPanel : public FrogPilotListWidget {
  Q_OBJECT

public:
  explicit FrogPilotVisualsPanel(SettingsWindow *parent);

signals:
  void closeParentToggle();
  void openParentToggle();

private:
  void hideEvent(QHideEvent *event);
  void hideSubToggles();
  void parentToggleClicked();
  void updateMetric();
  void updateToggles();

  std::set<QString> customOnroadUIKeys;
  std::set<QString> customThemeKeys;
  std::set<QString> modelUIKeys;

  std::map<std::string, ParamControl*> toggles;

  Params params;
  Params paramsMemory{"/dev/shm/params"};

  bool isMetric = params.getBool("IsMetric");
};
