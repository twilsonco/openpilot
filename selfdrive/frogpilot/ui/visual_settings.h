#pragma once

#include <set>

#include "selfdrive/frogpilot/ui/frogpilot_ui_functions.h"
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

  std::set<QString> alertVolumeControlKeys = {"EngageVolume", "DisengageVolume", "RefuseVolume", "PromptVolume", "PromptDistractedVolume", "WarningSoftVolume", "WarningImmediateVolume"};
  std::set<QString> customAlertsKeys = {"GreenLightAlert", "LeadDepartingAlert", "LoudBlindspotAlert"};
  std::set<QString> customOnroadUIKeys = {"AccelerationPath", "AdjacentPath", "BlindSpotPath", "FPSCounter", "LeadInfo"};
  std::set<QString> customThemeKeys = {"CustomColors", "CustomIcons", "CustomSignals", "CustomSounds"};
  std::set<QString> modelUIKeys = {"LaneLinesWidth", "PathEdgeWidth", "PathWidth", "RoadEdgesWidth", "UnlimitedLength"};
  std::set<QString> qolKeys = {"DriveStats", "FullMap", "HideSpeed"};

  std::map<std::string, ParamControl*> toggles;

  Params params;
  Params paramsMemory{"/dev/shm/params"};

  bool isMetric = params.getBool("IsMetric");
};
