#pragma once

#include <mutex>

#include "selfdrive/ui/qt/network/wifi_manager.h"
#include "selfdrive/ui/qt/offroad/settings.h"
#include "selfdrive/ui/ui.h"

class SetupMapbox : public QFrame {
  Q_OBJECT

public:
  explicit SetupMapbox(QWidget *parent = nullptr);

private:
  void paintEvent(QPaintEvent *event) override;
  void updateState();

  static constexpr const char *imagePath = "../assets/images/";

  bool mapboxPublicKeySet;
  bool mapboxSecretKeySet;
  bool setupCompleted;

  Params params;

  QImage currentImage;
  QString currentStep;
  QString lastStep;
};

class Primeless : public QWidget {
  Q_OBJECT

public:
  explicit Primeless(QWidget *parent = nullptr);

protected:
  void mousePressEvent(QMouseEvent *event) override;
  void hideEvent(QHideEvent *event) override;

private:
  void createMapboxKeyControl(ButtonControl *&control, const QString &label, const std::string &paramKey, const QString &prefix);
  void updateState();

  ListWidget *list;

  ButtonControl *publicMapboxKeyControl;
  ButtonControl *secretMapboxKeyControl;
  ButtonControl *setupButton;
  ButtonParamControl *searchInput;
  LabelControl *ipLabel;

  QPushButton *back;

  SetupMapbox *setupMapbox;
  WifiManager *wifi;

  Params params;

signals:
  void backPress();
};

class ManageMaps : public QFrame {
  Q_OBJECT

public:
  explicit ManageMaps(QWidget *parent = nullptr);

private:
  void hideEvent(QHideEvent *event);

  QStackedLayout *mapsLayout = nullptr;

  ListWidget *countriesList;
  ListWidget *statesList;

  LabelControl *midwestLabel;
  LabelControl *northeastLabel;
  LabelControl *southLabel;
  LabelControl *territoriesLabel;
  LabelControl *westLabel;

  LabelControl *africaLabel;
  LabelControl *antarcticaLabel;
  LabelControl *asiaLabel;
  LabelControl *europeLabel;
  LabelControl *northAmericaLabel;
  LabelControl *oceaniaLabel;
  LabelControl *southAmericaLabel;

  QPushButton *back_btn;
  QPushButton *states_btn;
  QPushButton *countries_btn;

  static QString activeButtonStyle;
  static QString normalButtonStyle;

signals:
  void backPress();
  void startDownload();
};

class FrogPilotNavigationPanel : public QFrame {
  Q_OBJECT

public:
  explicit FrogPilotNavigationPanel(QWidget *parent = 0);

private:
  void cancelDownload(QWidget *parent);
  void downloadMaps(QWidget *parent);
  void downloadSchedule();
  void hideEvent(QHideEvent *event);
  void removeMaps(QWidget *parent);
  void updateState();

  QStackedLayout *mainLayout = nullptr;
  QWidget *navigationWidget = nullptr;

  ManageMaps *mapsPanel;
  Primeless *primelessPanel;

  ButtonControl *cancelDownloadButton;
  ButtonControl *downloadOfflineMapsButton;
  ButtonControl *manageMapsButton;
  ButtonControl *manageNOOButton;
  ButtonControl *redownloadOfflineMapsButton;
  ButtonControl *removeOfflineMapsButton;

  ButtonParamControl *preferredSchedule;

  LabelControl *offlineMapsSize;
  LabelControl *offlineMapsStatus;
  LabelControl *offlineMapsETA;
  LabelControl *offlineMapsElapsed;

  qint64 fileSize;
  std::string previousOSMDownloadProgress;

  std::chrono::steady_clock::time_point startTime;

  bool scheduleCompleted;
  bool schedulePending;
  int schedule;

  Params params;
  Params paramsMemory{"/dev/shm/params"};
  UIScene &scene;

  std::mutex manageMapsMutex;
};
