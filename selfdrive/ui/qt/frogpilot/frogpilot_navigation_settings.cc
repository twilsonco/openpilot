#include <QMouseEvent>

#include "selfdrive/ui/qt/frogpilot/frogpilot_navigation_functions.h"
#include "selfdrive/ui/qt/frogpilot/frogpilot_navigation_settings.h"
#include "selfdrive/ui/qt/offroad/frogpilot_settings.h"

FrogPilotNavigationPanel::FrogPilotNavigationPanel(QWidget *parent) : QFrame(parent), scene(uiState()->scene) {
  mainLayout = new QStackedLayout(this);

  navigationWidget = new QWidget();
  QVBoxLayout *navigationLayout = new QVBoxLayout(navigationWidget);
  navigationLayout->setMargin(40);

  ListWidget *list = new ListWidget(navigationWidget);

  Primeless *primelessPanel = new Primeless(this);
  mainLayout->addWidget(primelessPanel);

  ButtonControl *manageNOOButton = new ButtonControl(tr("Manage Navigation Settings"), tr("MANAGE"), tr("Manage primeless navigate on openpilot settings."));
  QObject::connect(manageNOOButton, &ButtonControl::clicked, [=]() { mainLayout->setCurrentWidget(primelessPanel); });
  QObject::connect(primelessPanel, &Primeless::backPress, [=]() { mainLayout->setCurrentWidget(navigationWidget); });
  list->addItem(manageNOOButton);
  manageNOOButton->setVisible(!uiState()->hasPrime());

  std::vector<QString> scheduleOptions{tr("Manually"), tr("Weekly"), tr("Monthly")};
  ButtonParamControl *preferredSchedule = new ButtonParamControl("PreferredSchedule", tr("Maps Scheduler"),
                                          tr("Choose the frequency for updating maps with the latest OpenStreetMap (OSM) changes. "
                                          "Weekly updates begin at midnight every Sunday, while monthly updates start at midnight on the 1st of each month. "
                                          "If your device is off or offline during a scheduled update, the download will the next time you're offroad for more than 5 minutes."),
                                          "",
                                          scheduleOptions);
  schedule = params.getInt("PreferredSchedule");
  schedulePending =  params.getBool("SchedulePending");
  list->addItem(preferredSchedule);

  list->addItem(offlineMapsSize = new LabelControl(tr("Offline Maps Size"), formatSize(calculateDirectorySize(offlineFolderPath))));
  offlineMapsSize->setVisible(true);
  list->addItem(offlineMapsStatus = new LabelControl(tr("Offline Maps Status"), ""));
  offlineMapsStatus->setVisible(false);
  list->addItem(offlineMapsETA = new LabelControl(tr("Offline Maps ETA"), ""));
  offlineMapsETA->setVisible(false);
  list->addItem(offlineMapsElapsed = new LabelControl(tr("Time Elapsed"), ""));
  offlineMapsElapsed->setVisible(false);

  cancelDownloadButton = new ButtonControl(tr("Cancel Download"), tr("CANCEL"), tr("Cancel your current download."));
  QObject::connect(cancelDownloadButton, &ButtonControl::clicked, [this] { cancelDownload(this); });
  list->addItem(cancelDownloadButton);
  cancelDownloadButton->setVisible(false);

  downloadOfflineMapsButton = new ButtonControl(tr("Download Offline Maps"), tr("DOWNLOAD"), tr("Download your selected offline maps to use with openpilot."));
  QObject::connect(downloadOfflineMapsButton, &ButtonControl::clicked, [this] { downloadMaps(); });
  list->addItem(downloadOfflineMapsButton);
  downloadOfflineMapsButton->setVisible(!params.get("MapsSelected").empty());

  SelectMaps *mapsPanel = new SelectMaps(this);
  mainLayout->addWidget(mapsPanel);

  QObject::connect(mapsPanel, &SelectMaps::setMaps, [=]() { setMaps(); });

  ButtonControl *selectMapsButton = new ButtonControl(tr("Select Offline Maps"), tr("SELECT"), tr("Select your maps to use with OSM."));
  QObject::connect(selectMapsButton, &ButtonControl::clicked, [=]() { mainLayout->setCurrentWidget(mapsPanel); });
  QObject::connect(mapsPanel, &SelectMaps::backPress, [=]() { mainLayout->setCurrentWidget(navigationWidget); });
  list->addItem(selectMapsButton);

  removeOfflineMapsButton = new ButtonControl(tr("Remove Offline Maps"), tr("REMOVE"), tr("Remove your downloaded offline maps to clear up storage space."));
  QObject::connect(removeOfflineMapsButton, &ButtonControl::clicked, [this] { removeMaps(this); });
  list->addItem(removeOfflineMapsButton);
  removeOfflineMapsButton->setVisible(QDir(offlineFolderPath).exists());

  navigationLayout->addWidget(new ScrollView(list, navigationWidget));
  navigationWidget->setLayout(navigationLayout);
  mainLayout->addWidget(navigationWidget);
  mainLayout->setCurrentWidget(navigationWidget);

  QObject::connect(uiState(), &UIState::uiUpdate, this, &FrogPilotNavigationPanel::updateState);
}

void FrogPilotNavigationPanel::hideEvent(QHideEvent *event) {
  QWidget::hideEvent(event);

  mainLayout->setCurrentWidget(navigationWidget);
}

void FrogPilotNavigationPanel::updateState() {
  if (schedule) downloadSchedule();

  if (!isVisible()) return;

  if (downloadActive) updateStatuses();
}

void FrogPilotNavigationPanel::updateStatuses() {
  std::thread([&] {
    static std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
    osmDownloadProgress = params.get("OSMDownloadProgress");

    if (osmDownloadProgress != previousOSMDownloadProgress) {
      qint64 fileSize = calculateDirectorySize(offlineFolderPath);
      offlineMapsSize->setText(formatSize(fileSize));
      previousOSMDownloadProgress = osmDownloadProgress;
    }

    const QString elapsedTime = calculateElapsedTime(osmDownloadProgress, startTime);

    offlineMapsElapsed->setText(elapsedTime);
    offlineMapsETA->setText(calculateETA(osmDownloadProgress, startTime));
    offlineMapsStatus->setText(formatDownloadStatus(osmDownloadProgress));

    downloadActive = elapsedTime != "Downloaded";

    if (downloadActive != previousDownloadActive) {
      startTime = !downloadActive ? std::chrono::steady_clock::now() : startTime;
      updateVisibility(downloadActive);
      previousDownloadActive = downloadActive;
    }
  }).detach();
}

void FrogPilotNavigationPanel::updateVisibility(bool visibility) {
  cancelDownloadButton->setVisible(visibility);
  offlineMapsElapsed->setVisible(visibility);
  offlineMapsETA->setVisible(visibility);
  offlineMapsStatus->setVisible(visibility);
  downloadOfflineMapsButton->setVisible(!visibility);
  removeOfflineMapsButton->setVisible(!visibility);
}

void FrogPilotNavigationPanel::downloadSchedule() {
  const bool wifi = (*uiState()->sm)["deviceState"].getDeviceState().getNetworkType() == cereal::DeviceState::NetworkType::WIFI;

  const std::time_t t = std::time(nullptr);
  const std::tm *now = std::localtime(&t);

  const bool isScheduleTime = (schedule == 1 && now->tm_wday == 0) || (schedule == 2 && now->tm_mday == 1);

  if ((isScheduleTime || schedulePending) && !(scene.started || scheduleCompleted) && wifi) {
    downloadMaps();
    scheduleCompleted = true;
  } else if (!isScheduleTime) {
    scheduleCompleted = false;
  } else {
    if (!schedulePending) {
      params.putBool("SchedulePending", true);
    }
    schedulePending = true;
  }
}

void FrogPilotNavigationPanel::cancelDownload(QWidget *parent) {
  if (ConfirmationDialog::yesorno("Are you sure you want to cancel the download?", parent)) {
    std::thread([&] {
      std::system("pkill mapd");
    }).detach();
    if (ConfirmationDialog::toggle("Reboot required to re-enable map downloads", "Reboot Now", parent)) {
      Hardware::reboot();
    }
    downloadActive = false;
    updateVisibility(downloadActive);
    downloadOfflineMapsButton->setVisible(downloadActive);
  }
}

void FrogPilotNavigationPanel::downloadMaps() {
  paramsMemory.put("OSMDownloadLocations", params.get("MapsSelected"));
  removeOfflineMapsButton->setVisible(true);
  downloadActive = true;
}

void FrogPilotNavigationPanel::removeMaps(QWidget *parent) {
  if (ConfirmationDialog::yesorno("Are you sure you want to delete all of your downloaded maps?", parent)) {
    std::thread([&] {
      removeOfflineMapsButton->setVisible(false);
      offlineMapsSize->setText(formatSize(0));
      std::system("rm -rf /data/media/0/osm/offline");
    }).detach();
  }
}

void FrogPilotNavigationPanel::setMaps() {
  std::thread([&] {
    QStringList states = ButtonSelectionControl::selectedStates.split(',', QString::SkipEmptyParts);
    QStringList countries = ButtonSelectionControl::selectedCountries.split(',', QString::SkipEmptyParts);

    if (!states.isEmpty() || !countries.isEmpty()) {
      QJsonObject json;
      json.insert("states", QJsonArray::fromStringList(states));
      json.insert("nations", QJsonArray::fromStringList(countries));

      params.put("MapsSelected", QJsonDocument(json).toJson(QJsonDocument::Compact).toStdString());
      downloadOfflineMapsButton->setVisible(true);
    }
  }).detach();
}

SelectMaps::SelectMaps(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *mainLayout = new QVBoxLayout(this);

  QHBoxLayout *buttonsLayout = new QHBoxLayout();
  buttonsLayout->setContentsMargins(20, 40, 20, 0);

  backButton = new QPushButton(tr("Back"), this);
  statesButton = new QPushButton(tr("States"), this);
  countriesButton = new QPushButton(tr("Countries"), this);

  backButton->setFixedSize(400, 100);
  statesButton->setFixedSize(400, 100);
  countriesButton->setFixedSize(400, 100);

  buttonsLayout->addWidget(backButton);
  buttonsLayout->addStretch();
  buttonsLayout->addWidget(statesButton);
  buttonsLayout->addStretch();
  buttonsLayout->addWidget(countriesButton);
  mainLayout->addLayout(buttonsLayout);

  mainLayout->addWidget(horizontalLine());
  mainLayout->setSpacing(20);

  mapsLayout = new QStackedLayout();
  mapsLayout->setMargin(40);
  mapsLayout->setSpacing(20);
  mainLayout->addLayout(mapsLayout);

  QObject::connect(backButton, &QPushButton::clicked, this, [this]() { emit backPress(), emit setMaps(); });

  ListWidget *statesList = new ListWidget();

  LabelControl *northeastLabel = new LabelControl(tr("United States - Northeast"), "");
  statesList->addItem(northeastLabel);

  ButtonSelectionControl *northeastControl = new ButtonSelectionControl("", tr(""), tr(""), northeastMap, false);
  statesList->addItem(northeastControl);

  LabelControl *midwestLabel = new LabelControl(tr("United States - Midwest"), "");
  statesList->addItem(midwestLabel);

  ButtonSelectionControl *midwestControl = new ButtonSelectionControl("", tr(""), tr(""), midwestMap, false);
  statesList->addItem(midwestControl);

  LabelControl *southLabel = new LabelControl(tr("United States - South"), "");
  statesList->addItem(southLabel);

  ButtonSelectionControl *southControl = new ButtonSelectionControl("", tr(""), tr(""), southMap, false);
  statesList->addItem(southControl);

  LabelControl *westLabel = new LabelControl(tr("United States - West"), "");
  statesList->addItem(westLabel);

  ButtonSelectionControl *westControl = new ButtonSelectionControl("", tr(""), tr(""), westMap, false);
  statesList->addItem(westControl);

  LabelControl *territoriesLabel = new LabelControl(tr("United States - Territories"), "");
  statesList->addItem(territoriesLabel);

  ButtonSelectionControl *territoriesControl = new ButtonSelectionControl("", tr(""), tr(""), territoriesMap, false);
  statesList->addItem(territoriesControl);

  statesScrollView = new ScrollView(statesList);
  mapsLayout->addWidget(statesScrollView);

  QObject::connect(statesButton, &QPushButton::clicked, this, [this]() {
    mapsLayout->setCurrentWidget(statesScrollView);
    statesButton->setStyleSheet(activeButtonStyle);
    countriesButton->setStyleSheet(normalButtonStyle);
  });

  ListWidget *countriesList = new ListWidget();

  LabelControl *africaLabel = new LabelControl(tr("Africa"), "");
  countriesList->addItem(africaLabel);

  ButtonSelectionControl *africaControl = new ButtonSelectionControl("", tr(""), tr(""), africaMap, true);
  countriesList->addItem(africaControl);

  LabelControl *antarcticaLabel = new LabelControl(tr("Antarctica"), "");
  countriesList->addItem(antarcticaLabel);

  ButtonSelectionControl *antarcticaControl = new ButtonSelectionControl("", tr(""), tr(""), antarcticaMap, true);
  countriesList->addItem(antarcticaControl);

  LabelControl *asiaLabel = new LabelControl(tr("Asia"), "");
  countriesList->addItem(asiaLabel);

  ButtonSelectionControl *asiaControl = new ButtonSelectionControl("", tr(""), tr(""), asiaMap, true);
  countriesList->addItem(asiaControl);

  LabelControl *europeLabel = new LabelControl(tr("Europe"), "");
  countriesList->addItem(europeLabel);

  ButtonSelectionControl *europeControl = new ButtonSelectionControl("", tr(""), tr(""), europeMap, true);
  countriesList->addItem(europeControl);

  LabelControl *northAmericaLabel = new LabelControl(tr("North America"), "");
  countriesList->addItem(northAmericaLabel);

  ButtonSelectionControl *northAmericaControl = new ButtonSelectionControl("", tr(""), tr(""), northAmericaMap, true);
  countriesList->addItem(northAmericaControl);

  LabelControl *oceaniaLabel = new LabelControl(tr("Oceania"), "");
  countriesList->addItem(oceaniaLabel);

  ButtonSelectionControl *oceaniaControl = new ButtonSelectionControl("", tr(""), tr(""), oceaniaMap, true);
  countriesList->addItem(oceaniaControl);

  LabelControl *southAmericaLabel = new LabelControl(tr("South America"), "");
  countriesList->addItem(southAmericaLabel);

  ButtonSelectionControl *southAmericaControl = new ButtonSelectionControl("", tr(""), tr(""), southAmericaMap, true);
  countriesList->addItem(southAmericaControl);

  countriesScrollView = new ScrollView(countriesList);
  mapsLayout->addWidget(countriesScrollView);

  QObject::connect(countriesButton, &QPushButton::clicked, this, [this]() {
    mapsLayout->setCurrentWidget(countriesScrollView);
    statesButton->setStyleSheet(normalButtonStyle);
    countriesButton->setStyleSheet(activeButtonStyle);
  });

  mapsLayout->setCurrentWidget(statesScrollView);
  statesButton->setStyleSheet(activeButtonStyle);

  setStyleSheet(R"(
    QPushButton {
      font-size: 50px;
      margin: 0px;
      padding: 15px;
      border-width: 0;
      border-radius: 30px;
      color: #dddddd;
      background-color: #393939;
    }
    QPushButton:pressed {
      background-color: #4a4a4a;
    }
  )");
}

QString SelectMaps::activeButtonStyle = R"(
  font-size: 50px;
  margin: 0px;
  padding: 15px;
  border-width: 0;
  border-radius: 30px;
  color: #dddddd;
  background-color: #33Ab4C;
)";

QString SelectMaps::normalButtonStyle = R"(
  font-size: 50px;
  margin: 0px;
  padding: 15px;
  border-width: 0;
  border-radius: 30px;
  color: #dddddd;
  background-color: #393939;
)";

QFrame *SelectMaps::horizontalLine(QWidget *parent) const {
  QFrame *line = new QFrame(parent);

  line->setFrameShape(QFrame::StyledPanel);
  line->setStyleSheet(R"(
    border-width: 2px;
    border-bottom-style: solid;
    border-color: gray;
  )");
  line->setFixedHeight(2);

  return line;
}

void SelectMaps::hideEvent(QHideEvent *event) {
  QWidget::hideEvent(event);
  emit setMaps();
}

Primeless::Primeless(QWidget *parent) : QWidget(parent) {
  QStackedLayout *primelessLayout = new QStackedLayout(this);

  QWidget *mainWidget = new QWidget();
  mainLayout = new QVBoxLayout(mainWidget);
  mainLayout->setMargin(40);

  backButton = new QPushButton(tr("Back"), this);
  backButton->setObjectName("backButton");
  backButton->setFixedSize(400, 100);
  QObject::connect(backButton, &QPushButton::clicked, this, [this]() { emit backPress(); });
  mainLayout->addWidget(backButton, 0, Qt::AlignLeft);

  list = new ListWidget(mainWidget);

  wifi = new WifiManager(this);
  ipLabel = new LabelControl(tr("Manage Your Settings At"), QString("%1:8082").arg(wifi->getIp4Address()));
  list->addItem(ipLabel);

  std::vector<QString> searchOptions{tr("MapBox"), tr("Amap"), tr("Google")};
  ButtonParamControl *searchInput = new ButtonParamControl("SearchInput", tr("Destination Search Provider"), 
                                       tr("Select a search provider for destination queries in Navigate on Openpilot. Options include MapBox (recommended), Amap, and Google Maps."),
                                       "", searchOptions);
  list->addItem(searchInput);

  createMapboxKeyControl(publicMapboxKeyControl, tr("Public Mapbox Key"), "MapboxPublicKey", "pk.");
  createMapboxKeyControl(secretMapboxKeyControl, tr("Secret Mapbox Key"), "MapboxSecretKey", "sk.");

  mapboxPublicKeySet = !params.get("MapboxPublicKey").empty();
  mapboxSecretKeySet = !params.get("MapboxSecretKey").empty();
  setupCompleted = mapboxPublicKeySet && mapboxSecretKeySet;

  QHBoxLayout *setupLayout = new QHBoxLayout();
  setupLayout->setMargin(0);

  imageLabel = new QLabel(this);
  pixmap.load(currentStep);
  imageLabel->setPixmap(pixmap.scaledToWidth(1500, Qt::SmoothTransformation));
  setupLayout->addWidget(imageLabel, 0, Qt::AlignCenter);
  imageLabel->hide();

  ButtonControl *setupButton = new ButtonControl(tr("Mapbox Setup Instructions"), tr("VIEW"), tr("View the instructions to set up MapBox for Primeless Navigation."), this);
  QObject::connect(setupButton, &ButtonControl::clicked, this, [this]() {
    updateStep();
    backButton->hide();
    list->setVisible(false);
    imageLabel->show();
  });
  list->addItem(setupButton);

  QObject::connect(uiState(), &UIState::uiUpdate, this, &Primeless::updateState);

  mainLayout->addLayout(setupLayout);
  mainLayout->addWidget(new ScrollView(list, mainWidget));
  mainWidget->setLayout(mainLayout);
  primelessLayout->addWidget(mainWidget);

  setLayout(primelessLayout);

  setStyleSheet(R"(
    QPushButton {
      font-size: 50px;
      margin: 0px;
      padding: 15px;
      border-width: 0;
      border-radius: 30px;
      color: #dddddd;
      background-color: #393939;
    }
    QPushButton:pressed {
      background-color: #4a4a4a;
    }
  )");
}

void Primeless::hideEvent(QHideEvent *event) {
  QWidget::hideEvent(event);
  backButton->show();
  list->setVisible(true);
  imageLabel->hide();
}

void Primeless::mousePressEvent(QMouseEvent *event) {
  backButton->show();
  list->setVisible(true);
  imageLabel->hide();
}

void Primeless::updateState() {
  if (!isVisible()) return;

  QString ipAddress = wifi->getIp4Address();
  ipLabel->setText(ipAddress.isEmpty() ? tr("Device Offline") : QString("%1:8082").arg(ipAddress));

  mapboxPublicKeySet = !params.get("MapboxPublicKey").empty();
  mapboxSecretKeySet = !params.get("MapboxSecretKey").empty();
  setupCompleted = mapboxPublicKeySet && mapboxSecretKeySet && setupCompleted;

  publicMapboxKeyControl->setText(mapboxPublicKeySet ? tr("REMOVE") : tr("ADD"));
  secretMapboxKeyControl->setText(mapboxSecretKeySet ? tr("REMOVE") : tr("ADD"));

  if (imageLabel->isVisible()) {
    updateStep();
  }
}

void Primeless::createMapboxKeyControl(ButtonControl *&control, const QString &label, const std::string &paramKey, const QString &prefix) {
  control = new ButtonControl(label, "", tr("Manage your %1."), this);
  QObject::connect(control, &ButtonControl::clicked, this, [this, control, label, paramKey, prefix] {
    if (control->text() == tr("ADD")) {
      QString key = InputDialog::getText(tr("Enter your %1").arg(label), this);
      if (!key.startsWith(prefix)) {
        key = prefix + key;
      }
      if (key.length() >= 80) {
        params.put(paramKey, key.toStdString());
      }
    } else {
      params.remove(paramKey);
    }
  });
  list->addItem(control);
  control->setText(params.get(paramKey).empty() ? tr("ADD") : tr("REMOVE"));
}

void Primeless::updateStep() {
  currentStep = setupCompleted ? "../assets/images/setup_completed.png" : 
                (mapboxPublicKeySet && mapboxSecretKeySet) ? "../assets/images/both_keys_set.png" :
                mapboxPublicKeySet ? "../assets/images/public_key_set.png" : "../assets/images/no_keys_set.png";

  pixmap.load(currentStep);
  imageLabel->setPixmap(pixmap.scaledToWidth(1500, Qt::SmoothTransformation));
}
