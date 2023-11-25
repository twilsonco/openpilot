#include <QMouseEvent>

#include "selfdrive/ui/qt/frogpilot/frogpilot_navigation_functions.h"
#include "selfdrive/ui/qt/frogpilot/frogpilot_navigation_settings.h"
#include "selfdrive/ui/qt/offroad/frogpilot_settings.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"

FrogPilotNavigationPanel::FrogPilotNavigationPanel(QWidget *parent) : QFrame(parent), scene(uiState()->scene) {
  mainLayout = new QStackedLayout(this);

  navigationWidget = new QWidget();
  QVBoxLayout *navigationLayout = new QVBoxLayout(navigationWidget);
  navigationLayout->setMargin(40);

  ListWidget *list = new ListWidget(navigationWidget);

  primelessPanel = new Primeless(this);
  mainLayout->addWidget(primelessPanel);

  manageNOOButton = new ButtonControl(tr("Manage Navigation Settings"), tr("MANAGE"), tr("Manage primeless navigate on openpilot settings."));
  QObject::connect(manageNOOButton, &ButtonControl::clicked, [=]() { mainLayout->setCurrentWidget(primelessPanel); });
  QObject::connect(primelessPanel, &Primeless::backPress, [=]() { mainLayout->setCurrentWidget(navigationWidget); });
  list->addItem(manageNOOButton);
  manageNOOButton->setVisible(!uiState()->hasPrime());

  QObject::connect(uiState(), &UIState::primeTypeChanged, this, [=](PrimeType prime_type) {
    bool notPrime = prime_type == PrimeType::NONE || prime_type == PrimeType::UNKNOWN;
    manageNOOButton->setVisible(notPrime);
  });

  std::vector<QString> scheduleOptions{tr("Manually"), tr("Weekly"), tr("Monthly")};
  preferredSchedule = new ButtonParamControl("PreferredSchedule", tr("Maps Scheduler"),
                                          tr("Choose the frequency for updating maps with the latest OpenStreetMap (OSM) changes. "
                                          "Weekly updates begin at midnight every Sunday, while monthly updates start at midnight on the 1st of each month. "
                                          "If your device is off or offline during a scheduled update, the download will the next time you're offroad for more than 5 minutes."),
                                          "",
                                          scheduleOptions);
  schedule = params.getInt("PreferredSchedule");
  list->addItem(preferredSchedule);

  list->addItem(offlineMapsSize = new LabelControl(tr("Offline Maps Size"), ""));
  list->addItem(offlineMapsStatus = new LabelControl(tr("Offline Maps Status"), ""));
  list->addItem(offlineMapsETA = new LabelControl(tr("Offline Maps ETA"), ""));
  list->addItem(offlineMapsElapsed = new LabelControl(tr("Time Elapsed"), ""));

  cancelDownloadButton = new ButtonControl(tr("Cancel Download"), tr("CANCEL"), tr("Cancel the downloading of the currently selected maps."));
  QObject::connect(cancelDownloadButton, &ButtonControl::clicked, [this] { cancelDownload(this); });
  list->addItem(cancelDownloadButton);

  downloadOfflineMapsButton = new ButtonControl(tr("Download Offline Maps"), tr("DOWNLOAD"), tr("Download your selected offline maps to use with openpilot."));
  QObject::connect(downloadOfflineMapsButton, &ButtonControl::clicked, [this] { downloadMaps(this); });
  list->addItem(downloadOfflineMapsButton);

  mapsPanel = new ManageMaps(this);
  mainLayout->addWidget(mapsPanel);

  manageMapsButton = new ButtonControl(tr("Manage Offline Maps"), tr("MANAGE"), tr("Manage your maps to use with OSM."));
  QObject::connect(manageMapsButton, &ButtonControl::clicked, [=]() { mainLayout->setCurrentWidget(mapsPanel); });
  QObject::connect(mapsPanel, &ManageMaps::backPress, [=]() { mainLayout->setCurrentWidget(navigationWidget); });
  list->addItem(manageMapsButton);

  redownloadOfflineMapsButton = new ButtonControl(tr("Redownload Offline Maps"), tr("DOWNLOAD"), tr("Redownload your selected offline maps to use with openpilot."));
  QObject::connect(redownloadOfflineMapsButton, &ButtonControl::clicked, [this] { downloadMaps(this); });
  list->addItem(redownloadOfflineMapsButton);

  removeOfflineMapsButton = new ButtonControl(tr("Remove Offline Maps"), tr("REMOVE"), tr("Remove your downloaded offline maps to clear up storage space."));
  QObject::connect(removeOfflineMapsButton, &ButtonControl::clicked, [this] { removeMaps(this); });
  list->addItem(removeOfflineMapsButton);

  navigationLayout->addWidget(new ScrollView(list, navigationWidget));
  navigationLayout->addStretch(1);
  navigationWidget->setLayout(navigationLayout);
  mainLayout->addWidget(navigationWidget);
  mainLayout->setCurrentWidget(navigationWidget);

  QObject::connect(uiState(), &UIState::uiUpdate, this, &FrogPilotNavigationPanel::downloadSchedule);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &FrogPilotNavigationPanel::updateState);

  QObject::connect(mapsPanel, &ManageMaps::startDownload, [=]() { downloadMaps(this); });
}

void FrogPilotNavigationPanel::hideEvent(QHideEvent *event) {
  QWidget::hideEvent(event);
  mainLayout->setCurrentWidget(navigationWidget);
}

void FrogPilotNavigationPanel::updateState() {
  if (!isVisible()) return;

  const QString offlineFolderPath = "/data/media/0/osm/offline";
  const bool dirExists = QDir(offlineFolderPath).exists();
  const bool mapsSelected = !params.get("MapsSelected").empty();
  const std::string osmDownloadProgress = params.get("OSMDownloadProgress");

  if (osmDownloadProgress != previousOSMDownloadProgress || !(fileSize || dirExists)) {
    fileSize = 0;
    offlineMapsSize->setText("0 MB");
  }

  previousOSMDownloadProgress = osmDownloadProgress;

  const QString elapsedTime = calculateElapsedTime(osmDownloadProgress, startTime);
  const bool isDownloaded = elapsedTime == "Downloaded";

  cancelDownloadButton->setVisible(!isDownloaded);

  offlineMapsElapsed->setVisible(!isDownloaded);
  offlineMapsETA->setVisible(!isDownloaded);

  offlineMapsElapsed->setText(elapsedTime);
  offlineMapsETA->setText(calculateETA(osmDownloadProgress, startTime));
  offlineMapsStatus->setText(formatDownloadStatus(osmDownloadProgress));

  downloadOfflineMapsButton->setVisible(!dirExists && mapsSelected);
  redownloadOfflineMapsButton->setVisible(dirExists && mapsSelected && osmDownloadProgress.empty());
  removeOfflineMapsButton->setVisible(dirExists && osmDownloadProgress.empty());

  offlineMapsSize->setVisible(false);
}

void FrogPilotNavigationPanel::downloadSchedule() {
  if (!schedule) return;

  std::time_t t = std::time(nullptr);
  std::tm *now = std::localtime(&t);

  bool isScheduleTime = (schedule == 1 && now->tm_wday == 0) || (schedule == 2 && now->tm_mday == 1);
  bool wifi = (*uiState()->sm)["deviceState"].getDeviceState().getNetworkType() == cereal::DeviceState::NetworkType::WIFI;

  if ((isScheduleTime || schedulePending) && !(scene.started || scheduleCompleted) && wifi) {
    downloadMaps(this);
    scheduleCompleted = true;
  } else if (!isScheduleTime) {
    scheduleCompleted = false;
  } else {
    schedulePending = true;
  }
}

void FrogPilotNavigationPanel::cancelDownload(QWidget *parent) {
  std::lock_guard<std::mutex> lock(manageMapsMutex);

  if (ConfirmationDialog::yesorno("Are you sure you want to cancel the download?", parent)) {
    paramsMemory.putBool("OSM", false);
    paramsMemory.remove("OSMDownloadLocations");
    params.remove("OSMDownloadProgress");
    std::thread([&] {
      std::system("pkill mapd");
      std::system("rm -rf /data/media/0/osm/offline");
    }).detach();
    if (ConfirmationDialog::toggle("Reboot required to enable map downloads", "Reboot Now", parent)) {
      Hardware::reboot();
    }
  }
}

void FrogPilotNavigationPanel::downloadMaps(QWidget *parent) {
  std::lock_guard<std::mutex> lock(manageMapsMutex);

  std::thread([&] {
    QStringList states = ButtonSelectionControl::selectedStates.split(',', QString::SkipEmptyParts);
    QStringList countries = ButtonSelectionControl::selectedCountries.split(',', QString::SkipEmptyParts);

    states.removeAll(QString());
    countries.removeAll(QString());

    QJsonObject json;
    if (!states.isEmpty()) {
      json.insert("states", QJsonArray::fromStringList(states));
    }
    if (!countries.isEmpty()) {
      json.insert("nations", QJsonArray::fromStringList(countries));
    }

    paramsMemory.put("OSMDownloadLocations", QJsonDocument(json).toJson(QJsonDocument::Compact).toStdString());
    params.put("MapsSelected", QJsonDocument(json).toJson(QJsonDocument::Compact).toStdString());
  }).detach();

  startTime = std::chrono::steady_clock::now();
}

void FrogPilotNavigationPanel::removeMaps(QWidget *parent) {
  std::lock_guard<std::mutex> lock(manageMapsMutex);
  if (ConfirmationDialog::yesorno("Are you sure you want to delete all of your downloaded maps?", parent)) {
    std::thread([&] {
      std::system("rm -rf /data/media/0/osm/offline");
    }).detach();
  }
}

ManageMaps::ManageMaps(QWidget *parent) : QFrame(parent) {
  back_btn = new QPushButton(tr("Back"), this);
  states_btn = new QPushButton(tr("States"), this);
  countries_btn = new QPushButton(tr("Countries"), this);

  back_btn->setFixedSize(400, 100);
  states_btn->setFixedSize(400, 100);
  countries_btn->setFixedSize(400, 100);

  QHBoxLayout *buttonsLayout = new QHBoxLayout();
  buttonsLayout->addWidget(back_btn);
  buttonsLayout->addWidget(states_btn);
  buttonsLayout->addWidget(countries_btn);

  mapsLayout = new QStackedLayout();
  mapsLayout->setMargin(40);
  mapsLayout->setSpacing(20);

  QVBoxLayout *mainLayout = new QVBoxLayout(this);
  mainLayout->addLayout(buttonsLayout);
  mainLayout->addLayout(mapsLayout);

  QWidget *buttonsWidget = new QWidget();
  buttonsWidget->setLayout(buttonsLayout);
  mapsLayout->addWidget(buttonsWidget);

  QObject::connect(back_btn, &QPushButton::clicked, this, [this]() { emit backPress(); });

  statesList = new ListWidget();

  northeastLabel = new LabelControl(tr("United States - Northeast"), "");
  statesList->addItem(northeastLabel);

  ButtonSelectionControl *northeastControl = new ButtonSelectionControl("", tr(""), tr(""), northeastMap, false);
  statesList->addItem(northeastControl);

  midwestLabel = new LabelControl(tr("United States - Midwest"), "");
  statesList->addItem(midwestLabel);

  ButtonSelectionControl *midwestControl = new ButtonSelectionControl("", tr(""), tr(""), midwestMap, false);
  statesList->addItem(midwestControl);

  southLabel = new LabelControl(tr("United States - South"), "");
  statesList->addItem(southLabel);

  ButtonSelectionControl *southControl = new ButtonSelectionControl("", tr(""), tr(""), southMap, false);
  statesList->addItem(southControl);

  westLabel = new LabelControl(tr("United States - West"), "");
  statesList->addItem(westLabel);

  ButtonSelectionControl *westControl = new ButtonSelectionControl("", tr(""), tr(""), westMap, false);
  statesList->addItem(westControl);

  territoriesLabel = new LabelControl(tr("United States - Territories"), "");
  statesList->addItem(territoriesLabel);

  ButtonSelectionControl *territoriesControl = new ButtonSelectionControl("", tr(""), tr(""), territoriesMap, false);
  statesList->addItem(territoriesControl);

  ScrollView *statesScrollView = new ScrollView(statesList);
  mapsLayout->addWidget(statesScrollView);

  QObject::connect(states_btn, &QPushButton::clicked, this, [this, statesScrollView]() {
    mapsLayout->setCurrentWidget(statesScrollView);
    states_btn->setStyleSheet(activeButtonStyle);
    countries_btn->setStyleSheet(normalButtonStyle);
  });

  countriesList = new ListWidget();

  africaLabel = new LabelControl(tr("Africa"), "");
  countriesList->addItem(africaLabel);

  ButtonSelectionControl *africaControl = new ButtonSelectionControl("", tr(""), tr(""), africaMap, true);
  countriesList->addItem(africaControl);

  antarcticaLabel = new LabelControl(tr("Antarctica"), "");
  countriesList->addItem(antarcticaLabel);

  ButtonSelectionControl *antarcticaControl = new ButtonSelectionControl("", tr(""), tr(""), antarcticaMap, true);
  countriesList->addItem(antarcticaControl);

  asiaLabel = new LabelControl(tr("Asia"), "");
  countriesList->addItem(asiaLabel);

  ButtonSelectionControl *asiaControl = new ButtonSelectionControl("", tr(""), tr(""), asiaMap, true);
  countriesList->addItem(asiaControl);

  europeLabel = new LabelControl(tr("Europe"), "");
  countriesList->addItem(europeLabel);

  ButtonSelectionControl *europeControl = new ButtonSelectionControl("", tr(""), tr(""), europeMap, true);
  countriesList->addItem(europeControl);

  northAmericaLabel = new LabelControl(tr("North America"), "");
  countriesList->addItem(northAmericaLabel);

  ButtonSelectionControl *northAmericaControl = new ButtonSelectionControl("", tr(""), tr(""), northAmericaMap, true);
  countriesList->addItem(northAmericaControl);

  oceaniaLabel = new LabelControl(tr("Oceania"), "");
  countriesList->addItem(oceaniaLabel);

  ButtonSelectionControl *oceaniaControl = new ButtonSelectionControl("", tr(""), tr(""), oceaniaMap, true);
  countriesList->addItem(oceaniaControl);

  southAmericaLabel = new LabelControl(tr("South America"), "");
  countriesList->addItem(southAmericaLabel);

  ButtonSelectionControl *southAmericaControl = new ButtonSelectionControl("", tr(""), tr(""), southAmericaMap, true);
  countriesList->addItem(southAmericaControl);

  ScrollView *countriesScrollView = new ScrollView(countriesList);
  mapsLayout->addWidget(countriesScrollView);

  QObject::connect(countries_btn, &QPushButton::clicked, this, [this, countriesScrollView]() {
    mapsLayout->setCurrentWidget(countriesScrollView);
    states_btn->setStyleSheet(normalButtonStyle);
    countries_btn->setStyleSheet(activeButtonStyle);
  });

  mapsLayout->setCurrentWidget(statesScrollView);
  states_btn->setStyleSheet(activeButtonStyle);

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

QString ManageMaps::activeButtonStyle = R"(
  font-size: 50px;
  margin: 0px;
  padding: 15px;
  border-width: 0;
  border-radius: 30px;
  color: #dddddd;
  background-color: #33Ab4C;
)";

QString ManageMaps::normalButtonStyle = R"(
  font-size: 50px;
  margin: 0px;
  padding: 15px;
  border-width: 0;
  border-radius: 30px;
  color: #dddddd;
  background-color: #393939;
)";

void ManageMaps::hideEvent(QHideEvent *event) {
  QWidget::hideEvent(event);

  emit startDownload();
}

Primeless::Primeless(QWidget *parent) : QWidget(parent), wifi(new WifiManager(this)), list(new ListWidget(this)), 
    setupMapbox(new SetupMapbox(this)), back(new QPushButton(tr("Back"), this)) {

  QVBoxLayout *primelessLayout = new QVBoxLayout(this);
  primelessLayout->setMargin(40);
  primelessLayout->setSpacing(20);

  back->setObjectName("back_btn");
  back->setFixedSize(400, 100);
  QObject::connect(back, &QPushButton::clicked, this, [this]() { emit backPress(); });
  primelessLayout->addWidget(back, 0, Qt::AlignLeft);

  ipLabel = new LabelControl(tr("Manage Your Settings At"), QString("%1:8082").arg(wifi->getIp4Address()));
  list->addItem(ipLabel);

  std::vector<QString> searchOptions{tr("MapBox"), tr("Amap"), tr("Google")};
  searchInput = new ButtonParamControl("SearchInput", tr("Destination Search Provider"),
                                          tr("Select a search provider for destination queries in Navigate on Openpilot. Options include MapBox (recommended), Amap, and Google Maps."),
                                          "",
                                          searchOptions);
  list->addItem(searchInput);

  createMapboxKeyControl(publicMapboxKeyControl, tr("Public Mapbox Key"), "MapboxPublicKey", "pk.");
  createMapboxKeyControl(secretMapboxKeyControl, tr("Secret Mapbox Key"), "MapboxSecretKey", "sk.");

  setupMapbox->setMinimumSize(QSize(1625, 1050));
  setupMapbox->hide();

  setupButton = new ButtonControl(tr("Mapbox Setup Instructions"), tr("VIEW"), tr("View the instructions to set up MapBox for Primeless Navigation."), this);
  QObject::connect(setupButton, &ButtonControl::clicked, this, [this]() {
    back->hide();
    list->setVisible(false);
    setupMapbox->show();
  });
  list->addItem(setupButton);

  QObject::connect(uiState(), &UIState::uiUpdate, this, &Primeless::updateState);

  primelessLayout->addWidget(new ScrollView(list, this));

  setStyleSheet(R"(
    #setupMapbox > QPushButton, #back_btn {
      font-size: 50px;
      margin: 0px;
      padding: 15px;
      border-width: 0;
      border-radius: 30px;
      color: #dddddd;
      background-color: #393939;
    }
    #back_btn:pressed {
      background-color: #4a4a4a;
    }
  )");
}

void Primeless::hideEvent(QHideEvent *event) {
  QWidget::hideEvent(event);
  list->setVisible(true);
  setupMapbox->hide();
}

void Primeless::mousePressEvent(QMouseEvent *event) {
  back->show();
  list->setVisible(true);
  setupMapbox->hide();
}

void Primeless::updateState() {
  if (!isVisible()) return;

  const bool mapboxPublicKeySet = !params.get("MapboxPublicKey").empty();
  const bool mapboxSecretKeySet = !params.get("MapboxSecretKey").empty();

  publicMapboxKeyControl->setText(mapboxPublicKeySet ? tr("REMOVE") : tr("ADD"));
  secretMapboxKeyControl->setText(mapboxSecretKeySet ? tr("REMOVE") : tr("ADD"));

  QString ipAddress = wifi->getIp4Address();
  ipLabel->setText(ipAddress.isEmpty() ? tr("Device Offline") : QString("%1:8082").arg(ipAddress));
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

SetupMapbox::SetupMapbox(QWidget *parent) : QFrame(parent) {
  setAttribute(Qt::WA_OpaquePaintEvent);

  mapboxPublicKeySet = !params.get("MapboxPublicKey").empty();
  mapboxSecretKeySet = !params.get("MapboxSecretKey").empty();
  setupCompleted = mapboxPublicKeySet && mapboxSecretKeySet;

  QObject::connect(uiState(), &UIState::uiUpdate, this, &SetupMapbox::updateState);
}

void SetupMapbox::updateState() {
  if (!isVisible()) return;

  mapboxPublicKeySet = !params.get("MapboxPublicKey").empty();
  mapboxSecretKeySet = !params.get("MapboxSecretKey").empty();

  if (!mapboxPublicKeySet || !mapboxSecretKeySet) {
    setupCompleted = false;
  }

  QString newStep = setupCompleted ? "setup_completed" : 
                    mapboxPublicKeySet && mapboxSecretKeySet ? "both_keys_set" :
                    mapboxPublicKeySet ? "public_key_set" : "no_keys_set";

  if (newStep != currentStep) {
    currentStep = newStep;
    currentImage = QImage(QString::fromUtf8(imagePath) + currentStep + ".png");
    repaint();
  }
}

void SetupMapbox::paintEvent(QPaintEvent *event) {
  QPainter painter(this);

  if (!currentImage.isNull()) {
    int x = qMax(0, (width() - currentImage.width()) / 2);
    int y = qMax(0, (height() - currentImage.height()) / 2);

    painter.drawImage(QPoint(x, y), currentImage);
  }
}
