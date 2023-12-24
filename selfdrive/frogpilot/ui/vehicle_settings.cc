#include <filesystem>
#include <QDir>
#include <QRegularExpression>
#include <QTextStream>

#include "selfdrive/frogpilot/ui/vehicle_settings.h"
#include "selfdrive/ui/ui.h"

QStringList getCarNames(const QString &dirPath, const QString &carMake) {
  QMap<QString, QString> makeMap;
  makeMap["acura"] = "honda";
  makeMap["audi"] = "volkswagen";
  makeMap["buick"] = "gm";
  makeMap["cadillac"] = "gm";
  makeMap["chevrolet"] = "gm";
  makeMap["chrysler"] = "chrysler";
  makeMap["dodge"] = "chrysler";
  makeMap["ford"] = "ford";
  makeMap["gm"] = "gm";
  makeMap["gmc"] = "gm";
  makeMap["genesis"] = "hyundai";
  makeMap["honda"] = "honda";
  makeMap["hyundai"] = "hyundai";
  makeMap["infiniti"] = "nissan";
  makeMap["jeep"] = "chrysler";
  makeMap["kia"] = "hyundai";
  makeMap["lexus"] = "toyota";
  makeMap["lincoln"] = "ford";
  makeMap["man"] = "volkswagen";
  makeMap["mazda"] = "mazda";
  makeMap["nissan"] = "nissan";
  makeMap["ram"] = "chrysler";
  makeMap["seat"] = "volkswagen";
  makeMap["subaru"] = "subaru";
  makeMap["tesla"] = "tesla";
  makeMap["toyota"] = "toyota";
  makeMap["volkswagen"] = "volkswagen";
  makeMap["skoda"] = "volkswagen";

  QStringList names;
  QDir dir(dirPath);
  QString lowerCaseCarMake = carMake.toLower();

  QString targetFolder = makeMap.value(lowerCaseCarMake, lowerCaseCarMake);

  foreach (const QString &folder, dir.entryList(QDir::Dirs | QDir::NoDotAndDotDot)) {
    if (folder.toLower() == targetFolder) {
      QFile file(dirPath + "/" + folder + "/values.py");
      if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QRegularExpression regex("class CAR\\(StrEnum\\):([\\s\\S]*?)(?=^\\w)", QRegularExpression::MultilineOption);
        QRegularExpressionMatch match = regex.match(QTextStream(&file).readAll());
        file.close();

        if (match.hasMatch()) {
          QRegularExpression nameRegex("=\\s*\"([^\"]+)\"");
          QRegularExpressionMatchIterator it = nameRegex.globalMatch(match.captured(1));
          while (it.hasNext()) {
            names << it.next().captured(1);
          }
        }
      }
    }
  }

  std::sort(names.begin(), names.end());
  return names;
}

FrogPilotVehiclesPanel::FrogPilotVehiclesPanel(SettingsWindow *parent) : ListWidget(parent) {
  selectMakeButton = new ButtonControl(tr("Select Make"), tr("SELECT"));
  connect(selectMakeButton, &ButtonControl::clicked, [this]() {
    std::string currentModel = params.get("CarMake");
    QStringList makes = {
      "Acura", "Audi", "BMW", "Buick", "Cadillac", "Chevrolet", "Chrysler", "Dodge", "Ford", "GM", "GMC",
      "Genesis", "Honda", "Hyundai", "Infiniti", "Jeep", "Kia", "Lexus", "Lincoln", "MAN", "Mazda",
      "Mercedes", "Nissan", "Ram", "SEAT", "Subaru", "Tesla", "Toyota", "Volkswagen", "Volvo", "Škoda",
    };

    const QString newMakeSelection = MultiOptionDialog::getSelection(tr("Select a Make"), makes, QString::fromStdString(currentModel), this);
    if (!newMakeSelection.isEmpty()) {
      params.put("CarMake", newMakeSelection.toStdString());
      selectMakeButton->setValue(newMakeSelection);
      setModels();
    }
  });
  addItem(selectMakeButton);

  selectModelButton = new ButtonControl(tr("Select Model"), tr("SELECT"));
  const QString modelSelection = QString::fromStdString(params.get("CarModel"));
  connect(selectModelButton, &ButtonControl::clicked, [this]() {
    const std::string currentModel = params.get("CarModel");
    const QString newModelSelection = MultiOptionDialog::getSelection(tr("Select a Model"), models, QString::fromStdString(currentModel), this);
    if (!newModelSelection.isEmpty()) {
      params.put("CarModel", newModelSelection.toStdString());
      selectModelButton->setValue(newModelSelection);
    }
  });
  selectModelButton->setValue(modelSelection);
  addItem(selectModelButton);

  noToggles = new QLabel(tr("No additional options available for the selected make."));
  noToggles->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  noToggles->setAlignment(Qt::AlignCenter);
  addItem(noToggles);

  const std::vector<std::tuple<QString, QString, QString, QString>> vehicleToggles {
    {"EVTable", "EV Lookup Tables", "Smoothen out the gas and brake controls for EV vehicles.", ""},
    {"LongPitch", "Long Pitch Compensation", "Reduce speed and acceleration error for greater passenger comfort and improved vehicle efficiency.", ""},
    {"LowerVolt", "Lower Volt Enable Speed", "Lower the Volt's minimum enable speed to enable openpilot at any speed.", ""},

    {"LockDoors", "Lock Doors In Drive", "Automatically lock the doors when in drive and unlock when in park.", ""},
    {"SNGHack", "Stop and Go Hack", "Enable the 'Stop and Go' hack for vehicles without stock stop and go functionality.", ""},
    {"TSS2Tune", "TSS2 Tune", "Tuning profile based on the tuning profile from DragonPilot for TSS2 vehicles.", ""}
  };

  for (const auto &[param, title, desc, icon] : vehicleToggles) {
    ParamControl *toggle = new ParamControl(param, title, desc, icon, this);
    addItem(toggle);
    toggles[param.toStdString()] = toggle;

    connect(toggles["TSS2Tune"], &ToggleControl::toggleFlipped, [=]() {
      if (ConfirmationDialog::toggle("Reboot required to take effect.", "Reboot Now", parent)) {
        Hardware::reboot();
      }
    });

    connect(toggle, &ToggleControl::toggleFlipped, [this]() {
      paramsMemory.putBool("FrogPilotTogglesUpdated", true);
    });
  }

  QObject::connect(uiState(), &UIState::uiUpdate, this, [this]() {
    if (this->isVisible()) {
      this->setModels();
    }
  });

  gmKeys = {"EVTable", "LongPitch", "LowerVolt"};
  toyotaKeys = {"LockDoors", "SNGHack", "TSS2Tune"};

  setDefaults();
  setModels();
}

void FrogPilotVehiclesPanel::setModels() {
  std::thread([this] {
    carMake = QString::fromStdString(params.get("CarMake"));
    const QString dirPath = "../../selfdrive/car";
    models = getCarNames(dirPath, carMake);
    setToggles();
  }).detach();
}

void FrogPilotVehiclesPanel::setToggles() {
  std::thread([this] {
    selectModelButton->setVisible(!carMake.isEmpty());

    if (!carMake.isEmpty()) {
      if (previousCarMake == carMake) return;
    }

    previousCarMake = carMake;
    selectMakeButton->setValue(carMake);

    const bool gm = carMake == "Buick" || carMake == "Cadillac" || carMake == "Chevrolet" || carMake == "GM" || carMake == "GMC";
    const bool toyota = carMake == "Lexus" || carMake == "Toyota";

    for (const auto &[key, toggle] : toggles) {
      const bool gmToggles = gmKeys.find(key.c_str()) != gmKeys.end();
      const bool toyotaToggles = toyotaKeys.find(key.c_str()) != toyotaKeys.end();

      if (gm) {
        toggle->setVisible(gmToggles);
      } else if (toyota) {
        toggle->setVisible(toyotaToggles);
      } else {
        toggle->setVisible(false);
      }

      noToggles->setVisible(!(gm || toyota));
    }
  }).detach();
}

void FrogPilotVehiclesPanel::setDefaults() {
  const bool FrogsGoMoo = params.get("DongleId").substr(0, 3) == "be6";

  const std::map<std::string, std::string> defaultValues {
    {"EVTable", FrogsGoMoo ? "0" : "1"},
    {"LongPitch", FrogsGoMoo ? "0" : "1"},
    {"LowerVolt", FrogsGoMoo ? "0" : "1"},
    {"LockDoors", "0"},
    {"SNGHack", "1"},
    {"TSS2Tune", "1"},
  };

  bool rebootRequired = false;
  for (const auto &[key, value] : defaultValues) {
    if (params.get(key).empty()) {
      params.put(key, value);
      rebootRequired = true;
    }
  }

  if (rebootRequired) {
    while (!std::filesystem::exists("/data/openpilot/prebuilt")) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    Hardware::reboot();
  }
}
