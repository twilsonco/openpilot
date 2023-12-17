#include <QDir>
#include <QRegularExpression>
#include <QTextStream>

#include "selfdrive/frogpilot/ui/vehicle_settings.h"

QStringList getCarNames(const QString &dirPath, const QString &carBrand) {
  QMap<QString, QString> brandMap;
  brandMap["acura"] = "honda";
  brandMap["audi"] = "volkswagen";
  brandMap["buick"] = "gm";
  brandMap["cadillac"] = "gm";
  brandMap["chevrolet"] = "gm";
  brandMap["chrysler"] = "chrysler";
  brandMap["dodge"] = "chrysler";
  brandMap["ford"] = "ford";
  brandMap["gm"] = "gm";
  brandMap["gmc"] = "gm";
  brandMap["genesis"] = "hyundai";
  brandMap["honda"] = "honda";
  brandMap["hyundai"] = "hyundai";
  brandMap["infiniti"] = "nissan";
  brandMap["jeep"] = "chrysler";
  brandMap["kia"] = "hyundai";
  brandMap["lexus"] = "toyota";
  brandMap["lincoln"] = "ford";
  brandMap["man"] = "volkswagen";
  brandMap["mazda"] = "mazda";
  brandMap["nissan"] = "nissan";
  brandMap["ram"] = "chrysler";
  brandMap["seat"] = "volkswagen";
  brandMap["subaru"] = "subaru";
  brandMap["tesla"] = "tesla";
  brandMap["toyota"] = "toyota";
  brandMap["volkswagen"] = "volkswagen";
  brandMap["skoda"] = "volkswagen";

  QStringList names;
  QDir dir(dirPath);
  QString lowerCaseCarBrand = carBrand.toLower();

  QString targetFolder = brandMap.value(lowerCaseCarBrand, lowerCaseCarBrand);

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
  brandSelection = QString::fromStdString(params.get("CarBrand"));
  connect(selectMakeButton, &ButtonControl::clicked, [this]() {
    std::string currentBrand = params.get("CarBrand");
    QStringList makes = {
      "Acura", "Audi", "BMW", "Buick", "Cadillac", "Chevrolet", "Chrysler", "Dodge", "Ford", "GM", "GMC", 
      "Genesis", "Honda", "Hyundai", "Infiniti", "Jeep", "Kia", "Lexus", "Lincoln", "MAN", "Mazda", 
      "Mercedes", "Nissan", "Ram", "SEAT", "Subaru", "Tesla", "Toyota", "Volkswagen", "Volvo", "Škoda"
    };

    brandSelection = MultiOptionDialog::getSelection(tr("Select a Make"), makes, QString::fromStdString(currentBrand), this);
    if (!brandSelection.isEmpty()) {
      params.put("CarBrand", brandSelection.toStdString());
      selectMakeButton->setValue(brandSelection);
      setModels();
      setToggles();
    }
  });
  selectMakeButton->setValue(brandSelection);
  addItem(selectMakeButton);

  selectModelButton = new ButtonControl(tr("Select Model"), tr("SELECT"));
  connect(selectModelButton, &ButtonControl::clicked, [this]() {
    std::string currentModel = params.get("CarModel");
    QString modelSelection = MultiOptionDialog::getSelection(tr("Select a Model"), models, QString::fromStdString(currentModel), this);
    if (!modelSelection.isEmpty()) {
      params.put("CarModel", modelSelection.toStdString());
      selectModelButton->setValue(modelSelection);
    }
  });
  selectModelButton->setValue(QString::fromStdString(params.get("CarModel")));
  addItem(selectModelButton);

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

  setModels();
  setToggles();
}

void FrogPilotVehiclesPanel::setModels() {
  std::string carBrand = params.get("CarBrand");
  QString dirPath = "../../selfdrive/car";
  models = getCarNames(dirPath, QString::fromStdString(carBrand));
}

void FrogPilotVehiclesPanel::setToggles() {
  this->update();
}
