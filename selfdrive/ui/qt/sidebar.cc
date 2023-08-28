#include "selfdrive/ui/qt/sidebar.h"

#include <QMouseEvent>

#include "common/params.h"
#include "selfdrive/ui/qt/util.h"

void Sidebar::drawMetric(QPainter &p, const QPair<QString, QString> &label, QColor c, int y) {
  const QRect rect = {30, y, 240, 126};

  p.setPen(Qt::NoPen);
  p.setBrush(QBrush(c));
  p.setClipRect(rect.x() + 4, rect.y(), 18, rect.height(), Qt::ClipOperation::ReplaceClip);
  p.drawRoundedRect(QRect(rect.x() + 4, rect.y() + 4, 100, 118), 18, 18);
  p.setClipping(false);

  QPen pen = QPen(QColor(0xff, 0xff, 0xff, 0x55));
  pen.setWidth(2);
  p.setPen(pen);
  p.setBrush(Qt::NoBrush);
  p.drawRoundedRect(rect, 20, 20);

  p.setPen(QColor(0xff, 0xff, 0xff));
  p.setFont(InterFont(35, QFont::DemiBold));
  p.drawText(rect.adjusted(22, 0, 0, 0), Qt::AlignCenter, label.first + "\n" + label.second);
}

Sidebar::Sidebar(QWidget *parent) : QFrame(parent), onroad(false), flag_pressed(false), settings_pressed(false) {
  home_img = loadPixmap("../assets/images/button_home.png", home_btn.size());
  flag_img = loadPixmap("../assets/images/button_flag.png", home_btn.size());
  settings_img = loadPixmap("../assets/images/button_settings.png", settings_btn.size(), Qt::IgnoreAspectRatio);

  connect(this, &Sidebar::valueChanged, [=] { update(); });

  setAttribute(Qt::WA_OpaquePaintEvent);
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
  setFixedWidth(300);

  QObject::connect(uiState(), &UIState::uiUpdate, this, &Sidebar::updateState);

  pm = std::make_unique<PubMaster, const std::initializer_list<const char *>>({"userFlag"});

  // FrogPilot variables
  static auto params = Params();
  isDeveloperUI = params.getInt("DeveloperUI");

  const bool isFrogTheme = params.getBool("FrogTheme");
  isFrogColors = isFrogTheme && params.getBool("FrogColors");
  const bool isFrogIcons = isFrogTheme && params.getBool("FrogIcons");

  isFahrenheit = params.getBool("Fahrenheit");
  isNumericalTemp = params.getBool("NumericalTemp");

  if (isFrogIcons) {
    flag_img = loadPixmap("../assets/images/frog_button_home.png", home_btn.size());
    home_img = loadPixmap("../assets/images/frog_button_home.png", home_btn.size());
    settings_img = loadPixmap("../assets/images/frog_button_settings.png", settings_btn.size(), Qt::IgnoreAspectRatio);
  }
}

void Sidebar::mousePressEvent(QMouseEvent *event) {
  QRect tempRect = {30, 338, 240, 126};
  if (tempRect.contains(event->pos()) && isNumericalTemp) {
    isFahrenheit = !isFahrenheit;
    Params().putBool("Fahrenheit", isFahrenheit);
    update();
  } else if (onroad && home_btn.contains(event->pos())) {
    flag_pressed = true;
    update();
  } else if (settings_btn.contains(event->pos())) {
    settings_pressed = true;
    update();
  }
}

void Sidebar::mouseReleaseEvent(QMouseEvent *event) {
  if (flag_pressed || settings_pressed) {
    flag_pressed = settings_pressed = false;
    update();
  }
  if (home_btn.contains(event->pos())) {
    MessageBuilder msg;
    msg.initEvent().initUserFlag();
    pm->send("userFlag", msg);
  } else if (settings_btn.contains(event->pos())) {
    emit openSettings();
  }
}

void Sidebar::offroadTransition(bool offroad) {
  onroad = !offroad;
  update();
}

void Sidebar::updateState(const UIState &s) {
  if (!isVisible()) return;

  auto &sm = *(s.sm);

  auto deviceState = sm["deviceState"].getDeviceState();
  setProperty("netType", network_type[deviceState.getNetworkType()]);
  int strength = (int)deviceState.getNetworkStrength();
  setProperty("netStrength", strength > 0 ? strength + 1 : 0);

  // FrogPilot properties
  auto cpu_loads = deviceState.getCpuUsagePercent();
  int cpu_usage = std::accumulate(cpu_loads.begin(), cpu_loads.end(), 0) / cpu_loads.size();
  int maxTempC = deviceState.getMaxTempC();
  int memory_usage = deviceState.getMemoryUsagePercent();
  QString cpu = QString::number(cpu_usage) + "%";
  QString max_temp = isFahrenheit || (isDeveloperUI == 1 && isFahrenheit) ? QString::number(maxTempC * 9 / 5 + 32) + "°F" : QString::number(maxTempC) + "°C";
  QString memory = QString::number(memory_usage) + "%";

  // Developer UI
  if (isDeveloperUI) {
    ItemStatus cpuStatus = {{tr("CPU"), cpu}, isFrogColors ? frog_color : good_color};
    if (cpu_usage >= 85) {
      cpuStatus = {{tr("CPU"), cpu}, danger_color};
    } else if (cpu_usage >= 70) {
      cpuStatus = {{tr("CPU"), cpu}, warning_color};
    }
    ItemStatus memoryStatus = {{tr("MEMORY"), memory}, isFrogColors ? frog_color : good_color};
    if (memory_usage >= 85) {
      memoryStatus = {{tr("MEMORY"), memory}, danger_color};
    } else if (memory_usage >= 70) {
      memoryStatus = {{tr("MEMORY"), memory}, warning_color};
    }
    setProperty("cpuStatus", QVariant::fromValue(cpuStatus));
    setProperty("memoryStatus", QVariant::fromValue(memoryStatus));
  }

  ItemStatus connectStatus;
  auto last_ping = deviceState.getLastAthenaPingTime();
  if (last_ping == 0) {
    connectStatus = ItemStatus{{tr("CONNECT"), tr("OFFLINE")}, warning_color};
  } else {
    connectStatus = nanos_since_boot() - last_ping < 80e9
                        ? ItemStatus{{tr("CONNECT"), tr("ONLINE")}, isFrogColors ? frog_color : good_color}
                        : ItemStatus{{tr("CONNECT"), tr("ERROR")}, danger_color};
  }
  setProperty("connectStatus", QVariant::fromValue(connectStatus));

  ItemStatus tempStatus = {{tr("TEMP"), isNumericalTemp ? max_temp : tr("HIGH")}, danger_color};
  auto ts = deviceState.getThermalStatus();
  if (ts == cereal::DeviceState::ThermalStatus::GREEN) {
    tempStatus = {{tr("TEMP"), isNumericalTemp ? max_temp : tr("GOOD")}, isFrogColors ? frog_color : good_color};
  } else if (ts == cereal::DeviceState::ThermalStatus::YELLOW) {
    tempStatus = {{tr("TEMP"), isNumericalTemp ? max_temp : tr("OK")}, warning_color};
  }
  setProperty("tempStatus", QVariant::fromValue(tempStatus));

  ItemStatus pandaStatus = {{tr("VEHICLE"), tr("ONLINE")}, isFrogColors ? frog_color : good_color};
  if (s.scene.pandaType == cereal::PandaState::PandaType::UNKNOWN) {
    pandaStatus = {{tr("NO"), tr("PANDA")}, danger_color};
  } else if (s.scene.started && !sm["liveLocationKalman"].getLiveLocationKalman().getGpsOK()) {
    pandaStatus = {{tr("GPS"), tr("SEARCH")}, warning_color};
  }
  setProperty("pandaStatus", QVariant::fromValue(pandaStatus));
}

void Sidebar::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setPen(Qt::NoPen);
  p.setRenderHint(QPainter::Antialiasing);

  p.fillRect(rect(), QColor(57, 57, 57));

  // buttons
  p.setOpacity(settings_pressed ? 0.65 : 1.0);
  p.drawPixmap(settings_btn.x(), settings_btn.y(), settings_img);
  p.setOpacity(onroad && flag_pressed ? 0.65 : 1.0);
  p.drawPixmap(home_btn.x(), home_btn.y(), onroad ? flag_img : home_img);
  p.setOpacity(1.0);

  // network
  int x = 58;
  const QColor gray(0x54, 0x54, 0x54);
  for (int i = 0; i < 5; ++i) {
    p.setBrush(i < net_strength ? Qt::white : gray);
    p.drawEllipse(x, 196, 27, 27);
    x += 37;
  }

  p.setFont(InterFont(35));
  p.setPen(QColor(0xff, 0xff, 0xff));
  const QRect r = QRect(50, 247, 100, 50);
  p.drawText(r, Qt::AlignCenter, net_type);

  // metrics
  if (isDeveloperUI) {
    drawMetric(p, temp_status.first, temp_status.second, 338);
    drawMetric(p, cpu_status.first, cpu_status.second, 496);
    drawMetric(p, memory_status.first, memory_status.second, 654);
  } else {
    drawMetric(p, temp_status.first, temp_status.second, 338);
    drawMetric(p, panda_status.first, panda_status.second, 496);
    drawMetric(p, connect_status.first, connect_status.second, 654);
  }
}
