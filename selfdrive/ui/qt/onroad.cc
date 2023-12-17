#include "selfdrive/ui/qt/onroad.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <sstream>

#include <QApplication>
#include <QDebug>
#include <QMouseEvent>

#include "common/timing.h"
#include "selfdrive/ui/qt/util.h"
#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map_helpers.h"
#include "selfdrive/ui/qt/maps/map_panel.h"
#endif

static void drawIcon(QPainter &p, const QPoint &center, const QPixmap &img, const QBrush &bg, float opacity) {
  p.setRenderHint(QPainter::Antialiasing);
  p.setOpacity(1.0);  // bg dictates opacity of ellipse
  p.setPen(Qt::NoPen);
  p.setBrush(bg);
  p.drawEllipse(center, btn_size / 2, btn_size / 2);
  p.setOpacity(opacity);
  p.drawPixmap(center - QPoint(img.width() / 2, img.height() / 2), img);
  p.setOpacity(1.0);
}

static void drawIconRotate(QPainter &p, const QPoint &center, const QPixmap &img, const QBrush &bg, float opacity, const int angle) {
  p.setRenderHint(QPainter::Antialiasing);
  p.setOpacity(1.0);  // bg dictates opacity of ellipse
  p.setPen(Qt::NoPen);
  p.setBrush(bg);
  p.drawEllipse(center, btn_size / 2, btn_size / 2);
  p.save();
  p.translate(center);
  p.rotate(-angle);
  p.setOpacity(opacity);
  p.drawPixmap(-QPoint(img.width() / 2, img.height() / 2), img); 
  p.setOpacity(1.0);
  p.restore();
}

OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent), scene(uiState()->scene) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  main_layout->setMargin(UI_BORDER_SIZE);
  QStackedLayout *stacked_layout = new QStackedLayout;
  stacked_layout->setStackingMode(QStackedLayout::StackAll);
  main_layout->addLayout(stacked_layout);

  nvg = new AnnotatedCameraWidget(VISION_STREAM_ROAD, this);

  QWidget * split_wrapper = new QWidget;
  split = new QHBoxLayout(split_wrapper);
  split->setContentsMargins(0, 0, 0, 0);
  split->setSpacing(0);
  split->addWidget(nvg);

  if (getenv("DUAL_CAMERA_VIEW")) {
    CameraWidget *arCam = new CameraWidget("camerad", VISION_STREAM_ROAD, true, this);
    split->insertWidget(0, arCam);
  }

  if (getenv("MAP_RENDER_VIEW")) {
    CameraWidget *map_render = new CameraWidget("navd", VISION_STREAM_MAP, false, this);
    split->insertWidget(0, map_render);
  }

  stacked_layout->addWidget(split_wrapper);

  alerts = new OnroadAlerts(this);
  alerts->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  stacked_layout->addWidget(alerts);

  // setup stacking order
  alerts->raise();

  setAttribute(Qt::WA_OpaquePaintEvent);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &OnroadWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &OnroadWindow::offroadTransition);
  QObject::connect(uiState(), &UIState::primeChanged, this, &OnroadWindow::primeChanged);

  QObject::connect(&clickTimer, &QTimer::timeout, this, [this]() {
    clickTimer.stop();
    QMouseEvent *event = new QMouseEvent(QEvent::MouseButtonPress, timeoutPoint, Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::postEvent(this, event);
  });
}

void OnroadWindow::updateState(const UIState &s) {
  if (!s.scene.started) {
    return;
  }

  QColor bgColor = bg_colors[s.status];
  Alert alert = Alert::get(*(s.sm), s.scene.started_frame);
  alerts->updateAlert(alert);

  if (s.scene.map_on_left) {
    split->setDirection(QBoxLayout::LeftToRight);
  } else {
    split->setDirection(QBoxLayout::RightToLeft);
  }

  nvg->updateState(s);

  if (bg != bgColor) {
    // repaint border
    bg = bgColor;
    update();
  }

#ifdef ENABLE_MAPS
  // Make sure the sidebar is always closed when the map is ope
  if (map != nullptr) {
    if (geometry().x() > 0 && map->isVisible()) {
      clickTimer.start(1);
    }
  }
#endif
}

void OnroadWindow::mousePressEvent(QMouseEvent* e) {
  // FrogPilot clickable widgets
  bool widgetClicked = false;

  // Change cruise control increments button
  const QRect maxSpeedRect(7, 25, 225, 225);
  const bool isMaxSpeedClicked = maxSpeedRect.contains(e->pos());

  // Hide speed button
  const QRect speedRect(rect().center().x() - 175, 50, 350, 350);
  const bool isSpeedClicked = speedRect.contains(e->pos());

  if (isMaxSpeedClicked || isSpeedClicked) {
    // Check if the click was within the max speed area
    if (isMaxSpeedClicked) {
      reverseCruise = !params.getBool("ReverseCruise");
      params.putBoolNonBlocking("ReverseCruise", reverseCruise);
      paramsMemory.putBoolNonBlocking("FrogPilotTogglesUpdated", true);
    // Check if the click was within the speed text area
    } else {
      speedHidden = !params.getBool("HideSpeed");
      params.putBoolNonBlocking("HideSpeed", speedHidden);
    }
    widgetClicked = true;
  // If the click wasn't for anything specific, change the value of "ExperimentalMode"
  } else if (scene.experimental_mode_via_press && e->pos() != timeoutPoint) {
    if (clickTimer.isActive()) {
      clickTimer.stop();
      if (scene.conditional_experimental) {
        const int override_value = (scene.conditional_status >= 1 && scene.conditional_status <= 4) ? 0 : scene.conditional_status >= 5 ? 3 : 4;
        paramsMemory.putIntNonBlocking("CEStatus", override_value);
      } else {
        const bool experimentalMode = params.getBool("ExperimentalMode");
        params.putBoolNonBlocking("ExperimentalMode", !experimentalMode);
      }
    } else {
      clickTimer.start(500);
    }
    widgetClicked = true;
  }

#ifdef ENABLE_MAPS
  if (map != nullptr && !widgetClicked) {
    // Switch between map and sidebar when using navigate on openpilot
    bool sidebarVisible = geometry().x() > 0;
    bool show_map = uiState()->scene.navigate_on_openpilot ? sidebarVisible : !sidebarVisible;
    if (!scene.experimental_mode_via_press || map->isVisible()) {
      map->setVisible(show_map && !map->isVisible());
    }
  }
#endif
  // propagation event to parent(HomeWindow)
  if (!widgetClicked) {
    QWidget::mousePressEvent(e);
  }
}

void OnroadWindow::offroadTransition(bool offroad) {
#ifdef ENABLE_MAPS
  if (!offroad) {
    if (map == nullptr && (uiState()->hasPrime() || !MAPBOX_TOKEN.isEmpty())) {
      auto m = new MapPanel(get_mapbox_settings());
      map = m;

      QObject::connect(m, &MapPanel::mapPanelRequested, this, &OnroadWindow::mapPanelRequested);
      QObject::connect(nvg->map_settings_btn, &MapSettingsButton::clicked, m, &MapPanel::toggleMapSettings);
      QObject::connect(nvg->map_settings_btn_bottom, &MapSettingsButton::clicked, m, &MapPanel::toggleMapSettings);
      nvg->map_settings_btn->setEnabled(true);

      m->setFixedWidth(topWidget(this)->width() / 2 - UI_BORDER_SIZE);
      split->insertWidget(0, m);

      // hidden by default, made visible when navRoute is published
      m->setVisible(false);
    }
  }
#endif

  alerts->updateAlert({});
}

void OnroadWindow::primeChanged(bool prime) {
#ifdef ENABLE_MAPS
  if (map && (!prime && MAPBOX_TOKEN.isEmpty())) {
    nvg->map_settings_btn->setEnabled(false);
    nvg->map_settings_btn->setVisible(false);
    map->deleteLater();
    map = nullptr;
  }
#endif
}

void OnroadWindow::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.fillRect(rect(), QColor(bg.red(), bg.green(), bg.blue(), 255));

  // Draw FPS on screen
  if (scene.show_fps) {
    constexpr double minAllowedFPS = 0.1;
    constexpr double maxAllowedFPS = 99.9;
    constexpr qint64 oneMinuteInMilliseconds = 60000;
    static double avgFPS;
    static double maxFPS;
    static double minFPS;
    static double totalFPS;
    static qint64 frameCount;

    // Store the last reset time
    static qint64 lastResetTime = QDateTime::currentMSecsSinceEpoch();
    const qint64 currentMillis = QDateTime::currentMSecsSinceEpoch();

    // Reset the counter if it's been 60 seconds
    if (currentMillis - lastResetTime >= oneMinuteInMilliseconds) {
      avgFPS = 0;
      frameCount = 0;
      minFPS = maxAllowedFPS;
      maxFPS = minAllowedFPS;
      totalFPS = 0;
      lastResetTime = currentMillis;
    }

    // Update the FPS variables
    fps = qBound(minAllowedFPS, fps, maxAllowedFPS);
    minFPS = qMin(minFPS, fps);
    maxFPS = qMax(maxFPS, fps);
    frameCount++;
    totalFPS += fps;
    avgFPS = totalFPS / frameCount;
    update();

    // Text declarations
    p.setFont(InterFont(30, QFont::DemiBold));
    p.setRenderHint(QPainter::TextAntialiasing);
    p.setPen(Qt::white);

    // Construct the FPS display string
    QString fpsDisplayString = QString("FPS: %1 (%2) | Min: %3 | Max: %4 | Avg: %5")
      .arg(fps, 0, 'f', 2)
      .arg(paramsMemory.getInt("CameraFPS"))
      .arg(minFPS, 0, 'f', 2)
      .arg(maxFPS, 0, 'f', 2)
      .arg(avgFPS, 0, 'f', 2);

    // Calculate text positioning
    const QRect currentRect = rect();
    const int textWidth = p.fontMetrics().horizontalAdvance(fpsDisplayString);
    const int xPos = (currentRect.width() - textWidth) / 2;
    const int yPos = currentRect.bottom() - 5;
    p.drawText(xPos, yPos, fpsDisplayString);
  }
}

// ***** onroad widgets *****

// OnroadAlerts
void OnroadAlerts::updateAlert(const Alert &a) {
  if (!alert.equal(a)) {
    alert = a;
    update();
  }
}

void OnroadAlerts::paintEvent(QPaintEvent *event) {
  if (alert.size == cereal::ControlsState::AlertSize::NONE || scene.show_driver_camera) {
    return;
  }
  static std::map<cereal::ControlsState::AlertSize, const int> alert_heights = {
    {cereal::ControlsState::AlertSize::SMALL, 271},
    {cereal::ControlsState::AlertSize::MID, 420},
    {cereal::ControlsState::AlertSize::FULL, height()},
  };
  int h = alert_heights[alert.size];

  int margin = 40;
  int radius = 30;
  int offset = scene.always_on_lateral || scene.conditional_experimental || scene.road_name_ui ? 25 : 0;
  if (alert.size == cereal::ControlsState::AlertSize::FULL) {
    margin = 0;
    radius = 0;
    offset = 0;
  }
  QRect r = QRect(0 + margin, height() - h + margin - offset, width() - margin*2, h - margin*2);

  QPainter p(this);

  // draw background + gradient
  p.setPen(Qt::NoPen);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);
  p.setBrush(QBrush(alert_colors[alert.status]));
  p.drawRoundedRect(r, radius, radius);

  QLinearGradient g(0, r.y(), 0, r.bottom());
  g.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.05));
  g.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0.35));

  p.setCompositionMode(QPainter::CompositionMode_DestinationOver);
  p.setBrush(QBrush(g));
  p.drawRoundedRect(r, radius, radius);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);

  // text
  const QPoint c = r.center();
  p.setPen(QColor(0xff, 0xff, 0xff));
  p.setRenderHint(QPainter::TextAntialiasing);
  if (alert.size == cereal::ControlsState::AlertSize::SMALL) {
    p.setFont(InterFont(74, QFont::DemiBold));
    p.drawText(r, Qt::AlignCenter, alert.text1);
  } else if (alert.size == cereal::ControlsState::AlertSize::MID) {
    p.setFont(InterFont(88, QFont::Bold));
    p.drawText(QRect(0, c.y() - 125, width(), 150), Qt::AlignHCenter | Qt::AlignTop, alert.text1);
    p.setFont(InterFont(66));
    p.drawText(QRect(0, c.y() + 21, width(), 90), Qt::AlignHCenter, alert.text2);
  } else if (alert.size == cereal::ControlsState::AlertSize::FULL) {
    bool l = alert.text1.length() > 15;
    p.setFont(InterFont(l ? 132 : 177, QFont::Bold));
    p.drawText(QRect(0, r.y() + (l ? 240 : 270), width(), 600), Qt::AlignHCenter | Qt::TextWordWrap, alert.text1);
    p.setFont(InterFont(88));
    p.drawText(QRect(0, r.height() - (l ? 361 : 420), width(), 300), Qt::AlignHCenter | Qt::TextWordWrap, alert.text2);
  }
}

// ExperimentalButton
ExperimentalButton::ExperimentalButton(QWidget *parent) : experimental_mode(false), engageable(false), QPushButton(parent), scene(uiState()->scene) {
  setFixedSize(btn_size, btn_size + 10);

  engage_img = loadPixmap("../assets/img_chffr_wheel.png", {img_size, img_size});
  experimental_img = loadPixmap("../assets/img_experimental.svg", {img_size, img_size});
  QObject::connect(this, &QPushButton::clicked, this, &ExperimentalButton::changeMode);

  // Custom steering wheel images
  wheelImages = {
    {0, loadPixmap("../assets/img_chffr_wheel.png", {img_size, img_size})},
    {1, loadPixmap("../frogpilot/assets/wheel_images/lexus.png", {img_size, img_size})},
    {2, loadPixmap("../frogpilot/assets/wheel_images/toyota.png", {img_size, img_size})},
    {3, loadPixmap("../frogpilot/assets/wheel_images/frog.png", {img_size, img_size})},
    {4, loadPixmap("../frogpilot/assets/wheel_images/rocket.png", {img_size, img_size})},
    {5, loadPixmap("../frogpilot/assets/wheel_images/hyundai.png", {img_size, img_size})},
    {6, loadPixmap("../frogpilot/assets/wheel_images/stalin.png", {img_size, img_size})}
  };
}

void ExperimentalButton::changeMode() {
  const auto cp = (*uiState()->sm)["carParams"].getCarParams();
  bool can_change = hasLongitudinalControl(cp) && (params.getBool("ExperimentalModeConfirmed") || scene.experimental_mode_via_press);
  if (can_change) {
    if (scene.conditional_experimental) {
      const int override_value = (scene.conditional_status >= 1 && scene.conditional_status <= 4) ? 0 : scene.conditional_status >= 5 ? 3 : 4;
      paramsMemory.putIntNonBlocking("ConditionalStatus", override_value);
    } else {
      params.putBoolNonBlocking("ExperimentalMode", !experimental_mode);
    }
  }
}

void ExperimentalButton::updateState(const UIState &s, bool leadInfo) {
  const auto cs = (*s.sm)["controlsState"].getControlsState();
  bool eng = cs.getEngageable() || cs.getEnabled();
  if ((cs.getExperimentalMode() != experimental_mode) || (eng != engageable)) {
    engageable = eng;
    experimental_mode = cs.getExperimentalMode();
    update();
  }

  // FrogPilot variables
  rotatingWheel = scene.rotating_wheel;
  wheelIcon = scene.wheel_icon;

  y_offset = leadInfo ? 10 : 0;

  // Update the icon so the steering wheel rotates in real time
  if (rotatingWheel && steeringAngleDeg != scene.steering_angle_deg) {
    steeringAngleDeg = scene.steering_angle_deg;
    update();
  }
}

void ExperimentalButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  // Custom steering wheel icon
  engage_img = wheelImages[wheelIcon];
  QPixmap img = wheelIcon ? engage_img : (experimental_mode ? experimental_img : engage_img);

  const QColor background_color = wheelIcon && !isDown() && engageable ?
      (scene.conditional_status == 1 ? QColor(255, 246, 0, 255) :
      (experimental_mode ? QColor(218, 111, 37, 241) :
      (scene.navigate_on_openpilot ? QColor(49, 161, 238, 255) : QColor(0, 0, 0, 166)))) :
      (scene.always_on_lateral_active ? QColor(10, 186, 181, 255) :
      QColor(0, 0, 0, 166));

  if (!scene.show_driver_camera) {
    if (rotatingWheel) {
      drawIconRotate(p, QPoint(btn_size / 2, btn_size / 2 + y_offset), img, background_color, (isDown() || (!engageable && !scene.always_on_lateral_active)) ? 0.6 : 1.0, steeringAngleDeg);
    } else {
      drawIcon(p, QPoint(btn_size / 2, btn_size / 2 + y_offset), img, background_color, (isDown() || (!engageable && !scene.always_on_lateral_active)) ? 0.6 : 1.0);
    }
  }
}


// MapSettingsButton
MapSettingsButton::MapSettingsButton(QWidget *parent) : QPushButton(parent) {
  setFixedSize(btn_size, btn_size + 20);
  settings_img = loadPixmap("../assets/navigation/icon_directions_outlined.svg", {img_size, img_size});

  // hidden by default, made visible if map is created (has prime or mapbox token)
  setVisible(false);
  setEnabled(false);
}

void MapSettingsButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  drawIcon(p, QPoint(btn_size / 2, btn_size / 2), settings_img, QColor(0, 0, 0, 166), isDown() ? 0.6 : 1.0);
}


// Window that shows camera view and variety of info drawn on top
AnnotatedCameraWidget::AnnotatedCameraWidget(VisionStreamType type, QWidget* parent) : fps_filter(UI_FREQ, 3, 1. / UI_FREQ), CameraWidget("camerad", type, true, parent), scene(uiState()->scene) {
  pm = std::make_unique<PubMaster, const std::initializer_list<const char *>>({"uiDebug"});

  main_layout = new QVBoxLayout(this);
  main_layout->setMargin(UI_BORDER_SIZE);
  main_layout->setSpacing(0);

  // Neokii screen recorder
  QHBoxLayout *top_right_layout = new QHBoxLayout();
  top_right_layout->setSpacing(0);
  recorder_btn = new ScreenRecorder(this);
  top_right_layout->addWidget(recorder_btn);

  experimental_btn = new ExperimentalButton(this);
  top_right_layout->addWidget(experimental_btn);

  main_layout->addLayout(top_right_layout, 0);
  main_layout->setAlignment(top_right_layout, Qt::AlignTop | Qt::AlignRight);

  map_settings_btn = new MapSettingsButton(this);
  main_layout->addWidget(map_settings_btn, 0, Qt::AlignBottom | Qt::AlignRight);

  dm_img = loadPixmap("../assets/img_driver_face.png", {img_size + 5, img_size + 5});

  // Initialize FrogPilot widgets
  initializeFrogPilotWidgets();
}

void AnnotatedCameraWidget::updateState(const UIState &s) {
  const int SET_SPEED_NA = 255;
  const SubMaster &sm = *(s.sm);

  const bool cs_alive = sm.alive("controlsState");
  const bool nav_alive = sm.alive("navInstruction") && sm["navInstruction"].getValid();
  const auto cs = sm["controlsState"].getControlsState();
  const auto car_state = sm["carState"].getCarState();
  const auto nav_instruction = sm["navInstruction"].getNavInstruction();

  // Handle older routes where vCruiseCluster is not set
  float v_cruise =  cs.getVCruiseCluster() == 0.0 ? cs.getVCruise() : cs.getVCruiseCluster();
  setSpeed = cs_alive ? v_cruise : SET_SPEED_NA;
  is_cruise_set = setSpeed > 0 && (int)setSpeed != SET_SPEED_NA;
  if (is_cruise_set && !s.scene.is_metric) {
    setSpeed *= KM_TO_MILE;
  }

  // Handle older routes where vEgoCluster is not set
  v_ego_cluster_seen = v_ego_cluster_seen || car_state.getVEgoCluster() != 0.0;
  float v_ego = v_ego_cluster_seen ? car_state.getVEgoCluster() : car_state.getVEgo();
  speed = cs_alive ? std::max<float>(0.0, v_ego) : 0.0;
  speed *= s.scene.is_metric ? MS_TO_KPH : MS_TO_MPH;

  auto speed_limit_sign = nav_instruction.getSpeedLimitSign();
  speedLimit = nav_alive ? nav_instruction.getSpeedLimit() : 0.0;
  speedLimit *= (s.scene.is_metric ? MS_TO_KPH : MS_TO_MPH);

  has_us_speed_limit = (nav_alive && speed_limit_sign == cereal::NavInstruction::SpeedLimitSign::MUTCD);
  has_eu_speed_limit = (nav_alive && speed_limit_sign == cereal::NavInstruction::SpeedLimitSign::VIENNA);
  is_metric = s.scene.is_metric;
  speedUnit =  s.scene.is_metric ? tr("km/h") : tr("mph");
  hideBottomIcons = (cs.getAlertSize() != cereal::ControlsState::AlertSize::NONE || customSignals && (turnSignalLeft || turnSignalRight)) || showDriverCamera;
  status = s.status;

  // update engageability/experimental mode button
  experimental_btn->updateState(s, leadInfo);

  // update DM icon
  auto dm_state = sm["driverMonitoringState"].getDriverMonitoringState();
  dmActive = dm_state.getIsActiveMode();
  rightHandDM = dm_state.getIsRHD();
  // DM icon transition
  dm_fade_state = std::clamp(dm_fade_state+0.2*(0.5-dmActive), 0.0, 1.0);

  // hide map settings button for alerts and flip for right hand DM
  if (map_settings_btn->isEnabled()) {
    map_settings_btn->setVisible(!hideBottomIcons && compass);
    main_layout->setAlignment(map_settings_btn, (rightHandDM ? Qt::AlignLeft : Qt::AlignRight) | (compass ? Qt::AlignTop : Qt::AlignBottom));
  }
}

void AnnotatedCameraWidget::drawHud(QPainter &p) {
  p.save();

  // Header gradient
  QLinearGradient bg(0, UI_HEADER_HEIGHT - (UI_HEADER_HEIGHT / 2.5), 0, UI_HEADER_HEIGHT);
  bg.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.45));
  bg.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));
  p.fillRect(0, 0, width(), UI_HEADER_HEIGHT, bg);

  QString speedLimitStr = (speedLimit > 1) ? QString::number(std::nearbyint(speedLimit)) : "–";
  QString speedStr = QString::number(std::nearbyint(speed));
  QString setSpeedStr = is_cruise_set ? QString::number(std::nearbyint(setSpeed)) : "–";

  // Draw outer box + border to contain set speed and speed limit
  const int sign_margin = 12;
  const int us_sign_height = 186;
  const int eu_sign_size = 176;

  const QSize default_size = {172, 204};
  QSize set_speed_size = default_size;
  if (is_metric || has_eu_speed_limit) set_speed_size.rwidth() = 200;
  if (has_us_speed_limit && speedLimitStr.size() >= 3) set_speed_size.rwidth() = 223;

  if (has_us_speed_limit) set_speed_size.rheight() += us_sign_height + sign_margin;
  else if (has_eu_speed_limit) set_speed_size.rheight() += eu_sign_size + sign_margin;

  int top_radius = 32;
  int bottom_radius = has_eu_speed_limit ? 100 : 32;

  QRect set_speed_rect(QPoint(60 + (default_size.width() - set_speed_size.width()) / 2, 45), set_speed_size);
  if (reverseCruise) {
    p.setPen(QPen(QColor(0, 150, 255), 6));
  } else {
    p.setPen(QPen(whiteColor(75), 6));
  }
  p.setBrush(blackColor(166));
  drawRoundedRect(p, set_speed_rect, top_radius, top_radius, bottom_radius, bottom_radius);

  // Draw MAX
  QColor max_color = QColor(0x80, 0xd8, 0xa6, 0xff);
  QColor set_speed_color = whiteColor();
  if (is_cruise_set) {
    if (status == STATUS_DISENGAGED) {
      max_color = whiteColor();
    } else if (status == STATUS_OVERRIDE) {
      max_color = QColor(0x91, 0x9b, 0x95, 0xff);
    } else if (speedLimit > 0) {
      auto interp_color = [=](QColor c1, QColor c2, QColor c3) {
        return speedLimit > 0 ? interpColor(setSpeed, {speedLimit + 5, speedLimit + 15, speedLimit + 25}, {c1, c2, c3}) : c1;
      };
      max_color = interp_color(max_color, QColor(0xff, 0xe4, 0xbf), QColor(0xff, 0xbf, 0xbf));
      set_speed_color = interp_color(set_speed_color, QColor(0xff, 0x95, 0x00), QColor(0xff, 0x00, 0x00));
    }
  } else {
    max_color = QColor(0xa6, 0xa6, 0xa6, 0xff);
    set_speed_color = QColor(0x72, 0x72, 0x72, 0xff);
  }
  p.setFont(InterFont(40, QFont::DemiBold));
  p.setPen(max_color);
  p.drawText(set_speed_rect.adjusted(0, 27, 0, 0), Qt::AlignTop | Qt::AlignHCenter, tr("MAX"));
  p.setFont(InterFont(90, QFont::Bold));
  p.setPen(set_speed_color);
  p.drawText(set_speed_rect.adjusted(0, 77, 0, 0), Qt::AlignTop | Qt::AlignHCenter, setSpeedStr);

  const QRect sign_rect = set_speed_rect.adjusted(sign_margin, default_size.height(), -sign_margin, -sign_margin);
  // US/Canada (MUTCD style) sign
  if (has_us_speed_limit) {
    p.setPen(Qt::NoPen);
    p.setBrush(whiteColor());
    p.drawRoundedRect(sign_rect, 24, 24);
    p.setPen(QPen(blackColor(), 6));
    p.drawRoundedRect(sign_rect.adjusted(9, 9, -9, -9), 16, 16);

    p.setFont(InterFont(28, QFont::DemiBold));
    p.drawText(sign_rect.adjusted(0, 22, 0, 0), Qt::AlignTop | Qt::AlignHCenter, tr("SPEED"));
    p.drawText(sign_rect.adjusted(0, 51, 0, 0), Qt::AlignTop | Qt::AlignHCenter, tr("LIMIT"));
    p.setFont(InterFont(70, QFont::Bold));
    p.drawText(sign_rect.adjusted(0, 85, 0, 0), Qt::AlignTop | Qt::AlignHCenter, speedLimitStr);
  }

  // EU (Vienna style) sign
  if (has_eu_speed_limit) {
    p.setPen(Qt::NoPen);
    p.setBrush(whiteColor());
    p.drawEllipse(sign_rect);
    p.setPen(QPen(Qt::red, 20));
    p.drawEllipse(sign_rect.adjusted(16, 16, -16, -16));

    p.setFont(InterFont((speedLimitStr.size() >= 3) ? 60 : 70, QFont::Bold));
    p.setPen(blackColor());
    p.drawText(sign_rect, Qt::AlignCenter, speedLimitStr);
  }

  // current speed
  if (!speedHidden) {
    p.setFont(InterFont(176, QFont::Bold));
    drawText(p, rect().center().x(), 210, speedStr);
    p.setFont(InterFont(66));
    drawText(p, rect().center().x(), 290, speedUnit, 200);
  }

  p.restore();
}

void AnnotatedCameraWidget::drawText(QPainter &p, int x, int y, const QString &text, int alpha) {
  QRect real_rect = p.fontMetrics().boundingRect(text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(QColor(0xff, 0xff, 0xff, alpha));
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

void AnnotatedCameraWidget::initializeGL() {
  CameraWidget::initializeGL();
  qInfo() << "OpenGL version:" << QString((const char*)glGetString(GL_VERSION));
  qInfo() << "OpenGL vendor:" << QString((const char*)glGetString(GL_VENDOR));
  qInfo() << "OpenGL renderer:" << QString((const char*)glGetString(GL_RENDERER));
  qInfo() << "OpenGL language version:" << QString((const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

  prev_draw_t = millis_since_boot();
  setBackgroundColor(bg_colors[STATUS_DISENGAGED]);
}

void AnnotatedCameraWidget::updateFrameMat() {
  CameraWidget::updateFrameMat();
  UIState *s = uiState();
  int w = width(), h = height();

  s->fb_w = w;
  s->fb_h = h;

  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  // 2) Apply same scaling as video
  // 3) Put (0, 0) in top left corner of video
  s->car_space_transform.reset();
  s->car_space_transform.translate(w / 2 - x_offset, h / 2 - y_offset)
      .scale(zoom, zoom)
      .translate(-intrinsic_matrix.v[2], -intrinsic_matrix.v[5]);
}

void AnnotatedCameraWidget::drawLaneLines(QPainter &painter, const UIState *s) {
  painter.save();

  SubMaster &sm = *(s->sm);

  // lanelines
  for (int i = 0; i < std::size(scene.lane_line_vertices); ++i) {
    if (customColors != 0) {
      painter.setBrush(themeConfiguration[customColors].second.first);
    } else {
      painter.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, std::clamp<float>(scene.lane_line_probs[i], 0.0, 0.7)));
    }
    painter.drawPolygon(scene.lane_line_vertices[i]);
  }

  // road edges
  for (int i = 0; i < std::size(scene.road_edge_vertices); ++i) {
    if (customColors != 0) {
      painter.setBrush(themeConfiguration[customColors].second.first);
    } else {
      painter.setBrush(QColor::fromRgbF(1.0, 0, 0, std::clamp<float>(1.0 - scene.road_edge_stds[i], 0.0, 1.0)));
    }
    painter.drawPolygon(scene.road_edge_vertices[i]);
  }

  // paint path
  QLinearGradient bg(0, height(), 0, 0);
  if (sm["controlsState"].getControlsState().getExperimentalMode() || accelerationPath) {
    // The first half of track_vertices are the points for the right side of the path
    // and the indices match the positions of accel from uiPlan
    const auto &acceleration_const = sm["uiPlan"].getUiPlan().getAccel();
    const int max_len = std::min<int>(scene.track_vertices.length() / 2, acceleration_const.size());

    // Copy of the acceleration vector
    std::vector<float> acceleration;
    for (int i = 0; i < acceleration_const.size(); i++) {
      acceleration.push_back(acceleration_const[i]);
    }

    for (int i = 0; i < max_len; ++i) {
      // Some points are out of frame
      if (scene.track_vertices[i].y() < 0 || scene.track_vertices[i].y() > height()) continue;

      // Flip so 0 is bottom of frame
      float lin_grad_point = (height() - scene.track_vertices[i].y()) / height();

      // If acceleration is between -0.2 and 0.2, resort to the theme color
      if (std::abs(acceleration[i]) < 0.2 && (customColors != 0)) {
        const auto &colorMap = themeConfiguration[customColors].second.second;
        for (const auto &[position, brush] : colorMap) {
          bg.setColorAt(position, brush.color());
        }
      } else {
        // speed up: 120, slow down: 0
        float path_hue = fmax(fmin(60 + acceleration[i] * 35, 120), 0);
        // FIXME: painter.drawPolygon can be slow if hue is not rounded
        path_hue = int(path_hue * 100 + 0.5) / 100;

        float saturation = fmin(fabs(acceleration[i] * 1.5), 1);
        float lightness = util::map_val(saturation, 0.0f, 1.0f, 0.95f, 0.62f);  // lighter when grey
        float alpha = util::map_val(lin_grad_point, 0.75f / 2.f, 0.75f, 0.4f, 0.0f);  // matches previous alpha fade
        bg.setColorAt(lin_grad_point, QColor::fromHslF(path_hue / 360., saturation, lightness, alpha));

        // Skip a point, unless next is last
        i += (i + 2) < max_len ? 1 : 0;
      }
    }

  } else if (customColors != 0) {
    const auto &colorMap = themeConfiguration[customColors].second.second;
    for (const auto &[position, brush] : colorMap) {
      bg.setColorAt(position, brush.color());
    }
  } else {
    bg.setColorAt(0.0, QColor::fromHslF(148 / 360., 0.94, 0.51, 0.4));
    bg.setColorAt(0.5, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.35));
    bg.setColorAt(1.0, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.0));
  }

  painter.setBrush(bg);
  painter.drawPolygon(scene.track_vertices);

  // create new path with track vertices and track edge vertices
  QPainterPath path;
  path.addPolygon(scene.track_vertices);
  path.addPolygon(scene.track_edge_vertices);

  // paint path edges
  QLinearGradient pe(0, height(), 0, 0);
  if (alwaysOnLateral) {
    pe.setColorAt(0.0, QColor::fromHslF(178 / 360., 0.90, 0.38, 1.0));
    pe.setColorAt(0.5, QColor::fromHslF(178 / 360., 0.90, 0.38, 0.5));
    pe.setColorAt(1.0, QColor::fromHslF(178 / 360., 0.90, 0.38, 0.1));
  } else if (conditionalStatus == 1 || conditionalStatus == 3) {
    pe.setColorAt(0.0, QColor::fromHslF(58 / 360., 1.00, 0.50, 1.0));
    pe.setColorAt(0.5, QColor::fromHslF(58 / 360., 1.00, 0.50, 0.5));
    pe.setColorAt(1.0, QColor::fromHslF(58 / 360., 1.00, 0.50, 0.1));
  } else if (experimentalMode) {
    pe.setColorAt(0.0, QColor::fromHslF(25 / 360., 0.71, 0.50, 1.0));
    pe.setColorAt(0.5, QColor::fromHslF(25 / 360., 0.71, 0.50, 0.5));
    pe.setColorAt(1.0, QColor::fromHslF(25 / 360., 0.71, 0.50, 0.1));
  } else if (scene.navigate_on_openpilot) {
    pe.setColorAt(0.0, QColor::fromHslF(205 / 360., 0.85, 0.56, 1.0));
    pe.setColorAt(0.5, QColor::fromHslF(205 / 360., 0.85, 0.56, 0.5));
    pe.setColorAt(1.0, QColor::fromHslF(205 / 360., 0.85, 0.56, 0.1));
  } else if (customColors != 0) {
    const auto &colorMap = themeConfiguration[customColors].second.second;
    for (const auto &[position, brush] : colorMap) {
      QColor darkerColor = brush.color().darker(120);
      pe.setColorAt(position, darkerColor);
    }
  } else {
    pe.setColorAt(0.0, QColor::fromHslF(148 / 360., 0.94, 0.51, 1.0));
    pe.setColorAt(0.5, QColor::fromHslF(112 / 360., 1.00, 0.68, 0.5));
    pe.setColorAt(1.0, QColor::fromHslF(112 / 360., 1.00, 0.68, 0.1));
  }

  painter.setBrush(pe);
  painter.drawPath(path);

  // paint blindspot path
  QLinearGradient bs(0, height(), 0, 0);
  if (blindSpotLeft || blindSpotRight) {
    bs.setColorAt(0.0, QColor::fromHslF(0 / 360., 0.75, 0.50, 0.6));
    bs.setColorAt(0.5, QColor::fromHslF(0 / 360., 0.75, 0.50, 0.4));
    bs.setColorAt(1.0, QColor::fromHslF(0 / 360., 0.75, 0.50, 0.2));
  }

  painter.setBrush(bs);
  if (blindSpotLeft) {
    painter.drawPolygon(scene.track_left_adjacent_lane_vertices);
  }
  if (blindSpotRight) {
    painter.drawPolygon(scene.track_right_adjacent_lane_vertices);
  }

  // paint adjacent lane paths
  if (adjacentPath && (laneWidthLeft != 0 || laneWidthRight != 0)) {
    // Set up the units
    const double conversionFactor = is_metric ? 1.0 : 3.28084;
    const QString unit_d = is_metric ? " meters" : " feet";

    // Declare the lane width thresholds
    constexpr float minLaneWidth = 2.5f;
    constexpr float maxLaneWidth = 3.5f;

    // Set gradient colors based on laneWidth and blindspot
    const auto setGradientColors = [](QLinearGradient &gradient, const float laneWidth, const bool blindspot) {
      // Make the path red for smaller paths or if there's a car in the blindspot and green for larger paths
      const double hue = (laneWidth < minLaneWidth || blindspot) ? 0.0 : 
                         (laneWidth >= maxLaneWidth) ? 120.0 : 
                          120.0 * (laneWidth - minLaneWidth) / (maxLaneWidth - minLaneWidth);
      const auto hue_ratio = hue / 360.0;
      gradient.setColorAt(0.0, QColor::fromHslF(hue_ratio, 0.75, 0.50, 0.6));
      gradient.setColorAt(0.5, QColor::fromHslF(hue_ratio, 0.75, 0.50, 0.4));
      gradient.setColorAt(1.0, QColor::fromHslF(hue_ratio, 0.75, 0.50, 0.2));
    };

    // Paint the lanes
    const auto paintLane = [&](QPainter &painter, const QPolygonF &lane, const float laneWidth, const bool blindspot) {
      QLinearGradient gradient(0, height(), 0, 0);
      setGradientColors(gradient, laneWidth, blindspot);
      painter.setFont(InterFont(30, QFont::DemiBold));
      painter.setBrush(gradient);
      painter.setPen(Qt::transparent);
      painter.drawPolygon(lane);
      painter.setPen(Qt::white);

      QRectF boundingRect = lane.boundingRect();
      painter.drawText(boundingRect.center(), blindspot ? "Vehicle in blind spot" : 
                       QString("%1%2").arg(laneWidth * conversionFactor, 0, 'f', 2).arg(unit_d));
      painter.setPen(Qt::NoPen);
    };

    // Paint lanes
    paintLane(painter, scene.track_left_adjacent_lane_vertices, laneWidthLeft, blindSpotLeft);
    paintLane(painter, scene.track_right_adjacent_lane_vertices, laneWidthRight, blindSpotRight);
  }

  painter.restore();
}

void AnnotatedCameraWidget::drawDriverState(QPainter &painter, const UIState *s) {
  painter.save();

  // base icon
  int offset = UI_BORDER_SIZE + btn_size / 2;
  offset += alwaysOnLateral || conditionalExperimental || roadNameUI ? 25 : 0;
  int x = rightHandDM ? width() - offset : offset;
  int y = height() - offset;
  float opacity = dmActive ? 0.65 : 0.2;
  drawIcon(painter, QPoint(x, y), dm_img, blackColor(70), opacity);

  // face
  QPointF face_kpts_draw[std::size(default_face_kpts_3d)];
  float kp;
  for (int i = 0; i < std::size(default_face_kpts_3d); ++i) {
    kp = (scene.face_kpts_draw[i].v[2] - 8) / 120 + 1.0;
    face_kpts_draw[i] = QPointF(scene.face_kpts_draw[i].v[0] * kp + x, scene.face_kpts_draw[i].v[1] * kp + y);
  }

  painter.setPen(QPen(QColor::fromRgbF(1.0, 1.0, 1.0, opacity), 5.2, Qt::SolidLine, Qt::RoundCap));
  painter.drawPolyline(face_kpts_draw, std::size(default_face_kpts_3d));

  // tracking arcs
  const int arc_l = 133;
  const float arc_t_default = 6.7;
  const float arc_t_extend = 12.0;
  QColor arc_color = QColor::fromRgbF(0.545 - 0.445 * s->engaged(),
                                      0.545 + 0.4 * s->engaged(),
                                      0.545 - 0.285 * s->engaged(),
                                      0.4 * (1.0 - dm_fade_state));
  float delta_x = -scene.driver_pose_sins[1] * arc_l / 2;
  float delta_y = -scene.driver_pose_sins[0] * arc_l / 2;
  painter.setPen(QPen(arc_color, arc_t_default+arc_t_extend*fmin(1.0, scene.driver_pose_diff[1] * 5.0), Qt::SolidLine, Qt::RoundCap));
  painter.drawArc(QRectF(std::fmin(x + delta_x, x), y - arc_l / 2, fabs(delta_x), arc_l), (scene.driver_pose_sins[1]>0 ? 90 : -90) * 16, 180 * 16);
  painter.setPen(QPen(arc_color, arc_t_default+arc_t_extend*fmin(1.0, scene.driver_pose_diff[0] * 5.0), Qt::SolidLine, Qt::RoundCap));
  painter.drawArc(QRectF(x - arc_l / 2, std::fmin(y + delta_y, y), arc_l, fabs(delta_y)), (scene.driver_pose_sins[0]>0 ? 0 : 180) * 16, 180 * 16);

  painter.restore();
}

void AnnotatedCameraWidget::drawLead(QPainter &painter, const cereal::RadarState::LeadData::Reader &lead_data, const QPointF &vd) {
  painter.save();

  const float speedBuff = customColors ? 25. : 10.;  // Make the center of the chevron appear sooner if a custom theme is active
  const float leadBuff = customColors ? 100. : 40.;  // Make the center of the chevron appear sooner if a custom theme is active
  const float d_rel = lead_data.getDRel();
  const float v_rel = lead_data.getVRel();

  float fillAlpha = 0;
  if (d_rel < leadBuff) {
    fillAlpha = 255 * (1.0 - (d_rel / leadBuff));
    if (v_rel < 0) {
      fillAlpha += 255 * (-1 * (v_rel / speedBuff));
    }
    fillAlpha = (int)(fmin(fillAlpha, 255));
  }

  float sz = std::clamp((25 * 30) / (d_rel / 3 + 30), 15.0f, 30.0f) * 2.35;
  float x = std::clamp((float)vd.x(), 0.f, width() - sz / 2);
  float y = std::fmin(height() - sz * .6, (float)vd.y());

  float g_xo = sz / 5;
  float g_yo = sz / 10;

  QPointF glow[] = {{x + (sz * 1.35) + g_xo, y + sz + g_yo}, {x, y - g_yo}, {x - (sz * 1.35) - g_xo, y + sz + g_yo}};
  painter.setBrush(QColor(218, 202, 37, 255));
  painter.drawPolygon(glow, std::size(glow));

  // chevron
  QPointF chevron[] = {{x + (sz * 1.25), y + sz}, {x, y}, {x - (sz * 1.25), y + sz}};
  if (customColors != 0) {
    painter.setBrush(themeConfiguration[customColors].second.first);
  } else {
    painter.setBrush(redColor(fillAlpha));
  }
  painter.drawPolygon(chevron, std::size(chevron));

  // Add lead info
  if (leadInfo) {
    // Declare the variables
    QString unit_d = "meters";
    QString unit_s = "km/h";
    float distance = d_rel;
    float lead_speed = std::max(lead_data.getVLead(), 0.0f);  // Ensure speed doesn't go under 0 m/s cause that's dumb

    // Conversion factors and units
    constexpr float toFeet = 3.28084f;
    constexpr float toMph = 2.23694f;
    constexpr float toKmph = 3.6f;

    // Metric speed conversion
    if (is_metric) {
      lead_speed *= toKmph;
    } else {
    // US imperial conversion
      distance *= toFeet;
      lead_speed *= toMph;
      unit_d = "feet";
      unit_s = "mph";
    }

    // Form the text and center it below the chevron
    painter.setPen(Qt::white);
    painter.setFont(InterFont(35, QFont::Bold));

    const QString text = QString("%1 %2 | %3 %4")
                        .arg(distance, 0, 'f', 2, '0')
                        .arg(unit_d)
                        .arg(lead_speed, 0, 'f', 2, '0')
                        .arg(unit_s);

    // Calculate the text starting position
    const QFontMetrics metrics(painter.font());
    const int middle_x = (chevron[2].x() + chevron[0].x()) / 2;
    const int textWidth = metrics.horizontalAdvance(text);
    painter.drawText(middle_x - textWidth / 2, chevron[0].y() + metrics.height() + 5, text);
  }

  painter.restore();
}

void AnnotatedCameraWidget::paintGL() {
}

void AnnotatedCameraWidget::paintEvent(QPaintEvent *event) {
  UIState *s = uiState();
  SubMaster &sm = *(s->sm);
  QPainter painter(this);
  const double start_draw_t = millis_since_boot();
  const cereal::ModelDataV2::Reader &model = sm["modelV2"].getModelV2();

  // draw camera frame
  {
    std::lock_guard lk(frame_lock);

    if (frames.empty()) {
      if (skip_frame_count > 0) {
        skip_frame_count--;
        qDebug() << "skipping frame, not ready";
        return;
      }
    } else {
      // skip drawing up to this many frames if we're
      // missing camera frames. this smooths out the
      // transitions from the narrow and wide cameras
      skip_frame_count = 5;
    }

    // Wide or narrow cam dependent on speed
    bool has_wide_cam = available_streams.count(VISION_STREAM_WIDE_ROAD);
    if (has_wide_cam) {
      float v_ego = sm["carState"].getCarState().getVEgo();
      if ((v_ego < 10) || available_streams.size() == 1) {
        wide_cam_requested = true;
      } else if (v_ego > 15) {
        wide_cam_requested = false;
      }
      wide_cam_requested = wide_cam_requested && sm["controlsState"].getControlsState().getExperimentalMode() || cameraView == 2;
      // for replay of old routes, never go to widecam
      wide_cam_requested = wide_cam_requested && s->scene.calibration_wide_valid;
    }
    paramsMemory.putBoolNonBlocking("WideCamera", wide_cam_requested);
    CameraWidget::setStreamType(showDriverCamera || cameraView == 3 ? VISION_STREAM_DRIVER : 
                                wide_cam_requested && cameraView != 1 ? VISION_STREAM_WIDE_ROAD : VISION_STREAM_ROAD);

    s->scene.wide_cam = CameraWidget::getStreamType() == VISION_STREAM_WIDE_ROAD;
    if (s->scene.calibration_valid) {
      auto calib = s->scene.wide_cam ? s->scene.view_from_wide_calib : s->scene.view_from_calib;
      CameraWidget::updateCalibration(calib);
    } else {
      CameraWidget::updateCalibration(DEFAULT_CALIBRATION);
    }
    painter.beginNativePainting();
    CameraWidget::setFrameId(model.getFrameId());
    CameraWidget::paintGL();
    painter.endNativePainting();
  }

  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(Qt::NoPen);

  if (s->scene.world_objects_visible && !showDriverCamera) {
    update_model(s, model, sm["uiPlan"].getUiPlan());
    drawLaneLines(painter, s);

    if (s->scene.longitudinal_control && sm.rcv_frame("radarState") > s->scene.started_frame) {
      auto radar_state = sm["radarState"].getRadarState();
      update_leads(s, radar_state, model.getPosition());
      auto lead_one = radar_state.getLeadOne();
      auto lead_two = radar_state.getLeadTwo();
      if (lead_one.getStatus()) {
        drawLead(painter, lead_one, s->scene.lead_vertices[0]);
      }
      if (lead_two.getStatus() && (std::abs(lead_one.getDRel() - lead_two.getDRel()) > 3.0)) {
        drawLead(painter, lead_two, s->scene.lead_vertices[1]);
      }
    }
  }

  // DMoji
  if (!hideBottomIcons && (sm.rcv_frame("driverStateV2") > s->scene.started_frame) && !muteDM) {
    update_dmonitoring(s, sm["driverStateV2"].getDriverStateV2(), dm_fade_state, rightHandDM);
    drawDriverState(painter, s);
  }

  drawHud(painter);

  double cur_draw_t = millis_since_boot();
  double dt = cur_draw_t - prev_draw_t;
  fps = fps_filter.update(1. / dt * 1000);
  if (fps < 15) {
    LOGW("slow frame rate: %.2f fps", fps);
  }
  prev_draw_t = cur_draw_t;

  // publish debug msg
  MessageBuilder msg;
  auto m = msg.initEvent().initUiDebug();
  m.setDrawTimeMillis(cur_draw_t - start_draw_t);
  pm->send("uiDebug", msg);

  // Update FrogPilot widgets
  updateFrogPilotWidgets(painter);
}

void AnnotatedCameraWidget::showEvent(QShowEvent *event) {
  CameraWidget::showEvent(event);

  ui_update_params(uiState());
  prev_draw_t = millis_since_boot();
}

// FrogPilot widgets
void AnnotatedCameraWidget::initializeFrogPilotWidgets() {
  bottom_layout = new QHBoxLayout();

  QSpacerItem *spacer = new QSpacerItem(0, 0, QSizePolicy::Expanding, QSizePolicy::Minimum);
  bottom_layout->addItem(spacer);

  compass_img = new Compass(this);
  bottom_layout->addWidget(compass_img);

  map_settings_btn_bottom = new MapSettingsButton(this);
  bottom_layout->addWidget(map_settings_btn_bottom);

  main_layout->addLayout(bottom_layout);

  if (params.getBool("HideSpeed")) {
    speedHidden = true;
  }

  if (params.getBool("ReverseCruise")) {
    reverseCruise = true;
  }

  // Custom themes configuration
  themeConfiguration = {
    {1, {QString("frog_theme"), {QColor(23, 134, 68, 242), {{0.0, QBrush(QColor::fromHslF(144 / 360., 0.71, 0.31, 0.9))},
                                                            {0.5, QBrush(QColor::fromHslF(144 / 360., 0.71, 0.31, 0.5))},
                                                            {1.0, QBrush(QColor::fromHslF(144 / 360., 0.71, 0.31, 0.1))}}}}},
    {2, {QString("tesla_theme"), {QColor(0, 72, 255, 255), {{0.0, QBrush(QColor::fromHslF(223 / 360., 1.0, 0.5, 0.9))},
                                                            {0.5, QBrush(QColor::fromHslF(223 / 360., 1.0, 0.5, 0.5))},
                                                            {1.0, QBrush(QColor::fromHslF(223 / 360., 1.0, 0.5, 0.1))}}}}},
    {3, {QString("stalin_theme"), {QColor(255, 0, 0, 255), {{0.0, QBrush(QColor::fromHslF(0 / 360., 1.0, 0.5, 0.9))},
                                                            {0.5, QBrush(QColor::fromHslF(0 / 360., 1.0, 0.5, 0.5))},
                                                            {1.0, QBrush(QColor::fromHslF(0 / 360., 1.0, 0.5, 0.1))}}}}}
  };

  // Initialize the timer for the turn signal animation
  animationTimer = new QTimer(this);
  connect(animationTimer, &QTimer::timeout, this, [this] {
    animationFrameIndex = (animationFrameIndex + 1) % totalFrames;
  });

  // Initialize the timer for the screen recorder
  QTimer *record_timer = new QTimer(this);
  connect(record_timer, &QTimer::timeout, this, [this]() {
    if (this->recorder_btn) {
      this->recorder_btn->update_screen();
    }
  });
  record_timer->start(1000 / UI_FREQ);
}

void AnnotatedCameraWidget::updateFrogPilotWidgets(QPainter &p) {
  accelerationPath = scene.acceleration_path;
  adjacentPath = scene.adjacent_path;
  alwaysOnLateral = scene.always_on_lateral_active;
  blindSpotLeft = scene.blind_spot_left;
  blindSpotRight = scene.blind_spot_right;
  cameraView = scene.camera_view;
  compass = scene.compass;
  conditionalExperimental = scene.conditional_experimental;
  conditionalSpeed = scene.conditional_speed;
  conditionalSpeedLead = scene.conditional_speed_lead;
  conditionalStatus = scene.conditional_status;
  customColors = scene.custom_colors;
  desiredFollow = scene.desired_follow;
  experimentalMode = scene.experimental_mode;
  laneWidthLeft = scene.lane_width_left;
  laneWidthRight = scene.lane_width_right;
  leadInfo = scene.lead_info;
  mapOpen = scene.map_open;
  muteDM = scene.mute_dm;
  obstacleDistance = scene.obstacle_distance;
  obstacleDistanceStock = scene.obstacle_distance_stock;
  roadNameUI = scene.road_name_ui;
  showDriverCamera = scene.show_driver_camera;
  stoppedEquivalence = scene.stopped_equivalence;
  stoppedEquivalenceStock = scene.stopped_equivalence_stock;
  turnSignalLeft = scene.turn_signal_left;
  turnSignalRight = scene.turn_signal_right;

  if (!showDriverCamera) {
    if (leadInfo) {
      drawLeadInfo(p);
    }

    if (alwaysOnLateral || conditionalExperimental || roadNameUI) {
      drawStatusBar(p);
    }

    if (customSignals && (turnSignalLeft || turnSignalRight)) {
      if (!animationTimer->isActive()) {
        animationTimer->start(totalFrames * 11);  // 440 milliseconds per loop; syncs up perfectly with my 2019 Lexus ES 350 turn signal clicks
      }
      drawTurnSignals(p);
    } else if (animationTimer->isActive()) {
      animationTimer->stop();
    }
  }

  const bool enableCompass = compass && !hideBottomIcons;
  compass_img->setVisible(enableCompass);
  if (enableCompass) {
    if (bearingDeg != scene.bearing_deg) {
      bearingDeg = scene.bearing_deg;
      compass_img->updateState(bearingDeg);
    }
    bottom_layout->setAlignment(compass_img, (rightHandDM ? Qt::AlignLeft : Qt::AlignRight));
  }

  map_settings_btn_bottom->setEnabled(map_settings_btn->isEnabled());
  if (map_settings_btn_bottom->isEnabled()) {
    map_settings_btn_bottom->setVisible(!hideBottomIcons && !compass);
    bottom_layout->setAlignment(map_settings_btn_bottom, rightHandDM ? Qt::AlignLeft : Qt::AlignRight);
  }

  // Update the turn signal animation images upon toggle change
  if (customSignals != scene.custom_signals) {
    customSignals = scene.custom_signals;

    const QString theme_path = QString("../frogpilot/assets/custom_themes/%1/images").arg(themeConfiguration.find(customSignals) != themeConfiguration.end() ? 
                                       themeConfiguration[customSignals].first : "stock_theme");
    const QStringList imagePaths = {
      theme_path + "/turn_signal_1.png",
      theme_path + "/turn_signal_2.png",
      theme_path + "/turn_signal_3.png",
      theme_path + "/turn_signal_4.png"
    };

    signalImgVector.clear();
    signalImgVector.reserve(4 * imagePaths.size() + 2);  // Reserve space for both regular and flipped images
    for (int i = 0; i < 2; ++i) {
      for (const QString &imagePath : imagePaths) {
        QPixmap pixmap(imagePath);
        signalImgVector.push_back(pixmap);  // Regular image
        signalImgVector.push_back(pixmap.transformed(QTransform().scale(-1, 1)));  // Flipped image
      }
    }
    signalImgVector.push_back(QPixmap(theme_path + "/turn_signal_1_red.png"));  // Regular blindspot image
    signalImgVector.push_back(QPixmap(theme_path + "/turn_signal_1_red.png").transformed(QTransform().scale(-1, 1)));  // Flipped blindspot image
  }
}

Compass::Compass(QWidget *parent) : QWidget(parent) {
  setFixedSize(btn_size * 1.5, btn_size * 1.5);

  compassSize = btn_size;
  circleOffset = compassSize / 2;
  degreeLabelOffset = circleOffset + 25;
  innerCompass = compassSize / 2;
  x = (btn_size * 1.5) / 2 + 20;
  y = (btn_size * 1.5) / 2;

  compassInnerImg = loadPixmap("../frogpilot/assets/other_images/compass_inner.png", QSize(compassSize / 1.75, compassSize / 1.75));

  initializeStaticElements();
}

void Compass::updateState(int bearing_deg) {
  bearingDeg = bearing_deg;
  update();
}

void Compass::initializeStaticElements() {
  staticElements = QPixmap(size());
  staticElements.fill(Qt::transparent);
  QPainter p(&staticElements);

  p.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);

  // Configure the circles
  QPen whitePen(Qt::white, 2);
  p.setPen(whitePen);

  // Draw the circle background and white inner circle
  p.setOpacity(1.0);
  p.setBrush(QColor(0, 0, 0, 100));
  p.drawEllipse(x - circleOffset, y - circleOffset, circleOffset * 2, circleOffset * 2);

  // Draw the white circles
  p.setBrush(Qt::NoBrush);
  p.drawEllipse(x - (innerCompass + 5), y - (innerCompass + 5), (innerCompass + 5) * 2, (innerCompass + 5) * 2);
  p.drawEllipse(x - degreeLabelOffset, y - degreeLabelOffset, degreeLabelOffset * 2, degreeLabelOffset * 2);

  // Draw the black background for the bearing degrees
  QPainterPath outerCircle, innerCircle;
  outerCircle.addEllipse(x - degreeLabelOffset, y - degreeLabelOffset, degreeLabelOffset * 2, degreeLabelOffset * 2);
  innerCircle.addEllipse(x - circleOffset, y - circleOffset, compassSize, compassSize);
  p.fillPath(outerCircle.subtracted(innerCircle), Qt::black);

  // Draw the static degree lines
  for (int i = 0; i < 360; i += 15) {
    const bool isCardinalDirection = i % 90 == 0;
    const int lineLength = isCardinalDirection ? 15 : 10;
    p.setPen(QPen(Qt::white, isCardinalDirection ? 3 : 1));
    p.save();
    p.translate(x, y);
    p.rotate(i);
    p.drawLine(0, -(compassSize / 2 - lineLength), 0, -(compassSize / 2));
    p.restore();
  }
}

void Compass::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);

  // Draw static elements
  p.drawPixmap(0, 0, staticElements);

  // Rotate and draw the compassInnerImg image
  p.translate(x, y);
  p.rotate(bearingDeg);
  p.drawPixmap(-compassInnerImg.width() / 2, -compassInnerImg.height() / 2, compassInnerImg);
  p.rotate(-bearingDeg);
  p.translate(-x, -y);

  // Draw the dynamic bearing degree numbers and lines
  QFont font = InterFont(10, QFont::Normal);
  for (int i = 0; i < 360; i += 15) {
    const bool isBold = abs(i - static_cast<int>(bearingDeg)) <= 7;
    font.setWeight(isBold ? QFont::Bold : QFont::Normal);
    p.setFont(font);
    p.setPen(QPen(Qt::white, i % 90 == 0 ? 2 : 1));

    p.save();
    p.translate(x, y);
    p.rotate(i);
    p.drawLine(0, -(compassSize / 2 - (i % 90 == 0 ? 12 : 8)), 0, -(compassSize / 2));
    p.translate(0, -(compassSize / 2 + 12));
    p.rotate(-i);
    p.drawText(QRect(-20, -10, 40, 20), Qt::AlignCenter, QString::number(i));
    p.restore();
  }

  // Draw cardinal directions
  p.setFont(InterFont(20, QFont::Bold));
  const QString directions[] = {"N", "E", "S", "W", "N"};
  const int fromAngles[] = {337, 68, 158, 248, 337};
  const int toAngles[] = {22, 112, 202, 292, 360};
  const int alignmentFlags[] = {Qt::AlignTop | Qt::AlignHCenter, Qt::AlignRight | Qt::AlignVCenter, Qt::AlignBottom | Qt::AlignHCenter, Qt::AlignLeft | Qt::AlignVCenter, Qt::AlignTop | Qt::AlignHCenter};
  int directionOffset = 20;

  for (int i = 0; i < 5; ++i) {
    const int offset = (directions[i] == "E") ? -5 : (directions[i] == "W" ? 5 : 0);
    p.setOpacity((bearingDeg >= fromAngles[i] && bearingDeg < toAngles[i]) ? 1.0 : 0.2);
    QRect textRect(x - innerCompass + offset + directionOffset, y - innerCompass + directionOffset, innerCompass * 2 - 2 * directionOffset, innerCompass * 2 - 2 * directionOffset);
    p.drawText(textRect, alignmentFlags[i], directions[i]);
  }
}

void AnnotatedCameraWidget::drawLeadInfo(QPainter &p) {
  const SubMaster &sm = *uiState()->sm;

  // Declare the variables
  static QElapsedTimer timer;
  static bool isFiveSecondsPassed = false;
  constexpr int maxAccelDuration = 5000;

  // Constants for units and conversions
  QString unit_a = " m/s²";
  QString unit_d = mapOpen ? "m" : "meters";
  QString unit_s = "kmh";
  float distanceConversion = 1.0f;
  float speedConversion = 1.0f;

  // Conversion factors and units
  constexpr float toFeet = 3.28084f;
  constexpr float toMph = 2.23694f;

  // Metric speed conversion
  if (!is_metric) {
    // US imperial conversion
    unit_a = " ft/s²";
    unit_d = mapOpen ? "ft" : "feet";
    unit_s = "mph";
    distanceConversion = toFeet;
    speedConversion = toMph;
  }

  // Update acceleration
  const double currentAcceleration = std::round(sm["carState"].getCarState().getAEgo() * 100) / 100;
  static double maxAcceleration = 0.0;

  if (currentAcceleration > maxAcceleration && status == STATUS_ENGAGED) {
    maxAcceleration = currentAcceleration;
    isFiveSecondsPassed = false;
    timer.start();
  } else {
    isFiveSecondsPassed = timer.hasExpired(maxAccelDuration);
  }

  // Construct text segments
  const auto createText = [&](const QString &title, const double data) {
    return title + QString::number(std::round(data * distanceConversion)) + " " + unit_d;
  };

  // Create segments for insights
  const QString accelText = QString("Accel: %1%2")
    .arg(currentAcceleration * speedConversion, 0, 'f', 2)
    .arg(unit_a);

  const QString maxAccSuffix = QString(mapOpen ? "" : " - Max: %1%2")
    .arg(maxAcceleration * speedConversion, 0, 'f', 2)
    .arg(unit_a);

  const QString obstacleText = createText(mapOpen ? " | Obstacle: " : "  |  Obstacle Factor: ", obstacleDistance);
  const QString stopText = createText(mapOpen ? " - Stop: " : "  -  Stop Factor: ", stoppedEquivalence);
  const QString followText = " = " + createText(mapOpen ? "Follow: " : "Follow Distance: ", desiredFollow);

  // Check if the longitudinal toggles have an impact on the driving logics
  const auto createDiffText = [&](const double data, const double stockData) {
    const double difference = std::round((data - stockData) * distanceConversion);
    return difference != 0 ? QString(" (%1%2)").arg(difference > 0 ? "+" : "").arg(difference) : QString();
  };

  // Prepare rectangle for insights
  p.save();
  const QRect insightsRect(rect().left() - 1, rect().top() - 60, rect().width() + 2, 100);
  p.setBrush(QColor(0, 0, 0, 150));
  p.drawRoundedRect(insightsRect, 30, 30);
  p.setFont(InterFont(30, QFont::DemiBold));
  p.setRenderHint(QPainter::TextAntialiasing);

  // Calculate positioning for text drawing
  const QRect adjustedRect = insightsRect.adjusted(0, 27, 0, 27);
  const int textBaseLine = adjustedRect.y() + (adjustedRect.height() + p.fontMetrics().height()) / 2 - p.fontMetrics().descent();

  // Calculate the entire text width to ensure perfect centering
  const int totalTextWidth = p.fontMetrics().horizontalAdvance(accelText)
                           + p.fontMetrics().horizontalAdvance(maxAccSuffix)
                           + p.fontMetrics().horizontalAdvance(obstacleText)
                           + p.fontMetrics().horizontalAdvance(createDiffText(obstacleDistance, obstacleDistanceStock))
                           + p.fontMetrics().horizontalAdvance(stopText)
                           + p.fontMetrics().horizontalAdvance(createDiffText(stoppedEquivalence, stoppedEquivalenceStock))
                           + p.fontMetrics().horizontalAdvance(followText);

  int textStartPos = adjustedRect.x() + (adjustedRect.width() - totalTextWidth) / 2;

  // Draw the text
  const auto drawText = [&](const QString &text, const QColor color) {
    p.setPen(color);
    p.drawText(textStartPos, textBaseLine, text);
    textStartPos += p.fontMetrics().horizontalAdvance(text);
  };

  drawText(accelText, Qt::white);
  drawText(maxAccSuffix, isFiveSecondsPassed ? Qt::white : Qt::red);
  drawText(obstacleText, Qt::white);
  drawText(createDiffText(obstacleDistance, obstacleDistanceStock), (obstacleDistance - obstacleDistanceStock) > 0 ? Qt::green : Qt::red);
  drawText(stopText, Qt::white);
  drawText(createDiffText(stoppedEquivalence, stoppedEquivalenceStock), (stoppedEquivalence - stoppedEquivalenceStock) > 0 ? Qt::green : Qt::red);
  drawText(followText, Qt::white);

  p.restore();
}

void AnnotatedCameraWidget::drawStatusBar(QPainter &p) {
  p.save();

  // Variable declarations
  static QElapsedTimer timer;
  static QString lastShownStatus;
  static QString newStatus;

  static bool displayStatusText = false;

  constexpr qreal fadeDuration = 1500.0;
  constexpr qreal textDuration = 5000.0;

  const QString screenSuffix = ". Double tap the screen to revert";
  const QString wheelSuffix = ". Double press the \"LKAS\" button to revert";

  const QString roadName = roadNameUI ? QString::fromStdString(paramsMemory.get("RoadName")) : QString();

  QMap<int, QString> conditionalStatusMap = {
    {0, "Conditional Experimental Mode ready"},
    {1, "Conditional Experimental overridden"},
    {2, "Experimental Mode manually activated"},
    {3, "Conditional Experimental overridden"},
    {4, "Experimental Mode manually activated"},
    {5, "Experimental Mode activated for navigation" + (mapOpen ? "" : QString(" instructions input"))},
    {6, "Experimental Mode activated due to" + (mapOpen ? "SLC" : QString(" no speed limit set"))},
    {7, "Experimental Mode activated due to" + (mapOpen ? " speed" : " speed being less than " + QString::number(conditionalSpeedLead) + (is_metric ? " kph" : " mph"))},
    {8, "Experimental Mode activated due to" + (mapOpen ? " speed" : " speed being less than " + QString::number(conditionalSpeed) + (is_metric ? " kph" : " mph"))},
    {9, "Experimental Mode activated for slower lead"},
    {10, "Experimental Mode activated for turn" + (mapOpen ? "" : QString(" / lane change"))},
    {11, "Experimental Mode activated for curve"},
    {12, "Experimental Mode activated for stop" + (mapOpen ? "" : QString(" sign / stop light"))},
  };

  if (alwaysOnLateral) {
    newStatus = QString("Always On Lateral active") + (mapOpen ? "" : ". Press the \"Cruise Control\" button to disable");
  } else if (conditionalExperimental) {
    newStatus = conditionalStatusMap.contains(conditionalStatus) && status != STATUS_DISENGAGED ? conditionalStatusMap[conditionalStatus] : conditionalStatusMap[0];
  }

  // Check if status has changed or if the road name is empty
  if (newStatus != lastShownStatus || roadName.isEmpty()) {
    displayStatusText = true;
    lastShownStatus = newStatus;
    timer.restart();
  } else if (displayStatusText && timer.hasExpired(textDuration + fadeDuration)) {
    displayStatusText = false;
  }
  if (!alwaysOnLateral && !mapOpen && status != STATUS_DISENGAGED && !newStatus.isEmpty()) {
    newStatus += (conditionalStatus == 3 || conditionalStatus == 4) ? screenSuffix : (conditionalStatus == 1 || conditionalStatus == 2) ? wheelSuffix : "";
  }

  // Calculate opacities
  static qreal roadNameOpacity;
  static qreal statusTextOpacity;
  const int elapsed = timer.elapsed();
  if (displayStatusText) {
    statusTextOpacity = qBound(0.0, 1.0 - (elapsed - textDuration) / fadeDuration, 1.0);
    roadNameOpacity = 1.0 - statusTextOpacity;
  } else {
    roadNameOpacity = qBound(0.0, elapsed / fadeDuration, 1.0);
    statusTextOpacity = 1.0 - roadNameOpacity;
  }

  // Draw status bar
  const QRect currentRect = rect();
  const QRect statusBarRect(currentRect.left() - 1, currentRect.bottom() - 50, currentRect.width() + 2, 100);
  p.setBrush(QColor(0, 0, 0, 150));
  p.setOpacity(1.0);
  p.drawRoundedRect(statusBarRect, 30, 30);

  // Configure the text
  p.setFont(InterFont(40, QFont::Bold));
  p.setPen(Qt::white);
  p.setRenderHint(QPainter::TextAntialiasing);

  // Draw the status text with the calculated opacity
  p.setOpacity(statusTextOpacity);
  QRect textRect = p.fontMetrics().boundingRect(statusBarRect, Qt::AlignCenter | Qt::TextWordWrap, newStatus);
  textRect.moveBottom(statusBarRect.bottom() - 50);
  p.drawText(textRect, Qt::AlignCenter | Qt::TextWordWrap, newStatus);

  // Draw the road name with the calculated opacity
  if (!roadName.isEmpty()) {
    p.setOpacity(roadNameOpacity);
    textRect = p.fontMetrics().boundingRect(statusBarRect, Qt::AlignCenter | Qt::TextWordWrap, roadName);
    textRect.moveBottom(statusBarRect.bottom() - 50);
    p.drawText(textRect, Qt::AlignCenter | Qt::TextWordWrap, roadName);
  }

  p.restore();
}

void AnnotatedCameraWidget::drawTurnSignals(QPainter &p) {
  // Declare the turn signal size
  constexpr int signalHeight = 480;
  constexpr int signalWidth = 360;

  // Calculate the vertical position for the turn signals
  const int baseYPosition = (height() - signalHeight) / 2 + (alwaysOnLateral || conditionalExperimental || roadNameUI ? 225 : 300);
  // Calculate the x-coordinates for the turn signals
  const int leftSignalXPosition = 75 + width() - signalWidth - 300 * (blindSpotLeft ? 0 : animationFrameIndex);
  const int rightSignalXPosition = -75 + 300 * (blindSpotRight ? 0 : animationFrameIndex);

  // Enable Antialiasing
  p.setRenderHint(QPainter::Antialiasing);

  // Draw the turn signals
  if (animationFrameIndex < signalImgVector.size()) {
    const auto drawSignal = [&](const bool signalActivated, const int xPosition, const bool flip, const bool blindspot) {
      if (signalActivated) {
        // Get the appropriate image from the signalImgVector
        const int uniqueImages = signalImgVector.size() / 4;  // Each image has a regular, flipped, and two blindspot versions
        const int index = (blindspot ? 2 * uniqueImages : 2 * animationFrameIndex % totalFrames) + (flip ? 1 : 0);
        const QPixmap &signal = signalImgVector[index];
        p.drawPixmap(xPosition, baseYPosition, signalWidth, signalHeight, signal);
      }
    };

    // Display the animation based on which signal is activated
    drawSignal(turnSignalLeft, leftSignalXPosition, false, blindSpotLeft);
    drawSignal(turnSignalRight, rightSignalXPosition, true, blindSpotRight);
  }
}
