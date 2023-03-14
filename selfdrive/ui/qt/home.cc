#include "selfdrive/ui/qt/home.h"

#include <QDateTime>
#include <QHBoxLayout>
#include <QMouseEvent>
#include <QVBoxLayout>

#include "selfdrive/common/params.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/widgets/drive_stats.h"
#include "selfdrive/ui/qt/widgets/prime.h"

// HomeWindow: the container for the offroad and onroad UIs

HomeWindow::HomeWindow(QWidget* parent) : QWidget(parent) {
  QHBoxLayout *main_layout = new QHBoxLayout(this);
  main_layout->setMargin(0);
  main_layout->setSpacing(0);

  sidebar = new Sidebar(this);
  main_layout->addWidget(sidebar);
  QObject::connect(this, &HomeWindow::update, sidebar, &Sidebar::updateState);
  QObject::connect(sidebar, &Sidebar::openSettings, this, &HomeWindow::openSettings);

  slayout = new QStackedLayout();
  main_layout->addLayout(slayout);

  home = new OffroadHome();
  slayout->addWidget(home);

  onroad = new OnroadWindow(this);
  slayout->addWidget(onroad);

  QObject::connect(this, &HomeWindow::update, onroad, &OnroadWindow::updateStateSignal);
  QObject::connect(this, &HomeWindow::offroadTransitionSignal, onroad, &OnroadWindow::offroadTransitionSignal);

  driver_view = new DriverViewWindow(this);
  connect(driver_view, &DriverViewWindow::done, [=] {
    showDriverView(false);
  });
  slayout->addWidget(driver_view);
  setAttribute(Qt::WA_NoSystemBackground);
}

void HomeWindow::showSidebar(bool show) {
  sidebar->setVisible(show);
}

void HomeWindow::offroadTransition(bool offroad) {
  sidebar->setVisible(offroad);
  if (offroad) {
    slayout->setCurrentWidget(home);
  } else {
    slayout->setCurrentWidget(onroad);
  }
  emit offroadTransitionSignal(offroad);
}

void HomeWindow::showDriverView(bool show) {
  if (show) {
    emit closeSettings();
    slayout->setCurrentWidget(driver_view);
  } else {
    slayout->setCurrentWidget(home);
  }
  sidebar->setVisible(show == false);
}

void HomeWindow::mousePressEvent(QMouseEvent* e) {
  if (!QUIState::ui_state.scene.screen_tapped){
    QUIState::ui_state.scene.screen_tapped = true;
    Params().putBool("ScreenTapped", true);
    return;
  }

  // Toggle speed limit control enabled
  Rect touch_rect = QUIState::ui_state.scene.speed_limit_sign_touch_rect;
  SubMaster &sm = *(QUIState::ui_state.sm);
  if (QUIState::ui_state.scene.started && sm["longitudinalPlan"].getLongitudinalPlan().getSpeedLimit() > 0.0 &&
      touch_rect.ptInRect(e->x(), e->y())) {
    // If touching the speed limit sign area when visible
    QUIState::ui_state.scene.last_speed_limit_sign_tap = seconds_since_boot();
    QUIState::ui_state.scene.speed_limit_control_enabled = !QUIState::ui_state.scene.speed_limit_control_enabled;
    Params().putBool("SpeedLimitControl", QUIState::ui_state.scene.speed_limit_control_enabled);
		return;
  }

  // toggle adjacent lead info printing style
  if (QUIState::ui_state.scene.started  &&
    QUIState::ui_state.scene.adjacent_lead_info_touch_rect.ptInRect(e->x(), e->y())) {
    // If touching the speed limit sign area when visible
    QUIState::ui_state.scene.adjacent_lead_info_print_at_lead = !QUIState::ui_state.scene.adjacent_lead_info_print_at_lead;
    Params().putBool("PrintAdjacentLeadSpeedsAtLead", QUIState::ui_state.scene.adjacent_lead_info_print_at_lead);
		return;
  }
  
  // Laneless mode
  touch_rect = QUIState::ui_state.scene.laneless_btn_touch_rect;
  if (QUIState::ui_state.scene.started && !sidebar->isVisible() && QUIState::ui_state.scene.end_to_end && touch_rect.ptInRect(e->x(), e->y())) {
    QUIState::ui_state.scene.laneless_mode = QUIState::ui_state.scene.laneless_mode + 1;
    if (QUIState::ui_state.scene.laneless_mode > 2) {
      QUIState::ui_state.scene.laneless_mode = 0;
    }
    if (QUIState::ui_state.scene.laneless_mode == 0) {
      Params().put("LanelessMode", "0", 1);
    } else if (QUIState::ui_state.scene.laneless_mode == 1) {
      Params().put("LanelessMode", "1", 1);
    } else if (QUIState::ui_state.scene.laneless_mode == 2) {
      Params().put("LanelessMode", "2", 1);
    }
    return;
  }
  
  // presses of measure boxes
  for (int ii = 0; ii < QUIState::ui_state.scene.measure_cur_num_slots; ++ii){
    int i = ii;
    if (QUIState::ui_state.scene.measure_cur_num_slots > QUIState::ui_state.scene.measure_max_rows && i >= QUIState::ui_state.scene.measure_num_rows){
      i += QUIState::ui_state.scene.measure_row_offset;
    }
    if (QUIState::ui_state.scene.lastTime - QUIState::ui_state.scene.measures_last_tap_t < QUIState::ui_state.scene.measures_touch_timeout && QUIState::ui_state.scene.started && QUIState::ui_state.scene.measure_slot_touch_rects[i].ptInRect(e->x(), e->y())){
      // user pressed one of the measure boxes. Need to increment the data shown.
      char slotName[16];
      snprintf(slotName, sizeof(slotName), "MeasureSlot%.2d", i);
      int slot_val = (QUIState::ui_state.scene.measure_slots[i] + 1) % QUIState::ui_state.scene.num_measures;
      if (!QUIState::ui_state.scene.car_is_ev){
        bool metric_is_dup = false;
        for (int j = 0; j < i && !metric_is_dup; ++j){
          metric_is_dup = (slot_val == QUIState::ui_state.scene.measure_slots[i]);
        }
        while (metric_is_dup || QUIState::ui_state.scene.EVMeasures.count(static_cast<UIMeasure>(slot_val))){
          slot_val = (QUIState::ui_state.scene.measure_slots[i]+1) % QUIState::ui_state.scene.num_measures;
          metric_is_dup = false;
          for (int j = 0; j < i && !metric_is_dup; ++j){
            metric_is_dup = (slot_val == QUIState::ui_state.scene.measure_slots[i]);
          }
        }
      }
      else{
        bool metric_is_dup = false;
        for (int j = 0; j < i && !metric_is_dup; ++j){
          metric_is_dup = (slot_val == QUIState::ui_state.scene.measure_slots[i]);
        }
        while (metric_is_dup){
          slot_val = (QUIState::ui_state.scene.measure_slots[i]+1) % QUIState::ui_state.scene.num_measures;
          metric_is_dup = false;
          for (int j = 0; j < i && !metric_is_dup; ++j){
            metric_is_dup = (slot_val == QUIState::ui_state.scene.measure_slots[i]);
          }
        }
      }
      QUIState::ui_state.scene.measure_slots[i] = slot_val;
      char val_str[6];
      sprintf(val_str, "%1d", slot_val);
      Params().put(slotName, val_str, strlen(val_str));
      QUIState::ui_state.scene.measures_last_tap_t = QUIState::ui_state.scene.lastTime;
      return;
    }
  }
  
  // presses of vehicle speed to increment number of measure boxes
  if (QUIState::ui_state.scene.started 
    && QUIState::ui_state.scene.speed_rect.ptInRect(e->x(), e->y())){
    if (QUIState::ui_state.scene.lastTime - QUIState::ui_state.scene.measures_last_tap_t < QUIState::ui_state.scene.measures_touch_timeout
        || QUIState::ui_state.scene.measure_config_num == 0){
      QUIState::ui_state.scene.measure_config_num = (QUIState::ui_state.scene.measure_config_num + 1) % QUIState::ui_state.scene.measure_config_list.size();
      QUIState::ui_state.scene.measure_cur_num_slots = QUIState::ui_state.scene.measure_config_list[QUIState::ui_state.scene.measure_config_num];
      QUIState::ui_state.scene.measure_num_rows = QUIState::ui_state.scene.measure_cur_num_slots;
      if (QUIState::ui_state.scene.measure_num_rows > QUIState::ui_state.scene.measure_max_rows){
        QUIState::ui_state.scene.measure_num_rows /= 2;
      }
      QUIState::ui_state.scene.measure_row_offset = QUIState::ui_state.scene.measure_max_rows - QUIState::ui_state.scene.measure_num_rows;
      char val_str[6];
      sprintf(val_str, "%1d", QUIState::ui_state.scene.measure_config_num);
      Params().put("MeasureConfigNum", val_str, strlen(val_str));
    }
    QUIState::ui_state.scene.measures_last_tap_t = QUIState::ui_state.scene.lastTime;
    return;
  }
  
  // presses of wheel to toggle vision/map curve braking
  if (QUIState::ui_state.scene.started 
    && QUIState::ui_state.scene.wheel_touch_rect.ptInRect(e->x(), e->y()))
  {
    bool vision_enabled = Params().getBool("TurnVisionControl");
    bool map_enabled = Params().getBool("TurnSpeedControl");
    if (vision_enabled && map_enabled){
      Params().putBool("TurnVisionControl", false);
      Params().putBool("TurnSpeedControl", false);
    }
    else if (!vision_enabled && !map_enabled){
      Params().putBool("TurnVisionControl", true);
    }
    else if (vision_enabled && !map_enabled){
      Params().putBool("TurnSpeedControl", true);
    }
    else {
      Params().putBool("TurnVisionControl", true);
      Params().putBool("TurnSpeedControl", true);
    }
    return;
  }
  
  // presses of maxspeed to toggle coasting
  if (QUIState::ui_state.scene.started 
    && QUIState::ui_state.scene.maxspeed_touch_rect.ptInRect(e->x(), e->y())
    && QUIState::ui_state.scene.one_pedal_fade <= 0.)
  {
    Params().putBool("CoastingActive", !Params().getBool("CoastingActive"));
    return;
  }
  
  // one-pedal mode button
  if (QUIState::ui_state.scene.started 
    && QUIState::ui_state.scene.one_pedal_touch_rect.ptInRect(e->x(), e->y())
    && QUIState::ui_state.scene.one_pedal_fade > 0.)
  {
    Params().putBool("MADSOnePedalMode", !Params().getBool("MADSOnePedalMode"));
    return;
  }
  
  // accel_mode button
  if (QUIState::ui_state.scene.started && QUIState::ui_state.scene.accel_mode_touch_rect.ptInRect(e->x(), e->y())){
    Params().put("AccelMode", std::to_string((std::stoi(Params().get("AccelMode")) + 1) % 3).c_str(), 1);
    return;
  }

  // weather button
  if (QUIState::ui_state.scene.started && QUIState::ui_state.scene.weather_touch_rect.ptInRect(e->x(), e->y())){
    Params().put("WeatherDisplayMode", std::to_string((std::stoi(Params().get("WeatherDisplayMode")) + 1) % 2).c_str(), 1);
    return;
  }
  
  // dynamic_follow_mode button
  if (QUIState::ui_state.scene.started && QUIState::ui_state.scene.dynamic_follow_mode_touch_rect.ptInRect(e->x(), e->y())){
    Params().putBool("DynamicFollow", !Params().getBool("DynamicFollow"));
    return;
  }

  // power meter text press to change units
  if (QUIState::ui_state.scene.started && QUIState::ui_state.scene.power_meter_text_rect.ptInRect(e->x(), e->y())){
    QUIState::ui_state.scene.power_meter_metric = !QUIState::ui_state.scene.power_meter_metric;
    Params().putBool("PowerMeterMetric", QUIState::ui_state.scene.power_meter_metric);
    return;
  }
  // brake indicator or power meter press
  if (QUIState::ui_state.scene.started && QUIState::ui_state.scene.brake_touch_rect.ptInRect(e->x(), e->y())){
    QUIState::ui_state.scene.power_meter_mode = (QUIState::ui_state.scene.power_meter_mode + 1) % 3;
    Params().put("PowerMeterMode", std::to_string(QUIState::ui_state.scene.power_meter_mode).c_str(), 1);
    return;
  }
  
  // screen dim button (dm face icon)
  if (QUIState::ui_state.scene.started && QUIState::ui_state.scene.screen_dim_touch_rect.ptInRect(e->x(), e->y())){
    int dim_mode = std::stoi(Params().get("ScreenDimMode")) - 1;
    if (dim_mode < 0){
      dim_mode = QUIState::ui_state.scene.screen_dim_mode_max;
    }
    QUIState::ui_state.scene.screen_dim_mode = dim_mode;
    Params().put("ScreenDimMode", std::to_string(dim_mode).c_str(), 1);
    return;
  }
  
  // lane position buttons (0., 1., -1. = center, left, right)
  if (QUIState::ui_state.scene.started && QUIState::ui_state.scene.lane_pos_enabled && QUIState::ui_state.scene.lane_pos_left_touch_rect.ptInRect(e->x(), e->y())){
    if (QUIState::ui_state.scene.auto_lane_pos_active){
      QUIState::ui_state.scene.auto_lane_pos_active = false;
      Params().putBool("AutoLanePositionActive", false);
      if (QUIState::ui_state.scene.lane_pos == -1){
        // user pressed left button when auto mode was on right position, so enable left position
        QUIState::ui_state.scene.lane_pos = 1;
        QUIState::ui_state.scene.lane_pos_timeout_dist = QUIState::ui_state.scene.lane_pos_dist_short;
        QUIState::ui_state.scene.lane_pos_set_t = QUIState::ui_state.scene.lastTime;
        QUIState::ui_state.scene.lane_pos_dist_since_set = 0.;
        Params().put("LanePosition", "1", 1);
      }
      else{
        QUIState::ui_state.scene.lane_pos = 0;
        Params().put("LanePosition", "0", 1);
        QUIState::ui_state.scene.lane_pos_set_t = 0;
      }
    }
    else{
      if (QUIState::ui_state.scene.lane_pos == 1){
        if (QUIState::ui_state.scene.lastTime - QUIState::ui_state.scene.lane_pos_set_t < 2.){
          QUIState::ui_state.scene.lane_pos_timeout_dist = QUIState::ui_state.scene.lane_pos_dist_long;
          QUIState::ui_state.scene.lane_pos_set_t = 0;
        }
        else{
          QUIState::ui_state.scene.lane_pos = 0;
          Params().put("LanePosition", "0", 1);
        }
      }
      else if (QUIState::ui_state.scene.lane_pos == -1 && QUIState::ui_state.scene.lastTime - QUIState::ui_state.scene.lane_pos_set_t < 2.){
        // activate auto mode
        QUIState::ui_state.scene.auto_lane_pos_active = true;
        Params().putBool("AutoLanePositionActive", true);
        QUIState::ui_state.scene.lane_pos = 0;
        Params().put("LanePosition", "0", 1);
        QUIState::ui_state.scene.lane_pos_set_t = 0;
      }
      else{
        QUIState::ui_state.scene.lane_pos = 1;
        QUIState::ui_state.scene.lane_pos_timeout_dist = QUIState::ui_state.scene.lane_pos_dist_short;
        QUIState::ui_state.scene.lane_pos_set_t = QUIState::ui_state.scene.lastTime;
        QUIState::ui_state.scene.lane_pos_dist_since_set = 0.;
        Params().put("LanePosition", "1", 1);
      }
    }
    return;
  }
  if (QUIState::ui_state.scene.started && QUIState::ui_state.scene.lane_pos_enabled && QUIState::ui_state.scene.lane_pos_right_touch_rect.ptInRect(e->x(), e->y())){
    if (QUIState::ui_state.scene.auto_lane_pos_active){
      QUIState::ui_state.scene.auto_lane_pos_active = false;
      Params().putBool("AutoLanePositionActive", false);
      if (QUIState::ui_state.scene.lane_pos == 1){
        // user pressed left button when auto mode was on right position, so enable left position
        QUIState::ui_state.scene.lane_pos = -1;
        QUIState::ui_state.scene.lane_pos_timeout_dist = QUIState::ui_state.scene.lane_pos_dist_short;
        QUIState::ui_state.scene.lane_pos_set_t = QUIState::ui_state.scene.lastTime;
        QUIState::ui_state.scene.lane_pos_dist_since_set = 0.;
        Params().put("LanePosition", "-1", 2);
      }
      else{
        QUIState::ui_state.scene.lane_pos = 0;
        Params().put("LanePosition", "0", 1);
        QUIState::ui_state.scene.lane_pos_set_t = 0;
      }
    }
    else{
      if (QUIState::ui_state.scene.lane_pos == -1){
        if (QUIState::ui_state.scene.lastTime - QUIState::ui_state.scene.lane_pos_set_t < 2.){
          QUIState::ui_state.scene.lane_pos_timeout_dist = QUIState::ui_state.scene.lane_pos_dist_long;
          QUIState::ui_state.scene.lane_pos_set_t = 0;
        }
        else{
          QUIState::ui_state.scene.lane_pos = 0;
          Params().put("LanePosition", "0", 1);
        }
      }
      else if (QUIState::ui_state.scene.lane_pos == 1 && QUIState::ui_state.scene.lastTime - QUIState::ui_state.scene.lane_pos_set_t < 2.){
        // activate auto mode
        QUIState::ui_state.scene.auto_lane_pos_active = true;
        Params().putBool("AutoLanePositionActive", true);
        QUIState::ui_state.scene.lane_pos = 0;
        Params().put("LanePosition", "0", 1);
        QUIState::ui_state.scene.lane_pos_set_t = 0;
      }
      else{
        QUIState::ui_state.scene.lane_pos = -1;
        QUIState::ui_state.scene.lane_pos_timeout_dist = QUIState::ui_state.scene.lane_pos_dist_short;
        QUIState::ui_state.scene.lane_pos_set_t = QUIState::ui_state.scene.lastTime;
        QUIState::ui_state.scene.lane_pos_dist_since_set = 0.;
        Params().put("LanePosition", "-1", 2);
      }
    }
    return;
  }

  // if metrics are in edit mode, then don't switch to map or sidebar
  if (QUIState::ui_state.scene.lastTime - QUIState::ui_state.scene.measures_last_tap_t < QUIState::ui_state.scene.measures_touch_timeout && QUIState::ui_state.scene.started){
    return;
  }

  // Handle sidebar collapsing
  if (onroad->isVisible() && (!sidebar->isVisible() || e->x() > sidebar->width())) {
    sidebar->setVisible(!sidebar->isVisible() && !onroad->isMapVisible());
  }
}

// OffroadHome: the offroad home page

OffroadHome::OffroadHome(QWidget* parent) : QFrame(parent) {
  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(40, 40, 40, 45);

  // top header
  QHBoxLayout* header_layout = new QHBoxLayout();
  header_layout->setContentsMargins(15, 15, 15, 0);
  header_layout->setSpacing(16);

  date = new QLabel();
  header_layout->addWidget(date, 1, Qt::AlignHCenter | Qt::AlignLeft);

  update_notif = new QPushButton("UPDATE");
  update_notif->setVisible(false);
  update_notif->setStyleSheet("background-color: #364DEF;");
  QObject::connect(update_notif, &QPushButton::clicked, [=]() { center_layout->setCurrentIndex(1); });
  header_layout->addWidget(update_notif, 0, Qt::AlignHCenter | Qt::AlignRight);

  alert_notif = new QPushButton();
  alert_notif->setVisible(false);
  alert_notif->setStyleSheet("background-color: #E22C2C;");
  QObject::connect(alert_notif, &QPushButton::clicked, [=] { center_layout->setCurrentIndex(2); });
  header_layout->addWidget(alert_notif, 0, Qt::AlignHCenter | Qt::AlignRight);

  header_layout->addWidget(new QLabel(getBrandVersion()), 0, Qt::AlignHCenter | Qt::AlignRight);

  main_layout->addLayout(header_layout);

  // main content
  main_layout->addSpacing(25);
  center_layout = new QStackedLayout();

  QWidget* statsAndSetupWidget = new QWidget(this);
  QHBoxLayout* statsAndSetup = new QHBoxLayout(statsAndSetupWidget);
  statsAndSetup->setMargin(0);
  statsAndSetup->setSpacing(30);
  statsAndSetup->addWidget(new DriveStats, 1);
  statsAndSetup->addWidget(new SetupWidget);

  center_layout->addWidget(statsAndSetupWidget);

  // add update & alerts widgets
  update_widget = new UpdateAlert();
  QObject::connect(update_widget, &UpdateAlert::dismiss, [=]() { center_layout->setCurrentIndex(0); });
  center_layout->addWidget(update_widget);
  alerts_widget = new OffroadAlert();
  QObject::connect(alerts_widget, &OffroadAlert::dismiss, [=]() { center_layout->setCurrentIndex(0); });
  center_layout->addWidget(alerts_widget);

  main_layout->addLayout(center_layout, 1);

  // set up refresh timer
  timer = new QTimer(this);
  timer->callOnTimeout(this, &OffroadHome::refresh);

  setStyleSheet(R"(
    * {
     color: white;
    }
    OffroadHome {
      background-color: black;
    }
    OffroadHome > QPushButton {
      padding: 15px 30px;
      border-radius: 5px;
      font-size: 40px;
      font-weight: 500;
    }
    OffroadHome > QLabel {
      font-size: 55px;
    }
  )");
}

void OffroadHome::showEvent(QShowEvent *event) {
  refresh();
  timer->start(10 * 1000);
}

void OffroadHome::hideEvent(QHideEvent *event) {
  timer->stop();
}

void OffroadHome::refresh() {
  date->setText(QDateTime::currentDateTime().toString("dddd, MMMM d"));

  bool updateAvailable = update_widget->refresh();
  int alerts = alerts_widget->refresh();

  // pop-up new notification
  int idx = center_layout->currentIndex();
  if (!updateAvailable && !alerts) {
    idx = 0;
  } else if (updateAvailable && (!update_notif->isVisible() || (!alerts && idx == 2))) {
    idx = 1;
  } else if (alerts && (!alert_notif->isVisible() || (!updateAvailable && idx == 1))) {
    idx = 2;
  }
  center_layout->setCurrentIndex(idx);

  update_notif->setVisible(updateAvailable);
  alert_notif->setVisible(alerts);
  if (alerts) {
    alert_notif->setText(QString::number(alerts) + " ALERT" + (alerts > 1 ? "S" : ""));
  }
}
