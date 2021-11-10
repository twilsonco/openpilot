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
  for (int i = 0; i < QUIState::ui_state.scene.measure_cur_num_slots; ++i){
    if (QUIState::ui_state.scene.started && QUIState::ui_state.scene.measure_slot_touch_rects[i].ptInRect(e->x(), e->y())){
      // user pressed one of the measure boxes. Need to increment the data shown.
      char slotName[16];
      snprintf(slotName, sizeof(slotName), "MeasureSlot%.2d", i);
      int slot_val = (QUIState::ui_state.scene.measure_slots[i] + 1) % QUIState::ui_state.scene.num_measures;
      QUIState::ui_state.scene.measure_slots[i] = slot_val;
      char val_str[6];
      sprintf(val_str, "%1d", slot_val);
      Params().put(slotName, val_str, strlen(val_str));
      return;
    }
  }
  
  // presses of vehicle speed to increment number of measure boxes
  if (QUIState::ui_state.scene.started 
    && QUIState::ui_state.scene.speed_rect.ptInRect(e->x(), e->y())){
      
    int num_slots = QUIState::ui_state.scene.measure_cur_num_slots + 1; 
    if (num_slots > QUIState::ui_state.scene.measure_max_num_slots){
      num_slots = QUIState::ui_state.scene.measure_min_num_slots;
    }
    else if (num_slots > QUIState::ui_state.scene.measure_max_num_slots / 2){
      num_slots = QUIState::ui_state.scene.measure_max_num_slots;
    }
    QUIState::ui_state.scene.measure_cur_num_slots = num_slots;
    char val_str[6];
    sprintf(val_str, "%1d", num_slots);
    Params().put("MeasureNumSlots", val_str, strlen(val_str));
    return;
  }
  
  // presses of wheen to toggle rotation
  if (QUIState::ui_state.scene.started 
    && QUIState::ui_state.scene.wheel_touch_rect.ptInRect(e->x(), e->y())){
    
    QUIState::ui_state.scene.wheel_rotates = !QUIState::ui_state.scene.wheel_rotates;
    return;
  }
  
  // presses of maxspeed to toggle coasting
  if (QUIState::ui_state.scene.started 
    && QUIState::ui_state.scene.maxspeed_touch_rect.ptInRect(e->x(), e->y())){
    Params().putBool("Coasting", !Params().getBool("Coasting"));
    return;
  }
  
  // one-pedal mode button
  if (QUIState::ui_state.scene.started && QUIState::ui_state.scene.one_pedal_touch_rect.ptInRect(e->x(), e->y())){
    Params().putBool("OnePedalModeEngageOnGas", !Params().getBool("OnePedalModeEngageOnGas"));
  }

  // Handle sidebar collapsing
  else if (onroad->isVisible() && (!sidebar->isVisible() || e->x() > sidebar->width())) {
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
