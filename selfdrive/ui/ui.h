#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <string>

#include <QObject>
#include <QTimer>
#include <QColor>

#include "nanovg.h"

#include "cereal/messaging/messaging.h"
#include "cereal/visionipc/visionipc.h"
#include "cereal/visionipc/visionipc_client.h"
#include "common/transformations/orientation.hpp"
#include "selfdrive/camerad/cameras/camera_common.h"
#include "selfdrive/common/glutil.h"
#include "selfdrive/common/mat.h"
#include "selfdrive/common/modeldata.h"
#include "selfdrive/common/params.h"
#include "selfdrive/common/util.h"
#include "selfdrive/common/visionimg.h"

#define COLOR_BLACK nvgRGBA(0, 0, 0, 255)
#define COLOR_BLACK_ALPHA(x) nvgRGBA(0, 0, 0, x)
#define COLOR_WHITE nvgRGBA(255, 255, 255, 255)
#define COLOR_WHITE_ALPHA(x) nvgRGBA(255, 255, 255, x)
#define COLOR_RED_ALPHA(x) nvgRGBA(201, 34, 49, x)
#define COLOR_YELLOW nvgRGBA(218, 202, 37, 255)
#define COLOR_RED nvgRGBA(201, 34, 49, 255)
#define COLOR_OCHRE nvgRGBA(218, 111, 37, 255)
#define COLOR_OCHRE_ALPHA(x) nvgRGBA(218, 111, 37, x)
#define COLOR_GREEN nvgRGBA(0, 255, 0, 255)
#define COLOR_GREEN_ALPHA(x) nvgRGBA(0, 255, 0, x)
#define COLOR_BLUE nvgRGBA(0, 0, 255, 255)
#define COLOR_BLUE_ALPHA(x) nvgRGBA(0, 0, 255, x)
#define COLOR_ORANGE nvgRGBA(255, 175, 3, 255)
#define COLOR_ORANGE_ALPHA(x) nvgRGBA(255, 175, 3, x)
#define COLOR_YELLOW_ALPHA(x) nvgRGBA(218, 202, 37, x)
#define COLOR_GREY nvgRGBA(191, 191, 191, 1)


#define MAX(A,B) A > B ? A : B
#define MIN(A,B) A < B ? A : B
#define CLIP(A,L,H) A < L ? L : (A > H ? H : A)

typedef cereal::CarControl::HUDControl::AudibleAlert AudibleAlert;

// TODO: this is also hardcoded in common/transformations/camera.py
// TODO: choose based on frame input size
const float y_offset = Hardware::TICI() ? 150.0 : 0.0;
const float ZOOM = Hardware::TICI() ? 2912.8 : 2138.5;

typedef struct Rect {
  int x, y, w, h;
  int centerX() const { return x + w / 2; }
  int centerY() const { return y + h / 2; }
  int right() const { return x + w; }
  int bottom() const { return y + h; }
  bool ptInRect(int px, int py) const {
    return px >= x && px < (x + w) && py >= y && py < (y + h);
  }
} Rect;

typedef struct Alert {
  QString text1;
  QString text2;
  QString type;
  cereal::ControlsState::AlertSize size;
  AudibleAlert sound;
  bool equal(const Alert &a2) {
    return text1 == a2.text1 && text2 == a2.text2 && type == a2.type;
  }
} Alert;

const Alert CONTROLS_WAITING_ALERT = {"openpilot Unavailable", "Waiting for controls to start", 
                                      "controlsWaiting", cereal::ControlsState::AlertSize::MID,
                                      AudibleAlert::NONE};

const Alert CONTROLS_UNRESPONSIVE_ALERT = {"TAKE CONTROL IMMEDIATELY", "Controls Unresponsive",
                                           "controlsUnresponsive", cereal::ControlsState::AlertSize::FULL,
                                           AudibleAlert::CHIME_WARNING_REPEAT};
const int CONTROLS_TIMEOUT = 5;

const int bdr_s = 30;
const int header_h = 420;
const int footer_h = 280;
const int laneless_btn_touch_pad = 80;

const int brake_size = 90;
const int face_wheel_radius = 88;

const int speed_sgn_r = 96;
const int speed_sgn_touch_pad = 60;

const int UI_FREQ = 20;   // Hz

typedef enum UIStatus {
  STATUS_DISENGAGED,
  STATUS_ENGAGED,
  STATUS_WARNING,
  STATUS_ALERT,
} UIStatus;

const QColor bg_colors [] = {
  [STATUS_DISENGAGED] =  QColor(0x17, 0x33, 0x49, 0xc8),
  [STATUS_ENGAGED] = QColor(0x17, 0x86, 0x44, 0xf1),
  [STATUS_WARNING] = QColor(0xDA, 0x6F, 0x25, 0xf1),
  [STATUS_ALERT] = QColor(0xC9, 0x22, 0x31, 0xf1),
};

const QColor tcs_colors [] = {
  [int(cereal::LongitudinalPlan::VisionTurnControllerState::DISABLED)] =  QColor(0x0, 0x0, 0x0, 0xff),
  [int(cereal::LongitudinalPlan::VisionTurnControllerState::ENTERING)] = QColor(0xC9, 0x22, 0x31, 0xf1),
  [int(cereal::LongitudinalPlan::VisionTurnControllerState::TURNING)] = QColor(0xDA, 0x6F, 0x25, 0xf1),
  [int(cereal::LongitudinalPlan::VisionTurnControllerState::LEAVING)
  ] = QColor(0x17, 0x86, 0x44, 0xf1),
};

typedef struct {
  float x, y;
} vertex_data;

typedef struct {
  vertex_data v[TRAJECTORY_SIZE * 2];
  int cnt;
} line_vertices_data;

typedef enum UIMeasure { //rearrange here to adjust order when cycling measures
  STEERING_ANGLE = 0,
  DESIRED_STEERING_ANGLE,
  STEERING_TORQUE_EPS,
  ENGINE_RPM,
  ENGINE_RPM_TEMPC,
  ENGINE_RPM_TEMPF,
  COOLANT_TEMPC,
  COOLANT_TEMPF,
  ACCELERATION,
  LAT_ACCEL,//JERK,
  ALTITUDE,
  PERCENT_GRADE,
  PERCENT_GRADE_DEVICE,
  FOLLOW_LEVEL,
  LEAD_TTC,
  LEAD_DISTANCE_LENGTH,
  LEAD_DISTANCE_TIME,
  LEAD_DESIRED_DISTANCE_LENGTH,
  LEAD_DESIRED_DISTANCE_TIME,
  LEAD_COSTS,
  LEAD_VELOCITY_RELATIVE,
  LEAD_VELOCITY_ABS,
  GPS_ACCURACY,
  CPU_TEMP_AND_PERCENTF,
  CPU_TEMP_AND_PERCENTC,
  CPU_TEMPF,
  CPU_TEMPC,
  CPU_PERCENT,
  MEMORY_TEMPF,
  MEMORY_TEMPC,
  AMBIENT_TEMPF,
  AMBIENT_TEMPC,
  FANSPEED_PERCENT,
  FANSPEED_RPM,
  MEMORY_USAGE_PERCENT,
  FREESPACE_STORAGE,
  DEVICE_BATTERY,
  HVB_VOLTAGE,
  HVB_CURRENT,
  HVB_WATTAGE,
  HVB_WATTVOLT,
  VISION_CURLATACCEL,
  VISION_MAXVFORCURCURV,
  VISION_MAXPREDLATACCEL,
  LANE_WIDTH,
  
  NUM_MEASURES
} UIMeasure;

typedef struct UIScene {

  mat3 view_from_calib;
  bool world_objects_visible;

  // Debug UI
  bool show_debug_ui;

  bool is_using_torque_control = false;

  // Speed limit control
  bool speed_limit_control_enabled;
  bool speed_limit_perc_offset;
  Rect speed_limit_sign_touch_rect;
  double last_speed_limit_sign_tap;
  
  // adjustable lane position
  Rect lane_pos_left_touch_rect = {1,1,1,1}, lane_pos_right_touch_rect = {1,1,1,1};
  bool lane_pos_enabled = false;
  int lane_pos = 0; // 0, 1, -1 = center, left, right
  float lane_pos_timeout_short_t = 15.; // 30s short timeout
  float lane_pos_timeout_long_t = 600.; // 10 minute long timeout
  float lane_pos_timeout = lane_pos_timeout_short_t;
  float lane_pos_set_t = 0.;
  
  Rect wheel_touch_rect;
  bool wheel_rotates = true;
  
  float screen_dim_modes_v[3] = {0.01, 0.5, 1.};
  int screen_dim_mode_max = 2;
  int screen_dim_mode_cur = screen_dim_mode_max; 
  int screen_dim_mode = screen_dim_mode_cur, 
    screen_dim_mode_last = screen_dim_mode_cur;
  float screen_dim_fade = -1., screen_dim_fade_last_t = 0., screen_dim_fade_step = 1;
  float screen_dim_fade_dur_up = 0.5, screen_dim_fade_dur_down = 2.;
  Rect screen_dim_touch_rect;

  cereal::PandaState::PandaType pandaType;
  
// measures
  int measure_min_num_slots = 0;
  int measure_max_num_slots = 10;
  int measure_cur_num_slots = 3;
  int measure_slots[10];
  Rect measure_slots_rect;
  Rect measure_slot_touch_rects[10];
  int num_measures = UIMeasure::NUM_MEASURES; // the number of cases handled in ui_draw_measures() in paint.cc
  Rect speed_rect;
  
  // actual measures
  float angleSteers;
  float angleSteersDes;
  float lateralCorrection;
  int engineRPM;
  bool steerOverride;
  int thermalStatus;
  int percentGradeRollingIter = 0, percentGradeNumSamples = 10;
  float percentGradeAltitudes[10], percentGradePositions[10], percentGrades[10], percentGradeCurDist = 0., percentGradeLenStep = 5., percentGradeLastTime = 0., percentGrade = 0., percentGradeMinDist = 200.;
  bool percentGradeIterRolled = false;
  float desiredFollowDistance, followDistanceCost, followAccelCost;
  float stoppingDistance;
  float percentGradeDevice;
  int fanspeed_rpm = 0;
  
  float lastTime = 0., sessionInitTime = 0.;
  float paramsCheckLast = 0., paramsCheckFreq = 0.1; // check params at 10Hz
  bool onePedalModeActive, disableDisengageOnGasEnabled, onePedalEngageOnGasEnabled, visionBrakingEnabled;

  int lead_status;
  float lead_d_rel, lead_v_rel, lead_v;

  // gps
  int satelliteCount;
  bool gpsOK;
  
  // brake indicator
  int brake_percent;
  float brake_indicator_alpha;
  float brake_indicator_last_t;
  
  // accel mode button
  bool accel_mode_button_enabled;
  Rect accel_mode_touch_rect;
  int accel_mode;
  
  // dynamic follow mode button
  bool dynamic_follow_mode_button_enabled;
  Rect dynamic_follow_mode_touch_rect;
  bool dynamic_follow_active;
  float dynamic_follow_level, dynamic_follow_level_ui, dynamic_follow_last_t;
  std::string dynamic_follow_strs[3] = {"Close","Med.","Far"};
  int dynamic_follow_r[3] = {0, 157, 74};
  int dynamic_follow_b[3] = {100, 157, 132};
  int dynamic_follow_g[3] = {255, 157, 23};
  int dynamic_follow_bg_r[3] = {0, 0, 74};
  int dynamic_follow_bg_b[3] = {100, 0, 132};
  int dynamic_follow_bg_g[3] = {255, 0, 23};
  
  // one-pedal mode fading. maxspeed rect at -1, fades away by 0, and one-pedal icon fades in by 1
  float one_pedal_fade = -1., one_pedal_fade_last_t = 0.;
  Rect one_pedal_touch_rect;
  Rect maxspeed_touch_rect;
  Rect brake_touch_rect;
  
  int laneless_mode;
  Rect laneless_btn_touch_rect;

  cereal::DeviceState::Reader deviceState;
  cereal::RadarState::LeadData::Reader lead_data[2];
  cereal::CarState::Reader car_state;
  cereal::ControlsState::Reader controls_state;
  cereal::LateralPlan::Reader lateral_plan;
  cereal::LongitudinalPlan::Reader longitudinal_plan;
  cereal::DriverState::Reader driver_state;
  cereal::DriverMonitoringState::Reader dmonitoring_state;

  // modelV2
  float lane_line_probs[4];
  float road_edge_stds[2];
  line_vertices_data track_vertices;
  line_vertices_data lane_line_vertices[4];
  line_vertices_data road_edge_vertices[2];

  bool dm_active, engageable;

  // lead
  vertex_data lead_vertices[2];

  float light_sensor, accel_sensor, gyro_sensor;
  bool started, ignition, is_metric, longitudinal_control, end_to_end;
  uint64_t started_frame;
  
  struct _LateralPlan
  {
    float laneWidth;

    float dProb;
    float lProb;
    float rProb;

    bool lanelessModeStatus;
  } lateralPlan;
  
} UIScene;

typedef struct UIState {
  VisionIpcClient * vipc_client;
  VisionIpcClient * vipc_client_rear;
  VisionIpcClient * vipc_client_wide;
  VisionBuf * last_frame;

  // framebuffer
  int fb_w, fb_h;

  // NVG
  NVGcontext *vg;

  // images
  std::map<std::string, int> images;

  std::unique_ptr<SubMaster> sm;

  UIStatus status;
  UIScene scene;

  // graphics
  std::unique_ptr<GLShader> gl_shader;
  std::unique_ptr<EGLImageTexture> texture[UI_BUF_COUNT];

  GLuint frame_vao, frame_vbo, frame_ibo;
  mat4 rear_frame_mat;

  bool awake;
  
  bool is_metric;

  float car_space_transform[6];
  bool wide_camera;
} UIState;


class QUIState : public QObject {
  Q_OBJECT

public:
  QUIState(QObject* parent = 0);

  // TODO: get rid of this, only use signal
  inline static UIState ui_state = {0};

signals:
  void uiUpdate(const UIState &s);
  void offroadTransition(bool offroad);

private slots:
  void update();

private:
  QTimer *timer;
  bool started_prev = true;
};


// device management class

class Device : public QObject {
  Q_OBJECT

public:
  Device(QObject *parent = 0);

private:
  // auto brightness
  const float accel_samples = 5*UI_FREQ;

  bool awake;
  int awake_timeout = 0;
  float accel_prev = 0;
  float gyro_prev = 0;
  float last_brightness = 0;
  FirstOrderFilter brightness_filter;

  QTimer *timer;

  void updateBrightness(const UIState &s);
  void updateWakefulness(const UIState &s);

signals:
  void displayPowerChanged(bool on);

public slots:
  void setAwake(bool on, bool reset);
  void update(const UIState &s);
};

NVGcolor interp_alert_color(float p, int a);

int offset_button_y(UIState *s, int center_y, int radius);

int offset_right_side_button_x(UIState *s, int center_x, int radius, bool doShift = false);