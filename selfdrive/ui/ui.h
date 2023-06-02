#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <deque>
#include <vector>
#include <set>

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
#define COLOR_GRACE_BLUE nvgRGBA(0,100,255, 255)
#define COLOR_GRACE_BLUE_ALPHA(x) nvgRGBA(0,100,255, x)
#define COLOR_ORANGE nvgRGBA(255, 175, 3, 255)
#define COLOR_ORANGE_ALPHA(x) nvgRGBA(255, 175, 3, x)
#define COLOR_YELLOW_ALPHA(x) nvgRGBA(218, 202, 37, x)
#define COLOR_GREY nvgRGBA(191, 191, 191, 1)


#define MAX(A,B) A > B ? A : B
#define MIN(A,B) A < B ? A : B
#define CLIP(A,L,H) A < L ? L : (A > H ? H : A)

typedef cereal::CarControl::HUDControl::AudibleAlert AudibleAlert;
typedef cereal::RadarState::LeadData::Reader LeadData;
typedef cereal::LateralPlan::LaneTraffic LaneTraffic;
typedef cereal::LateralPlan::LanePosition LanePosition;

// TODO: this is also hardcoded in common/transformations/camera.py
// TODO: choose based on frame input size
const float y_offset = Hardware::TICI() ? 150.0 : 0.0;
const float ZOOM = Hardware::TICI() ? 2912.8 : 2138.5;

const std::vector<std::string> ui_network_type = {
  "--",
  "WiFi",
  "2G",
  "3G",
  "LTE",
  "5G",
  "ETH"
};
  // above vector based on this cereal enum
  // enum NetworkType { 
  //   none @0;
  //   wifi @1;
  //   cell2G @2;
  //   cell3G @3;
  //   cell4G @4;
  //   cell5G @5;
  //   ethernet @6;
  // }

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

const QColor alt_bg_colors [] = {
  [STATUS_DISENGAGED] =  QColor(0x00, 0x30, 0x4A, 0xc8),
  [STATUS_ENGAGED] = QColor(0x00, 0x04, 0xA7, 0xf1),
  [STATUS_WARNING] = QColor(0xAB, 0x00, 0xAE, 0xf1),
  [STATUS_ALERT] = QColor(0x92, 0x00, 0x0D, 0xf1),
};

const QColor tcs_colors [] = {
  [int(cereal::LongitudinalPlan::VisionTurnControllerState::DISABLED)] =  QColor(0x0, 0x0, 0x0, 0xff),
  [int(cereal::LongitudinalPlan::VisionTurnControllerState::ENTERING)] = QColor(0xC9, 0x22, 0x31, 0xf1),
  [int(cereal::LongitudinalPlan::VisionTurnControllerState::TURNING)] = QColor(0xDA, 0x6F, 0x25, 0xf1),
  [int(cereal::LongitudinalPlan::VisionTurnControllerState::LEAVING)] = QColor(0x17, 0x86, 0x44, 0xf1),
};

typedef struct {
  float x, y;
} vertex_data;

typedef struct {
  float x, y, d, v;
} lead_vertex_data;

typedef struct {
  vertex_data v[TRAJECTORY_SIZE * 2];
  int cnt;
} line_vertices_data;

typedef struct {
  bool enabled = false;
  bool valid = false;
  long int time = 0;
  int display_mode = 0; // 0/1 = simple/full
  char desc_simple[16];
  char desc_simple1[16];
  char desc_simple2[16];
  char desc_full1[64];
  char desc_full2[64];
  char desc_full3[64];
  char desc_full4[64];
  char icon[8];
  bool has_precip = false;
} weather_info;

void s_to_time_str(char* val, int s);
void deg_to_str(char* val, float deg);

typedef enum UIMeasure { //rearrange here to adjust order when cycling measures
  // Vehicle info
  STEERING_ANGLE = 0,
  DESIRED_STEERING_ANGLE,
  STEERING_ANGLE_ERROR,
  STEERING_TORQUE_EPS,
  ENGINE_RPM,
  ENGINE_RPM_TEMPC,
  ENGINE_RPM_TEMPF,
  COOLANT_TEMPC,
  COOLANT_TEMPF,
  KINETIC_ENERGY,
  ACCELERATION,
  TIME_TO_STOP_CUR,
  TIME_TO_STOP_MIN,
  DIST_TO_STOP_CUR,
  DIST_TO_STOP_MIN,
  LAT_ACCEL,//JERK,
  DRAG_FORCE,
  DRAG_POWER,
  DRAG_POWER_HP,
  DRAG_LOSSES,
  ACCEL_FORCE,
  ACCEL_POWER,
  ACCEL_POWER_HP,
  EV_FORCE,
  EV_POWER,
  EV_POWER_HP,
  REGEN_FORCE,
  REGEN_POWER,
  REGEN_POWER_HP,
  BRAKE_FORCE,
  BRAKE_POWER,
  BRAKE_POWER_HP,
  DRIVE_POWER,
  DRIVE_POWER_HP,
  ICE_POWER,
  ICE_POWER_HP,
  // Location/road info
  ALTITUDE,
  BEARING,
  PERCENT_GRADE,
  PERCENT_GRADE_DEVICE,
  ROLL,
  ROLL_DEVICE,
  LANE_WIDTH,
  LANE_DIST_FROM_CENTER,
  DISTANCE_TRAVELED_SESSION,
  DISTANCE_TRAVELED_TOTAL,
  DISTANCE_ENGAGED,
  DISTANCE_ENGAGED_SESSION,
  DISTANCE_ENGAGED_TOTAL,
  DISTANCE_ENGAGED_PERCENT_SESSION,
  DISTANCE_ENGAGED_PERCENT_TOTAL,
  TIME_CAR_RUNNING_SESSION,
  TIME_CAR_RUNNING_TOTAL,
  TIME_OPENPILOT_ENGAGED_SESSION,
  TIME_OPENPILOT_ENGAGED_TOTAL,
  TIME_OPENPILOT_ENGAGED,
  TIME_ENGAGED_PERCENT_SESSION,
  TIME_ENGAGED_PERCENT_TOTAL,
  DISENGAGEMENT_COUNT_SESSION,
  DISENGAGEMENT_COUNT_TOTAL,
  INTERACTION_COUNT_SESSION,
  INTERACTION_COUNT_TOTAL,
  INTERVENTION_COUNT_SESSION,
  INTERVENTION_COUNT_TOTAL,
  DISTRACTION_COUNT_SESSION,
  DISTRACTION_COUNT_TOTAL,
  INTERACTION_DISTANCE,
  INTERVENTION_DISTANCE,
  DISTRACTION_DISTANCE,
  DISTANCE_PER_DISENGAGEMENT_SESSION,
  DISTANCE_PER_DISENGAGEMENT_TOTAL,
  DISTANCE_PER_INTERACTION_SESSION,
  DISTANCE_PER_INTERACTION_TOTAL,
  DISTANCE_PER_INTERVENTION_SESSION,
  DISTANCE_PER_INTERVENTION_TOTAL,
  DISTANCE_PER_DISTRACTION_SESSION,
  DISTANCE_PER_DISTRACTION_TOTAL,
  TIME_PER_DISENGAGEMENT_SESSION,
  TIME_PER_DISENGAGEMENT_TOTAL,
  TIME_PER_INTERACTION_SESSION,
  TIME_PER_INTERACTION_TOTAL,
  TIME_PER_INTERVENTION_SESSION,
  TIME_PER_INTERVENTION_TOTAL,
  TIME_PER_DISTRACTION_SESSION,
  TIME_PER_DISTRACTION_TOTAL,
  INTERACTION_TIMER,
  INTERVENTION_TIMER,
  DISTRACTION_TIMER,
  // Lead/traffic info
  FOLLOW_LEVEL,
  LEAD_TTC,
  LEAD_DISTANCE_LENGTH,
  LEAD_DISTANCE_TIME,
  LEAD_DESIRED_DISTANCE_LENGTH,
  LEAD_DESIRED_DISTANCE_TIME,
  LEAD_COSTS,
  LEAD_VELOCITY_RELATIVE,
  LEAD_VELOCITY_ABS,
  LANE_POSITION,
  LANE_OFFSET,
  TRAFFIC_COUNT_TOTAL,
  TRAFFIC_COUNT_ONCOMING,
  TRAFFIC_COUNT_ONGOING,
  TRAFFIC_COUNT_STOPPED,
  TRAFFIC_COUNT_ADJACENT_ONGOING,
  TRAFFIC_ADJ_ONGOING_MIN_DISTANCE,
  // EV info
  HVB_VOLTAGE,
  HVB_CURRENT,
  HVB_WATTAGE,
  HVB_WATTVOLT,
  EV_EFF_NOW,
  EV_EFF_RECENT,
  EV_EFF_TRIP,
  EV_CONSUM_NOW,
  EV_CONSUM_RECENT,
  EV_CONSUM_TRIP,
  EV_BOTH_NOW,
  EV_OBSERVED_DRIVETRAIN_EFF,
  // Device info
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
  GPS_ACCURACY,
  // vision turn speed controller
  VISION_CURLATACCEL,
  VISION_MAXVFORCURCURV,
  VISION_MAXPREDLATACCEL,
  VISION_VF,
  
  NUM_MEASURES
} UIMeasure;

typedef struct UIScene {

  mat3 view_from_calib;
  bool world_objects_visible;

  // Debug UI
  bool show_debug_ui;

  bool show_cur_speed;

  bool map_open;

  float mass = 2000.0;

  bool lead_info_print_enabled;
  std::deque<int> lead_x_vals, lead_y_vals;
  int lead_x, lead_y;
  int const lead_xy_num_vals = 5;

  bool adjacent_paths_enabled;
  bool adjacent_lead_info_print_enabled;
  bool adjacent_lead_info_print_at_lead;
  std::string adjacent_leads_left_str, adjacent_leads_right_str, adjacent_lead_left_ttp_str, adjacent_lead_right_ttp_str;
  std::vector<std::string> adjacent_leads_center_strs;
  Rect adjacent_lead_info_touch_rect = {1,1,1,1};

  LaneTraffic traffic_left, traffic_right;

  bool is_using_torque_control = false;

  // Speed limit control
  bool speed_limit_control_enabled;
  bool speed_limit_perc_offset;
  Rect speed_limit_sign_touch_rect;
  double last_speed_limit_sign_tap;
  int speed_limit_eu_style = false;

  std::string current_road_name;

  weather_info weather_info;
  Rect weather_touch_rect;
  bool weather_simple_show_percip = false;
  int weather_simple_alernate_wind_precip_update_freq = 4;
  bool auto_brightness_enabled = false;
  
  // adjustable lane position
  Rect lane_pos_left_touch_rect = {1,1,1,1}, lane_pos_right_touch_rect = {1,1,1,1};
  bool lane_pos_enabled = false;
  int lane_pos = 0; // 0, 1, -1 = center, left, right
  float lane_pos_offset = 0.0; // in meters
  float lane_pos_dist_short = 800.; // ≈1/3 mile short timeout
  float lane_pos_dist_long = 16000.; // 10 mile long timeout
  float lane_pos_timeout_dist = lane_pos_dist_short;
  float lane_pos_set_t = 0.;
  float lane_pos_dist_since_set = 0.;
  float lane_pos_dist_last_t = 0.;
  float lane_pos_max_steer_deg = 150.;
  bool auto_lane_pos_active = false;
  
  Rect wheel_touch_rect;
  bool wheel_rotates = true;

  bool car_is_ev = false;

  bool color_path = false;
  bool alt_engage_color_enabled = false;
  
  float screen_dim_modes_v[3] = {0.01, 0.3, 1.};
  int screen_dim_mode_max = 2;
  int screen_dim_mode_cur = screen_dim_mode_max; 
  int screen_dim_mode = screen_dim_mode_cur, 
    screen_dim_mode_last = screen_dim_mode_cur;
  float screen_dim_fade = -1., screen_dim_fade_last_t = 0., screen_dim_fade_step = 1;
  float screen_dim_fade_dur_up = 0.5, screen_dim_fade_dur_down = 2.;
  Rect screen_dim_touch_rect;
  bool screen_tapped = false;
  bool screen_tapped2 = false;

  cereal::PandaState::PandaType pandaType;

  std::string network_type_string;
  int network_strength;

  std::vector<float> power_cur {0.,0.,0.,0.}, 
      power_max {75., 100., 100.,  55.}; // [kW] starting upper limits for ICE, EV, brake, and regen power based on volt (volt has 149hp EV output, 101hp motor, a reported 55-60 kW of regen capacity. Brake is based on volt braking at 3.5m/s^2 at 45mph. these go up if more power is observed
  float power_meter_pow = 0;
  Rect power_meter_rect, power_meter_text_rect;
  int power_meter_mode = 0; // 0/1/2 for meter/meter+number/old style brake indicator
  bool power_meter_metric = true; // true/false for kW/hp power output
  float power_meter_ema_k = 0.2;

  
// measures
  int measure_min_num_slots = 0;
  int measure_max_num_slots = 10;
  int measure_max_rows = measure_max_num_slots / 2;
  int measure_cur_num_slots = 3;
  int measure_slots[10];
  Rect measure_slots_rect;
  Rect measure_slot_touch_rects[10];
  int num_measures = UIMeasure::NUM_MEASURES; // the number of cases handled in ui_draw_measures() in paint.cc
  std::vector<int> measure_config_list = {0,1,2,3,4,5,6,8,10}; // 6,8,10 are 3x2, 4x2, and 5x2
  int measure_config_num = 0;
  int measure_num_rows = 0;
  int measure_row_offset = 0;
  float measures_touch_timeout = 10.;
  float measures_last_tap_t = -measures_touch_timeout;
  std::set<UIMeasure> EVMeasures = {
    UIMeasure::EV_FORCE,
    UIMeasure::EV_POWER,
    UIMeasure::EV_POWER_HP,
    UIMeasure::REGEN_FORCE,
    UIMeasure::REGEN_POWER,
    UIMeasure::REGEN_POWER_HP,
    UIMeasure::HVB_VOLTAGE,
    UIMeasure::HVB_CURRENT,
    UIMeasure::HVB_WATTAGE,
    UIMeasure::HVB_WATTVOLT,
    UIMeasure::EV_EFF_NOW,
    UIMeasure::EV_EFF_RECENT,
    UIMeasure::EV_EFF_TRIP,
    UIMeasure::EV_CONSUM_NOW,
    UIMeasure::EV_CONSUM_RECENT,
    UIMeasure::EV_CONSUM_TRIP,
    UIMeasure::EV_BOTH_NOW,
    UIMeasure::EV_OBSERVED_DRIVETRAIN_EFF
  };
  
  Rect speed_rect;
  float road_roll, device_roll;
  
  // actual measures
  float angleSteers, angleSteersDes, angleSteersErr;
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
  float bearingAccuracy;
  float bearingDeg;
  
  float lastTime = 0., sessionInitTime = 0.;
  float paramsCheckLast = 0., paramsCheckFreq = 0.1; // check params at 10Hz
  bool onePedalModeActive, onePedalModeSimple, disableDisengageOnGasEnabled, visionBrakingEnabled, mapBrakingEnabled;

  int lead_status;
  float lead_d_rel, lead_v_rel, lead_v;

  // EV efficiency
  float ev_eff_distances[2] = {10.f, 8046.f};
  float ev_eff_distances_recip[2] = {1.f/ev_eff_distances[0], 1.f/ev_eff_distances[1]}; // [m] denominator for weighted average weights
  float ev_eff_stopped_kWh = 0.; // [kWh]
  float ev_eff_total_kWh = 0.; // [kWh]
  float ev_eff_total_dist = 0.; // [m]
  float ev_recip_eff_wa[2] = {0., 0.}; // [kW] weighted averages of mi(km)/kWh
  float ev_eff_total = 0.;
  float ev_recip_eff_wa_max = 250.;
  float ev_eff_last_time = 0.;
  int ev_eff_params_write_freq = 500; // save trip and 5mi efficiencies every 5s


  // gps
  float altitudeUblox, gpsAccuracyUblox = 0.;
  int satelliteCount;
  bool gpsOK;
  
  // brake indicator
  int brake_percent;
  float brake_indicator_alpha;
  float brake_indicator_last_t;
  bool brake_indicator_enabled;
  
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
  int dynamic_follow_r[3] = {201, 157, 0};
  int dynamic_follow_g[3] = {34, 157, 4};
  int dynamic_follow_b[3] = {49, 157, 196};
  int dynamic_follow_bg_r[3] = {201, 0, 0};
  int dynamic_follow_bg_g[3] = {34, 0, 4};
  int dynamic_follow_bg_b[3] = {49, 0, 196};
  
  // one-pedal mode fading. maxspeed rect at -1, fades away by 0, and one-pedal icon fades in by 1
  float one_pedal_fade = -1., one_pedal_fade_last_t = 0.;
  Rect one_pedal_touch_rect;
  Rect maxspeed_touch_rect;
  Rect brake_touch_rect;
  
  int laneless_mode;
  Rect laneless_btn_touch_rect;

  cereal::DeviceState::Reader deviceState;
  cereal::RadarState::Reader radarState;
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
  line_vertices_data track_inside_vertices;
  line_vertices_data lane_line_vertices[4];
  line_vertices_data road_edge_vertices[2];

  line_vertices_data lane_vertices_left, lane_vertices_right;

  bool dm_active, engageable;

  // lead
  vertex_data lead_vertices[2];
  std::vector<lead_vertex_data> lead_vertices_oncoming, lead_vertices_ongoing, lead_vertices_stopped;

  float light_sensor, accel_sensor, gyro_sensor;
  bool started, ignition, is_metric, longitudinal_control, end_to_end;
  uint64_t started_frame;
  
  struct _LateralPlan
  {
    float laneWidth;
    float laneCenter;

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
