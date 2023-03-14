#pragma once

#include "cereal/messaging/messaging.h"
#include "cereal/visionipc/visionipc_client.h"
#include "common/util.h"
#include "common/modeldata.h"
#include "selfdrive/modeld/models/commonmodel.h"
#include "selfdrive/modeld/runners/run.h"

constexpr int NAV_INPUT_SIZE = 256*256;
constexpr int NAV_FEATURE_LEN = 64;
constexpr int NAV_DESIRE_LEN = 32;
constexpr int NAV_INSTRUCTION_LEN = 41;

constexpr int K_START = 1;
constexpr int K_DESTINATION = 4;
constexpr int K_CONTINUE = 8;
constexpr int K_SLIGHT_RIGHT = 9;
constexpr int K_RIGHT = 10;
constexpr int K_SHARP_RIGHT = 11;
constexpr int K_UTURN_LEFT = 13;
constexpr int K_SHARP_LEFT = 14;
constexpr int K_LEFT = 15;
constexpr int K_SLIGHT_LEFT = 16;
constexpr int K_RAMP_STRAIGHT = 17;
constexpr int K_RAMP_RIGHT = 18;
constexpr int K_RAMP_LEFT = 19;
constexpr int K_EXIT_RIGHT = 20;
constexpr int K_EXIT_LEFT = 21;
constexpr int K_STAY_STRAIGHT = 22;
constexpr int K_STAY_RIGHT = 23;
constexpr int K_STAY_LEFT = 24;
constexpr int K_ROUNDABOUT_ENTER = 26;
constexpr int K_ROUNDABOUT_EXIT = 27;

struct NavModelOutputXY {
  float x;
  float y;
};
static_assert(sizeof(NavModelOutputXY) == sizeof(float)*2);

struct NavModelOutputPlan {
  std::array<NavModelOutputXY, TRAJECTORY_SIZE> mean;
  std::array<NavModelOutputXY, TRAJECTORY_SIZE> std;
};
static_assert(sizeof(NavModelOutputPlan) == sizeof(NavModelOutputXY)*TRAJECTORY_SIZE*2);

struct NavModelOutputDesirePrediction {
  std::array<float, NAV_DESIRE_LEN> values;
};
static_assert(sizeof(NavModelOutputDesirePrediction) == sizeof(float)*NAV_DESIRE_LEN);

struct NavModelOutputFeatures {
  std::array<float, NAV_FEATURE_LEN> values;
};
static_assert(sizeof(NavModelOutputFeatures) == sizeof(float)*NAV_FEATURE_LEN);

struct NavModelResult {
  const NavModelOutputPlan plan;
  const NavModelOutputDesirePrediction desire_pred;
  const NavModelOutputFeatures features;
  float dsp_execution_time;
};
static_assert(sizeof(NavModelResult) == sizeof(NavModelOutputPlan) + sizeof(NavModelOutputDesirePrediction) + sizeof(NavModelOutputFeatures) + sizeof(float));

constexpr int NAV_OUTPUT_SIZE = sizeof(NavModelResult) / sizeof(float);
constexpr int NAV_NET_OUTPUT_SIZE = NAV_OUTPUT_SIZE - 1;

struct NavModelState {
  RunModel *m;
  uint8_t net_input_buf[NAV_INPUT_SIZE];
  float output[NAV_OUTPUT_SIZE];
};

void navmodel_init(NavModelState* s);
NavModelResult* navmodel_eval_frame(NavModelState* s, VisionBuf* buf);
void navmodel_publish(PubMaster &pm, uint32_t frame_id, const NavModelResult &model_res, float execution_time);
void navmodel_free(NavModelState* s);
