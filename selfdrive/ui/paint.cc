#include "selfdrive/ui/paint.h"

#include <algorithm>
#include <cassert>
#include <string>
#include <cmath>

#include <QDateTime>

#ifdef __APPLE__
#include <OpenGL/gl3.h>
#define NANOVG_GL3_IMPLEMENTATION
#define nvgCreate nvgCreateGL3
#else
#include <GLES3/gl3.h>
#define NANOVG_GLES3_IMPLEMENTATION
#define nvgCreate nvgCreateGLES3
#endif

#define NANOVG_GLES3_IMPLEMENTATION
#include <nanovg_gl.h>
#include <nanovg_gl_utils.h>

#include "selfdrive/common/timing.h"
#include "selfdrive/common/util.h"
#include "selfdrive/hardware/hw.h"

#include "selfdrive/ui/ui.h"

static void ui_draw_text(const UIState *s, float x, float y, const char *string, float size, NVGcolor color, const char *font_name) {
  nvgFontFace(s->vg, font_name);
  nvgFontSize(s->vg, size);
  nvgFillColor(s->vg, color);
  nvgText(s->vg, x, y, string, NULL);
}

static void ui_draw_circle(UIState *s, float x, float y, float size, NVGcolor color) {
  nvgBeginPath(s->vg);
  nvgCircle(s->vg, x, y, size);
  nvgFillColor(s->vg, color);
  nvgFill(s->vg);
}

static void ui_draw_speed_sign(UIState *s, float x, float y, int size, float speed, const char *subtext, 
                               float subtext_size, const char *font_name, bool is_map_sourced, bool is_active) {
  NVGcolor ring_color = is_active ? COLOR_RED : COLOR_BLACK_ALPHA(.2f * 255);
  NVGcolor inner_color = is_active ? COLOR_WHITE : COLOR_WHITE_ALPHA(.35f * 255);
  NVGcolor text_color = is_active ? COLOR_BLACK : COLOR_BLACK_ALPHA(.3f * 255);

  ui_draw_circle(s, x, y, float(size), ring_color);
  ui_draw_circle(s, x, y, float(size) * 0.8, inner_color);

  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);

  const std::string speedlimit_str = std::to_string((int)std::nearbyint(speed));
  ui_draw_text(s, x, y, speedlimit_str.c_str(), 120, text_color, font_name);
  ui_draw_text(s, x, y + 55, subtext, subtext_size, text_color, font_name);

  if (is_map_sourced) {
    const int img_size = 35;
    const int img_y = int(y - 55);
    ui_draw_image(s, {int(x - (img_size / 2)), img_y - (img_size / 2), img_size, img_size}, "map_source_icon", 
                  is_active ? 1. : .3);
  }
}

const float OneOverSqrt3 = 1.0 / sqrt(3.0);
static void ui_draw_turn_speed_sign(UIState *s, float x, float y, int width, float speed, int curv_sign, 
                                    const char *subtext, const char *font_name, bool is_active) {
  const float stroke_w = 15.0;
  NVGcolor border_color = is_active ? COLOR_RED : COLOR_BLACK_ALPHA(.2f * 255);
  NVGcolor inner_color = is_active ? COLOR_WHITE : COLOR_WHITE_ALPHA(.35f * 255);
  NVGcolor text_color = is_active ? COLOR_BLACK : COLOR_BLACK_ALPHA(.3f * 255);

  const float cS = stroke_w * 0.5 + 4.5;  // half width of the stroke on the corners of the triangle
  const float R = width * 0.5 - stroke_w * 0.5;
  const float A = 0.73205;
  const float h2 = 2.0 * R / (1.0 + A);
  const float h1 = A * h2;
  const float L = 4.0 * R * OneOverSqrt3;

  // Draw the internal triangle, compensate for stroke width. Needed to improve rendering when in inactive 
  // state due to stroke transparency being different from inner transparency.
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, x, y - R + cS);
  nvgLineTo(s->vg, x - L * 0.5 + cS, y + h1 + h2 - R - stroke_w * 0.5);
  nvgLineTo(s->vg, x + L * 0.5 - cS, y + h1 + h2 - R - stroke_w * 0.5);
  nvgClosePath(s->vg);

  nvgFillColor(s->vg, inner_color);
  nvgFill(s->vg);
  
  // Draw the stroke
  nvgLineJoin(s->vg, NVG_ROUND);
  nvgStrokeWidth(s->vg, stroke_w);
  nvgStrokeColor(s->vg, border_color);

  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, x, y - R);
  nvgLineTo(s->vg, x - L * 0.5, y + h1 + h2 - R);
  nvgLineTo(s->vg, x + L * 0.5, y + h1 + h2 - R);
  nvgClosePath(s->vg);

  nvgStroke(s->vg);

  // Draw the turn sign
  if (curv_sign != 0) {
    const int img_size = 35;
    const int img_y = int(y - R + stroke_w + 30);
    ui_draw_image(s, {int(x - (img_size / 2)), img_y, img_size, img_size}, 
                  curv_sign > 0 ? "turn_left_icon" : "turn_right_icon", is_active ? 1. : .3);
  }

  // Draw the texts.
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
  const std::string speedlimit_str = std::to_string((int)std::nearbyint(speed));
  ui_draw_text(s, x, y + 25, speedlimit_str.c_str(), 90., text_color, font_name);
  ui_draw_text(s, x, y + 65, subtext, 30., text_color, font_name);
}

static void draw_chevron(UIState *s, float x, float y, float sz, NVGcolor fillColor, NVGcolor glowColor) {
  // glow
  float g_xo = sz * 0.2;
  float g_yo = sz * 0.1;
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, x+(sz*1.35)+g_xo, y+sz+g_yo);
  nvgLineTo(s->vg, x, y-g_xo);
  nvgLineTo(s->vg, x-(sz*1.35)-g_xo, y+sz+g_yo);
  nvgClosePath(s->vg);
  nvgFillColor(s->vg, glowColor);
  nvgFill(s->vg);

  // chevron
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, x+(sz*1.25), y+sz);
  nvgLineTo(s->vg, x, y);
  nvgLineTo(s->vg, x-(sz*1.25), y+sz);
  nvgClosePath(s->vg);
  nvgFillColor(s->vg, fillColor);
  nvgFill(s->vg);
}

static void ui_draw_circle_image(const UIState *s, int center_x, int center_y, int radius, const char *image, NVGcolor color, float img_alpha) {
  nvgBeginPath(s->vg);
  nvgCircle(s->vg, center_x, center_y, radius);
  nvgFillColor(s->vg, color);
  nvgFill(s->vg);
  const int img_size = radius * 1.5;
  ui_draw_image(s, {center_x - (img_size / 2), center_y - (img_size / 2), img_size, img_size}, image, img_alpha);
}

static void ui_draw_circle_image(const UIState *s, int center_x, int center_y, int radius, const char *image, bool active) {
  float bg_alpha = active ? 0.3f : 0.1f;
  float img_alpha = active ? 1.0f : 0.15f;
  ui_draw_circle_image(s, center_x, center_y, radius, image, nvgRGBA(0, 0, 0, (255 * bg_alpha)), img_alpha);
}


static void draw_lead(UIState *s, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data, const vertex_data &vd) {
  // Draw lead car indicator
  auto [x, y] = vd;

  float fillAlpha = 0;
  float speedBuff = 10.;
  float leadBuff = 40.;
  float d_rel = lead_data.getX()[0];
  float v_rel = lead_data.getV()[0];
  if (d_rel < leadBuff) {
    fillAlpha = 255*(1.0-(d_rel/leadBuff));
    if (v_rel < 0) {
      fillAlpha += 255*(-1*(v_rel/speedBuff));
    }
    fillAlpha = (int)(fmin(fillAlpha, 255));
  }

  float sz = std::clamp((25 * 30) / (d_rel * 0.33333f + 30), 15.0f, 30.0f) * 2.35;
  x = std::clamp(x, 0.f, s->fb_w - sz * 0.5f);
  y = std::fmin(s->fb_h - sz * .6, y);
  draw_chevron(s, x, y, sz, nvgRGBA(201, 34, 49, fillAlpha), COLOR_YELLOW);
}

static void ui_draw_line(UIState *s, const line_vertices_data &vd, NVGcolor *color, NVGpaint *paint) {
  if (vd.cnt == 0) return;

  const vertex_data *v = &vd.v[0];
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, v[0].x, v[0].y);
  for (int i = 1; i < vd.cnt; i++) {
    nvgLineTo(s->vg, v[i].x, v[i].y);
  }
  nvgClosePath(s->vg);
  if (color) {
    nvgFillColor(s->vg, *color);
  } else if (paint) {
    nvgFillPaint(s->vg, *paint);
  }
  nvgFill(s->vg);
}

static void draw_vision_frame(UIState *s) {
  glBindVertexArray(s->frame_vao);
  mat4 *out_mat = &s->rear_frame_mat;
  glActiveTexture(GL_TEXTURE0);

  if (s->last_frame) {
    glBindTexture(GL_TEXTURE_2D, s->texture[s->last_frame->idx]->frame_tex);
    if (!Hardware::EON()) {
      // this is handled in ion on QCOM
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, s->last_frame->width, s->last_frame->height,
                   0, GL_RGB, GL_UNSIGNED_BYTE, s->last_frame->addr);
    }
  }

  glUseProgram(s->gl_shader->prog);
  glUniform1i(s->gl_shader->getUniformLocation("uTexture"), 0);
  glUniformMatrix4fv(s->gl_shader->getUniformLocation("uTransform"), 1, GL_TRUE, out_mat->v);

  assert(glGetError() == GL_NO_ERROR);
  glEnableVertexAttribArray(0);
  glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_BYTE, (const void *)0);
  glDisableVertexAttribArray(0);
  glBindVertexArray(0);
}

// sunnyhaibin's colored lane line
static void ui_draw_vision_lane_lines(UIState *s) {
  const UIScene &scene = s->scene;
  NVGpaint track_bg;
  int steerOverride = scene.car_state.getSteeringPressed();
  int red_lvl = 0;
  int green_lvl = 0;

  float red_lvl_line = 0;
  float green_lvl_line = 0;
  //if (!scene.end_to_end) {
  if (!scene.lateralPlan.lanelessModeStatus) {
    // paint lanelines
    for (int i = 0; i < std::size(scene.lane_line_vertices); i++) {
      if (scene.lane_line_probs[i] > 0.4){
        red_lvl_line = 1.0 - ((scene.lane_line_probs[i] - 0.4) * 2.5);
        green_lvl_line = 1.0;
      } else {
        red_lvl_line = 1.0;
        green_lvl_line = 1.0 - ((0.4 - scene.lane_line_probs[i]) * 2.5);
      }
      NVGcolor color = nvgRGBAf(1.0, 1.0, 1.0, scene.lane_line_probs[i]);
      color = nvgRGBAf(red_lvl_line, green_lvl_line, 0, 1);
      ui_draw_line(s, scene.lane_line_vertices[i], &color, nullptr);
    }

    // paint road edges
    for (int i = 0; i < std::size(scene.road_edge_vertices); i++) {
      NVGcolor color = nvgRGBAf(1.0, 0.0, 0.0, std::clamp<float>(1.0 - scene.road_edge_stds[i], 0.0, 1.0));
      ui_draw_line(s, scene.road_edge_vertices[i], &color, nullptr);
    }
  }
  if (scene.controls_state.getEnabled()) {
    if (steerOverride) {
      track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h*.4,
        COLOR_BLACK_ALPHA(80), COLOR_BLACK_ALPHA(20));
    } else if (!scene.lateralPlan.lanelessModeStatus) {
      red_lvl = 0;
      green_lvl = 200;
      track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h*.4,
        nvgRGBA(red_lvl, green_lvl, 0, 250), nvgRGBA(red_lvl, green_lvl, 0, 50));
    } else { // differentiate laneless mode color (Grace blue)
        track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
          nvgRGBA(0, 100, 255, 250), nvgRGBA(0, 100, 255, 50));
    }
  } else {
    // Draw white vision track
    track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
                                          COLOR_WHITE_ALPHA(150), COLOR_WHITE_ALPHA(20));
  }
  // paint path
  ui_draw_line(s, scene.track_vertices, nullptr, &track_bg);
}

// Draw all world space objects.
static void ui_draw_world(UIState *s) {
  nvgScissor(s->vg, 0, 0, s->fb_w, s->fb_h);

  // Draw lane edges and vision/mpc tracks
  ui_draw_vision_lane_lines(s);

  // Draw lead indicators if openpilot is handling longitudinal
  if (s->scene.longitudinal_control) {
    auto lead_one = (*s->sm)["modelV2"].getModelV2().getLeadsV3()[0];
    auto lead_two = (*s->sm)["modelV2"].getModelV2().getLeadsV3()[1];
    if (lead_one.getProb() > .5) {
      draw_lead(s, lead_one, s->scene.lead_vertices[0]);
    }
   if (lead_two.getProb() > .5 && (std::abs(lead_one.getX()[0] - lead_two.getX()[0]) > 3.0)) {
      draw_lead(s, lead_two, s->scene.lead_vertices[1]);
    }
  }
  nvgResetScissor(s->vg);
}

static void ui_draw_vision_maxspeed(UIState *s) {
  const int SET_SPEED_NA = 255;
  float maxspeed = (*s->sm)["controlsState"].getControlsState().getVCruise();
  const Rect rect = {bdr_s * 2, int(bdr_s * 1.5), 184, 202};
  if (s->scene.one_pedal_fade > 0.){
    const QColor &color = bg_colors[s->scene.car_state.getOnePedalBrakeMode() + 1];
    NVGcolor nvg_color = nvgRGBA(color.red(), color.green(), color.blue(), int(s->scene.one_pedal_fade * float(color.alpha())));
    ui_draw_circle_image(s, rect.centerX(), rect.centerY(), brake_size, "one_pedal_mode", nvg_color, s->scene.one_pedal_fade);
  }
  else{
    const bool is_cruise_set = maxspeed != 0 && maxspeed != SET_SPEED_NA;
    if (is_cruise_set && !s->scene.is_metric) { maxspeed *= 0.6225; }

    ui_fill_rect(s->vg, rect, COLOR_BLACK_ALPHA(int(-s->scene.one_pedal_fade * 100.)), 30.);
    ui_draw_rect(s->vg, rect, COLOR_WHITE_ALPHA(int(-s->scene.one_pedal_fade * 100.)), 10, 20.);

    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
    ui_draw_text(s, rect.centerX(), 118, "MAX", 26 * 2.5, COLOR_WHITE_ALPHA(is_cruise_set ? int(-s->scene.one_pedal_fade * 200.) : int(-s->scene.one_pedal_fade * 100.)), "sans-regular");
    if (is_cruise_set) {
      const std::string maxspeed_str = std::to_string((int)std::nearbyint(maxspeed));
      ui_draw_text(s, rect.centerX(), 212, maxspeed_str.c_str(), 48 * 2.5, COLOR_WHITE, "sans-bold");
    } else {
      ui_draw_text(s, rect.centerX(), 212, "N/A", 42 * 2.5, COLOR_WHITE_ALPHA(int(-s->scene.one_pedal_fade * 100.)), "sans-semibold");
    }
  }
}

static void ui_draw_vision_speedlimit(UIState *s) {
  auto longitudinal_plan = (*s->sm)["longitudinalPlan"].getLongitudinalPlan();
  const float speedLimit = longitudinal_plan.getSpeedLimit();
  const float speedLimitOffset = longitudinal_plan.getSpeedLimitOffset();

  if (speedLimit > 0.0 && s->scene.engageable) {
    const Rect maxspeed_rect = {bdr_s * 2, int(bdr_s * 1.5), 184, 202};
    Rect speed_sign_rect = Rect{maxspeed_rect.centerX() - speed_sgn_r, 
      maxspeed_rect.bottom() + bdr_s, 
      2 * speed_sgn_r, 
      2 * speed_sgn_r};
    const float speed = speedLimit * (s->scene.is_metric ? 3.6 : 2.2369362921);
    const float speed_offset = speedLimitOffset * (s->scene.is_metric ? 3.6 : 2.2369362921);

    auto speedLimitControlState = longitudinal_plan.getSpeedLimitControlState();
    const bool force_active = s->scene.speed_limit_control_enabled && 
                              seconds_since_boot() < s->scene.last_speed_limit_sign_tap + 2.0;
    const bool inactive = !force_active && (!s->scene.speed_limit_control_enabled || 
                          speedLimitControlState == cereal::LongitudinalPlan::SpeedLimitControlState::INACTIVE);
    const bool temp_inactive = !force_active && (s->scene.speed_limit_control_enabled && 
                               speedLimitControlState == cereal::LongitudinalPlan::SpeedLimitControlState::TEMP_INACTIVE);

    const int distToSpeedLimit = int(longitudinal_plan.getDistToSpeedLimit() * 
                                     (s->scene.is_metric ? 1.0 : 3.28084) / 10) * 10;
    const bool is_map_sourced = longitudinal_plan.getIsMapSpeedLimit();
    const std::string distance_str = std::to_string(distToSpeedLimit) + (s->scene.is_metric ? "m" : "f");
    const std::string offset_str = speed_offset > 0.0 ? "+" + std::to_string((int)std::nearbyint(speed_offset)) : "";
    const std::string inactive_str = temp_inactive ? "TEMP" : "";
    const std::string substring = inactive || temp_inactive ? inactive_str : 
                                                              distToSpeedLimit > 0 ? distance_str : offset_str;
    const float substring_size = inactive || temp_inactive || distToSpeedLimit > 0 ? 30.0 : 50.0;

    ui_draw_speed_sign(s, speed_sign_rect.centerX(), speed_sign_rect.centerY(), speed_sgn_r, speed, substring.c_str(), 
                       substring_size, "sans-bold", is_map_sourced, !inactive && !temp_inactive);

    s->scene.speed_limit_sign_touch_rect = Rect{speed_sign_rect.x - speed_sgn_touch_pad, 
                                                speed_sign_rect.y - speed_sgn_touch_pad,
                                                speed_sign_rect.w + 2 * speed_sgn_touch_pad, 
                                                speed_sign_rect.h + 2 * speed_sgn_touch_pad};
  }
}


NVGcolor color_from_thermal_status(int thermalStatus){
  switch (thermalStatus){
    case 0: return nvgRGBA(0, 255, 0, 200);
    case 1: return nvgRGBA(255, 128, 0, 200);
    default: return nvgRGBA(255, 0, 0, 200);
  }
}

static void ui_draw_measures(UIState *s){
  if (s->scene.measure_cur_num_slots){
    const Rect maxspeed_rect = {bdr_s * 2, int(bdr_s * 1.5), 184, 202};
    const int center_x = s->fb_w - face_wheel_radius - bdr_s * 2;
    const int brake_y = s->fb_h - footer_h / 2;
    const int y_min = maxspeed_rect.bottom() + bdr_s / 2;
    const int y_max = brake_y - brake_size - bdr_s / 2;
    const int y_rng = y_max - y_min;
    const int slot_y_rng = y_rng / s->scene.measure_max_num_slots * 2; // two columns
    const int y_mid = (y_max + y_min) / 2;
    const int slots_y_rng = slot_y_rng * (s->scene.measure_cur_num_slots <= 5 ? s->scene.measure_cur_num_slots : 5);
    const int slots_y_min = y_mid - (slots_y_rng / 2);
  
    NVGcolor default_name_color = nvgRGBA(255, 255, 255, 200);
    NVGcolor default_unit_color = nvgRGBA(255, 255, 255, 200);
    NVGcolor default_val_color = nvgRGBA(255, 255, 255, 200);
    int default_val_font_size = 78;
    int default_name_font_size = 32;
    int default_unit_font_size = 38;
  
    // determine bounding rectangle
    const int slots_r = brake_size + 4;
    const int slots_w = (s->scene.measure_cur_num_slots <= 5 ? 2 : 4) * slots_r;
    const int slots_x = (s->scene.measure_cur_num_slots <= 5 ? center_x - slots_r : center_x - 3 * slots_r);
    const Rect slots_rect = {slots_x, slots_y_min, slots_w, slots_y_rng};
    // draw bounding rectangle
    nvgBeginPath(s->vg);
    nvgRoundedRect(s->vg, slots_rect.x, slots_rect.y, slots_rect.w, slots_rect.h, 20);
    nvgStrokeColor(s->vg, nvgRGBA(255,255,255,200));
    nvgStrokeWidth(s->vg, 6);
    nvgStroke(s->vg);
    nvgFillColor(s->vg, nvgRGBA(0,0,0,100));
    nvgFill(s->vg);

    UIScene &scene = s->scene;
    
    // now start from the top and draw the current set of metrics
    for (int i = 0; i < scene.measure_cur_num_slots; ++i){
      char name[16], val[16], unit[6];
      snprintf(name, sizeof(name), "");
      snprintf(val, sizeof(val), "");
      snprintf(unit, sizeof(unit), "");
      NVGcolor val_color, label_color, unit_color;
      int val_font_size, label_font_size, unit_font_size;
      int g, b;
      float p;
      
      val_color = default_val_color;
      label_color = default_name_color;
      unit_color = default_unit_color;
      val_font_size = default_val_font_size;
      label_font_size = default_name_font_size;
      unit_font_size = default_unit_font_size;
    
      // switch to get metric strings/colors 
      switch (scene.measure_slots[i]){

        case UIMeasure::CPU_TEMP_AND_PERCENT: 
          {
          val_color = color_from_thermal_status(int(scene.deviceState.getThermalStatus()));
          if (s->is_metric) {
            snprintf(val, sizeof(val), "%.0f°C", scene.cpuTemp);
          }
          else{
            snprintf(val, sizeof(val), "%.0f°F", scene.cpuTemp * 1.8 + 32.);
          }
          snprintf(unit, sizeof(unit), "%d%%", scene.cpuPerc);
          snprintf(name, sizeof(name), "CPU");}
          break;
        
        case UIMeasure::CPU_TEMP: 
          {
          val_color = color_from_thermal_status(int(scene.deviceState.getThermalStatus()));
          if (s->is_metric) {
            snprintf(val, sizeof(val), "%.0f", scene.cpuTemp);
            snprintf(unit, sizeof(unit), "°C");
          }
          else{
            snprintf(val, sizeof(val), "%.0f", scene.cpuTemp * 1.8 + 32.);
            snprintf(unit, sizeof(unit), "°F");
          }
          snprintf(name, sizeof(name), "CPU TEMP");}
          break;
        
        case UIMeasure::MEMORY_TEMP: 
          {
          val_color = color_from_thermal_status(int(scene.deviceState.getThermalStatus()));
          if (s->is_metric) {
            snprintf(val, sizeof(val), "%.0f", scene.deviceState.getMemoryTempC());
            snprintf(unit, sizeof(unit), "°C");
          }
          else{
            snprintf(val, sizeof(val), "%.0f", scene.deviceState.getMemoryTempC() * 1.8 + 32.);
            snprintf(unit, sizeof(unit), "°F");
          }
          snprintf(name, sizeof(name), "MEM TEMP");}
          break;
        
        case UIMeasure::CPU_PERCENT: 
          {
          val_color = color_from_thermal_status(int(scene.deviceState.getThermalStatus()));
          snprintf(val, sizeof(val), "%d%%", scene.cpuPerc);
          snprintf(name, sizeof(name), "CPU PERC");}
          break;
        
        case UIMeasure::AMBIENT_TEMP: 
          {
          val_color = color_from_thermal_status(int(scene.deviceState.getThermalStatus()));
          if (s->is_metric) {
            snprintf(val, sizeof(val), "%.0f", scene.deviceState.getAmbientTempC());
            snprintf(unit, sizeof(unit), "°C");
          }
          else{
            snprintf(val, sizeof(val), "%.0f", scene.deviceState.getAmbientTempC() * 1.8 + 32.);
            snprintf(unit, sizeof(unit), "°F");
          }
          snprintf(name, sizeof(name), "AMB TEMP");}
          break;
        
        case UIMeasure::FANSPEED_PERCENT: 
          {
          val_color = color_from_thermal_status(int(scene.deviceState.getThermalStatus()));
          snprintf(val, sizeof(val), "%d%%", scene.deviceState.getFanSpeedPercentDesired());
          snprintf(name, sizeof(name), "FAN");}
          break;
        
        case UIMeasure::MEMORY_USAGE_PERCENT: 
          {
          int mem_perc = scene.deviceState.getMemoryUsagePercent();
          g = 255; 
          b = 255;
          p = 0.011764706 * (mem_perc); // red by 85%
          g -= int(0.5 * p * 255.);
          b -= int(p * 255.);
          g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
          b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
          val_color = nvgRGBA(255, g, b, 200);
          snprintf(val, sizeof(val), "%d%%", mem_perc);
          snprintf(name, sizeof(name), "MEM USED");}
          break;
        
        case UIMeasure::FREESPACE_STORAGE: 
          {
          int free_perc = scene.deviceState.getFreeSpacePercent();
          g = 0;
          b = 0;
          p = 0.05 * free_perc; // white at or above 20% freespace
          g += int((0.5+p) * 255.);
          b += int(p * 255.);
          g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
          b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
          val_color = nvgRGBA(255, g, b, 200);
          snprintf(val, sizeof(val), "%d%%", free_perc);
          snprintf(name, sizeof(name), "SSD FREE");}
          break;

        case UIMeasure::GPS_ACCURACY:
          {
          snprintf(name, sizeof(name), "GPS PREC");
          if (scene.gpsAccuracyUblox != 0.00) {
            //show red/orange if gps accuracy is low
            if(scene.gpsAccuracyUblox > 0.85) {
               val_color = nvgRGBA(255, 188, 3, 200);
            }
            if(scene.gpsAccuracyUblox > 1.3) {
               val_color = nvgRGBA(255, 0, 0, 200);
            }
            // gps accuracy is always in meters
            if(scene.gpsAccuracyUblox > 99 || scene.gpsAccuracyUblox == 0) {
               snprintf(val, sizeof(val), "None");
            }else if(scene.gpsAccuracyUblox > 9.99) {
              snprintf(val, sizeof(val), "%.1f", scene.gpsAccuracyUblox);
            }
            else {
              snprintf(val, sizeof(val), "%.2f", scene.gpsAccuracyUblox);
            }
            snprintf(unit, sizeof(unit), "%d", scene.satelliteCount);
          }}
          break;

        case UIMeasure::ALTITUDE:
          {
          snprintf(name, sizeof(name), "ALTITUDE");
          if (scene.gpsAccuracyUblox != 0.00) {
            float tmp_val;
            if (s->is_metric) {
              tmp_val = scene.altitudeUblox;
              snprintf(val, sizeof(val), "%.0f", scene.altitudeUblox);
              snprintf(unit, sizeof(unit), "m");
            } else {
              tmp_val = scene.altitudeUblox * 3.2808399;
              snprintf(val, sizeof(val), "%.0f", tmp_val);
              snprintf(unit, sizeof(unit), "ft");
            }
            if (log10(tmp_val) >= 4){
              val_font_size -= 10;
            }
          }}
          break;

        case UIMeasure::STEERING_TORQUE_EPS:
          {
          snprintf(name, sizeof(name), "EPS TRQ");
          //TODO: Add orange/red color depending on torque intensity. <1x limit = white, btwn 1x-2x limit = orange, >2x limit = red
          snprintf(val, sizeof(val), "%.0f", scene.steeringTorqueEps);
          snprintf(unit, sizeof(unit), "Nm");
          break;}

        case UIMeasure::ACCELERATION:
          {
          snprintf(name, sizeof(name), "ACCEL");
          snprintf(val, sizeof(val), "%.1f", scene.aEgo);
          snprintf(unit, sizeof(unit), "m/s²");
          break;}
        
        case UIMeasure::JERK:
          {
          snprintf(name, sizeof(name), "JERK");
          snprintf(val, sizeof(val), "%.1f", scene.jEgo);
          snprintf(unit, sizeof(unit), "m/s³");
          break;}

        case UIMeasure::LEAD_DISTANCE_LENGTH:
          {
            snprintf(name, sizeof(name), "REL DIST");
            if (scene.lead_status) {
              if (s->is_metric) {
                g = 0;
                b = 0;
                p = 0.0333 * scene.lead_d_rel;
                g += int((0.5+p) * 255.);
                b += int(p * 255.);
                g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
                b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
                val_color = nvgRGBA(255, g, b, 200);
                snprintf(val, sizeof(val), "%d", (int)scene.lead_d_rel);
              }
              else{
                g = 0;
                b = 0;
                p = 0.01 * scene.lead_d_rel;
                g += int((0.5+p) * 255.);
                b += int(p * 255.);
                g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
                b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
                val_color = nvgRGBA(255, g, b, 200);
                snprintf(val, sizeof(val), "%d", (int)(scene.lead_d_rel * 3.281));
              }
            } else {
               snprintf(val, sizeof(val), "-");
            }
            if (s->is_metric) {
              snprintf(unit, sizeof(unit), "m");
            }
            else{
              snprintf(unit, sizeof(unit), "ft");
            }
          }
          break;
          
        case UIMeasure::LEAD_DESIRED_DISTANCE_LENGTH:
          {
            snprintf(name, sizeof(name), "DES DIST");
            if (scene.lead_status) {
              auto follow_d = scene.desiredFollowDistance * scene.car_state.getVEgo();
              if (s->is_metric) {
                snprintf(val, sizeof(val), "%d", (int)follow_d);
              }
              else{
                snprintf(val, sizeof(val), "%d", (int)(follow_d * 3.281));
              }
            } else {
               snprintf(val, sizeof(val), "-");
            }
            if (s->is_metric) {
              snprintf(unit, sizeof(unit), "m");
            }
            else{
              snprintf(unit, sizeof(unit), "ft");
            }
          }
          break;
          
        case UIMeasure::LEAD_DISTANCE_TIME:
          {
          snprintf(name, sizeof(name), "REL DIST");
          if (scene.lead_status && scene.car_state.getVEgo() > 0.1) {
            float follow_t = scene.lead_d_rel / scene.car_state.getVEgo();
            g = 0;
            b = 0;
            p = 0.6667 * follow_t;
            g += int((0.5+p) * 255.);
            b += int(p * 255.);
            g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
            b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
            val_color = nvgRGBA(255, g, b, 200);
            snprintf(val, sizeof(val), "%.1f", follow_t);
          } else {
             snprintf(val, sizeof(val), "-");
          }
          snprintf(unit, sizeof(unit), "s");}
          break;
          
        case UIMeasure::LEAD_DESIRED_DISTANCE_TIME:
          {
          snprintf(name, sizeof(name), "DES DIST");
          if (scene.lead_status && scene.car_state.getVEgo() > 0.1) {
            snprintf(val, sizeof(val), "%.1f", scene.desiredFollowDistance);
          } else {
             snprintf(val, sizeof(val), "-");
          }
          snprintf(unit, sizeof(unit), "s");}
          break;
        
        case UIMeasure::LEAD_DISTANCE_COST:
          {
            snprintf(name, sizeof(name), "DIST COST");
            if (scene.lead_status && scene.car_state.getVEgo() > 0.1) {
              snprintf(val, sizeof(val), "%.1f", scene.followDistanceCost);
            } else {
               snprintf(val, sizeof(val), "-");
            }
          }
          break;

        case UIMeasure::LEAD_VELOCITY_RELATIVE:
          {
          snprintf(name, sizeof(name), "REL SPEED");
          if (scene.lead_status) {
            g = 255; 
            b = 255;
            p = -0.2 * (scene.lead_v_rel);
            g -= int(0.5 * p * 255.);
            b -= int(p * 255.);
            g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
            b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
            val_color = nvgRGBA(255, g, b, 200);
            // lead car relative speed is always in meters
            if (s->is_metric) {
               snprintf(val, sizeof(val), "%d", (int)(scene.lead_v_rel * 3.6 + 0.5));
            } else {
               snprintf(val, sizeof(val), "%d", (int)(scene.lead_v_rel * 2.2374144 + 0.5));
            }
          } else {
             snprintf(val, sizeof(val), "-");
          }
          if (s->is_metric) {
            snprintf(unit, sizeof(unit), "km/h");;
          } else {
            snprintf(unit, sizeof(unit), "mph");
          }}
          break;
        
        case UIMeasure::LEAD_VELOCITY_ABS: 
          {
          snprintf(name, sizeof(name), "LEAD SPD");
          if (scene.lead_status) {
            if (s->is_metric) {
               snprintf(val, sizeof(val), "%d", (int)(scene.lead_v * 3.6 + 0.5));
            } else {
               snprintf(val, sizeof(val), "%d", (int)(scene.lead_v * 2.2374144 + 0.5));
            }
          } else {
             snprintf(val, sizeof(val), "-");
          }
          if (s->is_metric) {
            snprintf(unit, sizeof(unit), "km/h");;
          } else {
            snprintf(unit, sizeof(unit), "mph");
          }}
          break;

        case UIMeasure::STEERING_ANGLE: 
          {
          snprintf(name, sizeof(name), "REAL STEER");
          float angleSteers = scene.angleSteers > 0. ? scene.angleSteers : -scene.angleSteers;
          g = 255;
          b = 255;
          p = 0.0333 * angleSteers;
          g -= int(0.5 * p * 255.);
          b -= int(p * 255.);
          g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
          b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
          val_color = nvgRGBA(255, g, b, 200);
          // steering is in degrees
          snprintf(val, sizeof(val), "%.0f°", scene.angleSteers);
          snprintf(unit, sizeof(unit), "");}
          break;

        case UIMeasure::DESIRED_STEERING_ANGLE: 
          {
          snprintf(name, sizeof(name), "DESIRE STR.");
          if (scene.controls_state.getEnabled()) {
            float angleSteers = scene.angleSteersDes > 0. ? scene.angleSteersDes : -scene.angleSteersDes;
            g = 255;
            b = 255;
            p = 0.0333 * angleSteers;
            g -= int(0.5 * p * 255.);
            b -= int(p * 255.);
            g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
            b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
            val_color = nvgRGBA(255, g, b, 200);
            // steering is in degrees
            snprintf(val, sizeof(val), "%.0f°", scene.angleSteersDes);
          } else {
             snprintf(val, sizeof(val), "-");
          }
          snprintf(unit, sizeof(unit), "");}
          break;

        case UIMeasure::ENGINE_RPM: 
          {
          snprintf(name, sizeof(name), "ENG RPM");
          if(scene.engineRPM == 0) {
            snprintf(val, sizeof(val), "OFF");
          }
          else {snprintf(val, sizeof(val), "%d", scene.engineRPM);}
          snprintf(unit, sizeof(unit), "");}
          break;
        
        case UIMeasure::PERCENT_GRADE:
          {
          snprintf(name, sizeof(name), "GRADE");
          if (scene.percentGradeIterRolled && scene.gpsAccuracyUblox != 0.00){
            // use stored values of length and altitude (in meters)
            float rise = scene.percentGradeAltitudes[scene.percentGradeRollingIter] - scene.percentGradeAltitudes[(scene.percentGradeRollingIter+1)%5];
            float run = scene.percentGradeLenStep * 5.;
            scene.percentGrade = int(round(rise/run * 100.));
            g = 255;
            b = 255;
            p = 0.05 * scene.percentGrade; // red by 20% grade
            g -= int(0.5 * p * 255.);
            b -= int(p * 255.);
            g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
            b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
            val_color = nvgRGBA(255, g, b, 200);
          }
          snprintf(val, sizeof(val), "%d%%", scene.percentGrade);
          snprintf(unit, sizeof(unit), "");}
          break;

        default: {// invalid number
          snprintf(name, sizeof(name), "INVALID");
          snprintf(val, sizeof(val), "⚠️");}
          break;
      }

      nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
      // now print the metric
      // first value
      
      int slot_x = slots_rect.x + (scene.measure_cur_num_slots <= 5 ? 0 : (i < 5 ? slots_r * 2 : 0));
      int x = slot_x + slots_r - unit_font_size / 2;
      if (i >= 5){
        x = slot_x + slots_r + unit_font_size / 2;
      }
      int slot_y = slots_rect.y + (i % 5) * slot_y_rng;
      int slot_y_mid = slot_y + slot_y_rng / 2;
      int y = slot_y_mid + slot_y_rng / 2 - 8 - label_font_size;
      nvgFontFace(s->vg, "sans-semibold");
      nvgFontSize(s->vg, val_font_size);
      nvgFillColor(s->vg, val_color);
      nvgText(s->vg, x, y, val, NULL);
    
      // now label
      y = slot_y_mid + slot_y_rng / 2 - 9;
      nvgFontFace(s->vg, "sans-regular");
      nvgFontSize(s->vg, label_font_size);
      nvgFillColor(s->vg, label_color);
      nvgText(s->vg, x, y, name, NULL);
    
      // now unit
      if (strlen(unit) > 0){
        nvgSave(s->vg);
        int rx = slot_x + slots_r * 2;
        if (i >= 5){
          rx = slot_x;
          nvgTranslate(s->vg, rx + 13, slot_y_mid);
          nvgRotate(s->vg, 1.5708); //-90deg in radians
        }
        else{
          nvgTranslate(s->vg, rx - 13, slot_y_mid);
          nvgRotate(s->vg, -1.5708); //-90deg in radians
        }
        nvgFontFace(s->vg, "sans-regular");
        nvgFontSize(s->vg, unit_font_size);
        nvgFillColor(s->vg, unit_color);
        nvgText(s->vg, 0, 0, unit, NULL);
        nvgRestore(s->vg);
      }
    
      // update touch rect
      scene.measure_slot_touch_rects[i] = {slot_x, slot_y, slots_r * 2, slot_y_rng};
    }
  }
}


static void ui_draw_vision_turnspeed(UIState *s) {
  auto longitudinal_plan = (*s->sm)["longitudinalPlan"].getLongitudinalPlan();
  const float turnSpeed = longitudinal_plan.getTurnSpeed();
  const float vEgo = (*s->sm)["carState"].getCarState().getVEgo();
  const bool show = turnSpeed > 0.0 && (turnSpeed < vEgo || s->scene.show_debug_ui);

  if (show) {
    const Rect maxspeed_rect = {bdr_s * 2, int(bdr_s * 1.5), 184, 202};
    Rect speed_sign_rect = Rect{maxspeed_rect.centerX() - speed_sgn_r, 
      maxspeed_rect.bottom() + 2 * (bdr_s + speed_sgn_r), 
      2 * speed_sgn_r, 
      maxspeed_rect.h};
    const float speed = turnSpeed * (s->scene.is_metric ? 3.6 : 2.2369362921);

    auto turnSpeedControlState = longitudinal_plan.getTurnSpeedControlState();
    const bool is_active = turnSpeedControlState > cereal::LongitudinalPlan::SpeedLimitControlState::TEMP_INACTIVE;

    const int curveSign = longitudinal_plan.getTurnSign();
    const int distToTurn = int(longitudinal_plan.getDistToTurn() * 
                               (s->scene.is_metric ? 1.0 : 3.28084) / 10) * 10;
    const std::string distance_str = std::to_string(distToTurn) + (s->scene.is_metric ? "m" : "f");

    ui_draw_turn_speed_sign(s, speed_sign_rect.centerX(), speed_sign_rect.centerY(), speed_sign_rect.w, speed, 
                            curveSign, distToTurn > 0 ? distance_str.c_str() : "", "sans-bold", is_active);
  }
}

static void ui_draw_vision_speed(UIState *s) {
  const float speed = std::max(0.0, (*s->sm)["carState"].getCarState().getVEgo() * (s->scene.is_metric ? 3.6 : 2.2369363));
  const std::string speed_str = std::to_string((int)std::nearbyint(speed));
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
  ui_draw_text(s, s->fb_w / 2, 210, speed_str.c_str(), 96 * 2.5, COLOR_WHITE, "sans-bold");
  ui_draw_text(s, s->fb_w / 2, 290, s->scene.is_metric ? "km/h" : "mph", 36 * 2.5, COLOR_WHITE_ALPHA(200), "sans-regular");
  s->scene.speed_rect = {s->fb_w / 2 - 50, 150, 150, 300};
}

static void ui_draw_vision_event(UIState *s) {
  auto longitudinal_plan = (*s->sm)["longitudinalPlan"].getLongitudinalPlan();
  auto visionTurnControllerState = longitudinal_plan.getVisionTurnControllerState();
  if (s->scene.show_debug_ui && 
      visionTurnControllerState > cereal::LongitudinalPlan::VisionTurnControllerState::DISABLED && 
      s->scene.engageable) {
    // draw a rectangle with colors indicating the state with the value of the acceleration inside.
    const int size = 184;
    const Rect rect = {s->fb_w - size - bdr_s, int(bdr_s * 1.5), size, size};
    ui_fill_rect(s->vg, rect, COLOR_BLACK_ALPHA(100), 30.);

    auto source = longitudinal_plan.getLongitudinalPlanSource();
    const int alpha = source == cereal::LongitudinalPlan::LongitudinalPlanSource::TURN ? 255 : 100;
    const QColor &color = tcs_colors[int(visionTurnControllerState)];
    NVGcolor nvg_color = nvgRGBA(color.red(), color.green(), color.blue(), alpha);
    ui_draw_rect(s->vg, rect, nvg_color, 10, 20.);
    
    const float vision_turn_speed = longitudinal_plan.getVisionTurnSpeed() * (s->scene.is_metric ? 3.6 : 2.2369363);
    std::string acc_str = std::to_string((int)std::nearbyint(vision_turn_speed));
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
    ui_draw_text(s, rect.centerX(), rect.centerY(), acc_str.c_str(), 56, COLOR_WHITE_ALPHA(alpha), "sans-bold");
  } else if (s->scene.engageable) {
    // draw steering wheel
    const float rot_angle = -s->scene.angleSteers * 0.01745329252;
    const int radius = 88;
    const int center_x = s->fb_w - radius - bdr_s * 2;
    const int center_y = radius  + (bdr_s * 1.5);
    const QColor &color = bg_colors[s->status];
    NVGcolor nvg_color = nvgRGBA(color.red(), color.green(), color.blue(), color.alpha());
    
    nvgBeginPath(s->vg);
    nvgCircle(s->vg, center_x, center_y, radius);
    nvgFillColor(s->vg, nvg_color);
    nvgFill(s->vg);
    
    nvgSave(s->vg);
    nvgTranslate(s->vg, center_x, center_y);
    nvgRotate(s->vg, rot_angle);
    nvgBeginPath(s->vg);
    NVGpaint imgPaint = nvgImagePattern(s->vg, -radius, -radius, 2*radius, 2*radius, 0, s->images.at("wheel"), 1.0f);
    nvgRect(s->vg, -radius, -radius, 2*radius, 2*radius);
    nvgFillPaint(s->vg, imgPaint);
    nvgFill(s->vg);
    nvgRestore(s->vg);
        
    // ui_draw_circle_image(s, center_x, center_y, radius, "wheel", nvg_color, 1.0f);
    
    // draw hands on wheel pictogram under wheel pictogram.
    auto handsOnWheelState = (*s->sm)["driverMonitoringState"].getDriverMonitoringState().getHandsOnWheelState();
    if (handsOnWheelState >= cereal::DriverMonitoringState::HandsOnWheelState::WARNING) {
      NVGcolor color = COLOR_RED;
      if (handsOnWheelState == cereal::DriverMonitoringState::HandsOnWheelState::WARNING) {
        color = COLOR_YELLOW;
      } 
      const int wheel_y = center_y + bdr_s + 2 * radius;
      ui_draw_circle_image(s, center_x, wheel_y, radius, "hands_on_wheel", color, 1.0f);
    }
  }
}

static void ui_draw_vision_face(UIState *s) {
  const Rect maxspeed_rect = {bdr_s * 2, int(bdr_s * 1.5), 184, 202};
  const int radius = 96;
  const int center_x = maxspeed_rect.centerX();
  const int center_y = s->fb_h - footer_h / 2;
  ui_draw_circle_image(s, center_x, center_y, radius, "driver_face", s->scene.dm_active);
}

static void ui_draw_vision_brake(UIState *s) {
  if (s->scene.brake_percent >= 0){
    const int brake_x = s->fb_w - face_wheel_radius - bdr_s * 2;
    const int brake_y = s->fb_h - footer_h / 2;
    float bg_alpha = 0.1 + 0.2 * s->scene.brake_indicator_alpha;
    float img_alpha = 0.15 + 0.85 * s->scene.brake_indicator_alpha;
    NVGcolor color = nvgRGBA(0, 0, 0, (255 * bg_alpha));
    if (s->scene.brake_percent > 0 && s->scene.brake_percent <= 100){
      int r = 0;
      if (s->scene.brake_percent >= 50){
        float p = 0.01 * float(s->scene.brake_percent - 50);
        bg_alpha += 0.3 * p;
        r = 200. * p;
      }
      color = nvgRGBA(r, 0, 0, (255 * bg_alpha));
    }
    ui_draw_circle_image(s, brake_x, brake_y, brake_size, "brake_disk", color, img_alpha);
    if (s->scene.brake_percent > 0 && s->scene.brake_percent <= 100){
      const int brake_r1 = 1;
      const int brake_r2 = brake_size / 3 + 4;
      const float brake_r_range = brake_r2 - brake_r1;
      float p = s->scene.brake_percent;
      const int brake_r = brake_r1 + int(brake_r_range * p * 0.01);
      nvgBeginPath(s->vg);
      nvgRoundedRect(s->vg, brake_x - brake_r, brake_y - brake_r, 2 * brake_r, 2 * brake_r, brake_r);
      nvgStrokeWidth(s->vg, 6);
      nvgFontSize(s->vg, s->scene.brake_percent < 100 ? 72 : 64);
      int r = 255, g = 255, b = 255, a = 200;
      p *= 0.01;
      g -= int(p * 255.);
      g = (g > 0 ? g : 0);
      b -= int((.6 + p) * 255.);
      b = (b > 0 ? b : 0); // goes from white to orange to red at p goes from 0 to 100
      nvgFillColor(s->vg, nvgRGBA(r,g,b,a));
      nvgStrokeColor(s->vg, nvgRGBA(r,g,b,255));
      nvgFill(s->vg);
      nvgStroke(s->vg);
    }
  }
}

static void draw_laneless_button(UIState *s) {
  if (s->vipc_client->connected) {
    const Rect maxspeed_rect = {bdr_s * 2, int(bdr_s * 1.5), 184, 202};
    const int vision_face_radius = 96;
    const int radius = 72;
    const int center_x = maxspeed_rect.centerX() + vision_face_radius + bdr_s + radius;
    const int center_y = s->fb_h - footer_h / 2 - radius / 2;
    int btn_w = radius * 2;
    int btn_h = radius * 2;
    int btn_x1 = center_x - 0.5 * radius;
    int btn_y = center_y - 0.5 * radius;
    int btn_xc1 = btn_x1 + radius;
    int btn_yc = btn_y + radius;
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
    nvgBeginPath(s->vg);
    nvgRoundedRect(s->vg, btn_x1, btn_y, btn_w, btn_h, 100);
    // nvgRoundedRect(s->vg, btn_x1, btn_y, btn_w, btn_h, 100);
    nvgStrokeColor(s->vg, nvgRGBA(0,0,0,80));
    nvgStrokeWidth(s->vg, 6);
    nvgStroke(s->vg);
    nvgFontSize(s->vg, 54);

    if (s->scene.laneless_mode == 0) {
      nvgStrokeColor(s->vg, nvgRGBA(0,125,0,255));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
      NVGcolor fillColor = nvgRGBA(0,125,0,80);
      nvgFillColor(s->vg, fillColor);
      nvgFill(s->vg);
      nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
      nvgText(s->vg,btn_xc1,btn_yc-20,"Lane",NULL);
      nvgText(s->vg,btn_xc1,btn_yc+20,"only",NULL);
    } else if (s->scene.laneless_mode == 1) {
      nvgStrokeColor(s->vg, nvgRGBA(0,100,255,255));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
      NVGcolor fillColor = nvgRGBA(0,100,255,80);
      nvgFillColor(s->vg, fillColor);
      nvgFill(s->vg);
      nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
      nvgText(s->vg,btn_xc1,btn_yc-20,"Lane",NULL);
      nvgText(s->vg,btn_xc1,btn_yc+20,"less",NULL);
    } else if (s->scene.laneless_mode == 2) {
      nvgStrokeColor(s->vg, nvgRGBA(125,0,125,255));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
      NVGcolor fillColor = nvgRGBA(125,0,125,80);
      nvgFillColor(s->vg, fillColor);
      nvgFill(s->vg);
      nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
      nvgText(s->vg,btn_xc1,btn_yc-20,"Auto",NULL);
      nvgText(s->vg,btn_xc1,btn_yc+20,"Lane",NULL);
    }
    
    s->scene.laneless_btn_touch_rect = Rect{center_x - laneless_btn_touch_pad, 
                                                center_y - laneless_btn_touch_pad,
                                                radius + 2 * laneless_btn_touch_pad, 
                                                radius + 2 * laneless_btn_touch_pad}; 
  }
}

static void ui_draw_vision_header(UIState *s) {
  NVGpaint gradient = nvgLinearGradient(s->vg, 0, header_h - (header_h * 0.4), 0, header_h,
                                        nvgRGBAf(0, 0, 0, 0.45), nvgRGBAf(0, 0, 0, 0));
  ui_fill_rect(s->vg, {0, 0, s->fb_w , header_h}, gradient);
  ui_draw_vision_maxspeed(s);
  ui_draw_vision_speedlimit(s);
  ui_draw_vision_speed(s);
  ui_draw_vision_turnspeed(s);
  ui_draw_vision_event(s);
}

static void ui_draw_vision(UIState *s) {
  const UIScene *scene = &s->scene;
  // Draw augmented elements
  if (scene->world_objects_visible) {
    ui_draw_world(s);
  }
  // Set Speed, Current Speed, Status/Events
  ui_draw_vision_header(s);
  if ((*s->sm)["controlsState"].getControlsState().getAlertSize() == cereal::ControlsState::AlertSize::NONE) {
    ui_draw_vision_face(s);
    ui_draw_vision_brake(s);
    ui_draw_measures(s);
    if (s->scene.end_to_end) {
      draw_laneless_button(s);
    }
  }
  if ((*s->sm)["controlsState"].getControlsState().getAlertSize() == cereal::ControlsState::AlertSize::SMALL) {
    ui_draw_measures(s);
  }
}

void ui_draw(UIState *s, int w, int h) {
  const bool draw_vision = s->scene.started && s->vipc_client->connected;

  glViewport(0, 0, s->fb_w, s->fb_h);
  if (draw_vision) {
    draw_vision_frame(s);
  }
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  // NVG drawing functions - should be no GL inside NVG frame
  nvgBeginFrame(s->vg, s->fb_w, s->fb_h, 1.0f);
  if (draw_vision) {
    ui_draw_vision(s);
  }
  nvgEndFrame(s->vg);
  glDisable(GL_BLEND);
}

void ui_draw_image(const UIState *s, const Rect &r, const char *name, float alpha) {
  nvgBeginPath(s->vg);
  NVGpaint imgPaint = nvgImagePattern(s->vg, r.x, r.y, r.w, r.h, 0, s->images.at(name), alpha);
  nvgRect(s->vg, r.x, r.y, r.w, r.h);
  nvgFillPaint(s->vg, imgPaint);
  nvgFill(s->vg);
}

void ui_draw_rect(NVGcontext *vg, const Rect &r, NVGcolor color, int width, float radius) {
  nvgBeginPath(vg);
  radius > 0 ? nvgRoundedRect(vg, r.x, r.y, r.w, r.h, radius) : nvgRect(vg, r.x, r.y, r.w, r.h);
  nvgStrokeColor(vg, color);
  nvgStrokeWidth(vg, width);
  nvgStroke(vg);
}

static inline void fill_rect(NVGcontext *vg, const Rect &r, const NVGcolor *color, const NVGpaint *paint, float radius) {
  nvgBeginPath(vg);
  radius > 0 ? nvgRoundedRect(vg, r.x, r.y, r.w, r.h, radius) : nvgRect(vg, r.x, r.y, r.w, r.h);
  if (color) nvgFillColor(vg, *color);
  if (paint) nvgFillPaint(vg, *paint);
  nvgFill(vg);
}
void ui_fill_rect(NVGcontext *vg, const Rect &r, const NVGcolor &color, float radius) {
  fill_rect(vg, r, &color, nullptr, radius);
}
void ui_fill_rect(NVGcontext *vg, const Rect &r, const NVGpaint &paint, float radius) {
  fill_rect(vg, r, nullptr, &paint, radius);
}

static const char frame_vertex_shader[] =
#ifdef NANOVG_GL3_IMPLEMENTATION
  "#version 150 core\n"
#else
  "#version 300 es\n"
#endif
  "in vec4 aPosition;\n"
  "in vec4 aTexCoord;\n"
  "uniform mat4 uTransform;\n"
  "out vec4 vTexCoord;\n"
  "void main() {\n"
  "  gl_Position = uTransform * aPosition;\n"
  "  vTexCoord = aTexCoord;\n"
  "}\n";

static const char frame_fragment_shader[] =
#ifdef NANOVG_GL3_IMPLEMENTATION
  "#version 150 core\n"
#else
  "#version 300 es\n"
#endif
  "precision mediump float;\n"
  "uniform sampler2D uTexture;\n"
  "in vec4 vTexCoord;\n"
  "out vec4 colorOut;\n"
  "void main() {\n"
  "  colorOut = texture(uTexture, vTexCoord.xy);\n"
#ifdef QCOM
  "  vec3 dz = vec3(0.0627f, 0.0627f, 0.0627f);\n"
  "  colorOut.rgb = ((vec3(1.0f, 1.0f, 1.0f) - dz) * colorOut.rgb / vec3(1.0f, 1.0f, 1.0f)) + dz;\n"
#endif
  "}\n";

static const mat4 device_transform = {{
  1.0,  0.0, 0.0, 0.0,
  0.0,  1.0, 0.0, 0.0,
  0.0,  0.0, 1.0, 0.0,
  0.0,  0.0, 0.0, 1.0,
}};

void ui_nvg_init(UIState *s) {
  // init drawing

  // on EON, we enable MSAA
  s->vg = Hardware::EON() ? nvgCreate(0) : nvgCreate(NVG_ANTIALIAS | NVG_STENCIL_STROKES | NVG_DEBUG);
  assert(s->vg);

  // init fonts
  std::pair<const char *, const char *> fonts[] = {
      {"sans-regular", "../assets/fonts/opensans_regular.ttf"},
      {"sans-semibold", "../assets/fonts/opensans_semibold.ttf"},
      {"sans-bold", "../assets/fonts/opensans_bold.ttf"},
  };
  for (auto [name, file] : fonts) {
    int font_id = nvgCreateFont(s->vg, name, file);
    assert(font_id >= 0);
  }

  // init images
  std::vector<std::pair<const char *, const char *>> images = {
    {"wheel", "../assets/img_chffr_wheel.png"},
    {"driver_face", "../assets/img_driver_face.png"},
    {"hands_on_wheel", "../assets/img_hands_on_wheel.png"},
    {"turn_left_icon", "../assets/img_turn_left_icon.png"},
    {"turn_right_icon", "../assets/img_turn_right_icon.png"},
    {"map_source_icon", "../assets/img_world_icon.png"},
    {"brake_disk", "../assets/img_brake.png"},
    {"one_pedal_mode", "../assets/offroad/icon_car_pedal.png"}
  };
  for (auto [name, file] : images) {
    s->images[name] = nvgCreateImage(s->vg, file, 1);
    assert(s->images[name] != 0);
  }

  // init gl
  s->gl_shader = std::make_unique<GLShader>(frame_vertex_shader, frame_fragment_shader);
  GLint frame_pos_loc = glGetAttribLocation(s->gl_shader->prog, "aPosition");
  GLint frame_texcoord_loc = glGetAttribLocation(s->gl_shader->prog, "aTexCoord");

  glViewport(0, 0, s->fb_w, s->fb_h);

  glDisable(GL_DEPTH_TEST);

  assert(glGetError() == GL_NO_ERROR);

  float x1 = 1.0, x2 = 0.0, y1 = 1.0, y2 = 0.0;
  const uint8_t frame_indicies[] = {0, 1, 2, 0, 2, 3};
  const float frame_coords[4][4] = {
    {-1.0, -1.0, x2, y1}, //bl
    {-1.0,  1.0, x2, y2}, //tl
    { 1.0,  1.0, x1, y2}, //tr
    { 1.0, -1.0, x1, y1}, //br
  };

  glGenVertexArrays(1, &s->frame_vao);
  glBindVertexArray(s->frame_vao);
  glGenBuffers(1, &s->frame_vbo);
  glBindBuffer(GL_ARRAY_BUFFER, s->frame_vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(frame_coords), frame_coords, GL_STATIC_DRAW);
  glEnableVertexAttribArray(frame_pos_loc);
  glVertexAttribPointer(frame_pos_loc, 2, GL_FLOAT, GL_FALSE,
                        sizeof(frame_coords[0]), (const void *)0);
  glEnableVertexAttribArray(frame_texcoord_loc);
  glVertexAttribPointer(frame_texcoord_loc, 2, GL_FLOAT, GL_FALSE,
                        sizeof(frame_coords[0]), (const void *)(sizeof(float) * 2));
  glGenBuffers(1, &s->frame_ibo);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, s->frame_ibo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(frame_indicies), frame_indicies, GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);

  ui_resize(s, s->fb_w, s->fb_h);
}

void ui_resize(UIState *s, int width, int height) {
  s->fb_w = width;
  s->fb_h = height;

  auto intrinsic_matrix = s->wide_camera ? ecam_intrinsic_matrix : fcam_intrinsic_matrix;

  float zoom = ZOOM / intrinsic_matrix.v[0];

  if (s->wide_camera) {
    zoom *= 0.5;
  }

  float zx = zoom * 2 * intrinsic_matrix.v[2] / width;
  float zy = zoom * 2 * intrinsic_matrix.v[5] / height;

  const mat4 frame_transform = {{
    zx, 0.0, 0.0, 0.0,
    0.0, zy, 0.0, -y_offset / height * 2,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
  }};

  s->rear_frame_mat = matmul(device_transform, frame_transform);

  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  nvgTranslate(s->vg, width / 2, height / 2 + y_offset);
  // 2) Apply same scaling as video
  nvgScale(s->vg, zoom, zoom);
  // 3) Put (0, 0) in top left corner of video
  nvgTranslate(s->vg, -intrinsic_matrix.v[2], -intrinsic_matrix.v[5]);

  nvgCurrentTransform(s->vg, s->car_space_transform);
  nvgResetTransform(s->vg);
}
