#include "selfdrive/ui/paint.h"

#include <algorithm>
#include <cassert>
#include <string>
#include <cmath>
#include <deque>

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
  std::string const speedlimit_str = std::to_string((int)std::nearbyint(speed));
  float one_pedal_fade = MAX(0.5,-s->scene.one_pedal_fade);
  if (s->scene.speed_limit_eu_style){ // eu style
    NVGcolor ring_color = is_active ? COLOR_RED_ALPHA(int(one_pedal_fade * 255.)) : COLOR_RED_ALPHA(int(.2 * 255.));
    NVGcolor inner_color = is_active ? COLOR_WHITE_ALPHA(int(one_pedal_fade * 255.)) : COLOR_WHITE_ALPHA(int(.5 * 255.));
    NVGcolor text_color = is_active ? COLOR_BLACK_ALPHA(int(one_pedal_fade * 255.)) : COLOR_BLACK_ALPHA(int(.3 * 255.));

    ui_draw_circle(s, x, y, float(size), ring_color);
    ui_draw_circle(s, x, y, float(size) * 0.8, inner_color);

    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);

    ui_draw_text(s, x, y, speedlimit_str.c_str(), 120, text_color, font_name);
    ui_draw_text(s, x, y + 55, subtext, subtext_size, text_color, font_name);

    if (is_map_sourced) {
      const int img_size = 35;
      const int img_y = int(y - 55);
      ui_draw_image(s, {int(x - (img_size / 2)), img_y - (img_size / 2), img_size, img_size}, "map_source_icon", 
                    is_active ? 1. : .3);
    }
    s->scene.speed_limit_sign_touch_rect = Rect{int(x) - speed_sgn_touch_pad, 
                                            int(y) - speed_sgn_touch_pad,
                                            2 * (speed_sgn_touch_pad + size), 
                                            2 * (speed_sgn_touch_pad + size)};
  }
  else{ // us/canada style
    const int border_width = 6;
    const int sign_width = 164;
    const int sign_height = 216;
    const Rect maxspeed_rect = {bdr_s * 2, int(bdr_s * 1.5), 184, 202};

    // White outer square
    Rect sign_rect_outer = {maxspeed_rect.x + 10, int(y-size), sign_width, sign_height};
    ui_fill_rect(s->vg, sign_rect_outer, is_active ? COLOR_WHITE_ALPHA(int(one_pedal_fade * 255.)) : COLOR_WHITE_ALPHA(int(.5 * 255.)), 24);

    // Smaller black border
    Rect sign_rect = {int(sign_rect_outer.x + 1.5 * border_width), int(sign_rect_outer.y + 1.5 * border_width), int(sign_width - 3 * border_width), int(sign_height - 3 * border_width)};
    ui_draw_rect(s->vg, sign_rect, is_active ? COLOR_BLACK_ALPHA(int(one_pedal_fade * 255.)) : COLOR_BLACK_ALPHA(int(.5 * 255.)), border_width, 16);

    

    // Speed limit value

    if (subtext_size > 0. && is_active && s->scene.one_pedal_fade <= 0.){
      // "SPEED"
      nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_TOP);
      ui_draw_text(s, sign_rect.centerX(), sign_rect.y + 7, "SPEED", 19 * 2.5, is_active ? COLOR_BLACK_ALPHA(int(one_pedal_fade * 255.)) : COLOR_BLACK_ALPHA(int(.5 * 255.)), "sans-semibold");

      // "LIMIT"
      ui_draw_text(s, sign_rect.centerX(), sign_rect.y + 45, "LIMIT", 19 * 2.5, is_active ? COLOR_BLACK_ALPHA(int(one_pedal_fade * 255.)) : COLOR_BLACK_ALPHA(int(.5 * 255.)), "sans-semibold");
      ui_draw_text(s, sign_rect.centerX(), sign_rect.y + 76, speedlimit_str.c_str(), 37 * 2.5, is_active ? COLOR_BLACK_ALPHA(int(one_pedal_fade * 255.)) : COLOR_BLACK_ALPHA(int(.5 * 255.)), "sans-bold");
      ui_draw_text(s, sign_rect.centerX(), sign_rect.y + 150, subtext, 20 * 2.5, is_active ? COLOR_BLACK_ALPHA(int(one_pedal_fade * 255.)) : COLOR_BLACK_ALPHA(int(.5 * 200.)), "sans-bold");
    }
    else{
      // "SPEED"
      nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_TOP);
      ui_draw_text(s, sign_rect.centerX(), sign_rect.y + 15, "SPEED", 19 * 2.5, is_active ? COLOR_BLACK_ALPHA(int(one_pedal_fade * 255.)) : COLOR_BLACK_ALPHA(int(.5 * 255.)), "sans-semibold");

      // "LIMIT"
      ui_draw_text(s, sign_rect.centerX(), sign_rect.y + 54, "LIMIT", 19 * 2.5, is_active ? COLOR_BLACK_ALPHA(int(one_pedal_fade * 255.)) : COLOR_BLACK_ALPHA(int(.5 * 255.)), "sans-semibold");
      ui_draw_text(s, sign_rect.centerX(), sign_rect.y + 83, speedlimit_str.c_str(), 48 * 2.5, is_active ? COLOR_BLACK_ALPHA(int(one_pedal_fade * 255.)) : COLOR_BLACK_ALPHA(int(.5 * 255.)), "sans-bold");
    }


    s->scene.speed_limit_sign_touch_rect = sign_rect_outer;
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


static void draw_lead(UIState *s, float d_rel, float v_rel, const vertex_data &vd, bool draw_info, bool is_voacc) {
  // Draw lead car indicator
  auto [x, y] = vd;

  float fillAlpha = 0;
  float speedBuff = 10.;
  float leadBuff = 40.;
  if (d_rel < leadBuff) {
    fillAlpha = 255*(1.0-(d_rel/leadBuff));
    if (v_rel < 0) {
      fillAlpha += 255*(-1*(v_rel/speedBuff));
    }
    fillAlpha = (int)(fmin(fillAlpha, 255));
  }

  float sz = std::clamp((25 * 30) / (d_rel * 0.33333f + 30), 15.0f, 30.0f) * 2.35;
  x = std::clamp(x, 0.f, s->fb_w - sz * 0.5f);
  y = std::fmin(s->fb_h - sz * .6f, y);
  draw_chevron(s, x, y, sz, nvgRGBA(201, 34, 49, fillAlpha), COLOR_YELLOW);
  if (is_voacc){
    const int r = 24;
    nvgBeginPath(s->vg);
    nvgRoundedRect(s->vg, x - r, y + sz/2 - r, 2 * r, 2 * r, r);
    nvgFillColor(s->vg, COLOR_GRACE_BLUE);
    nvgFill(s->vg);
  }

  if ((s->scene.lead_info_print_enabled || s->scene.adjacent_lead_info_print_enabled) && !s->scene.map_open && draw_info){
    // print lead info around chevron
    // Print relative distances to the left of the chevron
    int const x_offset = 100;
    int const y_offset = 48;
    int const y_max = s->fb_h - 4*bdr_s;
    s->scene.lead_x_vals.push_back(x);
    s->scene.lead_y_vals.push_back(y);
    while (s->scene.lead_x_vals.size() > s->scene.lead_xy_num_vals){
      s->scene.lead_x_vals.pop_front();
      s->scene.lead_y_vals.pop_front();
    }
    s->scene.lead_x_vals.shrink_to_fit();
    s->scene.lead_y_vals.shrink_to_fit();
    int lead_x = 0, lead_y = 0;
    for (int const & v : s->scene.lead_x_vals){
      lead_x += v;
    }
    lead_x /= float(s->scene.lead_x_vals.size());
    for (int const & v : s->scene.lead_y_vals){
      lead_y += v > y_max ? y_max : v;
    }
    lead_y /= float(s->scene.lead_y_vals.size());
    s->scene.lead_x = lead_x;
    s->scene.lead_y = lead_y;
    nvgFillColor(s->vg, nvgRGBA(255, 255, 255, 180));
    nvgFontFace(s->vg, "sans-semibold");
    if (s->scene.lead_info_print_enabled){
      nvgTextAlign(s->vg, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE);
      nvgBeginPath(s->vg);
      nvgFontSize(s->vg, 120);
      char val[16], unit[8];

      // first time distance
      if (s->scene.car_state.getVEgo() > 0.5){
        snprintf(unit, sizeof(unit), "s"); 
        float follow_t = d_rel / s->scene.car_state.getVEgo();
        snprintf(val, sizeof(val), "%.1f%s", follow_t, unit);
      }
      else{
        snprintf(val, sizeof(val), "-");
      }
      nvgText(s->vg,lead_x-x_offset,lead_y-y_offset,val,NULL);

      // then length distance
      if (s->is_metric){
        snprintf(unit, sizeof(unit), "m"); 
        if (s->scene.lead_d_rel < 10.){
          snprintf(val, sizeof(val), "%.1f%s", s->scene.lead_d_rel, unit);
        }
        else{
          snprintf(val, sizeof(val), "%.0f%s", s->scene.lead_d_rel, unit);
        }
      }
      else{
        snprintf(unit, sizeof(unit), "ft"); 
        float d_ft = s->scene.lead_d_rel * 3.281;
        if (d_ft < 10.){
          snprintf(val, sizeof(val), "%.1f%s", d_ft, unit);
        }
        else{
          snprintf(val, sizeof(val), "%.0f%s", d_ft, unit);
        }
      }
      nvgText(s->vg,lead_x-x_offset,lead_y+y_offset,val,NULL);

      // now abs and relative speed to the right

      nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);
      // first abs speed
      if (s->is_metric){
        snprintf(unit, sizeof(unit), "km/h"); 
        float v = (s->scene.lead_v * 3.6);
        if (v < 100.){
          snprintf(val, sizeof(val), "%.1f", v);
        }
        else{
          snprintf(val, sizeof(val), "%.0f", v);
        }
      }
      else{
        snprintf(unit, sizeof(unit), "mph"); 
        float v = (s->scene.lead_v * 2.2374144);
        if (v < 100.){
          snprintf(val, sizeof(val), "%.1f", v);
        }
        else{
          snprintf(val, sizeof(val), "%.0f", v);
        }
      }
      nvgText(s->vg,lead_x+x_offset,lead_y-(y_offset*1.3),val,NULL);

      // then relative speed
      if (s->is_metric) {
          snprintf(val, sizeof(val), "%s%.1f", s->scene.lead_v_rel >= 0. ? "+" : "", (s->scene.lead_v_rel * 3.6));
      } else {
          snprintf(val, sizeof(val), "%s%.1f", s->scene.lead_v_rel >= 0. ? "+" : "", (s->scene.lead_v_rel * 2.2374144));
      }
      nvgText(s->vg,lead_x+x_offset,lead_y+(y_offset*1.4),val,NULL);

      nvgFontSize(s->vg, 70);
      nvgText(s->vg,lead_x+x_offset+20,lead_y,unit,NULL);
    }
    else{
      nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_TOP);
      nvgBeginPath(s->vg);
      nvgFontSize(s->vg, 90);
      char val[16];
      float v = (s->scene.lead_v * (s->is_metric ? 3.6 : 2.2374144));
      if (v < 100.){
        snprintf(val, sizeof(val), "%.1f", v);
      }
      else{
        snprintf(val, sizeof(val), "%.0f", v);
      }
      nvgText(s->vg,lead_x,lead_y+60,val,NULL);
    }
  }
}

static void draw_other_leads(UIState *s, bool lead_drawn) {
  // Draw lead car circle
  if (s->scene.adjacent_lead_info_print_enabled){
    int r1 = 8, r2 = 75;
    int dr = r2 - r1;
    int i = 0;
    float fill_screen_dist_offset = (s->scene.lead_info_print_enabled ? 50 : 10);
    float fill_screen_dist_max = 400;
    float text_screen_dist_offset = (s->scene.lead_info_print_enabled ? 100 : 10);
    float text_screen_dist_max = 300;
    for (auto const & vd : s->scene.lead_vertices_ongoing){
      auto [x, y, d, v] = vd;
      // fade leads too close to the actual lead
      int alpha_fill = 80;
      int alpha_stroke = 200;
      int alpha_text = 200;
      if (lead_drawn){
        float screen_dist = std::clamp(std::fabs(x - s->scene.lead_x) - fill_screen_dist_offset, 0.f, fill_screen_dist_max);
        float alpha_factor = 1. - float(screen_dist) / fill_screen_dist_max;
        alpha_fill -= 60. * alpha_factor;
        alpha_stroke -= 160. * alpha_factor;

        screen_dist = std::clamp(std::fabs(x - s->scene.lead_x) - text_screen_dist_offset, 0.f, text_screen_dist_max);
        alpha_factor = 1. - float(screen_dist) / text_screen_dist_max;
        alpha_text -= 190. * alpha_factor;
      }
      int r = r2 - int(float(dr) * d / 180.);
      r = (r < r1 ? r1 : r);
      nvgBeginPath(s->vg);
      nvgRoundedRect(s->vg, x - r, y - r, 2 * r, 2 * r, r);
      nvgFillColor(s->vg, interp_alert_color(-1., alpha_fill));
      nvgFill(s->vg);
      nvgStrokeColor(s->vg, interp_alert_color(-1., alpha_stroke));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);

      if (s->scene.adjacent_lead_info_print_at_lead){
        nvgFontFace(s->vg, "sans-semibold");
        nvgBeginPath(s->vg);
        nvgFontSize(s->vg, 3 * r / 2);
        nvgTextAlign(s->vg, NVG_ALIGN_MIDDLE | NVG_ALIGN_CENTER);
        nvgFillColor(s->vg, COLOR_WHITE_ALPHA(s->scene.lead_info_print_enabled ? alpha_stroke : alpha_text));
        char val[16];
        snprintf(val, sizeof(val), "%.0f", v * (s->is_metric ? 3.6 : 2.2374144));
        nvgText(s->vg,x,y,val,NULL);
      }
      i++;
    }
    i = 0;
    for (auto const & vd : s->scene.lead_vertices_oncoming){
      auto [x, y, d, v] = vd;
      // fade leads too close to the actual lead
      int alpha_fill = 80;
      int alpha_stroke = 200;
      int alpha_text = 200;
      if (lead_drawn){
        float screen_dist = std::clamp(std::fabs(x - s->scene.lead_x) - fill_screen_dist_offset, 0.f, fill_screen_dist_max);
        float alpha_factor = 1. - float(screen_dist) / fill_screen_dist_max;
        alpha_fill -= 60. * alpha_factor;
        alpha_stroke -= 160. * alpha_factor;

        screen_dist = std::clamp(std::fabs(x - s->scene.lead_x) - text_screen_dist_offset, 0.f, text_screen_dist_max);
        alpha_factor = 1. - float(screen_dist) / text_screen_dist_max;
        alpha_text -= 190. * alpha_factor;
      }
      int r = r2 - int(float(dr) * d / 180.);
      r = (r < r1 ? r1 : r);
      nvgBeginPath(s->vg);
      nvgRoundedRect(s->vg, x - r, y - r, 2 * r, 2 * r, r);
      nvgFillColor(s->vg, interp_alert_color(1.1, alpha_fill));
      nvgFill(s->vg);
      nvgStrokeColor(s->vg, interp_alert_color(1.1, alpha_stroke));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);

      if (s->scene.adjacent_lead_info_print_at_lead){
        nvgFontFace(s->vg, "sans-semibold");
        nvgBeginPath(s->vg);
        nvgFontSize(s->vg, 3 * r / 2);
        nvgTextAlign(s->vg, NVG_ALIGN_MIDDLE | NVG_ALIGN_CENTER);
        nvgFillColor(s->vg, COLOR_WHITE_ALPHA(s->scene.lead_info_print_enabled ? alpha_stroke : alpha_text));
        char val[16];
        snprintf(val, sizeof(val), "%.0f", v * (s->is_metric ? 3.6 : 2.2374144));
        nvgText(s->vg,x,y,val,NULL);
      }
      i++;
    }
    i = 0;
    for (auto const & vd : s->scene.lead_vertices_stopped){
      auto [x, y, d, v] = vd;
      // fade leads too close to the actual lead
      int alpha_fill = 80;
      int alpha_stroke = 200;
      int alpha_text = 200;
      if (lead_drawn){
        float screen_dist = std::clamp(std::fabs(x - s->scene.lead_x) - fill_screen_dist_offset, 0.f, fill_screen_dist_max);
        float alpha_factor = 1. - float(screen_dist) / fill_screen_dist_max;
        alpha_fill -= 60. * alpha_factor;
        alpha_stroke -= 160. * alpha_factor;

        screen_dist = std::clamp(std::fabs(x - s->scene.lead_x) - text_screen_dist_offset, 0.f, text_screen_dist_max);
        alpha_factor = 1. - float(screen_dist) / text_screen_dist_max;
        alpha_text -= 190. * alpha_factor;
      }
      int r = r2 - int(float(dr) * d / 180.);
      r = (r < r1 ? r1 : r);
      nvgBeginPath(s->vg);
      nvgRoundedRect(s->vg, x - r, y - r, 2 * r, 2 * r, r);
      nvgFillColor(s->vg, COLOR_WHITE_ALPHA(alpha_fill));
      nvgFill(s->vg);
      nvgStrokeColor(s->vg, COLOR_WHITE_ALPHA(alpha_stroke));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);

      if (s->scene.adjacent_lead_info_print_at_lead){
        nvgFontFace(s->vg, "sans-semibold");
        nvgBeginPath(s->vg);
        nvgFontSize(s->vg, 3 * r / 2);
        nvgTextAlign(s->vg, NVG_ALIGN_MIDDLE | NVG_ALIGN_CENTER);
        nvgFillColor(s->vg, COLOR_WHITE_ALPHA(s->scene.lead_info_print_enabled ? alpha_stroke : alpha_text));
        char val[16];
        snprintf(val, sizeof(val), "%.0f", v * (s->is_metric ? 3.6 : 2.2374144));
        nvgText(s->vg,x,y,val,NULL);
      }
      i++;
    }
  }
}

static void draw_adjacent_lead_speeds(UIState *s, bool lead_drawn){
  if (s->scene.adjacent_lead_info_print_enabled && !s->scene.map_open){
    nvgFontFace(s->vg, "sans-semibold");
    nvgBeginPath(s->vg);
    nvgFontSize(s->vg, 90);
    int y = s->fb_h + 10;
    int x;

    if (!s->scene.adjacent_lead_info_print_at_lead){
      // left leads
      nvgTextAlign(s->vg, NVG_ALIGN_RIGHT | NVG_ALIGN_BOTTOM);
      nvgFillColor(s->vg, COLOR_WHITE_ALPHA(200));
      int x = s->fb_w * 11 / 32;
      nvgText(s->vg,x,y,s->scene.adjacent_leads_left_str.c_str(),NULL);
      nvgText(s->vg,x,y-60,s->scene.adjacent_lead_left_ttp_str.c_str(),NULL);

      // right leads
      nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_BOTTOM);
      nvgFillColor(s->vg, COLOR_WHITE_ALPHA(200));
      x = s->fb_w * 21 / 32;
      nvgText(s->vg,x,y,s->scene.adjacent_leads_right_str.c_str(),NULL);
      nvgText(s->vg,x,y-60,s->scene.adjacent_lead_right_ttp_str.c_str(),NULL);
    }
    else{ // still print time to pass along bottom
      // left lead
      nvgTextAlign(s->vg, NVG_ALIGN_RIGHT | NVG_ALIGN_BOTTOM);
      nvgFillColor(s->vg, COLOR_WHITE_ALPHA(200));
      int x = s->fb_w * 11 / 32;
      nvgText(s->vg,x,y,s->scene.adjacent_lead_left_ttp_str.c_str(),NULL);

      // right lead
      nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_BOTTOM);
      nvgFillColor(s->vg, COLOR_WHITE_ALPHA(200));
      x = s->fb_w * 21 / 32;
      nvgText(s->vg,x,y,s->scene.adjacent_lead_right_ttp_str.c_str(),NULL);
    }
     
    // center leads
    nvgFontSize(s->vg, 90);
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
    nvgFillColor(s->vg, COLOR_WHITE_ALPHA(200));
    if (lead_drawn){
      nvgFontFace(s->vg, "sans-semibold");
      x = s->scene.lead_x;
      y = s->scene.lead_y - 20;
    }
    else{
      x = s->fb_w / 2;
    }
    bool first = false;
    for (auto const & v : s->scene.adjacent_leads_center_strs){
      if (first && lead_drawn){
        auto l1p_v = s->scene.radarState.getLeadOnePlus().getVLeadK();
        if (s->scene.lead_v - l1p_v > 3.){
          nvgFontFace(s->vg, "sans-bold");
          nvgFillColor(s->vg, COLOR_RED_ALPHA(200));
          nvgFontSize(s->vg, 110);
          nvgText(s->vg,x,y,v.c_str(),NULL);
          y -= 75;
          nvgFontFace(s->vg, "sans-semibold");
          nvgFillColor(s->vg, COLOR_WHITE_ALPHA(200));
          nvgFontSize(s->vg, 90);
        }      
        else{
          nvgText(s->vg,x,y,v.c_str(),NULL);
          y -= 60;
        }
      }
      else{
        nvgText(s->vg,x,y,v.c_str(),NULL);
        y -= 60;
      }
      first = false;
    }

    x = s->fb_w / 2;
    y = s->fb_h;
    s->scene.adjacent_lead_info_touch_rect = {x-150,y-300,300,300};
  }
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
  //if (!scene.end_to_end) {
  // paint lanelines
  for (int i = 0; i < std::size(scene.lane_line_vertices); i++) {
    NVGcolor color;
    if (!scene.lateralPlan.lanelessModeStatus) {
      color = interp_alert_color(1.f - scene.lane_line_probs[i], 255);
    }
    else{
       color = COLOR_WHITE_ALPHA(int(scene.lane_line_probs[i] * 180.));
    }
    ui_draw_line(s, scene.lane_line_vertices[i], &color, nullptr);
  }
  if (!scene.lateralPlan.lanelessModeStatus) {
    // paint road edges
    for (int i = 0; i < std::size(scene.road_edge_vertices); i++) {
      NVGcolor color = nvgRGBAf(1.0, 0.0, 0.0, std::clamp<float>(1.0 - scene.road_edge_stds[i], 0.0, 1.0));
      ui_draw_line(s, scene.road_edge_vertices[i], &color, nullptr);
    }
  }
  if (scene.controls_state.getLatActive()) {
    if (steerOverride) {
      track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h*.4,
        COLOR_BLACK_ALPHA(80), COLOR_BLACK_ALPHA(20));
    } 
    else if (!scene.lateralPlan.lanelessModeStatus) {
      if (scene.car_state.getLkaEnabled()){
        if (scene.color_path){
          track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h*.4,
            interp_alert_color(fabs(scene.lateralCorrection), 90), 
            interp_alert_color(fabs(scene.lateralCorrection), 0));
        }
        else{
          track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h*.4,
            interp_alert_color(0., 150), 
            interp_alert_color(0., 0));
        }
      }
      else{
        track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
                                          COLOR_WHITE_ALPHA(100), COLOR_WHITE_ALPHA(0));
      }
    } 
    else { // differentiate laneless mode color (Grace blue)
      if (scene.car_state.getLkaEnabled()){
        if (scene.color_path){
          int g, r = 255.f * float(COLOR_GRACE_BLUE.b) * fabs(scene.lateralCorrection);
          r = CLIP(r, 255.f * COLOR_GRACE_BLUE.r, 255.f * COLOR_GRACE_BLUE.b);
          g = 255.f * COLOR_GRACE_BLUE.g + r;
          g = CLIP(g, 255.f * COLOR_GRACE_BLUE.g, 255.f * COLOR_GRACE_BLUE.b);
          track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
                                      nvgRGBA(r, g, 255.f * COLOR_GRACE_BLUE.b, 100), 
                                      nvgRGBA(r, g, 255.f * COLOR_GRACE_BLUE.b, 0));
        }
        else{
          track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
                                    COLOR_GRACE_BLUE_ALPHA(100), 
                                    COLOR_GRACE_BLUE_ALPHA(0));
        }
      }
      else{
        track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
                                          COLOR_WHITE_ALPHA(80), COLOR_WHITE_ALPHA(0));
      }
    }
  } else {
    // Draw white vision track
    track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
                                          COLOR_WHITE_ALPHA(80), COLOR_WHITE_ALPHA(0));
  }
  // paint path
  auto track_bg_big = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
                                  COLOR_WHITE_ALPHA(70), COLOR_WHITE_ALPHA(40));
  ui_draw_line(s, scene.track_vertices, nullptr, &track_bg_big);

  if (scene.controls_state.getLatActive() && scene.car_state.getLkaEnabled()){
    ui_draw_line(s, scene.track_inside_vertices, nullptr, &track_bg);
  }

  // now oncoming/ongoing lanes

  auto tf = scene.lateral_plan.getTrafficLeft();
  if (tf == LaneTraffic::ONCOMING){
    track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
                                          nvgRGBA(255, 30, 30, 150), nvgRGBA(255, 30, 30, 0));
    ui_draw_line(s, scene.lane_vertices_left, nullptr, &track_bg);
  }
  else if (tf == LaneTraffic::ONGOING){
    track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
                                          interp_alert_color(-1., 150), interp_alert_color(-1., 0));
    ui_draw_line(s, scene.lane_vertices_left, nullptr, &track_bg);
  }
  else if (tf == LaneTraffic::STOPPED){
    track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
                                          COLOR_WHITE_ALPHA(100), COLOR_WHITE_ALPHA(0));
    ui_draw_line(s, scene.lane_vertices_left, nullptr, &track_bg);
  }

  tf = scene.lateral_plan.getTrafficRight();
  if (tf == LaneTraffic::ONCOMING){
    track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
                                          nvgRGBA(255, 30, 30, 150), nvgRGBA(255, 30, 30, 0));
    ui_draw_line(s, scene.lane_vertices_right, nullptr, &track_bg);
  }
  else if (tf == LaneTraffic::ONGOING){
    track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
                                          interp_alert_color(-1., 150), interp_alert_color(-1., 0));
    ui_draw_line(s, scene.lane_vertices_right, nullptr, &track_bg);
  }
  else if (tf == LaneTraffic::STOPPED){
    track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
                                          COLOR_WHITE_ALPHA(100), COLOR_WHITE_ALPHA(0));
    ui_draw_line(s, scene.lane_vertices_right, nullptr, &track_bg);
  }

  // print lane and shoulder widths and probabilities
  if (s->scene.show_debug_ui && !s->scene.map_open){
    auto l_probs = s->scene.lateral_plan.getLaneProbs();
    auto road_edge_probs = s->scene.lateral_plan.getRoadEdgeProbs();
    if (l_probs.size() == 4 && road_edge_probs.size() == 2){
      const int width_font_size = 25;
      char cstr[32];
      int y = s->fb_h - 18;
      nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);

      // current lane
      snprintf(cstr, sizeof(cstr), "(%.1f) %.1f (%.1f)", l_probs[1], (!s->scene.is_metric ? 3.28084 : 1.0) * s->scene.lateral_plan.getLaneWidth(), l_probs[2]);
      ui_draw_text(s, s->fb_w / 2, y, cstr, width_font_size * 2.5, COLOR_WHITE, "sans-semibold");
      // left adjacent lane
      if (1||s->scene.lateral_plan.getLaneWidthMeanLeftAdjacent() > 0.){
        snprintf(cstr, sizeof(cstr), "(%.1f) %.1f", l_probs[0], (!s->scene.is_metric ? 3.28084 : 1.0) * s->scene.lateral_plan.getLaneWidthMeanLeftAdjacent());
        ui_draw_text(s, s->fb_w / 5, y, cstr, width_font_size * 2.5, COLOR_WHITE, "sans-semibold");
      }
      // right adjacent lane
      if (1||s->scene.lateral_plan.getLaneWidthMeanRightAdjacent() > 0.){
        snprintf(cstr, sizeof(cstr), "%.1f (%.1f)", (!s->scene.is_metric ? 3.28084 : 1.0) * s->scene.lateral_plan.getLaneWidthMeanRightAdjacent(), l_probs[3]);
        ui_draw_text(s, 4 * s->fb_w / 5, y, cstr, width_font_size * 2.5, COLOR_WHITE, "sans-semibold");
      }
      // left shoulder
      nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_BASELINE);
      if (1||s->scene.lateral_plan.getShoulderMeanWidthLeft() > 0.){
        snprintf(cstr, sizeof(cstr), "(%.1f) %.1f", road_edge_probs[0], (!s->scene.is_metric ? 3.28084 : 1.0) * s->scene.lateral_plan.getShoulderMeanWidthLeft());
        ui_draw_text(s, 50, y, cstr, width_font_size * 2.5, COLOR_RED, "sans-bold");
      }    
      // right shoulder
      nvgTextAlign(s->vg, NVG_ALIGN_RIGHT | NVG_ALIGN_BASELINE);
      if (1||s->scene.lateral_plan.getShoulderMeanWidthRight() > 0.){
        snprintf(cstr, sizeof(cstr), "%.1f (%.1f)", (!s->scene.is_metric ? 3.28084 : 1.0) * s->scene.lateral_plan.getShoulderMeanWidthRight(), road_edge_probs[1]);
        ui_draw_text(s, s->fb_w - 50, y, cstr, width_font_size * 2.5, COLOR_RED, "sans-bold");
      }
    }
  }
}

// Draw all world space objects.
static void ui_draw_world(UIState *s) {
  nvgScissor(s->vg, 0, 0, s->fb_w, s->fb_h);

  // Draw lane edges and vision/mpc tracks
  ui_draw_vision_lane_lines(s);

  // Draw lead indicators if openpilot is handling longitudinal
  bool lead_drawn = false;
  if (s->scene.longitudinal_control) {
    auto lead_one = (*s->sm)["modelV2"].getModelV2().getLeadsV3()[0];
    auto lead_two = (*s->sm)["modelV2"].getModelV2().getLeadsV3()[1];
    if (lead_one.getProb() > .5) {
      lead_drawn = true;
      draw_lead(s, lead_one.getX()[0], lead_one.getV()[0], s->scene.lead_vertices[0], true, false);
    }
    if (lead_two.getProb() > .5 && (std::abs(lead_one.getX()[0] - lead_two.getX()[0]) > 3.0)) {
      lead_drawn = true;
      draw_lead(s, lead_two.getX()[0], lead_two.getV()[0], s->scene.lead_vertices[1], lead_one.getProb() <= .5, false);
    }
    for (int i = 0; i < 2 && !lead_drawn; ++i){
      if (s->scene.lead_data[i].getStatus() && s->scene.lead_data[i].getDRel() > 60.){
        lead_drawn = true;
        draw_lead(s, s->scene.lead_data[i].getDRel(), s->scene.lead_data[i].getVRel(), s->scene.lead_vertices[i], true, true);
      }
    }
  }
  draw_other_leads(s, lead_drawn);
  draw_adjacent_lead_speeds(s, lead_drawn);
  nvgResetScissor(s->vg);
}

static void ui_draw_vision_maxspeed(UIState *s) {
  const int SET_SPEED_NA = 255;
  float maxspeed = s->scene.controls_state.getVCruise();
  const Rect rect = {bdr_s * 2, int(bdr_s * 1.5), 184, 202};
  auto const & bg_colors_ = (s->scene.alt_engage_color_enabled ? alt_bg_colors : bg_colors);
  if (s->scene.one_pedal_fade > 0.){
    const QColor &color = bg_colors_[s->scene.car_state.getMadsLeadBrakingActive() ? 3 : (s->scene.car_state.getOnePedalModeTemporary() ? 2 : (s->scene.car_state.getOnePedalModeActive() ? 1 : 0))];
    NVGcolor nvg_color = nvgRGBA(color.red(), color.green(), color.blue(), int(s->scene.one_pedal_fade * float(color.alpha())));
    const Rect pedal_rect = {rect.centerX() - brake_size, rect.centerY() - brake_size, brake_size * 2, brake_size * 2};
    ui_fill_rect(s->vg, pedal_rect, nvg_color, brake_size);
    if (s->scene.car_state.getOnePedalModeActive()){
      ui_draw_image(s, {rect.centerX() - brake_size, rect.centerY() - brake_size, brake_size * 2, brake_size * 2}, "one_pedal_mode", s->scene.one_pedal_fade);
    }
    else{
      ui_draw_image(s, {rect.centerX() - brake_size, rect.centerY() - brake_size, brake_size * 2, brake_size * 2}, "MADS", s->scene.one_pedal_fade);
    }
    s->scene.one_pedal_touch_rect = pedal_rect;
    s->scene.maxspeed_touch_rect = {1,1,1,1};
    
    // draw extra circle to indiate one-pedal lead braking is enabled
    if (!(s->scene.onePedalModeSimple)){
      nvgBeginPath(s->vg);
      const int r = int(float(brake_size) * 1.15);
      nvgRoundedRect(s->vg, rect.centerX() - r, rect.centerY() - r, 2 * r, 2 * r, r);
      nvgStrokeColor(s->vg, COLOR_WHITE_ALPHA(int(s->scene.one_pedal_fade * 255.)));
      nvgFillColor(s->vg, nvgRGBA(0,0,0,0));
      nvgFill(s->vg);
      nvgStrokeWidth(s->vg, 7);
      nvgStroke(s->vg);
    }
  }
  else{
    s->scene.one_pedal_touch_rect = {1,1,1,1};
    s->scene.maxspeed_touch_rect = rect;
    const bool is_cruise_set = s->scene.controls_state.getActive() && maxspeed != 0 && maxspeed != SET_SPEED_NA;
    if (is_cruise_set && !s->scene.is_metric) { maxspeed *= 0.6225; }

    ui_fill_rect(s->vg, rect, COLOR_BLACK_ALPHA(int(-s->scene.one_pedal_fade * 100.)), 30.);
    ui_draw_rect(s->vg, rect, COLOR_WHITE_ALPHA(int(-s->scene.one_pedal_fade * 100.)), 6, 20.);

    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
    NVGcolor max_color;
    if (is_cruise_set){
      if (s->scene.alt_engage_color_enabled){
        max_color = nvgRGBA(0x00, 0x9F, 0xFF, int(-s->scene.one_pedal_fade * 255.));
      }
      else{
        max_color = nvgRGBA(0x80, 0xd8, 0xa6, int(-s->scene.one_pedal_fade * 255.));
      }
    }
    else{
      max_color = nvgRGBA(0xa6, 0xa6, 0xa6, int(-s->scene.one_pedal_fade * 255.));
    }
    ui_draw_text(s, rect.centerX(), 118, "MAX", 26 * 2.5, max_color, is_cruise_set ? "sans-semibold" : "sans-regular");
    if (is_cruise_set) {
      std::string maxspeed_str = std::to_string((int)std::nearbyint(maxspeed));
      float font_size = 48 * 2.5;
      if (s->scene.car_state.getCoastingActive()){
        maxspeed_str += "+";
        font_size *= 0.9;
      }
      ui_draw_text(s, rect.centerX(), 212, maxspeed_str.c_str(), font_size, COLOR_WHITE_ALPHA(is_cruise_set ? int(-s->scene.one_pedal_fade * 200.) : int(-s->scene.one_pedal_fade * 100.)), "sans-bold");
    } else {
      ui_draw_text(s, rect.centerX(), 212, "N/A", 42 * 2.5, COLOR_WHITE_ALPHA(int(-s->scene.one_pedal_fade * 100.)), "sans-semibold");
    }
  }
}

static void ui_draw_vision_speedlimit(UIState *s) {
  const float speedLimit = s->scene.longitudinal_plan.getSpeedLimit();
  const float speedLimitOffset = s->scene.longitudinal_plan.getSpeedLimitOffset();

  if (speedLimit > 0.0 && s->scene.engageable) {
    const Rect maxspeed_rect = {bdr_s * 2, int(bdr_s * 1.5), 184, 202};
    Rect speed_sign_rect = Rect{maxspeed_rect.centerX() - speed_sgn_r, 
      maxspeed_rect.bottom() + bdr_s, 
      2 * speed_sgn_r, 
      2 * speed_sgn_r};
    const float speed = speedLimit * (s->scene.is_metric ? 3.6 : 2.2369362921);
    const float speed_offset = speedLimitOffset * (s->scene.is_metric ? 3.6 : 2.2369362921);

    auto speedLimitControlState = s->scene.longitudinal_plan.getSpeedLimitControlState();
    const bool force_active = s->scene.speed_limit_control_enabled && 
                              seconds_since_boot() < s->scene.last_speed_limit_sign_tap + 2.0;
    const bool inactive = !force_active && (!s->scene.speed_limit_control_enabled || 
                          speedLimitControlState == cereal::LongitudinalPlan::SpeedLimitControlState::INACTIVE);
    const bool temp_inactive = !force_active && (s->scene.speed_limit_control_enabled && 
                               speedLimitControlState == cereal::LongitudinalPlan::SpeedLimitControlState::TEMP_INACTIVE);

    const int distToSpeedLimit = int(s->scene.longitudinal_plan.getDistToSpeedLimit() * 
                                     (s->scene.is_metric ? 1.0 : 3.28084) / 10) * 10;
    const bool is_map_sourced = s->scene.longitudinal_plan.getIsMapSpeedLimit();
    const std::string distance_str = std::to_string(distToSpeedLimit) + (s->scene.is_metric ? "m" : "f");
    const std::string offset_str = speed_offset > 0.0 ? "+" + std::to_string((int)std::nearbyint(speed_offset)) : "";
    const std::string inactive_str = temp_inactive ? "TEMP" : "";
    const std::string substring = inactive || temp_inactive ? inactive_str : 
                                                              distToSpeedLimit > 0 ? distance_str : offset_str;
    const float substring_size = inactive || temp_inactive || distToSpeedLimit > 0 ? 30.0 : 50.0;

    ui_draw_speed_sign(s, speed_sign_rect.centerX(), speed_sign_rect.centerY(), speed_sgn_r, speed, substring.c_str(), 
                       substring_size, "sans-bold", is_map_sourced, !inactive && !temp_inactive);

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
    SubMaster &sm = *(s->sm);
    UIScene &scene = s->scene;
    const int measure_cur_num_slots = scene.measure_cur_num_slots;
    if (scene.map_open){
      scene.measure_cur_num_slots = MIN(scene.measure_num_rows, 4);
    }
    const Rect maxspeed_rect = {bdr_s * 2, int(bdr_s * 1.5), 184, 202};
    int center_x = s->fb_w - face_wheel_radius - bdr_s * 2;
    center_x -= s->scene.power_meter_rect.w + s->fb_w / 256;
    const int brake_y = s->fb_h - footer_h / 2;
    const int y_min = maxspeed_rect.bottom() + bdr_s / 2;
    const int y_max = brake_y - brake_size - bdr_s / 2;
    const int y_rng = y_max - y_min;
    int slot_y_rng;
    if (scene.measure_num_rows > 4 || scene.map_open){
      slot_y_rng = y_rng / scene.measure_max_rows;
    }
    else{
      slot_y_rng = y_rng / (scene.measure_num_rows < 3 ? 3 : scene.measure_num_rows);
    }
    const int slot_y_rng_orig = y_rng / scene.measure_max_rows; // two columns
    const float slot_aspect_ratio_ratio = float(slot_y_rng) / float(slot_y_rng_orig);
    const int y_mid = (y_max + y_min) / 2;
    const int slots_y_rng = slot_y_rng * (scene.measure_num_rows <= scene.measure_max_rows ? scene.measure_num_rows : scene.measure_max_rows);
    const int slots_y_min = y_mid - (slots_y_rng / 2);
  
    NVGcolor default_name_color = COLOR_WHITE_ALPHA(200);
    NVGcolor default_unit_color = COLOR_WHITE_ALPHA(200);
    NVGcolor default_val_color = COLOR_WHITE_ALPHA(200);
    int default_val_font_size = 78. * slot_aspect_ratio_ratio;
    int default_name_font_size = 32. * (slot_y_rng_orig > 1. ? 0.9 * slot_aspect_ratio_ratio : 1.);
    int default_unit_font_size = 38. * slot_aspect_ratio_ratio;
  
    // determine bounding rectangle
    int slots_r, slots_w, slots_x;

    const int slots_r_orig = brake_size + 6 + (s->scene.measure_cur_num_slots <= 5 ? 6 : 0);
    slots_r = brake_size * slot_aspect_ratio_ratio + 6 + (scene.measure_cur_num_slots <= scene.measure_max_rows ? 6 : 0);
    center_x -= slots_r - slots_r_orig;
    slots_w = (scene.measure_cur_num_slots <= scene.measure_max_rows ? 2 : 4) * slots_r;
    slots_x = (scene.measure_cur_num_slots <= scene.measure_max_rows ? center_x - slots_r : center_x - 3 * slots_r);

    scene.measure_slots_rect = {slots_x, slots_y_min, slots_w, slots_y_rng};
    // draw bounding rectangle
    nvgBeginPath(s->vg);
    nvgRoundedRect(s->vg, scene.measure_slots_rect.x, scene.measure_slots_rect.y, scene.measure_slots_rect.w, scene.measure_slots_rect.h, 20);
    if (QUIState::ui_state.scene.lastTime - QUIState::ui_state.scene.measures_last_tap_t > QUIState::ui_state.scene.measures_touch_timeout){
      nvgStrokeColor(s->vg, COLOR_WHITE_ALPHA(160));
    }
    else{
      nvgStrokeColor(s->vg, COLOR_GRACE_BLUE_ALPHA(200));
    }
    nvgStrokeWidth(s->vg, 6);
    nvgStroke(s->vg);
    nvgFillColor(s->vg, COLOR_BLACK_ALPHA(100));
    nvgFill(s->vg);

    char const * deg = Hardware::EON() ? "°" : "°";
    
    // now start from the top and draw the current set of metrics
    for (int ii = 0; ii < scene.measure_cur_num_slots; ++ii){
      try{
        int i = ii;
        if (scene.measure_cur_num_slots > scene.measure_max_rows && i >= scene.measure_num_rows){
          i += scene.measure_row_offset;
        }

        char name[32], val[24], unit[16];
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

          case UIMeasure::CPU_TEMP_AND_PERCENTF: 
            {
            auto cpus = scene.deviceState.getCpuUsagePercent();
            float cpu = 0.;
            int num_cpu = 0;
            for (auto c : cpus){
              cpu += c;
              num_cpu++;
            }
            if (num_cpu > 1){
              cpu /= num_cpu;
            }
            val_color = color_from_thermal_status(int(scene.deviceState.getThermalStatus()));
            snprintf(val, sizeof(val), "%.0f%sF", scene.deviceState.getCpuTempC()[0] * 1.8 + 32., deg);
            snprintf(unit, sizeof(unit), "%d%%", int(cpu));
            snprintf(name, sizeof(name), "CPU");}
            break;
          
          case UIMeasure::CPU_TEMPF: 
            {
            val_color = color_from_thermal_status(int(scene.deviceState.getThermalStatus()));
            snprintf(val, sizeof(val), "%.0f", scene.deviceState.getCpuTempC()[0] * 1.8 + 32.);
            snprintf(unit, sizeof(unit), "%sF", deg);
            snprintf(name, sizeof(name), "CPU TEMP");}
            break;
          
          case UIMeasure::MEMORY_TEMPF: 
            {
            val_color = color_from_thermal_status(int(scene.deviceState.getThermalStatus()));
            snprintf(val, sizeof(val), "%.0f", scene.deviceState.getMemoryTempC() * 1.8 + 32.);
            snprintf(unit, sizeof(unit), "%sF", deg);
            snprintf(name, sizeof(name), "MEM TEMP");}
            break;
          
          case UIMeasure::AMBIENT_TEMPF: 
            {
            val_color = color_from_thermal_status(int(scene.deviceState.getThermalStatus()));
            snprintf(val, sizeof(val), "%.0f", scene.deviceState.getAmbientTempC() * 1.8 + 32.);
            snprintf(unit, sizeof(unit), "%sF", deg);
            snprintf(name, sizeof(name), "AMB TEMP");}
            break;
          
          case UIMeasure::INTERACTION_TIMER: 
            {
            int s = scene.controls_state.getInteractionTimer();
            if (s < 5){
              val_color = nvgRGBA(255, 125, 100, 200);
            }
            s_to_time_str(val, s);
            snprintf(name, sizeof(name), "INTERACT");
            }
            break;

          case UIMeasure::INTERVENTION_TIMER: 
            {
            int s = scene.controls_state.getInterventionTimer();
            if (s < 5){
              val_color = nvgRGBA(255, 125, 100, 200);
            }
            s_to_time_str(val, s);
            snprintf(name, sizeof(name), "INTERVENE");
            }
            break;
          
          case UIMeasure::DISTRACTION_TIMER: 
            {
            int s = scene.controls_state.getDistractionTimer();
            if (s < 5){
              val_color = nvgRGBA(255, 125, 100, 200);
            }
            s_to_time_str(val, s);
            snprintf(name, sizeof(name), "DISTRACT");
            }
            break;

          case UIMeasure::TIME_CAR_RUNNING_SESSION: 
            {
            int s = scene.controls_state.getCarRunningTimerSession();
            if (s < 5){
              val_color = nvgRGBA(255, 125, 100, 200);
            }
            s_to_time_str(val, s);
            snprintf(name, sizeof(name), "DRIVE TIME");
            }
            break;

          case UIMeasure::TIME_CAR_RUNNING_TOTAL: 
            {
            int s = scene.controls_state.getCarRunningTimerTotal();
            if (s < 5){
              val_color = nvgRGBA(255, 125, 100, 200);
            }
            s_to_time_str(val, s);
            snprintf(name, sizeof(name), "TRIP TIME");
            }
            break;

          case UIMeasure::TIME_OPENPILOT_ENGAGED_SESSION: 
            {
            int s = scene.controls_state.getOpenpilotLongControlTimerSession();
            if (s < 5){
              val_color = nvgRGBA(255, 125, 100, 200);
            }
            s_to_time_str(val, s);
            snprintf(name, sizeof(name), "DRIVE ENGD.");
            }
            break;

          case UIMeasure::TIME_OPENPILOT_ENGAGED_TOTAL: 
            {
            int s = scene.controls_state.getOpenpilotLongControlTimerTotal();
            if (s < 5){
              val_color = nvgRGBA(255, 125, 100, 200);
            }
            s_to_time_str(val, s);
            snprintf(name, sizeof(name), "TRIP ENGD.");
            }
            break;

          case UIMeasure::TIME_OPENPILOT_ENGAGED: 
            {
            int s = scene.controls_state.getDisengagementTimer();
            if (s < 5){
              val_color = nvgRGBA(255, 125, 100, 200);
            }
            s_to_time_str(val, s);
            snprintf(name, sizeof(name), "ENGAGED");
            }
            break;

          case UIMeasure::TIME_ENGAGED_PERCENT_SESSION: 
            {
            float p = scene.controls_state.getPercentEngagedTimeSession();
            snprintf(val, sizeof(val), "%0.1f%%", p);
            snprintf(name, sizeof(name), "DRIVE ENGD.");
            snprintf(unit, sizeof(unit), "time");
            }
            break;

          case UIMeasure::TIME_ENGAGED_PERCENT_TOTAL: 
            {
            float p = scene.controls_state.getPercentEngagedTimeTotal();
            snprintf(val, sizeof(val), "%0.1f%%", p);
            snprintf(name, sizeof(name), "TRIP ENGD.");
            snprintf(unit, sizeof(unit), "time");
            }
            break;

          case UIMeasure::DISENGAGEMENT_COUNT_SESSION: 
            {
            int c = scene.controls_state.getDisengagementCountSession();
            snprintf(val, sizeof(val), "%d", c);
            snprintf(name, sizeof(name), "#DISENGAGE");
            snprintf(unit, sizeof(unit), "drive");
            }
            break;

          case UIMeasure::DISENGAGEMENT_COUNT_TOTAL: 
            {
            int c = scene.controls_state.getDisengagementCountTotal();
            snprintf(val, sizeof(val), "%d", c);
            snprintf(name, sizeof(name), "#DISENGAGE");
            snprintf(unit, sizeof(unit), "trip");
            }
            break;

          case UIMeasure::INTERACTION_COUNT_SESSION: 
            {
            int c = scene.controls_state.getInteractionCountSession();
            snprintf(val, sizeof(val), "%d", c);
            snprintf(name, sizeof(name), "#INTERACT");
            snprintf(unit, sizeof(unit), "drive");
            }
            break;

          case UIMeasure::INTERACTION_COUNT_TOTAL: 
            {
            int c = scene.controls_state.getInteractionCountTotal();
            snprintf(val, sizeof(val), "%d", c);
            snprintf(name, sizeof(name), "#INTERACT");
            snprintf(unit, sizeof(unit), "trip");
            }
            break;

          case UIMeasure::INTERVENTION_COUNT_SESSION: 
            {
            int c = scene.controls_state.getInterventionCountSession();
            snprintf(val, sizeof(val), "%d", c);
            snprintf(name, sizeof(name), "#INTERVENE");
            snprintf(unit, sizeof(unit), "drive");
            }
            break;

          case UIMeasure::INTERVENTION_COUNT_TOTAL: 
            {
            int c = scene.controls_state.getInterventionCountTotal();
            snprintf(val, sizeof(val), "%d", c);
            snprintf(name, sizeof(name), "#INTERVENE");
            snprintf(unit, sizeof(unit), "trip");
            }
            break;

          case UIMeasure::DISTRACTION_COUNT_SESSION: 
            {
            int c = scene.controls_state.getDistractionCountSession();
            snprintf(val, sizeof(val), "%d", c);
            snprintf(name, sizeof(name), "#DISTRACT");
            snprintf(unit, sizeof(unit), "drive");
            }
            break;

          case UIMeasure::DISTRACTION_COUNT_TOTAL: 
            {
            int c = scene.controls_state.getDistractionCountTotal();
            snprintf(val, sizeof(val), "%d", c);
            snprintf(name, sizeof(name), "#DISTRACT");
            snprintf(unit, sizeof(unit), "trip");
            }
            break;

          case UIMeasure::TIME_PER_DISENGAGEMENT_SESSION: 
            {
            int s = float(scene.controls_state.getOpenpilotLongControlTimerSession()) / MAX(1.0, float(scene.controls_state.getDisengagementCountSession()));
            s_to_time_str(val, s);
            snprintf(name, sizeof(name), "PER DISENGAGE");
            snprintf(unit, sizeof(unit), "drive time");
            }
            break;

          case UIMeasure::TIME_PER_DISENGAGEMENT_TOTAL: 
            {
            int s = float(scene.controls_state.getOpenpilotLongControlTimerTotal()) / MAX(1.0, scene.controls_state.getDisengagementCountTotal());
            s_to_time_str(val, s);
            snprintf(name, sizeof(name), "PER DISENGAGE");
            snprintf(unit, sizeof(unit), "trip time");
            }
            break;
            
          case UIMeasure::TIME_PER_INTERACTION_SESSION: 
            {
            int s = float(scene.controls_state.getOpenpilotLongControlTimerSession()) / MAX(1.0, scene.controls_state.getInteractionCountSession());
            s_to_time_str(val, s);
            snprintf(name, sizeof(name), "PER INTERACT");
            snprintf(unit, sizeof(unit), "drive time");
            }
            break;

          case UIMeasure::TIME_PER_INTERACTION_TOTAL: 
            {
            int s = float(scene.controls_state.getOpenpilotLongControlTimerTotal()) / MAX(1.0, scene.controls_state.getInteractionCountTotal());
            s_to_time_str(val, s);
            snprintf(name, sizeof(name), "PER INTERACT");
            snprintf(unit, sizeof(unit), "trip time");
            }
            break;

          case UIMeasure::TIME_PER_INTERVENTION_SESSION: 
            {
            int s = float(scene.controls_state.getOpenpilotLongControlTimerSession()) / MAX(1.0, scene.controls_state.getInterventionCountSession());
            s_to_time_str(val, s);
            snprintf(name, sizeof(name), "PER INTERVENE");
            snprintf(unit, sizeof(unit), "drive time");
            }
            break;

          case UIMeasure::TIME_PER_INTERVENTION_TOTAL: 
            {
            int s = float(scene.controls_state.getOpenpilotLongControlTimerTotal()) / MAX(1.0, scene.controls_state.getInterventionCountTotal());
            s_to_time_str(val, s);
            snprintf(name, sizeof(name), "PER INTERVENE");
            snprintf(unit, sizeof(unit), "trip time");
            }
            break;

          case UIMeasure::TIME_PER_DISTRACTION_SESSION: 
            {
            int s = float(scene.controls_state.getOpenpilotLongControlTimerSession()) / MAX(1.0, scene.controls_state.getInterventionCountSession());
            s_to_time_str(val, s);
            snprintf(name, sizeof(name), "PER DISTRACT");
            snprintf(unit, sizeof(unit), "drive time");
            }
            break;

          case UIMeasure::TIME_PER_DISTRACTION_TOTAL: 
            {
            int s = float(scene.controls_state.getOpenpilotLongControlTimerTotal()) / MAX(1.0, scene.controls_state.getInterventionCountTotal());
            s_to_time_str(val, s);
            snprintf(name, sizeof(name), "PER DISTRACT");
            snprintf(unit, sizeof(unit), "trip time");
            }
            break;

          case UIMeasure::CPU_TEMP_AND_PERCENTC: 
            {
            auto cpus = scene.deviceState.getCpuUsagePercent();
            float cpu = 0.;
            int num_cpu = 0;
            for (auto c : cpus){
              cpu += c;
              num_cpu++;
            }
            if (num_cpu > 1){
              cpu /= num_cpu;
            }
            val_color = color_from_thermal_status(int(scene.deviceState.getThermalStatus()));
              snprintf(val, sizeof(val), "%.0f%sC", scene.deviceState.getCpuTempC()[0], deg);
            snprintf(unit, sizeof(unit), "%d%%", int(cpu));
            snprintf(name, sizeof(name), "CPU");}
            break;
          
          case UIMeasure::CPU_TEMPC: 
            {
            val_color = color_from_thermal_status(int(scene.deviceState.getThermalStatus()));
            snprintf(val, sizeof(val), "%.0f", scene.deviceState.getCpuTempC()[0]);
            snprintf(unit, sizeof(unit), "%sC", deg);
            snprintf(name, sizeof(name), "CPU TEMP");}
            break;
          
          case UIMeasure::MEMORY_TEMPC: 
            {
            val_color = color_from_thermal_status(int(scene.deviceState.getThermalStatus()));
            snprintf(val, sizeof(val), "%.0f", scene.deviceState.getMemoryTempC());
            snprintf(unit, sizeof(unit), "%sC", deg);
            snprintf(name, sizeof(name), "MEM TEMP");}
            break;
          
          case UIMeasure::AMBIENT_TEMPC: 
            {
            val_color = color_from_thermal_status(int(scene.deviceState.getThermalStatus()));
            snprintf(val, sizeof(val), "%.0f", scene.deviceState.getAmbientTempC());
            snprintf(unit, sizeof(unit), "%sC", deg);
            snprintf(name, sizeof(name), "AMB TEMP");}
            break;
          
          case UIMeasure::CPU_PERCENT: 
            {
            auto cpus = scene.deviceState.getCpuUsagePercent();
            float cpu = 0.;
            int num_cpu = 0;
            for (auto c : cpus){
              cpu += c;
              num_cpu++;
            }
            if (num_cpu > 1){
              cpu /= num_cpu;
            }
            val_color = color_from_thermal_status(int(scene.deviceState.getThermalStatus()));
            snprintf(val, sizeof(val), "%d%%", int(cpu));
            snprintf(name, sizeof(name), "CPU PERC");}
            break;
            
          case UIMeasure::FANSPEED_PERCENT: 
            {
            val_color = color_from_thermal_status(int(scene.deviceState.getThermalStatus()));
            int fs = scene.deviceState.getFanSpeedPercentDesired();
            if (fs > 100){
              fs = scene.fanspeed_rpm;
              snprintf(unit, sizeof(unit), "RPM");
              snprintf(val, sizeof(val), "%d", fs);
            }
            else{
              snprintf(val, sizeof(val), "%d%%", fs);
            }
            snprintf(name, sizeof(name), "FAN");
            }
            break;
          case UIMeasure::FANSPEED_RPM: 
            {
            val_color = color_from_thermal_status(int(scene.deviceState.getThermalStatus()));
            snprintf(val, sizeof(val), "%d", scene.fanspeed_rpm);
            snprintf(name, sizeof(name), "FAN");
            snprintf(unit, sizeof(unit), "RPM");}
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
            if (sm.updated("ubloxGnss")) {
              auto data = sm["ubloxGnss"].getUbloxGnss();
              if (data.which() == cereal::UbloxGnss::MEASUREMENT_REPORT) {
                scene.satelliteCount = data.getMeasurementReport().getNumMeas();
              }
              auto data2 = sm["gpsLocationExternal"].getGpsLocationExternal();
              scene.gpsAccuracyUblox = data2.getAccuracy();
            }
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
            if (sm.updated("gpsLocationExternal")) {
              auto data2 = sm["gpsLocationExternal"].getGpsLocationExternal();
              scene.altitudeUblox = data2.getAltitude();
              scene.gpsAccuracyUblox = data2.getAccuracy();
            }
            snprintf(name, sizeof(name), "ELEVATION");
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

          case UIMeasure::BEARING:
            {
              snprintf(name, sizeof(name), "BEARING");
              if (scene.bearingAccuracy != 180.00) {
                snprintf(unit, sizeof(unit), "%.0d%s", (int)scene.bearingDeg, "°");
                if (((scene.bearingDeg >= 337.5) && (scene.bearingDeg <= 360)) || ((scene.bearingDeg >= 0) && (scene.bearingDeg <= 22.5))) {
                  snprintf(val, sizeof(val), "N");
                } else if ((scene.bearingDeg > 22.5) && (scene.bearingDeg < 67.5)) {
                  snprintf(val, sizeof(val), "NE");
                } else if ((scene.bearingDeg >= 67.5) && (scene.bearingDeg <= 112.5)) {
                  snprintf(val, sizeof(val), "E");
                } else if ((scene.bearingDeg > 112.5) && (scene.bearingDeg < 157.5)) {
                  snprintf(val, sizeof(val), "SE");
                } else if ((scene.bearingDeg >= 157.5) && (scene.bearingDeg <= 202.5)) {
                  snprintf(val, sizeof(val), "S");
                } else if ((scene.bearingDeg > 202.5) && (scene.bearingDeg < 247.5)) {
                  snprintf(val, sizeof(val), "SW");
                } else if ((scene.bearingDeg >= 247.5) && (scene.bearingDeg <= 292.5)) {
                  snprintf(val, sizeof(val), "W");
                } else if ((scene.bearingDeg > 292.5) && (scene.bearingDeg < 337.5)) {
                  snprintf(val, sizeof(val), "NW");
                }
              } else {
                snprintf(val, sizeof(val), "OFF");
                snprintf(unit, sizeof(unit), "-");
              }
            }
            break;

          case UIMeasure::STEERING_TORQUE_EPS:
            {
            snprintf(name, sizeof(name), "EPS TRQ");
            //TODO: Add orange/red color depending on torque intensity. <1x limit = white, btwn 1x-2x limit = orange, >2x limit = red
            snprintf(val, sizeof(val), "%.1f", scene.car_state.getSteeringTorqueEps());
            snprintf(unit, sizeof(unit), "Nm");
            break;}

          case UIMeasure::ACCELERATION:
            {
            snprintf(name, sizeof(name), "ACCEL");
            snprintf(val, sizeof(val), "%.1f", scene.car_state.getAEgo());
            snprintf(unit, sizeof(unit), "m/s²");
            break;}

          case UIMeasure::TIME_TO_STOP_CUR:
            {
            float a = scene.car_state.getAEgo();
            float t = a < -0.5 ? -scene.car_state.getVEgo() / a : 0.0;
            if (a > 0.0){
              snprintf(val, sizeof(val), "%.1f", t);
            }
            else{
              snprintf(val, sizeof(val), "---");
            }
            snprintf(name, sizeof(name), "STOP TIME");
            snprintf(unit, sizeof(unit), "s");
            break;}

          case UIMeasure::TIME_TO_STOP_MIN:
            {
            float t = scene.car_state.getVEgo() / 7.0;
            snprintf(name, sizeof(name), "MIN STOP TIME");
            snprintf(val, sizeof(val), "%.1f", t);
            snprintf(unit, sizeof(unit), "s");
            break;}

          case UIMeasure::DIST_TO_STOP_CUR:
            {
            float a = scene.car_state.getAEgo();
            float v = scene.car_state.getVEgo();
            float d = a < -0.5 ? -(v*v) / (2*a) : 0.0;
            snprintf(name, sizeof(name), "STOP DIST");
            if (!scene.is_metric){
              d *= 3.28;
              snprintf(unit, sizeof(unit), "ft");
            }
            else{
              snprintf(unit, sizeof(unit), "m");
            }
            if (d > 0.0){
              snprintf(val, sizeof(val), "%.0f", d);
            }
            else{
              snprintf(val, sizeof(val), "---");
            }
            break;}
          
          case UIMeasure::DIST_TO_STOP_MIN:
            {
            float a = 7.0;
            float v = scene.car_state.getVEgo();
            float d = v*v / (2*a);
            snprintf(name, sizeof(name), "MIN STOP DIST");
            if (!scene.is_metric){
              d *= 3.28;
              snprintf(val, sizeof(val), "%.1f", d);
              snprintf(unit, sizeof(unit), "ft");
            }
            else{
              snprintf(val, sizeof(val), "%.1f", d);
              snprintf(unit, sizeof(unit), "m");
            }
            break;}
          
          case UIMeasure::LAT_ACCEL:
            {
            snprintf(name, sizeof(name), "LAT ACC");
            snprintf(val, sizeof(val), "%.1f", sm["liveLocationKalman"].getLiveLocationKalman().getAccelerationCalibrated().getValue()[1]);
            snprintf(unit, sizeof(unit), "m/s²");
            break;}

          case UIMeasure::KINETIC_ENERGY:
            {
            snprintf(name, sizeof(name), "KIN EN");
            float v = scene.car_state.getVEgo();
            float T = v*v * scene.mass * 1e-3;
            if (T > 1e3){
              snprintf(val, sizeof(val), "%.2f", T * 1e-3);
              snprintf(unit, sizeof(unit), "MJ");
            }
            else{
              snprintf(val, sizeof(val), "%.0f", T);
              snprintf(unit, sizeof(unit), "KJ");
            }
            break;}

          case UIMeasure::DRAG_FORCE:
            {
            snprintf(name, sizeof(name), "DRAG FRC");
            float v = scene.car_state.getDragForce();
            v /= 1e3;
            if (fabs(v) > 100.){
              snprintf(val, sizeof(val), "%.0f", v);
            }
            else if (fabs(v) > 10.){
              snprintf(val, sizeof(val), "%.1f", v);
            }
            else{
              snprintf(val, sizeof(val), "%.2f", v);
            }
            snprintf(unit, sizeof(unit), "kN");
            break;}

          case UIMeasure::DRAG_POWER:
            {
            snprintf(name, sizeof(name), "DRAG POW");
            float v = scene.car_state.getDragPower();
            v /= 1e3;
            if (fabs(v) > 100.){
              snprintf(val, sizeof(val), "%.0f", v);
            }
            else if (fabs(v) > 10.){
              snprintf(val, sizeof(val), "%.1f", v);
            }
            else{
              snprintf(val, sizeof(val), "%.2f", v);
            }
            snprintf(unit, sizeof(unit), "kW");
            break;}

          case UIMeasure::DRAG_POWER_HP:
            {
            snprintf(name, sizeof(name), "DRAG POW");
            float v = scene.car_state.getDragPower();
            v /= 1e3;
            v *= 1.34;
            if (fabs(v) > 100.){
              snprintf(val, sizeof(val), "%.0f", v);
            }
            else if (fabs(v) > 10.){
              snprintf(val, sizeof(val), "%.1f", v);
            }
            else{
              snprintf(val, sizeof(val), "%.2f", v);
            }
            snprintf(unit, sizeof(unit), "hp");
            break;}

          case UIMeasure::DRAG_LOSSES:
            {
            snprintf(name, sizeof(name), "DRAG LOSS");
            if (scene.car_state.getDrivePower() != 0.){
              float v = scene.car_state.getDragPower() / scene.car_state.getDrivePower() * 100.;
              if (v >= 0. && v <= 100.){
                snprintf(val, sizeof(val), "%.0f%%", v);
              }
              else{
                float v = scene.car_state.getDragPower();
                v /= 1e3;
                if (fabs(v) > 100.){
                  snprintf(val, sizeof(val), "%.0f", v);
                }
                else if (fabs(v) > 10.){
                  snprintf(val, sizeof(val), "%.1f", v);
                }
                else{
                  snprintf(val, sizeof(val), "%.2f", v);
                }
                snprintf(unit, sizeof(unit), "kW");
              }
            }
            else{
              snprintf(val, sizeof(val), "--");
            }
            break;}
          
          case UIMeasure::ACCEL_FORCE:
            {
            snprintf(name, sizeof(name), "ACCEL FRC");
            float v = scene.car_state.getAccelForce();
            v /= 1e3;
            if (fabs(v) > 10.){
              snprintf(val, sizeof(val), "%.0f", v);
            }
            else {
              snprintf(val, sizeof(val), "%.1f", v);
            }
            snprintf(unit, sizeof(unit), "kN");
            break;}

          case UIMeasure::EV_FORCE:
            {
            snprintf(name, sizeof(name), "EV FRC");
            float v = scene.car_state.getEvForce();
            v /= 1e3;
            if (fabs(v) > 10.){
              snprintf(val, sizeof(val), "%.0f", v);
            }
            else {
              snprintf(val, sizeof(val), "%.1f", v);
            }
            snprintf(unit, sizeof(unit), "kN");
            break;}

          case UIMeasure::REGEN_FORCE:
            {
            snprintf(name, sizeof(name), "REGEN FRC");
            float v = scene.car_state.getRegenForce();
            v /= 1e3;
            if (fabs(v) > 10.){
              snprintf(val, sizeof(val), "%.0f", v);
            }
            else {
              snprintf(val, sizeof(val), "%.1f", v);
            }
            snprintf(unit, sizeof(unit), "kN");
            break;}

          case UIMeasure::BRAKE_FORCE:
            {
            snprintf(name, sizeof(name), "BRAKE FRC");
            float v = scene.car_state.getBrakeForce();
            v /= 1e3;
            if (fabs(v) > 10.){
              snprintf(val, sizeof(val), "%.0f", v);
            }
            else {
              snprintf(val, sizeof(val), "%.1f", v);
            }
            snprintf(unit, sizeof(unit), "kN");
            break;}

          case UIMeasure::ACCEL_POWER:
            {
            snprintf(name, sizeof(name), "ACCEL POW");
            float v = scene.car_state.getAccelPower();
            v /= 1e3;
            if (fabs(v) > 10.){
              snprintf(val, sizeof(val), "%.0f", v);
            }
            else {
              snprintf(val, sizeof(val), "%.1f", v);
            }
            snprintf(unit, sizeof(unit), "kW");
            break;}

          case UIMeasure::EV_POWER:
            {
            snprintf(name, sizeof(name), "EV POW");
            float v = scene.car_state.getEvPower();
            v /= 1e3;
            if (fabs(v) > 10.){
              snprintf(val, sizeof(val), "%.0f", v);
            }
            else {
              snprintf(val, sizeof(val), "%.1f", v);
            }
            snprintf(unit, sizeof(unit), "kW");
            break;}

          case UIMeasure::REGEN_POWER:
            {
            snprintf(name, sizeof(name), "REGEN POW");
            float v = scene.car_state.getRegenPower();
            v /= 1e3;
            if (fabs(v) > 10.){
              snprintf(val, sizeof(val), "%.0f", v);
            }
            else {
              snprintf(val, sizeof(val), "%.1f", v);
            }
            snprintf(unit, sizeof(unit), "kW");
            break;}

          case UIMeasure::BRAKE_POWER:
            {
            snprintf(name, sizeof(name), "BRAKE POW");
            float v = scene.car_state.getBrakePower();
            v /= 1e3;
            if (fabs(v) > 10.){
              snprintf(val, sizeof(val), "%.0f", v);
            }
            else {
              snprintf(val, sizeof(val), "%.1f", v);
            }
            snprintf(unit, sizeof(unit), "kW");
            break;}

          case UIMeasure::DRIVE_POWER:
            {
            snprintf(name, sizeof(name), "DRIVE POW");
            float v = scene.car_state.getDrivePower();
            v /= 1e3;
            if (fabs(v) > 10.){
              snprintf(val, sizeof(val), "%.0f", v);
            }
            else{
              snprintf(val, sizeof(val), "%.1f", v);
            }
            
            snprintf(unit, sizeof(unit), "kW");
            break;}

          case UIMeasure::ICE_POWER:
            {
            snprintf(name, sizeof(name), "ICE POW");
            float v = scene.car_state.getIcePower();
            v /= 1e3;
            if (fabs(v) > 10.){
              snprintf(val, sizeof(val), "%.0f", v);
            }
            else{
              snprintf(val, sizeof(val), "%.1f", v);
            }
            
            snprintf(unit, sizeof(unit), "kW");
            break;}
          
          case UIMeasure::ACCEL_POWER_HP:
            {
            snprintf(name, sizeof(name), "ACCEL POW");
            float v = scene.car_state.getAccelPower();
            v /= 1e3;
            v *= 1.34;
            if (fabs(v) > 10.){
              snprintf(val, sizeof(val), "%.0f", v);
            }
            else {
              snprintf(val, sizeof(val), "%.1f", v);
            }
            snprintf(unit, sizeof(unit), "hp");
            break;}

          case UIMeasure::EV_POWER_HP:
            {
            snprintf(name, sizeof(name), "EV POW");
            float v = scene.car_state.getEvPower();
            v /= 1e3;
            v *= 1.34;
            if (fabs(v) > 10.){
              snprintf(val, sizeof(val), "%.0f", v);
            }
            else {
              snprintf(val, sizeof(val), "%.1f", v);
            }
            snprintf(unit, sizeof(unit), "hp");
            break;}

          case UIMeasure::REGEN_POWER_HP:
            {
            snprintf(name, sizeof(name), "REGEN POW");
            float v = scene.car_state.getRegenPower();
            v /= 1e3;
            v *= 1.34;
            if (fabs(v) > 10.){
              snprintf(val, sizeof(val), "%.0f", v);
            }
            else {
              snprintf(val, sizeof(val), "%.1f", v);
            }
            snprintf(unit, sizeof(unit), "hp");
            break;}

          case UIMeasure::BRAKE_POWER_HP:
            {
            snprintf(name, sizeof(name), "BRAKE POW");
            float v = scene.car_state.getBrakePower();
            v /= 1e3;
            v *= 1.34;
            if (fabs(v) > 10.){
              snprintf(val, sizeof(val), "%.0f", v);
            }
            else {
              snprintf(val, sizeof(val), "%.1f", v);
            }
            snprintf(unit, sizeof(unit), "hp");
            break;}

          case UIMeasure::DRIVE_POWER_HP:
            {
            snprintf(name, sizeof(name), "DRIVE POW");
            float v = scene.car_state.getDrivePower();
            v /= 1e3;
            v *= 1.34;
            if (fabs(v) > 10.){
              snprintf(val, sizeof(val), "%.0f", v);
            }
            else{
              snprintf(val, sizeof(val), "%.1f", v);
            }
            snprintf(unit, sizeof(unit), "hp");
            break;}

          case UIMeasure::ICE_POWER_HP:
            {
            snprintf(name, sizeof(name), "ICE POW");
            float v = scene.car_state.getIcePower();
            v /= 1e3;
            v *= 1.34;
            if (fabs(v) > 100.){
              snprintf(val, sizeof(val), "%.0f", v);
            }
            else if (fabs(v) > 10.){
              snprintf(val, sizeof(val), "%.1f", v);
            }
            else{
              snprintf(val, sizeof(val), "%.2f", v);
            }
            snprintf(unit, sizeof(unit), "hp");
            break;}

          case UIMeasure::VISION_CURLATACCEL:
            {
            snprintf(name, sizeof(name), "V:LAT ACC");
            snprintf(val, sizeof(val), "%.1f", scene.longitudinal_plan.getVisionCurrentLateralAcceleration());
            snprintf(unit, sizeof(unit), "m/s²");
            break;}
          
          case UIMeasure::VISION_MAXVFORCURCURV:
            {
            snprintf(name, sizeof(name), "V:MX CUR V");
            snprintf(val, sizeof(val), "%.1f", scene.longitudinal_plan.getVisionMaxVForCurrentCurvature() * 2.24);
            snprintf(unit, sizeof(unit), "mph");
            break;}
          
          case UIMeasure::VISION_MAXPREDLATACCEL:
            {
            snprintf(name, sizeof(name), "V:MX PLA");
            snprintf(val, sizeof(val), "%.1f", scene.longitudinal_plan.getVisionMaxPredictedLateralAcceleration());
            snprintf(unit, sizeof(unit), "m/s²");
            break;}

          case UIMeasure::LANE_POSITION:
            {
            snprintf(name, sizeof(name), "LANE POS");
            auto dat = scene.lateral_plan.getLanePosition();
            if (dat == LanePosition::LEFT){
              snprintf(val, sizeof(val), "left");
            }
            else if (dat == LanePosition::RIGHT){
              snprintf(val, sizeof(val), "right");
            }
            else{
              snprintf(val, sizeof(val), "center");
            }
            break;}

          case UIMeasure::LANE_OFFSET:
            {
            snprintf(name, sizeof(name), "LN OFFSET");
            auto dat = scene.lateral_plan.getLaneOffset();
            if (s->is_metric) {
              snprintf(unit, sizeof(unit), "m");
              snprintf(val, sizeof(val), "%.1f", dat);
            }
            else{
              snprintf(unit, sizeof(unit), "ft");
              snprintf(val, sizeof(val), "%.1f", dat * 3.28084);
            }
            break;}
          
          case UIMeasure::TRAFFIC_COUNT_TOTAL:
            {
            snprintf(name, sizeof(name), "TOTAL");
            int dat = scene.lead_vertices_oncoming.size()
                      + scene.lead_vertices_ongoing.size()
                      + scene.lead_vertices_stopped.size()
                      + (scene.lead_data[0].getStatus() ? 1 : 0)
                      + (scene.radarState.getLeadsCenter()).size();
            snprintf(val, sizeof(val), "%d", dat);
            snprintf(unit, sizeof(unit), "cars");
            break;}

          case UIMeasure::TRAFFIC_COUNT_ONCOMING:
            {
            snprintf(name, sizeof(name), "ONCOMING");
            int dat = scene.lead_vertices_oncoming.size();
            snprintf(val, sizeof(val), "%d", dat);
            snprintf(unit, sizeof(unit), "cars");
            break;}

          case UIMeasure::TRAFFIC_COUNT_ONGOING:
            {
            snprintf(name, sizeof(name), "ONGOING");
            int dat = scene.lead_vertices_ongoing.size()
                      + (scene.lead_data[0].getStatus() ? 1 : 0)
                      + (scene.radarState.getLeadsCenter()).size();
            snprintf(val, sizeof(val), "%d", dat);
            snprintf(unit, sizeof(unit), "cars");
            break;}

          case UIMeasure::TRAFFIC_COUNT_STOPPED:
            {
            snprintf(name, sizeof(name), "STOPPED");
            int dat = scene.lead_vertices_stopped.size()
                      + (scene.lead_data[0].getStatus() 
                        && scene.lead_data[0].getVLeadK() < 3 
                          ? 1 + (scene.radarState.getLeadsCenter()).size() 
                          : 0);
            snprintf(val, sizeof(val), "%d", dat);
            snprintf(unit, sizeof(unit), "cars");
            break;}

          case UIMeasure::TRAFFIC_COUNT_ADJACENT_ONGOING:
            {
            snprintf(name, sizeof(name), "ADJ ONGOING");
            int dat1 = scene.lateral_plan.getTrafficCountLeft();
            int dat2 = scene.lateral_plan.getTrafficCountRight();
            snprintf(val, sizeof(val), "%d:%d", dat1, dat2);
            snprintf(unit, sizeof(unit), "cars");
            break;}

          case UIMeasure::TRAFFIC_ADJ_ONGOING_MIN_DISTANCE:
            {
            snprintf(name, sizeof(name), "MIN ADJ SEP");
            float dat1 = scene.lateral_plan.getTrafficMinSeperationLeft();
            float dat2 = scene.lateral_plan.getTrafficMinSeperationRight();
            snprintf(val, sizeof(val), "%.1f:%.1f", dat1, dat2);
            snprintf(unit, sizeof(unit), "s");
            break;}

          case UIMeasure::LEAD_TTC:
            {
            snprintf(name, sizeof(name), "TTC");
            if (scene.lead_status && scene.lead_v_rel < 0.) {
              float ttc = -scene.lead_d_rel / scene.lead_v_rel;
              g = 0;
              b = 0;
              p = 0.333 * ttc; // red for <= 3s
              g += int((0.5+p) * 255.);
              b += int(p * 255.);
              g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
              b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
              val_color = nvgRGBA(255, g, b, 200);
              if (ttc > 99.){
                snprintf(val, sizeof(val), "99+");
              }
              else if (ttc >= 10.){
                snprintf(val, sizeof(val), "%.0f", ttc);
              }
              else{
                snprintf(val, sizeof(val), "%.1f", ttc);
              }
            } else {
              snprintf(val, sizeof(val), "-");
            }
            snprintf(unit, sizeof(unit), "s");}
            break;

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
                  snprintf(val, sizeof(val), "%.0f", scene.lead_d_rel);
                }
                else{
                  g = 0;
                  b = 0;
                  p = 0.01 * scene.lead_d_rel * 3.281;
                  g += int((0.5+p) * 255.);
                  b += int(p * 255.);
                  g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
                  b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
                  val_color = nvgRGBA(255, g, b, 200);
                  float d_ft = scene.lead_d_rel * 3.281;
                  snprintf(val, sizeof(val), "%.0f", d_ft);
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
              snprintf(name, sizeof(name), "REL:DES DIST");
              auto follow_d = scene.desiredFollowDistance * scene.car_state.getVEgo() + scene.stoppingDistance;
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
                  snprintf(val, sizeof(val), "%d:%d", (int)scene.lead_d_rel, (int)follow_d);
                }
                else{
                  g = 0;
                  b = 0;
                  p = 0.01 * scene.lead_d_rel * 3.281;
                  g += int((0.5+p) * 255.);
                  b += int(p * 255.);
                  g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
                  b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
                  val_color = nvgRGBA(255, g, b, 200);
                  snprintf(val, sizeof(val), "%d:%d", (int)(scene.lead_d_rel * 3.281), (int)(follow_d * 3.281));
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
            if (scene.lead_status && scene.car_state.getVEgo() > 0.5) {
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
            snprintf(name, sizeof(name), "REL:DES DIST");
            if (scene.lead_status && scene.car_state.getVEgo() > 0.5) {
              float follow_t = scene.lead_d_rel / scene.car_state.getVEgo();
              float des_follow_t = scene.desiredFollowDistance + scene.stoppingDistance / scene.car_state.getVEgo();
              g = 0;
              b = 0;
              p = 0.6667 * follow_t;
              g += int((0.5+p) * 255.);
              b += int(p * 255.);
              g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
              b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
              val_color = nvgRGBA(255, g, b, 200);
              snprintf(val, sizeof(val), "%.1f:%.1f", follow_t, des_follow_t);
            } else {
              snprintf(val, sizeof(val), "-");
            }
            snprintf(unit, sizeof(unit), "s");}
            break;
          
          case UIMeasure::LEAD_COSTS:
            {
              snprintf(name, sizeof(name), "D:A COST");
              if (scene.lead_status && scene.car_state.getVEgo() > 0.5) {
                snprintf(val, sizeof(val), "%.1f:%.1f", scene.followDistanceCost, scene.followAccelCost);
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
                snprintf(val, sizeof(val), "%.1f", (scene.lead_v_rel * 3.6));
              } else {
                snprintf(val, sizeof(val), "%.1f", (scene.lead_v_rel * 2.2374144));
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
                float v = (scene.lead_v * 3.6);
                if (v < 100.){
                  snprintf(val, sizeof(val), "%.1f", v);
                }
                else{
                  snprintf(val, sizeof(val), "%.0f", v);
                }
              } else {
                float v = (scene.lead_v * 2.2374144);
                if (v < 100.){
                  snprintf(val, sizeof(val), "%.1f", v);
                }
                else{
                  snprintf(val, sizeof(val), "%.0f", v);
                }
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
            if (scene.angleSteers < 10. && scene.angleSteers > -10.){
              snprintf(val, sizeof(val), "%.1f%s", scene.angleSteers, deg);
            }
            else{
              snprintf(val, sizeof(val), "%.0f%s", scene.angleSteers, deg);
            }
            }
            break;

          case UIMeasure::DESIRED_STEERING_ANGLE: 
            {
            snprintf(name, sizeof(name), "REL:DES STR.");
            float angleSteers = scene.angleSteers > 0. ? scene.angleSteers : -scene.angleSteers;
            g = 255;
            b = 255;
            p = 0.0333 * angleSteers;
            g -= int(0.5 * p * 255.);
            b -= int(p * 255.);
            g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
            b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
            val_color = nvgRGBA(255, g, b, 200);
            if (scene.controls_state.getEnabled()) {
              // steering is in degrees
              snprintf(val, sizeof(val), "%.0f%s:%.0f%s", scene.angleSteers, deg, scene.angleSteersDes, deg);
              val_font_size += 12;
            }else{
              snprintf(val, sizeof(val), "%.0f%s", scene.angleSteers, deg);
            }
            }
            break;

          case UIMeasure::STEERING_ANGLE_ERROR: 
            {
            snprintf(name, sizeof(name), "STR. ERR.");
            float angleSteers = scene.angleSteersErr > 0. ? scene.angleSteersErr : -scene.angleSteersErr;
            if (scene.controls_state.getLatActive()) {
              g = 255;
              b = 255;
              p = 0.2 * angleSteers;
              g -= int(0.5 * p * 255.);
              b -= int(p * 255.);
              g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
              b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
              val_color = nvgRGBA(255, g, b, 200);
              // steering is in degrees
              if (angleSteers < 10. && angleSteers > -10.){
                snprintf(val, sizeof(val), "%.1f%s", scene.angleSteersErr, deg);
              }
              else{
                snprintf(val, sizeof(val), "%.0f%s", scene.angleSteersErr, deg);
              }
              val_font_size += 12;
            }else{
              snprintf(val, sizeof(val), "-");
            }
            }
            break;

          case UIMeasure::ENGINE_RPM: 
            {
              snprintf(name, sizeof(name), "ENG RPM");
              if(scene.engineRPM == 0) {
                snprintf(val, sizeof(val), "OFF");
              }
              else {
                snprintf(val, sizeof(val), "%d", scene.engineRPM);
              }
            }
            break;
            
          case UIMeasure::ENGINE_RPM_TEMPC: 
            {
              snprintf(name, sizeof(name), "ENGINE");
              int temp = scene.car_state.getEngineCoolantTemp();
              snprintf(unit, sizeof(unit), "%d%sC", temp, deg);
              if(scene.engineRPM == 0 && temp < 55) {
                snprintf(val, sizeof(val), "OFF");
              }
              else {
                snprintf(val, sizeof(val), "%d", scene.engineRPM);
                if (temp < 50){ 
                  unit_color = nvgRGBA(84, 207, 249, 200); // cyan if too cool
                }
                else if (temp < 66){ // Volt keeps engine temp between 50-65 when cold outside
                  unit_color = nvgRGBA(0, 255, 100, 200); // green
                }
                else if (temp > 108){
                  unit_color = nvgRGBA(255, 0, 0, 200); // red if too hot
                }
                else if (temp > 95){
                  unit_color = nvgRGBA(255, 169, 63, 200); // orange if close to too hot
                }
              }
            }
            break;

          case UIMeasure::ENGINE_RPM_TEMPF: 
            {
              snprintf(name, sizeof(name), "ENGINE");
              int temp = int(float(scene.car_state.getEngineCoolantTemp()) * 1.8 + 32.5);
              snprintf(unit, sizeof(unit), "%d%sF", temp, deg);
              if(scene.engineRPM == 0 && temp < 130) {
                snprintf(val, sizeof(val), "OFF");
              }
              else {
                snprintf(val, sizeof(val), "%d", scene.engineRPM);
                if (temp < 122){
                  unit_color = nvgRGBA(84, 207, 249, 200); // cyan if too cool
                }
                else if (temp < 151){ // Volt keeps engine temp between 50-65C when cold outside
                  unit_color = nvgRGBA(0, 255, 100, 200); // green
                }
                else if (temp > 226){
                  unit_color = nvgRGBA(255, 0, 0, 200); // red if too hot
                }
                else if (temp > 203){
                  unit_color = nvgRGBA(255, 169, 63, 200); // orange if close to too hot
                }
              }
            }
            break;
            
          case UIMeasure::COOLANT_TEMPC: 
            {
              snprintf(name, sizeof(name), "COOLANT");
              snprintf(unit, sizeof(unit), "%sC", deg);
              int temp = scene.car_state.getEngineCoolantTemp();
              snprintf(val, sizeof(val), "%d", temp);
              if(scene.engineRPM > 0 || temp >= 55) {
                if (temp < 50){
                  val_color = nvgRGBA(84, 207, 249, 200); // cyan if too cool
                }
                else if (temp < 66){ // Volt keeps engine temp between 50-65 when cold outside
                  val_color = nvgRGBA(0, 255, 100, 200); // green
                }
                else if (temp > 108){
                  val_color = nvgRGBA(255, 0, 0, 200); // red if too hot
                }
                else if (temp > 95){
                  val_color = nvgRGBA(255, 169, 63, 200); // orange if close to too hot
                }
              }
            }
            break;
          
          case UIMeasure::COOLANT_TEMPF: 
            {
              snprintf(name, sizeof(name), "COOLANT");
              snprintf(unit, sizeof(unit), "%sF", deg);
              int temp = int(float(scene.car_state.getEngineCoolantTemp()) * 1.8 + 32.5);
              snprintf(val, sizeof(val), "%d", temp);
              if(scene.engineRPM > 0 || temp >= 130) {
                if (temp < 122){
                  val_color = nvgRGBA(84, 207, 249, 200); // cyan if too cool
                }
                else if (temp < 151){ // Volt keeps engine temp between 50-65C when cold outside
                  val_color = nvgRGBA(0, 255, 100, 200); // green
                }
                else if (temp > 226){
                  val_color = nvgRGBA(255, 0, 0, 200); // red if too hot
                }
                else if (temp > 203){
                  val_color = nvgRGBA(255, 169, 63, 200); // orange if close to too hot
                }
              }
            }
            break;
          
          case UIMeasure::PERCENT_GRADE:
            {
            auto data2 = sm["gpsLocationExternal"].getGpsLocationExternal();
            float altitudeUblox = data2.getAltitude();
            float gpsAccuracyUblox = data2.getAccuracy();
            if (scene.car_state.getVEgo() > 0.0){
              scene.percentGradeCurDist += scene.car_state.getVEgo() * (scene.lastTime - scene.percentGradeLastTime);
              if (scene.percentGradeCurDist > scene.percentGradeLenStep){ // record position/elevation at even length intervals
                float prevDist = scene.percentGradePositions[scene.percentGradeRollingIter];
                scene.percentGradeRollingIter++;
                if (scene.percentGradeRollingIter >= scene.percentGradeNumSamples){
                  if (!scene.percentGradeIterRolled){
                    scene.percentGradeIterRolled = true;
                    // Calculate initial mean percent grade
                    float u = 0.;
                    for (int i = 0; i < scene.percentGradeNumSamples; ++i){
                      float rise = scene.percentGradeAltitudes[i] - scene.percentGradeAltitudes[(i+1)%scene.percentGradeNumSamples];
                      float run = scene.percentGradePositions[i] - scene.percentGradePositions[(i+1)%scene.percentGradeNumSamples];
                      if (run != 0.){
                        scene.percentGrades[i] = rise/run * 100.;
                        u += scene.percentGrades[i];
                      }
                    }
                    u /= float(scene.percentGradeNumSamples);
                    scene.percentGrade = u;
                  }
                  scene.percentGradeRollingIter = 0;
                }
                scene.percentGradeAltitudes[scene.percentGradeRollingIter] = altitudeUblox;
                scene.percentGradePositions[scene.percentGradeRollingIter] = prevDist + scene.percentGradeCurDist;
                if (scene.percentGradeIterRolled){
                  float rise = scene.percentGradeAltitudes[scene.percentGradeRollingIter] - scene.percentGradeAltitudes[(scene.percentGradeRollingIter+1)%scene.percentGradeNumSamples];
                  float run = scene.percentGradePositions[scene.percentGradeRollingIter] - scene.percentGradePositions[(scene.percentGradeRollingIter+1)%scene.percentGradeNumSamples];
                  if (run != 0.){
                    // update rolling average
                    float newGrade = rise/run * 100.;
                    scene.percentGrade -= scene.percentGrades[scene.percentGradeRollingIter] / float(scene.percentGradeNumSamples);
                    scene.percentGrade += newGrade / float(scene.percentGradeNumSamples);
                    scene.percentGrades[scene.percentGradeRollingIter] = newGrade;
                  }
                }
                scene.percentGradeCurDist = 0.;
              }
            }
            scene.percentGradeLastTime = scene.lastTime;

            snprintf(name, sizeof(name), "GRADE (GPS)");
            if (scene.percentGradeIterRolled && scene.percentGradePositions[scene.percentGradeRollingIter] >= scene.percentGradeMinDist && gpsAccuracyUblox != 0.00){
              g = 255;
              b = 255;
              p = 0.125 * (scene.percentGrade > 0 ? scene.percentGrade : -scene.percentGrade); // red by 8% grade
              g -= int(0.5 * p * 255.);
              b -= int(p * 255.);
              g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
              b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
              val_color = nvgRGBA(255, g, b, 200);
              snprintf(val, sizeof(val), "%.1f%%", scene.percentGrade);
            }
            else{
              snprintf(val, sizeof(val), "-");
            }}
            break;
          
          case UIMeasure::PERCENT_GRADE_DEVICE:
            {
            scene.percentGradeDevice = tan(scene.car_state.getPitch()) * 100.;
            snprintf(name, sizeof(name), "GRADE");
            g = 255;
            b = 255;
            p = 0.125 * (scene.percentGradeDevice > 0 ? scene.percentGradeDevice : -scene.percentGradeDevice); // red by 8% grade
            g -= int(0.5 * p * 255.);
            b -= int(p * 255.);
            g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
            b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
            val_color = nvgRGBA(255, g, b, 200);
            snprintf(val, sizeof(val), "%.1f%%", scene.percentGradeDevice);
            }
            break;
                    
          case UIMeasure::ROLL_DEVICE:
            {
            float degroll = nvgRadToDeg(scene.device_roll);
            snprintf(name, sizeof(name), "DEVICE ROLL");
            val_color = nvgRGBA(255, 255, 255, 200);
            snprintf(val, sizeof(val), "%.1f°", degroll);
            }
            break;

          case UIMeasure::ROLL:
            {
            float degroll = nvgRadToDeg(scene.road_roll);
            snprintf(name, sizeof(name), "ROAD ROLL");
            val_color = nvgRGBA(255, 255, 255, 200);
            snprintf(val, sizeof(val), "%.1f°", degroll);
            }
            break;

          case UIMeasure::FOLLOW_LEVEL: 
            {
              std::string gap;
              snprintf(name, sizeof(name), "GAP");
              if (scene.dynamic_follow_active){
                snprintf(val, sizeof(val), "%.1f", scene.dynamic_follow_level);
              }else
              {
                switch (int(scene.car_state.getReaddistancelines())){
                  case 1:
                  gap =  "I";
                  break;
                
                  case 2:
                  gap =  "I I";
                  break;
                
                  case 3:
                  gap =  "I I I";
                  break;
                
                  default:
                  gap =  "";
                  break;
                }
                snprintf(val, sizeof(val), "%s", gap.c_str());
              }
            }
            break;
            
          case UIMeasure::HVB_VOLTAGE: 
            {
              snprintf(name, sizeof(name), "HVB VOLT");
              snprintf(unit, sizeof(unit), "V");
              float temp = scene.car_state.getHvbVoltage();
              snprintf(val, sizeof(val), "%.0f", temp);
              g = 255;
              b = 255;
              p = temp - 360.;
              p = p > 0 ? p : -p;
              p *= 0.01666667; // red by the time voltage deviates from nominal voltage (360) by 60V deviation from nominal
              g -= int(0.5 * p * 255.);
              b -= int(p * 255.);
              g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
              b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
              val_color = nvgRGBA(255, g, b, 200);
            }
            break;
          
          case UIMeasure::HVB_CURRENT: 
            {
              snprintf(name, sizeof(name), "HVB CUR");
              snprintf(unit, sizeof(unit), "A");
              float temp = -scene.car_state.getHvbCurrent();
              if (abs(temp) >= 100.){
                snprintf(val, sizeof(val), "%.0f", temp);
              }
              else{
                snprintf(val, sizeof(val), "%.1f", temp);
              }
              g = 255;
              b = 255;
              p = scene.car_state.getHvbVoltage() - 360.;
              p = p > 0 ? p : -p;
              p *= 0.01666667; // red by the time voltage deviates from nominal voltage (360) by 60V deviation from nominal
              g -= int(0.5 * p * 255.);
              b -= int(p * 255.);
              g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
              b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
              val_color = nvgRGBA(255, g, b, 200);
            }
            break;

          case UIMeasure::HVB_WATTAGE:
            {
            snprintf(name, sizeof(name), "HVB POW");
            float v = -scene.car_state.getHvbWattage();
            v /= 1e3;
            if (v > 10){
              snprintf(val, sizeof(val), "%.1f", v);
            }
            else{
              snprintf(val, sizeof(val), "%.0f", v);
            }
            snprintf(unit, sizeof(unit), "kW");
            break;}
          
          case UIMeasure::HVB_WATTVOLT: 
            {
              snprintf(name, sizeof(name), "HVB kW");
              float temp = -scene.car_state.getHvbWattage();
              temp /= 1e3;
              if (abs(temp) >= 10.){
                snprintf(val, sizeof(val), "%.0f", temp);
              }
              else{
                snprintf(val, sizeof(val), "%.1f", temp);
              }
              temp = scene.car_state.getHvbVoltage();
              snprintf(unit, sizeof(unit), "%.0fV", temp);
              g = 255;
              b = 255;
              p = temp - 360.;
              p = p > 0 ? p : -p;
              p *= 0.01666667; // red by the time voltage deviates from nominal voltage (360) by 60V deviation from nominal
              g -= int(0.5 * p * 255.);
              b -= int(p * 255.);
              g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
              b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
              val_color = nvgRGBA(255, g, b, 200);
            }
            break;

          case UIMeasure::EV_BOTH_NOW: 
            {
              snprintf(name, sizeof(name), "EV NOW");
              float temp;
              if (scene.ev_recip_eff_wa[0] <= 0.f){
                if (scene.car_state.getVEgo() > 0.1){
                  temp = scene.ev_recip_eff_wa[0] * 1000.;
                  if (abs(temp) >= 9e5){
                    temp /= 1e6;
                    if (abs(temp) >= 10.){
                      snprintf(val, sizeof(val), "%.0fM", temp);
                    }
                    else{
                      snprintf(val, sizeof(val), "%.1fM", temp);
                    }
                  }
                  else if (abs(temp) >= 9e2){
                    temp /= 1e3;
                    if (abs(temp) >= 10.){
                      snprintf(val, sizeof(val), "%.0fk", temp);
                    }
                    else{
                      snprintf(val, sizeof(val), "%.1fk", temp);
                    }
                  }
                  else{
                    if (abs(temp) >= 10.){
                      snprintf(val, sizeof(val), "%.0f", temp);
                    }
                    else{
                      snprintf(val, sizeof(val), "%.1f", temp);
                    }
                  }
                }
                else {
                  snprintf(val, sizeof(val), "--");
                }
                snprintf(unit, sizeof(unit), (scene.is_metric ? "Wh/km" : "Wh/mi"));
              }
              else{
                temp = 1. / scene.ev_recip_eff_wa[0];
                if (abs(temp) >= scene.ev_recip_eff_wa_max){
                  snprintf(val, sizeof(val), (temp > 0. ? "%.0f+" : "%.0f-"), scene.ev_recip_eff_wa_max);
                }
                else if (abs(temp) >= 10.){
                  snprintf(val, sizeof(val), "%.0f", temp);
                }
                else{
                  snprintf(val, sizeof(val), "%.1f", temp);
                }
                snprintf(unit, sizeof(unit), (scene.is_metric ? "km/kWh" : "mi/kWh"));
              }
            }
            break;

          case UIMeasure::EV_EFF_NOW: 
            {
              snprintf(name, sizeof(name), "EV EFF NOW");
              if (scene.ev_recip_eff_wa[0] == 0.f){
                snprintf(val, sizeof(val), "--");
              }
              else{
                float temp = 1. / scene.ev_recip_eff_wa[0];
                if (abs(temp) >= scene.ev_recip_eff_wa_max){
                  snprintf(val, sizeof(val), (temp > 0. ? "%.0f+" : "%.0f-"), scene.ev_recip_eff_wa_max);
                }
                else if (abs(temp) >= 10.){
                  snprintf(val, sizeof(val), "%.0f", temp);
                }
                else{
                  snprintf(val, sizeof(val), "%.1f", temp);
                }
              }
              snprintf(unit, sizeof(unit), (scene.is_metric ? "km/kWh" : "mi/kWh"));
            }
            break;

          case UIMeasure::EV_EFF_RECENT: 
            {
              snprintf(name, sizeof(name), (scene.is_metric ? "EV EFF 8km" : "EV EFF 5mi"));
              if (scene.ev_recip_eff_wa[1] == 0.f){
                snprintf(val, sizeof(val), "--");
              }
              else{
                float temp = 1. / scene.ev_recip_eff_wa[1];
                if (abs(temp) >= scene.ev_recip_eff_wa_max){
                  snprintf(val, sizeof(val), (temp > 0. ? "%.0f+" : "%.0f-"), scene.ev_recip_eff_wa_max);
                }
                else if (abs(temp) >= 100.){
                  snprintf(val, sizeof(val), "%.0f", temp);
                }
                else{
                  snprintf(val, sizeof(val), "%.1f", temp);
                }
              }
              snprintf(unit, sizeof(unit), (scene.is_metric ? "km/kWh" : "mi/kWh"));
            }
            break;

          case UIMeasure::EV_EFF_TRIP: 
            {
              snprintf(name, sizeof(name), (scene.is_metric ? "EV EFF km/kWh" : "EV EFF mi/kWh"));
              float temp = scene.ev_eff_total;
              float dist = scene.ev_eff_total_dist / (scene.is_metric ? 1000. : 1609.);
              if (abs(temp) == scene.ev_recip_eff_wa_max){
                snprintf(val, sizeof(val), (temp > 0. ? "%.0f+" : "%.0f-"), temp);
              }
              else if (abs(temp) >= 100.){
                snprintf(val, sizeof(val), "%.0f", temp);
              }
              else if (abs(temp) >= 10.){
                snprintf(val, sizeof(val), "%.1f", temp);
              }
              else{
                snprintf(val, sizeof(val), "%.2f", temp);
              }
              if (dist >= 100.){
                snprintf(unit, sizeof(unit), "%.0f%s", dist, (scene.is_metric ? "km" : "mi"));
              }
              else{
                snprintf(unit, sizeof(unit), "%.1f%s", dist, (scene.is_metric ? "km" : "mi"));
              }
            }
            break;

          case UIMeasure::EV_CONSUM_NOW: 
            {
              snprintf(name, sizeof(name), "EV CON NOW");
              if (scene.car_state.getVEgo() > 0.1){
                float temp = scene.ev_recip_eff_wa[0] * 1000.;
                if (abs(temp) >= 9e5){
                  temp /= 1e6;
                  if (abs(temp) >= 10.){
                    snprintf(val, sizeof(val), "%.0fM", temp);
                  }
                  else{
                    snprintf(val, sizeof(val), "%.1fM", temp);
                  }
                }
                else if (abs(temp) >= 9e2){
                  temp /= 1e3;
                  if (abs(temp) >= 10.){
                    snprintf(val, sizeof(val), "%.0fk", temp);
                  }
                  else{
                    snprintf(val, sizeof(val), "%.1fk", temp);
                  }
                }
                else{
                  if (abs(temp) >= 10.){
                    snprintf(val, sizeof(val), "%.0f", temp);
                  }
                  else{
                    snprintf(val, sizeof(val), "%.1f", temp);
                  }
                }
              }
              else{
                snprintf(val, sizeof(val), "--");
              }
              snprintf(unit, sizeof(unit), (scene.is_metric ? "Wh/km" : "Wh/mi"));
            }
            break;

          case UIMeasure::EV_CONSUM_RECENT: 
            {
              snprintf(name, sizeof(name), (scene.is_metric ? "EV CON 8km" : "EV CON 5mi"));
              float temp = scene.ev_recip_eff_wa[1] * 1000.;
              if (abs(temp) >= 9e5){
                temp /= 1e6;
                if (abs(temp) >= 10.){
                  snprintf(val, sizeof(val), "%.0fM", temp);
                }
                else{
                  snprintf(val, sizeof(val), "%.1fM", temp);
                }
              }
              else if (abs(temp) >= 9e2){
                temp /= 1e3;
                if (abs(temp) >= 10.){
                  snprintf(val, sizeof(val), "%.0fk", temp);
                }
                else{
                  snprintf(val, sizeof(val), "%.1fk", temp);
                }
              }
              else{
                if (abs(temp) >= 10.){
                  snprintf(val, sizeof(val), "%.0f", temp);
                }
                else{
                  snprintf(val, sizeof(val), "%.1f", temp);
                }
              }
              snprintf(unit, sizeof(unit), (scene.is_metric ? "Wh/km" : "Wh/mi"));
            }
            break;

          case UIMeasure::EV_CONSUM_TRIP: 
            {
              snprintf(name, sizeof(name), (scene.is_metric ? "EV CON Wh/km" : "EV CON Wh/mi"));
              float dist = scene.ev_eff_total_dist / (scene.is_metric ? 1000. : 1609.);
              if (scene.ev_eff_total == 0.f){
                snprintf(val, sizeof(val), "--");
              }
              else{
                float temp = 1000./scene.ev_eff_total;
                if (abs(temp) >= 9e2){
                  temp /= 1e3;
                  if (abs(temp) >= 100.){
                    snprintf(val, sizeof(val), "%.0fk", temp);
                  }
                  else if (abs(temp) >= 10.){
                    snprintf(val, sizeof(val), "%.1fk", temp);
                  }
                  else{
                    snprintf(val, sizeof(val), "%.2fk", temp);
                  }
                }
                else{
                  if (abs(temp) >= 100.){
                    snprintf(val, sizeof(val), "%.0f", temp);
                  }
                  else if (abs(temp) >= 10.){
                    snprintf(val, sizeof(val), "%.1f", temp);
                  }
                  else{
                    snprintf(val, sizeof(val), "%.2f", temp);
                  }
                }
              }
              if (dist >= 100.){
                snprintf(unit, sizeof(unit), "%.0f%s", dist, (scene.is_metric ? "km" : "mi"));
              }
              else{
                snprintf(unit, sizeof(unit), "%.1f%s", dist, (scene.is_metric ? "km" : "mi"));
              }
            }
            break;
          
          case UIMeasure::EV_OBSERVED_DRIVETRAIN_EFF: 
            {
              snprintf(name, sizeof(name), "EV DRV EFF");
              float temp = scene.car_state.getObservedEVDrivetrainEfficiency();
              snprintf(val, sizeof(val), "%.2f", temp);
            }
            break;
            
          case UIMeasure::LANE_WIDTH: 
            {
              snprintf(name, sizeof(name), "LANE W");
              if (s->is_metric){
                snprintf(unit, sizeof(unit), "m");
                snprintf(val, sizeof(val), "%.1f", scene.lateralPlan.laneWidth);
              }
              else{
                snprintf(unit, sizeof(unit), "ft");
                snprintf(val, sizeof(val), "%.1f", scene.lateralPlan.laneWidth * 3.281);
              }
            }
            break;

          case UIMeasure::LANE_DIST_FROM_CENTER: 
            {
              snprintf(name, sizeof(name), "LANE CENTER");
              if (s->is_metric){
                snprintf(unit, sizeof(unit), "m");
                snprintf(val, sizeof(val), "%.1f", scene.lateralPlan.laneCenter);
              }
              else{
                snprintf(unit, sizeof(unit), "ft");
                snprintf(val, sizeof(val), "%.1f", scene.lateralPlan.laneCenter * 3.281);
              }
            }
            break;

          case UIMeasure::DISTANCE_TRAVELED_TOTAL: 
            {
              snprintf(name, sizeof(name), "TRIP DIST.");
              float temp = scene.controls_state.getDistanceTraveledTotal() / (scene.is_metric ? 1000. : 1609.);
              if (abs(temp) >= 100.){
                snprintf(val, sizeof(val), "%.0f", temp);
              }
              else if (abs(temp) >= 10.){
                snprintf(val, sizeof(val), "%.1f", temp);
              }
              else{
                snprintf(val, sizeof(val), "%.2f", temp);
              }
              snprintf(unit, sizeof(unit), (scene.is_metric ? "km" : "mi"));
            }
            break;

          case UIMeasure::DISTANCE_TRAVELED_SESSION: 
            {
              snprintf(name, sizeof(name), "DRIVE DIST.");
              float temp = scene.controls_state.getDistanceTraveledSession() / (scene.is_metric ? 1000. : 1609.);
              if (abs(temp) >= 100.){
                snprintf(val, sizeof(val), "%.0f", temp);
              }
              else if (abs(temp) >= 10.){
                snprintf(val, sizeof(val), "%.1f", temp);
              }
              else{
                snprintf(val, sizeof(val), "%.2f", temp);
              }
              snprintf(unit, sizeof(unit), (scene.is_metric ? "km" : "mi"));
            }
            break;
          
          case UIMeasure::DISTANCE_ENGAGED: 
            {
              snprintf(name, sizeof(name), "ENGAGED");
              float temp = scene.controls_state.getEngagedDistance() / (scene.is_metric ? 1000. : 1609.);
              if (abs(temp) >= 100.){
                snprintf(val, sizeof(val), "%.0f", temp);
              }
              else if (abs(temp) >= 10.){
                snprintf(val, sizeof(val), "%.1f", temp);
              }
              else{
                snprintf(val, sizeof(val), "%.2f", temp);
              }
              snprintf(unit, sizeof(unit), (scene.is_metric ? "km" : "mi"));
            }
            break;

          case UIMeasure::DISTANCE_ENGAGED_SESSION: 
            {
              snprintf(name, sizeof(name), "DRIVE ENGD.");
              float temp = scene.controls_state.getEngagedDistanceSession() / (scene.is_metric ? 1000. : 1609.);
              if (abs(temp) >= 100.){
                snprintf(val, sizeof(val), "%.0f", temp);
              }
              else if (abs(temp) >= 10.){
                snprintf(val, sizeof(val), "%.1f", temp);
              }
              else{
                snprintf(val, sizeof(val), "%.2f", temp);
              }
              snprintf(unit, sizeof(unit), (scene.is_metric ? "km" : "mi"));
            }
            break;

          case UIMeasure::DISTANCE_ENGAGED_TOTAL: 
            {
              snprintf(name, sizeof(name), "TRIP ENGD.");
              float temp = scene.controls_state.getEngagedDistanceTotal() / (scene.is_metric ? 1000. : 1609.);
              if (abs(temp) >= 100.){
                snprintf(val, sizeof(val), "%.0f", temp);
              }
              else if (abs(temp) >= 10.){
                snprintf(val, sizeof(val), "%.1f", temp);
              }
              else{
                snprintf(val, sizeof(val), "%.2f", temp);
              }
              snprintf(unit, sizeof(unit), (scene.is_metric ? "km" : "mi"));
            }
            break;

          case UIMeasure::DISTANCE_ENGAGED_PERCENT_SESSION: 
            {
              snprintf(name, sizeof(name), "DRIVE ENGD.");
              float temp = scene.controls_state.getPercentEngagedDistanceSession();
              snprintf(val, sizeof(val), "%.1f%%", temp);
              snprintf(unit, sizeof(unit), "distance");
            }
            break;

          case UIMeasure::DISTANCE_ENGAGED_PERCENT_TOTAL: 
            {
              snprintf(name, sizeof(name), "TRIP ENGD.");
              float temp = scene.controls_state.getPercentEngagedDistanceTotal();
              snprintf(val, sizeof(val), "%.1f%%", temp);
              snprintf(unit, sizeof(unit), "distance");
            }
            break;

          case UIMeasure::INTERACTION_DISTANCE: 
            {
              snprintf(name, sizeof(name), "INTERACT");
              float temp = scene.controls_state.getInteractionDistance() / (scene.is_metric ? 1000. : 1609.);
              if (abs(temp) >= 100.){
                snprintf(val, sizeof(val), "%.0f", temp);
              }
              else if (abs(temp) >= 10.){
                snprintf(val, sizeof(val), "%.1f", temp);
              }
              else{
                snprintf(val, sizeof(val), "%.2f", temp);
              }
              snprintf(unit, sizeof(unit), (scene.is_metric ? "km" : "mi"));
            }
            break;

          case UIMeasure::INTERVENTION_DISTANCE: 
            {
              snprintf(name, sizeof(name), "INTERVENE");
              float temp = scene.controls_state.getInterventionDistance() / (scene.is_metric ? 1000. : 1609.);
              if (abs(temp) >= 100.){
                snprintf(val, sizeof(val), "%.0f", temp);
              }
              else if (abs(temp) >= 10.){
                snprintf(val, sizeof(val), "%.1f", temp);
              }
              else{
                snprintf(val, sizeof(val), "%.2f", temp);
              }
              snprintf(unit, sizeof(unit), (scene.is_metric ? "km" : "mi"));
            }
            break;

          case UIMeasure::DISTRACTION_DISTANCE: 
            {
              snprintf(name, sizeof(name), "DISTRACT");
              float temp = scene.controls_state.getDistractionDistance() / (scene.is_metric ? 1000. : 1609.);
              if (abs(temp) >= 100.){
                snprintf(val, sizeof(val), "%.0f", temp);
              }
              else if (abs(temp) >= 10.){
                snprintf(val, sizeof(val), "%.1f", temp);
              }
              else{
                snprintf(val, sizeof(val), "%.2f", temp);
              }
              snprintf(unit, sizeof(unit), (scene.is_metric ? "km" : "mi"));
            }
            break;

          case UIMeasure::DISTANCE_PER_DISENGAGEMENT_SESSION: 
            {
              snprintf(name, sizeof(name), "PER DISENGAGE");
              float temp = scene.controls_state.getEngagedDistanceSession() / (scene.is_metric ? 1000. : 1609.);
              temp /= MAX(1.0, float(scene.controls_state.getDisengagementCountSession()));
              if (abs(temp) >= 100.){
                snprintf(val, sizeof(val), "%.0f", temp);
              }
              else if (abs(temp) >= 10.){
                snprintf(val, sizeof(val), "%.1f", temp);
              }
              else{
                snprintf(val, sizeof(val), "%.2f", temp);
              }
              snprintf(unit, sizeof(unit), "drive %s", (scene.is_metric ? "km" : "mi"));
            }
            break;

          case UIMeasure::DISTANCE_PER_DISENGAGEMENT_TOTAL: 
            {
              snprintf(name, sizeof(name), "PER DISENGAGE");
              float temp = scene.controls_state.getEngagedDistanceTotal() / (scene.is_metric ? 1000. : 1609.);
              temp /= MAX(1.0, float(scene.controls_state.getDisengagementCountTotal()));
              if (abs(temp) >= 100.){
                snprintf(val, sizeof(val), "%.0f", temp);
              }
              else if (abs(temp) >= 10.){
                snprintf(val, sizeof(val), "%.1f", temp);
              }
              else{
                snprintf(val, sizeof(val), "%.2f", temp);
              }
              snprintf(unit, sizeof(unit), "trip %s", (scene.is_metric ? "km" : "mi"));
            }
            break;

          case UIMeasure::DISTANCE_PER_INTERACTION_SESSION: 
            {
              snprintf(name, sizeof(name), "PER INTERACT");
              float temp = scene.controls_state.getEngagedDistanceSession() / (scene.is_metric ? 1000. : 1609.);
              temp /= MAX(1.0, float(scene.controls_state.getInteractionCountSession()));
              if (abs(temp) >= 100.){
                snprintf(val, sizeof(val), "%.0f", temp);
              }
              else if (abs(temp) >= 10.){
                snprintf(val, sizeof(val), "%.1f", temp);
              }
              else{
                snprintf(val, sizeof(val), "%.2f", temp);
              }
              snprintf(unit, sizeof(unit), "drive %s", (scene.is_metric ? "km" : "mi"));
            }
            break;

          case UIMeasure::DISTANCE_PER_INTERACTION_TOTAL: 
            {
              snprintf(name, sizeof(name), "PER INTERACT");
              float temp = scene.controls_state.getEngagedDistanceTotal() / (scene.is_metric ? 1000. : 1609.);
              temp /= MAX(1.0, float(scene.controls_state.getInteractionCountTotal()));
              if (abs(temp) >= 100.){
                snprintf(val, sizeof(val), "%.0f", temp);
              }
              else if (abs(temp) >= 10.){
                snprintf(val, sizeof(val), "%.1f", temp);
              }
              else{
                snprintf(val, sizeof(val), "%.2f", temp);
              }
              snprintf(unit, sizeof(unit), "trip %s", (scene.is_metric ? "km" : "mi"));
            }
            break;

          case UIMeasure::DISTANCE_PER_INTERVENTION_SESSION: 
            {
              snprintf(name, sizeof(name), "PER INTERVENE");
              float temp = scene.controls_state.getEngagedDistanceSession() / (scene.is_metric ? 1000. : 1609.);
              temp /= MAX(1.0, float(scene.controls_state.getInterventionCountSession()));
              if (abs(temp) >= 100.){
                snprintf(val, sizeof(val), "%.0f", temp);
              }
              else if (abs(temp) >= 10.){
                snprintf(val, sizeof(val), "%.1f", temp);
              }
              else{
                snprintf(val, sizeof(val), "%.2f", temp);
              }
              snprintf(unit, sizeof(unit), "drive %s", (scene.is_metric ? "km" : "mi"));
            }
            break;

          case UIMeasure::DISTANCE_PER_INTERVENTION_TOTAL: 
            {
              snprintf(name, sizeof(name), "PER INTERVENE");
              float temp = scene.controls_state.getEngagedDistanceTotal() / (scene.is_metric ? 1000. : 1609.);
              temp /= MAX(1.0, float(scene.controls_state.getInterventionCountTotal()));
              if (abs(temp) >= 100.){
                snprintf(val, sizeof(val), "%.0f", temp);
              }
              else if (abs(temp) >= 10.){
                snprintf(val, sizeof(val), "%.1f", temp);
              }
              else{
                snprintf(val, sizeof(val), "%.2f", temp);
              }
              snprintf(unit, sizeof(unit), "trip %s", (scene.is_metric ? "km" : "mi"));
            }
            break;

          case UIMeasure::DISTANCE_PER_DISTRACTION_SESSION: 
            {
              snprintf(name, sizeof(name), "PER DISTRACT");
              float temp = scene.controls_state.getEngagedDistanceSession() / (scene.is_metric ? 1000. : 1609.);
              temp /= MAX(1.0, float(scene.controls_state.getDistractionCountSession()));
              if (abs(temp) >= 100.){
                snprintf(val, sizeof(val), "%.0f", temp);
              }
              else if (abs(temp) >= 10.){
                snprintf(val, sizeof(val), "%.1f", temp);
              }
              else{
                snprintf(val, sizeof(val), "%.2f", temp);
              }
              snprintf(unit, sizeof(unit), "drive %s", (scene.is_metric ? "km" : "mi"));
            }
            break;

          case UIMeasure::DISTANCE_PER_DISTRACTION_TOTAL: 
            {
              snprintf(name, sizeof(name), "PER DISTRACT");
              float temp = scene.controls_state.getEngagedDistanceTotal() / (scene.is_metric ? 1000. : 1609.);
              temp /= MAX(1.0, float(scene.controls_state.getDistractionCountTotal()));
              if (abs(temp) >= 100.){
                snprintf(val, sizeof(val), "%.0f", temp);
              }
              else if (abs(temp) >= 10.){
                snprintf(val, sizeof(val), "%.1f", temp);
              }
              else{
                snprintf(val, sizeof(val), "%.2f", temp);
              }
              snprintf(unit, sizeof(unit), "trip %s", (scene.is_metric ? "km" : "mi"));
            }
            break;
            
          case UIMeasure::DEVICE_BATTERY: 
            {
              snprintf(name, sizeof(name), "DEVICE BATT.");
              snprintf(unit, sizeof(unit), "%.1f A", float(scene.deviceState.getBatteryCurrent()) * 1e-6);
              snprintf(val, sizeof(val), "%d", scene.deviceState.getBatteryPercent());
            }
            break;

          case UIMeasure::VISION_VF: 
            {
              snprintf(name, sizeof(name), "V: VF");
              snprintf(val, sizeof(val), "%.2f", (float)scene.longitudinal_plan.getVisionVf());
            }
            break;

          default: {// invalid number
            snprintf(name, sizeof(name), "INVALID");
            snprintf(val, sizeof(val), "42");}
            break;
        }

        nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
        // now print the metric
        // first value
        
        int vallen = strlen(val);
        if (vallen > 4){
          val_font_size -= 6 + (vallen - 4) * 8;
        }
        int unitlen = strlen(unit);
        if (unitlen > 4){
          unit_font_size -= 5 + (unitlen - 4) * 4;
        }
        int slot_x = scene.measure_slots_rect.x + (scene.measure_cur_num_slots <= scene.measure_max_rows ? 0 : (i < scene.measure_max_rows ? slots_r * 2 : 0));
        int x = slot_x + slots_r - unit_font_size / 2;
        if (i >= scene.measure_max_rows){
          x = slot_x + slots_r + unit_font_size / 2;
        }
        int slot_y = scene.measure_slots_rect.y + (i % scene.measure_max_rows) * slot_y_rng;
        int slot_y_mid = slot_y + slot_y_rng / 2;
        int y = slot_y_mid + slot_y_rng / 2 - 8 - label_font_size;
        if (strlen(name) == 0){
          y += label_font_size / 2;
        }
        if (strlen(unit) == 0){
          x = slot_x + slots_r;
        }
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
      catch(...){}
    }
    scene.measure_cur_num_slots = measure_cur_num_slots;
  }
}


static void ui_draw_vision_turnspeed(UIState *s) {
  const float mapTurnSpeed = s->scene.longitudinal_plan.getTurnSpeed();
  auto visionTurnControllerState = s->scene.longitudinal_plan.getVisionTurnControllerState();
  const bool vision_active = visionTurnControllerState > cereal::LongitudinalPlan::VisionTurnControllerState::DISABLED;
  const float visionTurnSpeed = vision_active ? s->scene.longitudinal_plan.getVisionTurnSpeed() : 0.;
  float turnSpeed;
  if (mapTurnSpeed > 0. && visionTurnSpeed > 0.){
    turnSpeed = mapTurnSpeed < visionTurnSpeed ? mapTurnSpeed : visionTurnSpeed;
  }
  else if (mapTurnSpeed > 0.){
    turnSpeed = mapTurnSpeed;
  }
  else if (visionTurnSpeed > 0.){
    turnSpeed = visionTurnSpeed;
  }
  else{
    turnSpeed = 0.;
  }
  const float vEgo = s->scene.car_state.getVEgo();  
  auto source = s->scene.longitudinal_plan.getLongitudinalPlanSource();
  const bool show = (s->scene.controls_state.getActive() && turnSpeed > 0.0 && (turnSpeed < vEgo+2.24 || s->scene.show_debug_ui));

  if (show) {
    const Rect maxspeed_rect = {bdr_s * 2, int(bdr_s * 1.5), 184, 202};
    Rect speed_sign_rect = Rect{maxspeed_rect.centerX() - speed_sgn_r, 
      maxspeed_rect.bottom() + 2 * (bdr_s + speed_sgn_r), 
      2 * speed_sgn_r, 
      maxspeed_rect.h};
    const float speed = turnSpeed * (s->scene.is_metric ? 3.6 : 2.2369362921);

    if (source == cereal::LongitudinalPlan::LongitudinalPlanSource::TURN){
      // vision turn controller, so need sign of curvature to know curve direction
      int curveSign = 0;
      if (visionTurnControllerState == cereal::LongitudinalPlan::VisionTurnControllerState::ENTERING){
        curveSign = s->scene.longitudinal_plan.getVisionMaxPredictedCurvature() > 0. ? -1 : 1;
      }
      else {
        auto curvatures = s->scene.lateral_plan.getCurvatures();
        for (auto curvature : curvatures){
          curveSign = curvature > 0. ? -1 : 1;
          break;
        }
      }

      const bool is_active = source == cereal::LongitudinalPlan::LongitudinalPlanSource::TURN;

      const int distToTurn = visionTurnControllerState > cereal::LongitudinalPlan::VisionTurnControllerState::ENTERING ? -1 : int(s->scene.longitudinal_plan.getVisionMaxPredictedLateralAccelerationDistance() * 
                                (s->scene.is_metric ? 1.0 : 3.28084) / 10) * 10;
      const std::string distance_str = visionTurnControllerState > cereal::LongitudinalPlan::VisionTurnControllerState::ENTERING ? "TURN" : "VIS";

      ui_draw_turn_speed_sign(s, speed_sign_rect.centerX(), speed_sign_rect.centerY(), 
                              speed_sign_rect.w, speed, 
                              curveSign, distToTurn > 0 ? distance_str.c_str() : "", "sans-bold", is_active);
    }
    else{
      const bool is_active = source == cereal::LongitudinalPlan::LongitudinalPlanSource::TURNLIMIT;

      

      const int curveSign = s->scene.longitudinal_plan.getTurnSign();
      const int distToTurn = int(s->scene.longitudinal_plan.getDistToTurn() * 
                                (s->scene.is_metric ? 1.0 : 3.28084) / 10) * 10;
      const std::string distance_str = std::to_string(distToTurn) + (s->scene.is_metric ? "m" : "f");

      ui_draw_turn_speed_sign(s, speed_sign_rect.centerX(), speed_sign_rect.centerY(), 
                              speed_sign_rect.w, speed, 
                              curveSign, distToTurn > 0 ? distance_str.c_str() : "", "sans-bold", is_active);
    }



  }
}

static void ui_draw_vision_speed(UIState *s) {
  if (s->scene.show_cur_speed){
    const int speed = s->scene.car_state.getClusterSpeed();
    const std::string speed_str = std::to_string(speed);
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
    ui_draw_text(s, s->fb_w / 2, 210, speed_str.c_str(), 96 * 2.5, COLOR_WHITE, "sans-bold");
    ui_draw_text(s, s->fb_w / 2, 290, s->scene.is_metric ? "km/h" : "mph", 36 * 2.5, COLOR_WHITE_ALPHA(200), "sans-regular");
  }
  s->scene.speed_rect = {s->fb_w / 2 - 50, 0, 200, 450};
}

static void ui_draw_vision_event(UIState *s) {
  s->scene.wheel_touch_rect = {1,1,1,1};
  if (s->scene.engageable) {
    // draw steering wheel
    const float rot_angle_multiplier = s->scene.car_state.getVEgo() / 5.0;
    const float rot_angle = -s->scene.angleSteers * 0.01745329252 * (rot_angle_multiplier > 1.0 ? rot_angle_multiplier : 1.0);
    const int radius = 88;
    const int center_x = s->fb_w - radius - bdr_s * 2;
    const int center_y = radius  + (bdr_s * 1.5);
    auto const & bg_colors_ = (s->scene.alt_engage_color_enabled ? alt_bg_colors : bg_colors);
    const QColor &color = bg_colors_[(s->scene.car_state.getLkaEnabled() ? MAX(1,s->status) : UIStatus::STATUS_DISENGAGED)];
    NVGcolor nvg_color = nvgRGBA(color.red(), color.green(), color.blue(), color.alpha());
  
    // draw circle behind wheel
    s->scene.wheel_touch_rect = {center_x - radius, center_y - radius, 2 * radius, 2 * radius};
    ui_fill_rect(s->vg, s->scene.wheel_touch_rect, nvg_color, radius);

    // now rotate and draw the wheel
    nvgSave(s->vg);
    nvgTranslate(s->vg, center_x, center_y);
    if (s->scene.wheel_rotates){
      nvgRotate(s->vg, rot_angle);
    }
    ui_draw_image(s, {-radius, -radius, 2*radius, 2*radius}, "wheel", 1.0f);
    nvgRestore(s->vg);
    
    // draw extra circle to indiate paused low-speed one-pedal blinker steering is enabled
    if (s->scene.visionBrakingEnabled and !(s->scene.mapBrakingEnabled)){
      nvgBeginPath(s->vg);
      const int r = int(float(radius) * 1.15);
      nvgRoundedRect(s->vg, center_x - r, center_y - r, 2 * r, 2 * r, r);
      nvgStrokeColor(s->vg, COLOR_WHITE_ALPHA(255));
      nvgFillColor(s->vg, nvgRGBA(0,0,0,0));
      nvgFill(s->vg);
      nvgStrokeWidth(s->vg, 7);
      nvgStroke(s->vg);
    }
    else if (s->scene.visionBrakingEnabled and s->scene.mapBrakingEnabled){
      nvgBeginPath(s->vg);
      const int r = int(float(radius) * 1.15);
      nvgRoundedRect(s->vg, center_x - r, center_y - r, 2 * r, 2 * r, r);
      nvgStrokeColor(s->vg, s->scene.network_strength > 0 ? (s->scene.alt_engage_color_enabled ? nvgRGBA(0,255,255,255) : COLOR_GREEN_ALPHA(255)) : COLOR_RED_ALPHA(255));
      nvgFillColor(s->vg, nvgRGBA(0,0,0,0));
      nvgFill(s->vg);
      nvgStrokeWidth(s->vg, 7);
      nvgStroke(s->vg);
    }
    
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
  // draw cell/wifi indicator if map-braking or speed limit control (which require data connection) enabled
  if (s->scene.mapBrakingEnabled || s->scene.speed_limit_control_enabled){
    const int r = 12;
    int x = bdr_s * 2;
    int y = bdr_s - 22;
    for (int i = 0; i < 5; ++i){
      nvgBeginPath(s->vg);
      nvgRoundedRect(s->vg, x, y, 2*r, 2*r, r);
      nvgStrokeColor(s->vg, COLOR_WHITE_ALPHA(200));
      nvgFillColor(s->vg, COLOR_WHITE_ALPHA(i < s->scene.network_strength ? 200 : 70));
      nvgFill(s->vg);
      nvgStrokeWidth(s->vg, 0);
      nvgStroke(s->vg);
      x += 2*r + 6;
    }
    if (s->scene.network_strength > 0){
      x += 5;
      y -= 9;
      nvgBeginPath(s->vg);
      nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_TOP);
      nvgFontFace(s->vg, "sans-semibold");
      nvgFontSize(s->vg, 40);
      nvgFillColor(s->vg, COLOR_WHITE_ALPHA(200));
      nvgText(s->vg, x, y, s->scene.network_type_string.c_str(), NULL);
    }
  }

  // current road name and heading

  nvgBeginPath(s->vg);
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_TOP);
  nvgFontFace(s->vg, "sans-regular");
  nvgFontSize(s->vg, 75);
  nvgFillColor(s->vg, COLOR_WHITE_ALPHA(255));
  char val[16];
  if (s->scene.bearingAccuracy != 180.00) {
    if (((s->scene.bearingDeg >= 337.5) && (s->scene.bearingDeg <= 360)) || ((s->scene.bearingDeg >= 0) && (s->scene.bearingDeg <= 22.5))) {
      snprintf(val, sizeof(val), "(N)");
    } else if ((s->scene.bearingDeg > 22.5) && (s->scene.bearingDeg < 67.5)) {
      snprintf(val, sizeof(val), "(NE)");
    } else if ((s->scene.bearingDeg >= 67.5) && (s->scene.bearingDeg <= 112.5)) {
      snprintf(val, sizeof(val), "(E)");
    } else if ((s->scene.bearingDeg > 112.5) && (s->scene.bearingDeg < 157.5)) {
      snprintf(val, sizeof(val), "(SE)");
    } else if ((s->scene.bearingDeg >= 157.5) && (s->scene.bearingDeg <= 202.5)) {
      snprintf(val, sizeof(val), "(S)");
    } else if ((s->scene.bearingDeg > 202.5) && (s->scene.bearingDeg < 247.5)) {
      snprintf(val, sizeof(val), "(SW)");
    } else if ((s->scene.bearingDeg >= 247.5) && (s->scene.bearingDeg <= 292.5)) {
      snprintf(val, sizeof(val), "(W)");
    } else if ((s->scene.bearingDeg > 292.5) && (s->scene.bearingDeg < 337.5)) {
      snprintf(val, sizeof(val), "(NW)");
    }
  }
  if (s->scene.network_strength > 0 && !s->scene.map_open){
    nvgText(s->vg, s->fb_w / 2, bdr_s - 31, (s->scene.current_road_name + " " + val + " ").c_str(), NULL);
  }
  else {
    nvgText(s->vg, s->fb_w / 2, bdr_s - 31, val, NULL);
  }
}

static void ui_draw_vision_face(UIState *s) {
  const Rect maxspeed_rect = {bdr_s * 2, int(bdr_s * 1.5), 184, 202};
  const int radius = 96;
  const int center_x = maxspeed_rect.centerX();
  int center_y = s->fb_h - footer_h / 2;
  center_y = offset_button_y(s, center_y, radius);
  ui_draw_circle_image(s, center_x, center_y, radius, "driver_face", s->scene.dm_active);
}

static void ui_draw_vision_power_meter(UIState *s) {
  Rect & outer_rect = s->scene.power_meter_rect;
  if (s->scene.brake_indicator_enabled && s->scene.power_meter_mode < 2){
    const int w = s->fb_w * 3 / 128;
    const int x = s->fb_w * 121 / 128 - 6;
    int alert_offset = s->scene.controls_state.getAlertSize() == cereal::ControlsState::AlertSize::SMALL
                    ? s->fb_h * 7 / 32
                     : s->scene.controls_state.getAlertSize() == cereal::ControlsState::AlertSize::MID
                     ? s->fb_h * 6 / 16 : 0;
    int h = (s->scene.power_meter_mode == 0 || alert_offset ? 22 : 21) * s->fb_h / 32 - 6;
    h -= alert_offset;
    const int hu = h / 2;
    const int hl = h - hu;
    int y = (s->scene.power_meter_mode == 0 || alert_offset ? 30 : 29) * s->fb_h / 32;
    y -= alert_offset;
    const int y_mid = y - hl;
    outer_rect = {x, y-h, 2 * w, h};
    s->scene.brake_touch_rect = outer_rect;
    const int y_offset = 2;

    int ipow = 0;
    float pow_cur[4];
    pow_cur[ipow++] = MAX(0., s->scene.car_state.getIcePower());
    pow_cur[ipow++] = MAX(0., s->scene.car_state.getEvPower());
    pow_cur[ipow++] = MAX(0., s->scene.car_state.getBrakePower());
    pow_cur[ipow++] = MAX(0., s->scene.car_state.getRegenPower());
    // float drag_power = s->scene.car_state.getDragPower() * 1e-3;
    // float rolling_resistance_power = s->scene.car_state.getRollingPower() * 1e-3;
    // float pitch_power = s->scene.car_state.getPitchPower() * 1e-3;
    for (ipow = 0; ipow < 4; ++ipow){
      pow_cur[ipow] *= 1e-3; // convert from W to kW
      s->scene.power_cur[ipow] = s->scene.power_meter_ema_k * pow_cur[ipow] + (1. - s->scene.power_meter_ema_k) * s->scene.power_cur[ipow];
    }

    const int inner_fill_alpha = 200;
    const int outer_fill_alpha = 20;
    const int rect_r = 10;

    // draw background
    nvgBeginPath(s->vg);
    nvgRoundedRect(s->vg, outer_rect.x, outer_rect.y, outer_rect.w, outer_rect.h, rect_r);
    nvgFillColor(s->vg, COLOR_BLACK_ALPHA(100)); // blue 
    nvgFill(s->vg);


    float pow_rel, pow_rel_max;
    int hi, wi;
    int xi = x;
    if (s->scene.car_is_ev){
      // ev power
      pow_rel = s->scene.power_cur[1] / s->scene.power_max[1];
      hi = hu * pow_rel;
      wi = (s->scene.car_state.getEngineRPM() == 0 ? 2 : 1) * w;

      // inner bar
      nvgBeginPath(s->vg);
      nvgRect(s->vg, xi, y_mid - hi - y_offset, wi, hi);
      nvgFillColor(s->vg, COLOR_GRACE_BLUE_ALPHA(inner_fill_alpha)); // blue 
      nvgFill(s->vg);
      // outer box for bar
      nvgBeginPath(s->vg);
      nvgRoundedRect(s->vg, xi, y_mid - hu - y_offset, wi, hu, rect_r);
      nvgFillColor(s->vg, COLOR_GRACE_BLUE_ALPHA(outer_fill_alpha));
      nvgFill(s->vg);

      // add lines to indicate drag and rolling resistance losses (I like the idea but it's too much)
      // if (pow_rel > 0.){
      //   int h_drag = hu * drag_power / s->scene.power_max[1];
      //   int h_rr = h_drag + hu * rolling_resistance_power / s->scene.power_max[1];
      //   int h_pitch = h_rr + hu * pitch_power / s->scene.power_max[1];
      //   nvgBeginPath(s->vg);
      //   nvgRect(s->vg, xi+2, y_mid - h_rr - h_pitch / 2, wi-4, h_pitch);
      //   nvgStrokeWidth(s->vg, 5);
      //   nvgStrokeColor(s->vg, COLOR_WHITE_ALPHA(150));
      //   nvgStroke(s->vg);
      //   nvgBeginPath(s->vg);
      //   nvgRect(s->vg, xi+2, y_mid - h_drag - h_rr / 2, wi-4, h_rr);
      //   nvgStrokeWidth(s->vg, 5);
      //   nvgStrokeColor(s->vg, COLOR_WHITE_ALPHA(150));
      //   nvgStroke(s->vg);
      //   nvgBeginPath(s->vg);
      //   nvgRect(s->vg, xi+2, y_mid - h_drag / 2, wi-4, h_drag);
      //   nvgStrokeWidth(s->vg, 5);
      //   nvgStrokeColor(s->vg, COLOR_WHITE_ALPHA(150));
      //   nvgStroke(s->vg);
      // }

      xi += wi;
      pow_rel_max = MAX(pow_rel, pow_rel_max);
    }

    if (!s->scene.car_is_ev || s->scene.car_state.getEngineRPM() > 0){
      // ice power
      pow_rel = s->scene.power_cur[0] / s->scene.power_max[0];
      int hu_ice = (s->scene.car_is_ev ? int(float(hu) * s->scene.power_max[0] / s->scene.power_max[1]) : hu);
      hi = hu_ice * pow_rel;
      wi = (s->scene.car_is_ev ? 1 : 2) * w;

      // inner bar
      nvgBeginPath(s->vg);
      nvgRect(s->vg, xi, y_mid - hi - y_offset, wi, hi);
      nvgFillColor(s->vg, nvgRGBA(249,240,1,inner_fill_alpha)); // yellow
      nvgFill(s->vg);
      // outer box for bar
      nvgBeginPath(s->vg);
      nvgRoundedRect(s->vg, xi, y_mid - hu_ice - y_offset, wi, hu_ice, rect_r);
      nvgFillColor(s->vg, nvgRGBA(249,240,1,outer_fill_alpha));
      nvgFill(s->vg);
      pow_rel_max = MAX(pow_rel, pow_rel_max);

            // add lines to indicate drag and rolling resistance losses
      // if (pow_rel > 0.){
      //   int h_drag = hu_ice * drag_power / s->scene.power_max[0];
      //   int h_rr = h_drag + hu_ice * rolling_resistance_power / s->scene.power_max[0];
      //   int h_pitch = h_rr + hu * pitch_power / s->scene.power_max[0];
      //   nvgBeginPath(s->vg);
      //   nvgRect(s->vg, xi+2, y_mid - h_rr - h_pitch / 2, wi-4, h_pitch);
      //   nvgStrokeWidth(s->vg, 5);
      //   nvgStrokeColor(s->vg, COLOR_WHITE_ALPHA(150));
      //   nvgStroke(s->vg);
      //   nvgBeginPath(s->vg);
      //   nvgRect(s->vg, xi+2, y_mid - h_drag - h_rr / 2, wi-4, h_rr);
      //   nvgStrokeWidth(s->vg, 5);
      //   nvgStrokeColor(s->vg, COLOR_WHITE_ALPHA(150));
      //   nvgStroke(s->vg);
      //   nvgBeginPath(s->vg);
      //   nvgRect(s->vg, xi+2, y_mid - h_drag / 2, wi-4, h_drag);
      //   nvgStrokeWidth(s->vg, 5);
      //   nvgStrokeColor(s->vg, COLOR_WHITE_ALPHA(150));
      //   nvgStroke(s->vg);
      // }
    }


    // regen/engine braking power
    pow_rel = s->scene.power_cur[3] / s->scene.power_max[3];
    hi = hl * pow_rel;
    wi = w;
    xi = x;

    // inner bar
    nvgBeginPath(s->vg);
    nvgRect(s->vg, xi, y_mid + y_offset, wi, hi);
    nvgFillColor(s->vg, nvgRGBA(0,230,27,inner_fill_alpha)); // green
    nvgFill(s->vg);
    // outer box for bar
    nvgBeginPath(s->vg);
    nvgRoundedRect(s->vg, xi, y_mid + y_offset, wi, hl, rect_r);
    nvgFillColor(s->vg, nvgRGBA(0,230,27,outer_fill_alpha));
    nvgFill(s->vg);

    pow_rel_max = MAX(pow_rel, pow_rel_max);


    // draw outer border here so that brakes can overwrite it
    nvgBeginPath(s->vg);
    nvgRoundedRect(s->vg, outer_rect.x, outer_rect.y, outer_rect.w, outer_rect.h, rect_r);
    nvgStrokeWidth(s->vg, 5);
    nvgStrokeColor(s->vg, COLOR_WHITE_ALPHA(160));
    nvgStroke(s->vg);
    
    // brake power
    pow_rel = (s->scene.brake_percent >= 51 ? float(s->scene.brake_percent - 51) * 0.02 : 0.);
    int pow_rel2 = s->scene.power_cur[2] / s->scene.power_max[2];
    if (pow_rel2 > pow_rel){
      pow_rel = pow_rel2;
    }
    hi = hl * pow_rel;
    wi = w;
    xi += w;

    // inner bar
    nvgBeginPath(s->vg);
    nvgRect(s->vg, xi, y_mid + y_offset, wi, hi);
    nvgFillColor(s->vg, nvgRGBA(255,21,0,inner_fill_alpha)); // red
    nvgFill(s->vg);
    // outer box for bar
    nvgBeginPath(s->vg);
    nvgRoundedRect(s->vg, xi, y_mid + y_offset, wi, hl, rect_r);
    nvgFillColor(s->vg, nvgRGBA(255,21,0,outer_fill_alpha + int(pow_rel > 0.) * 40.));
    nvgFill(s->vg);
    if (pow_rel > 0.){
      nvgStrokeWidth(s->vg, 5);
      nvgStrokeColor(s->vg, nvgRGBA(255,21,0,180));
      nvgStroke(s->vg);
    }

    // middle bar between +/- power
    nvgBeginPath(s->vg);
    nvgRect(s->vg, outer_rect.x, y_mid-8, outer_rect.w, 16);
    nvgFillColor(s->vg, COLOR_WHITE_ALPHA(200));
    nvgFill(s->vg);

    pow_rel_max = MAX(pow_rel, pow_rel_max);

    float pow = s->scene.car_state.getDrivePower();


    // number at bottom
    if (s->scene.power_meter_mode == 1 && !alert_offset){
      char val[16], unit[8];
      if (s->scene.car_is_ev){
        float batt_pow = -s->scene.car_state.getHvbWattage();
        if (abs(pow) < abs(batt_pow)){
         pow = batt_pow;
        }
      }
      s->scene.power_meter_pow = s->scene.power_meter_ema_k * pow + (1. - s->scene.power_meter_ema_k) * pow;
      pow = s->scene.power_meter_pow;
      pow *= 1e-3;
      if (!s->scene.power_meter_metric){
        pow *= 1.34; // kW to hp
        snprintf(unit, sizeof(unit), "hp"); 
      } 
      else {
        snprintf(unit, sizeof(unit), "kW"); 
      }
      nvgFillColor(s->vg, COLOR_WHITE_ALPHA(180));
      nvgFontFace(s->vg, "sans-semibold");
      snprintf(val, sizeof(val), "%.0f%s", pow, unit);
      nvgTextAlign(s->vg, NVG_ALIGN_RIGHT | NVG_ALIGN_TOP);
      nvgFontSize(s->vg, 100);
      nvgText(s->vg, outer_rect.right(), y + 5,val,NULL);
      s->scene.power_meter_text_rect = {outer_rect.x - 200, y, 1000, 1000};
    }
    else{
      s->scene.power_meter_text_rect = {1,1,1,1};
    }
  }
}

static void ui_draw_vision_brake(UIState *s) {
  if (s->scene.brake_percent >= 0){
    // scene.brake_percent in [0,50] is engine/regen
    // scene.brake_percent in [51,100] is friction
    int brake_x = s->fb_w - face_wheel_radius - bdr_s * 2;
    int brake_y = s->fb_h - footer_h / 2;
    brake_x = offset_right_side_button_x(s, brake_x, brake_size);
    brake_y = offset_button_y(s, brake_y, brake_size);
    const int brake_r1 = 1;
    const int brake_r2 = brake_size / 3 + 2;
    const float brake_r_range = brake_r2 - brake_r1;
    const int circ_offset = 1;
    float bg_alpha = 0.1 + 0.3 * s->scene.brake_indicator_alpha;
    float img_alpha = 0.15 + 0.85 * s->scene.brake_indicator_alpha;
    if (s->scene.brake_percent > 0 && s->scene.brake_percent <= 50){
      // engine/regen braking indicator only
      int bp = s->scene.brake_percent * 2;
      float p = bp;
      const int brake_r = brake_r1 + int(brake_r_range * p * 0.01);
      bg_alpha = (0.1 + (p * 0.004));
      if (bg_alpha > 0.3){
        bg_alpha = 0.3;
      }
      ui_draw_circle_image(s, brake_x, brake_y, brake_size, "brake_disk", nvgRGBA(0, 0, 0, int(bg_alpha * 255.)), img_alpha);
      nvgBeginPath(s->vg);
      nvgRoundedRect(s->vg, brake_x - brake_r + circ_offset, brake_y - brake_r + circ_offset, 2 * brake_r, 2 * brake_r, brake_r);
      nvgStrokeWidth(s->vg, 9);
      NVGcolor nvg_color = nvgRGBA(131,232,42, 200);
      nvgFillColor(s->vg, nvg_color);
      nvgStrokeColor(s->vg, nvg_color);
      nvgFill(s->vg);
      nvgStroke(s->vg);
    }
    else if (s->scene.brake_percent > 50){
      int bp = (s->scene.brake_percent - 50) * 2;
      bg_alpha = 0.3 + 0.1 * s->scene.brake_indicator_alpha;
      NVGcolor color = nvgRGBA(0, 0, 0, (255 * bg_alpha));
      if (bp > 0 && bp <= 100){
        int r = 0;
        if (bp >= 50){
          float p = 0.01 * float(bp - 50);
          bg_alpha += 0.3 * p;
          r = 200. * p;
        }
        color = nvgRGBA(r, 0, 0, (255 * bg_alpha));
      }
      ui_draw_circle_image(s, brake_x, brake_y, brake_size, "brake_disk", color, img_alpha);
      if (bp <= 100){
        float p = bp;
        
        // friction braking indicator starts at outside of regen indicator and grows from there 
        // do this by increasing radius while decreasing stroke width.
        nvgBeginPath(s->vg);
        const int start_r = brake_r2 + 3;
        const int end_r = brake_size;
        const int brake_r = start_r + float(end_r - start_r) * p * 0.01;
        const int stroke_width = brake_r - brake_r2;
        const int path_r = stroke_width / 2 + brake_r2;
        nvgRoundedRect(s->vg, brake_x - path_r + circ_offset, brake_y - path_r + circ_offset, 2 * path_r, 2 * path_r, path_r);
        nvgStrokeWidth(s->vg, stroke_width);
        int r = 255, g = 255, b = 255, a = 200;
        p *= 0.01;
        g -= int(p * 255.);
        g = (g > 0 ? g : 0);
        b -= int((.4 + p) * 255.);
        b = (b > 0 ? b : 0); // goes from white to orange to red as p goes from 0 to 100
        nvgFillColor(s->vg, nvgRGBA(0,0,0,0));
        nvgStrokeColor(s->vg, nvgRGBA(r,g,b,a));
        nvgFill(s->vg);
        nvgStroke(s->vg);
        
        // another brake image (this way the regen is on top of the background, while the brake disc itself occludes the other indicator)
        ui_draw_circle_image(s, brake_x, brake_y, brake_size, "brake_disk", nvgRGBA(0,0,0,0), img_alpha);
        
        // engine/regen braking indicator
        nvgBeginPath(s->vg);
        nvgRoundedRect(s->vg, brake_x - brake_r2 + circ_offset, brake_y - brake_r2 + circ_offset, 2 * brake_r2, 2 * brake_r2, brake_r2);
        nvgStrokeWidth(s->vg, 9);
        NVGcolor nvg_color = nvgRGBA(131,232,42, 200);
        nvgFillColor(s->vg, nvg_color);
        nvgStrokeColor(s->vg, nvg_color);
        nvgFill(s->vg);
        nvgStroke(s->vg);
      }
    }
    else{
      ui_draw_circle_image(s, brake_x, brake_y, brake_size, "brake_disk", nvgRGBA(0, 0, 0, bg_alpha), img_alpha);
    }
    s->scene.brake_touch_rect = {brake_x - brake_size, brake_y - brake_size, 2 * brake_size, 2 * brake_size};
  }
}

static void draw_lane_pos_buttons(UIState *s) {
  if (s->vipc_client->connected && s->scene.lane_pos_enabled) {
    const int radius = (s->scene.controls_state.getAlertSize() == cereal::ControlsState::AlertSize::NONE && !(s->scene.map_open) ? 185 : 100);
    const int right_x = (s->scene.measure_cur_num_slots > 0 
                          ? s->scene.measure_slots_rect.x - 4 * radius / 3
                          : 4 * s->fb_w / 5);
    int left_x = s->fb_w / 5 + 100;
    if (left_x > right_x - 2 * radius - 40){
      left_x = right_x - 2 * radius - 40;
    }
    const int y = offset_button_y(s, s->fb_h / 2, radius);

    // left button
    s->scene.lane_pos_left_touch_rect = {left_x - radius, y - radius, 2 * radius, 2 * radius};
    int radius_inner = 0;
    if (s->scene.lane_pos == 1 && s->scene.lateral_plan.getLanePosition() == LanePosition::LEFT){
      radius_inner = s->scene.auto_lane_pos_active ? radius : int(float(s->scene.lane_pos_timeout_dist - s->scene.lane_pos_dist_since_set) / float(s->scene.lane_pos_timeout_dist) * float(radius));
      if (radius_inner < 1){
        radius_inner = 1;
      }
      nvgBeginPath(s->vg);
      nvgRoundedRect(s->vg, left_x - radius_inner, y - radius_inner, 2 * radius_inner, 2 * radius_inner, radius_inner);
      nvgFillColor(s->vg, s->scene.auto_lane_pos_active ? COLOR_GRACE_BLUE_ALPHA(100) : COLOR_WHITE_ALPHA(200));
      nvgFill(s->vg);
      ui_draw_circle_image(s, left_x, y, radius, "lane_pos_left", COLOR_BLACK_ALPHA(80), 1.0);
    }
    else{
      ui_draw_circle_image(s, left_x, y, radius, "lane_pos_left", COLOR_BLACK_ALPHA(80), 0.4);
    }
    
    if ((s->scene.lane_pos == 1 && s->scene.lateral_plan.getLanePosition() == LanePosition::LEFT) || s->scene.auto_lane_pos_active){
      // outline of button when active
      nvgBeginPath(s->vg);
      nvgRoundedRect(s->vg, left_x - radius, y - radius, 2 * radius, 2 * radius, radius);
      if (s->scene.auto_lane_pos_active){
        nvgStrokeWidth(s->vg, 30);
      }
      nvgStrokeColor(s->vg, s->scene.auto_lane_pos_active ? COLOR_GRACE_BLUE_ALPHA(100) : COLOR_WHITE_ALPHA(200));
      nvgStroke(s->vg);
    }
    
    // right button
    s->scene.lane_pos_right_touch_rect = {right_x - radius, y - radius, 2 * radius, 2 * radius};
    radius_inner = 0;
    if (s->scene.lane_pos == -1 && s->scene.lateral_plan.getLanePosition() == LanePosition::RIGHT){
      radius_inner = s->scene.auto_lane_pos_active ? radius : int(float(s->scene.lane_pos_timeout_dist - s->scene.lane_pos_dist_since_set) / float(s->scene.lane_pos_timeout_dist) * float(radius));
      if (radius_inner < 1){
        radius_inner = 1;
      }
      nvgBeginPath(s->vg);
      nvgRoundedRect(s->vg, right_x - radius_inner, y - radius_inner, 2 * radius_inner, 2 * radius_inner, radius_inner);
      nvgFillColor(s->vg, s->scene.auto_lane_pos_active ? COLOR_GRACE_BLUE_ALPHA(100) : COLOR_WHITE_ALPHA(200));
      nvgFill(s->vg);
      ui_draw_circle_image(s, right_x, y, radius, "lane_pos_right", COLOR_BLACK_ALPHA(80), 1.0);
    }
    else{
      ui_draw_circle_image(s, right_x, y, radius, "lane_pos_right", COLOR_BLACK_ALPHA(80), 0.4);
    }
    if ((s->scene.lane_pos == -1 && s->scene.lateral_plan.getLanePosition() == LanePosition::RIGHT) || s->scene.auto_lane_pos_active){
      // outline of button when active
      nvgBeginPath(s->vg);
      nvgRoundedRect(s->vg, right_x - radius, y - radius, 2 * radius, 2 * radius, radius);
      if (s->scene.auto_lane_pos_active){
        nvgStrokeWidth(s->vg, 30);
      }
      nvgStrokeColor(s->vg, s->scene.auto_lane_pos_active ? COLOR_GRACE_BLUE_ALPHA(100) : COLOR_WHITE_ALPHA(200));
      nvgStroke(s->vg);
    }

    
  }
}

static void draw_accel_mode_button(UIState *s) {
  if (s->vipc_client->connected && s->scene.accel_mode_button_enabled) {
    const int radius = 72;
    int center_x = s->fb_w - face_wheel_radius - bdr_s * 2;
    if (s->scene.brake_percent >= 0){
      if (s->scene.power_meter_mode == 2){
        center_x -= brake_size + (s->scene.map_open ? 1.2 : 3) * bdr_s + radius;
      }
      else{
        if (s->scene.power_meter_mode == 0){
          center_x -= s->fb_w * 8 / 128;
        }
        else{
          center_x -= s->fb_w * 11 / 128;
        }
      }
    }
    int center_y = s->fb_h - footer_h / 2 - radius / 2;
    center_y = offset_button_y(s, center_y, radius);
    center_x = offset_right_side_button_x(s, center_x, radius);
    int btn_w = radius * 2;
    int btn_h = radius * 2;
    int btn_x1 = center_x - 0.5 * radius;
    int btn_y = center_y - 0.5 * radius;
    int btn_xc1 = btn_x1 + radius;
    int btn_yc = btn_y + radius;
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
    nvgBeginPath(s->vg);
    nvgRoundedRect(s->vg, btn_x1, btn_y, btn_w, btn_h, radius);
    // nvgRoundedRect(s->vg, btn_x1, btn_y, btn_w, btn_h, 100);
    nvgStrokeColor(s->vg, nvgRGBA(0,0,0,80));
    nvgStrokeWidth(s->vg, 6);
    nvgStroke(s->vg);
    nvgFontSize(s->vg, 52);

    if (s->scene.accel_mode == 0) { // normal
      nvgStrokeColor(s->vg, nvgRGBA(200,200,200,200));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
      NVGcolor fillColor = nvgRGBA(0,0,0,80);
      nvgFillColor(s->vg, fillColor);
      nvgFill(s->vg);
      nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
      nvgText(s->vg,btn_xc1,btn_yc-20,"Stock",NULL);
      nvgText(s->vg,btn_xc1,btn_yc+20,"accel",NULL);
    } else if (s->scene.accel_mode == 1) { // sport
      nvgStrokeColor(s->vg, interp_alert_color(2.,255));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
      NVGcolor fillColor = interp_alert_color(2.,80);
      nvgFillColor(s->vg, fillColor);
      nvgFill(s->vg);
      nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
      nvgText(s->vg,btn_xc1,btn_yc-20,"Sport",NULL);
      nvgText(s->vg,btn_xc1,btn_yc+20,"accel",NULL);
    } else if (s->scene.accel_mode == 2) { // eco
      nvgStrokeColor(s->vg, interp_alert_color(-1.,255));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
      NVGcolor fillColor = interp_alert_color(-1.,80);
      nvgFillColor(s->vg, fillColor);
      nvgFill(s->vg);
      nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
      nvgText(s->vg,btn_xc1,btn_yc-20,"Eco",NULL);
      nvgText(s->vg,btn_xc1,btn_yc+20,"accel",NULL);
    }
    
    
    s->scene.accel_mode_touch_rect = Rect{center_x - laneless_btn_touch_pad, 
                                                center_y - laneless_btn_touch_pad,
                                                radius + 2 * laneless_btn_touch_pad, 
                                                radius + 2 * laneless_btn_touch_pad}; 
  }
}

static void draw_weather(UIState *s){
  if (s->scene.weather_info.enabled == 0){
    return;
  }
  auto const & w = s->scene.weather_info;
  const Rect max_speed_rect = {bdr_s * 2, int(bdr_s * 1.5), 184, 202};
  if (w.display_mode == 0 || !w.valid || s->scene.map_open){
    Rect icon_rect = {max_speed_rect.right() + 20, max_speed_rect.y - 40, 180, 180};
    if (w.valid){
      ui_draw_image(s, icon_rect, w.icon, 1.0);
    }
    else{
      ui_draw_image(s, icon_rect, "weather_load", 1.0);
    }
    nvgBeginPath(s->vg);
    nvgFontSize(s->vg, 80);
    nvgStrokeColor(s->vg, COLOR_WHITE);
    nvgFillColor(s->vg, COLOR_WHITE);
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
    nvgText(s->vg, icon_rect.centerX(), icon_rect.bottom(), w.valid ? w.desc_simple : "", NULL);
    nvgFontSize(s->vg, 60);
    nvgText(s->vg, icon_rect.centerX(), icon_rect.bottom() + 50, w.desc_simple1, NULL);
    nvgText(s->vg, icon_rect.centerX(), icon_rect.bottom() + 100, w.desc_simple2, NULL);
    s->scene.weather_touch_rect = {icon_rect.x, icon_rect.y, icon_rect.w, icon_rect.h + 100};
  }
  else{
    const int x_offset = (s->scene.controls_state.getActive() || !s->scene.controls_state.getMadsEnabled()) ? 20 : 40;
    Rect icon_rect = {max_speed_rect.right() + x_offset, max_speed_rect.y, 100, 100};
    ui_draw_image(s, icon_rect, w.icon, 1.0);
    nvgBeginPath(s->vg);
    nvgStrokeColor(s->vg, COLOR_WHITE);
    nvgFillColor(s->vg, COLOR_WHITE);
    nvgFontSize(s->vg, 70);
    nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);
    nvgText(s->vg, icon_rect.right(), icon_rect.centerY(), w.desc_full1,NULL);
    nvgFontSize(s->vg, 55);
    nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_TOP);
    int y = 0, dy = 50;
    nvgText(s->vg, icon_rect.x + icon_rect.w / 5, icon_rect.bottom() - 20 + y, w.desc_full2,NULL);
    y += dy;
    if (w.has_precip){
      nvgText(s->vg, icon_rect.x + icon_rect.w / 5, icon_rect.bottom() - 20 + y, w.desc_full3,NULL);
      y += dy;
    }
    nvgText(s->vg, icon_rect.x + icon_rect.w / 5, icon_rect.bottom() - 20 + y, w.desc_full4,NULL);
    y += dy;
    s->scene.weather_touch_rect = {icon_rect.x, icon_rect.y, int(icon_rect.w * 2.5), int(icon_rect.h * 2.5)};
  }
}

static void draw_dynamic_follow_mode_button(UIState *s) {
  if (s->vipc_client->connected && s->scene.dynamic_follow_mode_button_enabled) {
    const int radius = 72;
    int center_x = s->fb_w - face_wheel_radius - bdr_s * 2;
    if (s->scene.brake_percent >= 0){
      if (s->scene.power_meter_mode == 2){
        center_x -= brake_size + (s->scene.map_open ? 1.2 : 3) * bdr_s + radius;
      }
      else{
        if (s->scene.power_meter_mode == 0){
          center_x -= s->fb_w * 8 / 128;
        }
        else{
          center_x -= s->fb_w * 11 / 128;
        }
      }
    }
    if (s->scene.accel_mode_button_enabled){
      center_x -= (s->scene.map_open ? 1.2 : 2) * bdr_s + 2 * radius;
    }
    int center_y = s->fb_h - footer_h / 2 - radius / 2;
    center_y = offset_button_y(s, center_y, radius);
    center_x = offset_right_side_button_x(s, center_x, radius);
    int btn_w = radius * 2;
    int btn_h = radius * 2;
    int btn_x1 = center_x - 0.5 * radius;
    int btn_y = center_y - 0.5 * radius;
    int btn_xc1 = btn_x1 + radius;
    int btn_yc = btn_y + radius;
    float df_level = s->scene.dynamic_follow_level_ui >= 0. ? s->scene.dynamic_follow_level_ui : 0.;
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
    nvgBeginPath(s->vg);
    nvgRoundedRect(s->vg, btn_x1, btn_y, btn_w, btn_h, radius);
    // nvgRoundedRect(s->vg, btn_x1, btn_y, btn_w, btn_h, 100);
    const bool df_active = s->scene.dynamic_follow_active && s->scene.controls_state.getActive();
    if (df_active){
      int r, g, b;
      int bg_r, bg_g, bg_b;
      for (int i = 1; i < 3; ++i){
        if (df_level <= i){
          float c = float(i) - df_level;
          r = float(s->scene.dynamic_follow_r[i-1]) * c + float(s->scene.dynamic_follow_r[i]) * (1. - c);
          g = float(s->scene.dynamic_follow_g[i-1]) * c + float(s->scene.dynamic_follow_g[i]) * (1. - c);
          b = float(s->scene.dynamic_follow_b[i-1]) * c + float(s->scene.dynamic_follow_b[i]) * (1. - c);
          bg_r = float(s->scene.dynamic_follow_bg_r[i-1]) * c + float(s->scene.dynamic_follow_bg_r[i]) * (1. - c);
          bg_g = float(s->scene.dynamic_follow_bg_g[i-1]) * c + float(s->scene.dynamic_follow_bg_g[i]) * (1. - c);
          bg_b = float(s->scene.dynamic_follow_bg_b[i-1]) * c + float(s->scene.dynamic_follow_bg_b[i]) * (1. - c);
          break;
        }
      }
      nvgStrokeColor(s->vg, nvgRGBA(r,g,b,255));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
      nvgFillColor(s->vg, nvgRGBA(bg_r,bg_g,bg_b,80));
      nvgFill(s->vg);
    } else{
      nvgStrokeColor(s->vg, nvgRGBA(0,0,0,80));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
      nvgStrokeColor(s->vg, nvgRGBA(200,200,200,80));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
      nvgFillColor(s->vg, nvgRGBA(0,0,0,80));
      nvgFill(s->vg);
    }
    
    // draw the three follow level strings. adjust alpha and y position to create a rolling effect
    const float dscale = 0.5;
    for (int i = 0; i < 3; ++i){
      char val[16];
      snprintf(val, sizeof(val), "%s", s->scene.dynamic_follow_strs[i].c_str());
      float alpha_f = abs(float(i) - df_level);
      alpha_f = (alpha_f > 1. ? 1. : alpha_f) * 1.5707963268;
      nvgFillColor(s->vg, nvgRGBA(255, 255, 255, int(cos(alpha_f) * (df_active ? 200. : 80.))));
                                
      nvgFontSize(s->vg, 40 + int(cos(alpha_f * 1.5707963268) * 16.));
      
      int text_y = btn_yc;
      if (df_level <= i){
        text_y -= float(radius) * sin(alpha_f) * dscale;
      }
      else{
        text_y += float(radius) * sin(alpha_f) * dscale;
      }
      nvgText(s->vg, btn_xc1, text_y, val, NULL);
    }
    
    s->scene.dynamic_follow_mode_touch_rect = Rect{center_x - laneless_btn_touch_pad, 
                                                center_y - laneless_btn_touch_pad,
                                                radius + 2 * laneless_btn_touch_pad, 
                                                radius + 2 * laneless_btn_touch_pad}; 
  }
}

static void draw_laneless_button(UIState *s) {
  if (s->vipc_client->connected) {
    const Rect maxspeed_rect = {bdr_s * 2, int(bdr_s * 1.5), 184, 202};
    const int vision_face_radius = 96;
    const int radius = 72;
    const int center_x = maxspeed_rect.centerX() + vision_face_radius + bdr_s * (s->scene.map_open ? 0 : 1) + radius;
    int center_y = s->fb_h - footer_h / 2 - radius / 2;
    center_y = offset_button_y(s, center_y, radius);
    int btn_w = radius * 2;
    int btn_h = radius * 2;
    int btn_x1 = center_x - 0.5 * radius;
    int btn_y = center_y - 0.5 * radius;
    int btn_xc1 = btn_x1 + radius;
    int btn_yc = btn_y + radius;
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
    nvgBeginPath(s->vg);
    nvgRoundedRect(s->vg, btn_x1, btn_y, btn_w, btn_h, radius);
    // nvgRoundedRect(s->vg, btn_x1, btn_y, btn_w, btn_h, 100);
    nvgStrokeColor(s->vg, nvgRGBA(0,0,0,80));
    nvgStrokeWidth(s->vg, 6);
    nvgStroke(s->vg);
    nvgFontSize(s->vg, 54);

    if (s->scene.laneless_mode == 0) {
      nvgStrokeColor(s->vg, (s->scene.alt_engage_color_enabled ? interp_alert_color(0.5,255): nvgRGBA(0,125,0,255)));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
      NVGcolor fillColor = (s->scene.alt_engage_color_enabled ? interp_alert_color(0.5,255): nvgRGBA(0,125,0,80));
      nvgFillColor(s->vg, fillColor);
      nvgFill(s->vg);
      nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
      nvgText(s->vg,btn_xc1,btn_yc-20,"Lane",NULL);
      nvgText(s->vg,btn_xc1,btn_yc+20,"only",NULL);
    } else if (s->scene.laneless_mode == 1) {
      nvgStrokeColor(s->vg, (s->scene.alt_engage_color_enabled ? interp_alert_color(-1.,255): nvgRGBA(0,100,255,255)));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
      NVGcolor fillColor = (s->scene.alt_engage_color_enabled ? interp_alert_color(-1.,255): nvgRGBA(0,100,255,255));
      nvgFillColor(s->vg, fillColor);
      nvgFill(s->vg);
      nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
      nvgText(s->vg,btn_xc1,btn_yc-20,"Lane",NULL);
      nvgText(s->vg,btn_xc1,btn_yc+20,"less",NULL);
    } else if (s->scene.laneless_mode == 2) {
      nvgStrokeColor(s->vg, nvgRGBA(200,200,200,255));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
      NVGcolor fillColor = nvgRGBA(0,0,0,80);
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
  ui_draw_vision_speed(s);
  if (s->scene.controls_state.getAlertSize() == cereal::ControlsState::AlertSize::NONE){
    ui_draw_vision_turnspeed(s);
    ui_draw_vision_speedlimit(s);
  }
  else if (s->scene.controls_state.getAlertSize() == cereal::ControlsState::AlertSize::SMALL){
    ui_draw_vision_speedlimit(s);
  }
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
  if (s->scene.controls_state.getAlertSize() == cereal::ControlsState::AlertSize::NONE
  || s->scene.controls_state.getAlertSize() == cereal::ControlsState::AlertSize::SMALL) {
    ui_draw_vision_face(s);
    if (s->scene.power_meter_mode < 2){
      ui_draw_vision_power_meter(s);
    }
    else{
      ui_draw_vision_brake(s);
      s->scene.power_meter_rect = {s->fb_w * 125 / 128, 1, 1, 1};
    }
    if (!s->scene.map_open || s->scene.controls_state.getAlertSize() == cereal::ControlsState::AlertSize::NONE){
      ui_draw_measures(s);
    }
    draw_weather(s);
  }
  else if (s->scene.controls_state.getAlertSize() == cereal::ControlsState::AlertSize::MID) {
    ui_draw_vision_face(s);
    draw_weather(s);
    if (s->scene.power_meter_mode < 2){
      ui_draw_vision_power_meter(s);
    }
    else{
      ui_draw_vision_brake(s);
      s->scene.power_meter_rect = {s->fb_w * 125 / 128, 1, 1, 1};
    }
  }
  if (s->scene.lane_pos_enabled){
    draw_lane_pos_buttons(s);
  }
  if (s->scene.end_to_end) {
    draw_laneless_button(s);
  }
  if (s->scene.accel_mode_button_enabled){
    draw_accel_mode_button(s);
  }
  if (s->scene.dynamic_follow_mode_button_enabled){
    draw_dynamic_follow_mode_button(s);
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
  std::vector<std::pair<const char *, const char *> > images = {
    {"eye", "../assets/img_eye_open_white.png"},
    {"wheel", "../assets/img_chffr_wheel.png"},
    {"driver_face", "../assets/img_driver_face.png"},
    {"hands_on_wheel", "../assets/img_hands_on_wheel.png"},
    {"turn_left_icon", "../assets/img_turn_left_icon.png"},
    {"turn_right_icon", "../assets/img_turn_right_icon.png"},
    {"map_source_icon", "../assets/img_world_icon.png"},
    {"brake_disk", "../assets/img_brake.png"},
    {"one_pedal_mode", "../assets/offroad/icon_car_pedal.png"},
    {"MADS", "../assets/offroad/icon_car_MADS.png"},
    {"lane_pos_left", "../assets/offroad/icon_lane_pos_left.png"},
    {"lane_pos_right", "../assets/offroad/icon_lane_pos_right.png"},
    {"01d", "../assets/weather/01d.png"},
    {"02d", "../assets/weather/02d.png"},
    {"03d", "../assets/weather/03d.png"},
    {"04d", "../assets/weather/04d.png"},
    {"09d", "../assets/weather/09d.png"},
    {"10d", "../assets/weather/10d.png"},
    {"11d", "../assets/weather/11d.png"},
    {"13d", "../assets/weather/13d.png"},
    {"50d", "../assets/weather/50d.png"},
    {"01n", "../assets/weather/01n.png"},
    {"02n", "../assets/weather/02n.png"},
    {"03n", "../assets/weather/03n.png"},
    {"04n", "../assets/weather/04n.png"},
    {"09n", "../assets/weather/09n.png"},
    {"10n", "../assets/weather/10n.png"},
    {"11n", "../assets/weather/11n.png"},
    {"13n", "../assets/weather/13n.png"},
    {"50n", "../assets/weather/50n.png"},
    {"weather_load", "../assets/weather/weatherload.png"}
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
