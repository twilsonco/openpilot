// board enforces
//   in-state
//      accel set/resume
//   out-state
//      cancel button
//      regen paddle
//      accel rising edge
//      brake rising edge
//      brake > 0mph

//const int GM_PARAM_HIGH_TORQUE = 1;
//const int GM_PARAM_ENABLE_FWD = 2;

typedef struct GM_LIMIT {
  const int GM_MAX_STEER;
  const int GM_MAX_RT_DELTA;
  const int GM_MAX_RATE_UP;
  const int GM_MAX_RATE_DOWN;
  const int GM_DRIVER_TORQUE_ALLOWANCE;
  const int GM_DRIVER_TORQUE_FACTOR;
  const int GM_MAX_GAS;
  const int GM_MAX_REGEN;
  const int GM_MAX_BRAKE;
}GM_LIMIT;

const GM_LIMIT GM_LIMITS[] =
{
  { // safety param 0 - Default
    .GM_MAX_STEER = 300,
    .GM_MAX_RT_DELTA = 256,
    .GM_MAX_RATE_UP = 36, 
    .GM_MAX_RATE_DOWN = 36,
    .GM_DRIVER_TORQUE_ALLOWANCE = 100,
    .GM_DRIVER_TORQUE_FACTOR = 4,
    .GM_MAX_GAS = 4095,
    .GM_MAX_REGEN = 1404,
    .GM_MAX_BRAKE = 500
  },
  { // safety param 1 - Trucks
    .GM_MAX_STEER = 600,
    .GM_MAX_RT_DELTA = 319,
    .GM_MAX_RATE_UP = 36, 
    .GM_MAX_RATE_DOWN = 36,
    .GM_DRIVER_TORQUE_ALLOWANCE = 100,
    .GM_DRIVER_TORQUE_FACTOR = 4,
    .GM_MAX_GAS = 4095,
    .GM_MAX_REGEN = 1404,
    .GM_MAX_BRAKE = 500
  },
};

int gm_safety_param = 0;
int gm_good_cam_cnt = 0;
bool gm_allow_fwd = true;
bool gm_block_fwd = false;
int gm_camera_bus = 2;
bool gm_has_relay = true;
uint32_t gm_start_ts = 0;
uint32_t gm_last_lkas_ts = 0;

#define GM_MAX_STEER (GM_LIMITS[gm_safety_param].GM_MAX_STEER)
#define GM_MAX_RT_DELTA (GM_LIMITS[gm_safety_param].GM_MAX_RT_DELTA)
#define GM_MAX_RATE_UP (GM_LIMITS[gm_safety_param].GM_MAX_RATE_UP)
#define GM_MAX_RATE_DOWN (GM_LIMITS[gm_safety_param].GM_MAX_RATE_DOWN)
#define GM_DRIVER_TORQUE_ALLOWANCE (GM_LIMITS[gm_safety_param].GM_DRIVER_TORQUE_ALLOWANCE)
#define GM_DRIVER_TORQUE_FACTOR (GM_LIMITS[gm_safety_param].GM_DRIVER_TORQUE_FACTOR)
#define GM_MAX_GAS (GM_LIMITS[gm_safety_param].GM_MAX_GAS)
#define GM_MAX_REGEN (GM_LIMITS[gm_safety_param].GM_MAX_REGEN)
#define GM_MAX_BRAKE (GM_LIMITS[gm_safety_param].GM_MAX_BRAKE)

const uint32_t GM_RT_INTERVAL = 250000;    // 250ms between real time checks
const int GM_GAS_INTERCEPTOR_THRESHOLD = 458;  // (610 + 306.25) / 2ratio between offset and gain from dbc file
#define GM_GET_INTERCEPTOR(msg) (((GET_BYTE((msg), 0) << 8) + GET_BYTE((msg), 1) + (GET_BYTE((msg), 2) << 8) + GET_BYTE((msg), 3)) / 2) // avg between 2 tracks

const CanMsg GM_TX_MSGS[] = {{384, 0, 4}, {1033, 0, 7}, {1034, 0, 7}, {715, 0, 8}, {880, 0, 6}, {512, 0, 6}, {789, 0, 5}, {800, 0, 6},  // pt bus
                             {161, 1, 7}, {774, 1, 8}, {776, 1, 7}, {784, 1, 2},   // obs bus
                             {789, 2, 5},  // ch bus
                             {0x104c006c, 3, 3}, {0x10400060, 3, 5}};  // gmlan

// TODO: do checksum and counter checks. Add correct timestep, 0.1s for now.
AddrCheckStruct gm_addr_checks[] = {
  {.msg = {{388, 0, 8, .expected_timestep = 100000U}, { 0 }, { 0 }}},
  {.msg = {{842, 0, 5, .expected_timestep = 100000U}, { 0 }, { 0 }}},
  {.msg = {{481, 0, 7, .expected_timestep = 100000U}, { 0 }, { 0 }}},
  {.msg = {{241, 0, 6, .expected_timestep = 100000U}, { 0 }, { 0 }}},
  {.msg = {{452, 0, 8, .expected_timestep = 100000U}, { 0 }, { 0 }}},
};
#define GM_RX_CHECK_LEN (sizeof(gm_addr_checks) / sizeof(gm_addr_checks[0]))
addr_checks gm_rx_checks = {gm_addr_checks, GM_RX_CHECK_LEN};

static int gm_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {

  bool valid = addr_safety_check(to_push, &gm_rx_checks, NULL, NULL, NULL);

  if (valid && (GET_BUS(to_push) == 0U)) {
    int addr = GET_ADDR(to_push);

    if (addr == 388) {
      int torque_driver_new = ((GET_BYTE(to_push, 6) & 0x7U) << 8) | GET_BYTE(to_push, 7);
      torque_driver_new = to_signed(torque_driver_new, 11);
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    // sample speed, really only care if car is moving or not
    // rear left wheel speed
    if (addr == 842) {
      vehicle_moving = GET_BYTE(to_push, 0) | GET_BYTE(to_push, 1);
    }

    //TODO: Should we be checking the CC status?

    // ACC steering wheel buttons
    if (addr == 481) {
      // lka button
      bool lfa_pressed = (GET_BYTE(to_push, 2) & 0x80U); // LKA on signal
      if (lfa_pressed && !lfa_pressed_prev)
      {
        controls_allowed = 1;
        lateral_allowed = 1;
      }
      lfa_pressed_prev = lfa_pressed;

      // acc buttons
      int button = (GET_BYTE(to_push, 5) & 0x70U) >> 4;
      switch (button) {
        case 2:  // resume
        case 3:  // set
          controls_allowed = 1;
          lateral_allowed = 1;
          break;
        case 6:  // cancel
          controls_allowed = 0;
          lateral_allowed = 1;
          break;
        default:
          break;  // any other button is irrelevant
      }
    }

    if (addr == 201) {
      bool acc_main_on = (GET_BYTE(to_push, 3) & 0x20U); // ACC main_on signal
      if (acc_main_on && !acc_main_on_prev)
      {
        controls_allowed = 1;
        lateral_allowed = 1;
      }
      else if (acc_main_on_prev != acc_main_on)
      {
        disengageFromBrakes = false;
        controls_allowed = 0;
        lateral_allowed = 0;
      }
      acc_main_on_prev = acc_main_on;
    }

    if (addr == 241) {
       // Brake pedal's potentiometer returns near-zero reading
       // even when pedal is not pressed
       brake_pressed = GET_BYTE(to_push, 1) >= 15U;
    }

    if (addr == 452) {
      gas_pressed = GET_BYTE(to_push, 5) >= 40U;
      gas_pressed_slight = GET_BYTE(to_push, 5) != 0U;
    }

    // exit controls on regen paddle if controls allowed
    //TODO: Evaluate impact of this change. Previous method could have caused controls mismatch...
    if (addr == 189) {
      brake_pressed = controls_allowed && GET_BYTE(to_push, 0) & 0x20U;
      // if (regen) {
      //   controls_allowed = 0;
      // }
    }

    // Pedal Interceptor
    if (addr == 513) {
      gas_interceptor_detected = 1;
      int gas_interceptor = GM_GET_INTERCEPTOR(to_push);
      gas_pressed = gas_interceptor > GM_GAS_INTERCEPTOR_THRESHOLD;
      gas_interceptor_prev = gas_interceptor;
    }

    // Check if ASCM or LKA camera are online
    // on powertrain bus.
    // 384 = ASCMLKASteeringCmd
    // 715 = ASCMGasRegenCmd
    //generic_rx_checks(((addr == 384) || (addr == 715)));
    generic_rx_checks(addr == 384);
    //TODO: relay malfunction firing when 715 is stock
  }
  return valid;
}

// all commands: gas/regen, friction brake and steering
// if controls_allowed and no pedals pressed
//     allow all commands up to limit
// else
//     block all commands that produce actuation

static int gm_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {

  int tx = 1;
  int addr = GET_ADDR(to_send);

  if (!msg_allowed(to_send, GM_TX_MSGS, sizeof(GM_TX_MSGS)/sizeof(GM_TX_MSGS[0]))) {
    tx = 0;
  }

  // disallow actuator commands if gas or brake (with vehicle moving) are pressed
  // and the the latching controls_allowed flag is True
  int pedal_pressed = brake_pressed_prev && vehicle_moving;
  bool unsafe_allow_gas = unsafe_mode & UNSAFE_DISABLE_DISENGAGE_ON_GAS;
  if (!unsafe_allow_gas) {
    pedal_pressed = pedal_pressed || gas_pressed_prev;
  }
  bool current_controls_allowed = controls_allowed && !pedal_pressed;

  // GAS: safety check (interceptor)
  if (addr == 512) {
    if (!current_controls_allowed) {
      if (GET_BYTE(to_send, 0) || GET_BYTE(to_send, 1)) {
        tx = 0;
      }
    }
  }

  // BRAKE: safety check
  if (addr == 789) {
    int brake = ((GET_BYTE(to_send, 0) & 0xFU) << 8) + GET_BYTE(to_send, 1);
    brake = (0x1000 - brake) & 0xFFF;
    if (!(current_controls_allowed || lateral_allowed) || gas_pressed_slight) {
      if (brake != 0) {
        tx = 0;
      }
    }
    if (brake > GM_MAX_BRAKE) {
      tx = 0;
    }
  }

  // LKA STEER: safety check
  if (addr == 384) {
    //int rolling_counter = GET_BYTE(to_send, 0) >> 4;
    int desired_torque = ((GET_BYTE(to_send, 0) & 0x7U) << 8) + GET_BYTE(to_send, 1);
    uint32_t ts = microsecond_timer_get();
    bool violation = 0;
    desired_torque = to_signed(desired_torque, 11);

    if (current_controls_allowed || lateral_allowed) {

      // *** global torque limit check ***
      violation |= max_limit_check(desired_torque, GM_MAX_STEER, -GM_MAX_STEER);

      // *** torque rate limit check ***
      violation |= driver_limit_check(desired_torque, desired_torque_last, &torque_driver,
        GM_MAX_STEER, GM_MAX_RATE_UP, GM_MAX_RATE_DOWN,
        GM_DRIVER_TORQUE_ALLOWANCE, GM_DRIVER_TORQUE_FACTOR);

      // used next time
      desired_torque_last = desired_torque;

      // TODO: JJS: Reenable after finding better numbers
      // // *** torque real time rate limit check ***
      // violation |= rt_rate_limit_check(desired_torque, rt_torque_last, GM_MAX_RT_DELTA);

      // // every RT_INTERVAL set the new limits
      // uint32_t ts_elapsed = get_ts_elapsed(ts, ts_last);
      // if (ts_elapsed > GM_RT_INTERVAL) {
      //   rt_torque_last = desired_torque;
      //   ts_last = ts;
      // }
    }

    // no torque if controls is not allowed
    if (!(current_controls_allowed || lateral_allowed) && (desired_torque != 0)) {
      violation = 1;
    }

    // reset to 0 if either controls is not allowed or there's a violation
    if (violation || !(current_controls_allowed || lateral_allowed)) {
      desired_torque_last = 0;
      rt_torque_last = 0;
      ts_last = ts;
    }

    if (violation) {
      tx = 0;
    }

    //Last chance to catch too-soon frame
    if (tx == 1) {
      uint32_t ts2 = microsecond_timer_get();
      uint32_t ts_elapsed = get_ts_elapsed(ts2, gm_last_lkas_ts);
      if (ts_elapsed <= 13000) { // Should be every 20ms, but it seems to tolerate down lower
        // Hard 20ms cutoff was dropping WAY too many frames
        tx = 0;
      }
      else {
        gm_last_lkas_ts = ts2;
      }
    }

  }

  // GAS/REGEN: safety check
  if (addr == 715) {
    int gas_regen = ((GET_BYTE(to_send, 2) & 0x7FU) << 5) + ((GET_BYTE(to_send, 3) & 0xF8U) >> 3);
    // Disabled message is !engaged with gas
    // value that corresponds to max regen.
    if (!current_controls_allowed) {
      // Stock ECU sends max regen when not enabled
      if (gas_regen != GM_MAX_REGEN) {
        tx = 0;
      }
    }
    if (!controls_allowed) {
      bool apply = GET_BYTE(to_send, 0) & 1U;
      if (apply) {
        tx = 0;
      }
    }
    if (gas_regen > GM_MAX_GAS) {
      tx = 0;
    }
  }

  // 1 allows the message through
  return tx;
}


static void gm_validate_camera(int addr, CAN_FIFOMailBox_TypeDef *to_fwd) {
  //TODO: Add more known good camera message ids
  if (addr == 384) {
    int len = GET_LEN(to_fwd);
    if (len == 4) {
      gm_allow_fwd = true;
      gm_good_cam_cnt++;
    }
    else {
      gm_good_cam_cnt = 0;
      gm_block_fwd = true;
    }
  }
  else if ((addr == 1120) // F_LRR_Obj_Header from object bus
        || (addr == 784) // ASCMHeadlight from object bus
        || (addr == 309) // LHT_CameraObjConfirmation_FO from object bus
        || (addr == 192) // Unknown id only on chassis bus
  ) {
    gm_good_cam_cnt = 0;
    gm_block_fwd = true;
  }

  // If we are forwarding, but we have seen no LKAS frames on cam bus for 3 seconds, stop forwarding!
  if (gm_allow_fwd && (gm_good_cam_cnt <= 0)) {
    uint32_t ts = microsecond_timer_get();
    uint32_t ts_elapsed = get_ts_elapsed(ts, gm_start_ts);
    if (ts_elapsed > 3000000) {
      gm_allow_fwd = false;
    }
  }

}

static int gm_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  int bus_fwd = -1;

  if (bus_num == 0) {
    if (gm_allow_fwd && !gm_block_fwd) {
      bus_fwd = gm_camera_bus;
    }
  }
  else if (bus_num == gm_camera_bus) {
    int addr = GET_ADDR(to_fwd);

    // Do extra testing on frames from the cam bus untill we have seen 2 good LKAS frames
    // If any bad frames are encountered before then, forwarding is blocked
    // For OBD-based connections, forwarding defaults to off and is enabled if we see good LKAS
    // For harness-based connections, forwarding defaults to on and is disabled if we see bad frames
    // TODO: Can this all be determined by OP and sent as a param?
    // OP FP may be able to detect all of this...
    // This testing is limited in duration to reduce load
    if (!gm_block_fwd && gm_good_cam_cnt <= 2) {
      gm_validate_camera(addr, to_fwd);
    }
    
    if (gm_allow_fwd && !gm_block_fwd) {
      // block stock lkas messages and stock acc messages (if OP is doing ACC)
      //TODO: Blocking stock camera ACC will need to be an option in custom fork to allow use of OP's VOACC.
      int is_lkas_msg = (addr == 384);
      int is_acc_msg = false;
      //int is_acc_msg = (addr == 0x343);
      int block_msg = is_lkas_msg || is_acc_msg;
      if (!block_msg) {
        bus_fwd = 0;
      }
    }
  }

  return bus_fwd;
}


static const addr_checks* gm_init(int16_t param) {
  gm_safety_param = (int)param;
  gm_good_cam_cnt = 0;
  gm_allow_fwd = true;
  gm_block_fwd = false;
  gm_camera_bus = 2;
  gm_has_relay = true;
  gm_start_ts = microsecond_timer_get();

  if (car_harness_status == HARNESS_STATUS_NC) {
    //TODO: It seems as though the OBD2 harness may present as having a harness w relay.
    // So this may not work to detect a relay as previously thought
    // Seems to work with grey panda tho
    //puts("gm_init: No harness attached, assuming OBD or Giraffe\n");
    //OBD harness and older pandas use bus 1 and no relay
    //Most likely if we are using the OBD Harness, we have an ASCM and don't want to forward.
    gm_has_relay = false;
    gm_camera_bus = 1;
    gm_allow_fwd = false;
  }

  disengageFromBrakes = false;
  lateral_allowed = true;
  controls_allowed = false;
  relay_malfunction_reset();
  return &gm_rx_checks;
}

const safety_hooks gm_hooks = {
  .init = gm_init,
  .rx = gm_rx_hook,
  .tx = gm_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = gm_fwd_hook,
};