
// Kia Hyundai Safety
//
// Based on GM Safety

const int HYUNDAI_STEER_ZERO = 1024;
const int HYUNDAI_MAX_STEER = 250; // This may be lifted highter AFTER a valid proof from users deems it necessary
const int HYUNDAI_MAX_RATE_UP = 3;
const int HYUNDAI_MAX_RATE_DOWN = 6;
const int HYUNDAI_DRIVER_TORQUE_ALLOWANCE = 100;
const int HYUNDAI_DRIVER_TORQUE_FACTOR = 100;
const int HYUNDAI_MAX_RT_DELTA = 128;
const int32_t HYUNDAI_RT_INTERVAL = 250000;
int hyundai_giraffe_switch_1 = 0;          // is giraffe switch 1 high?
int hyundai_rt_torque_last = 0;
int hyundai_desired_torque_last = 0;
uint32_t hyundai_ts_last = 0;
struct sample_t hyundai_torque_driver;         // last few driver torques measured

static int hyundai_ign_hook() {
  return true;
}
static void hyundai_init(int16_t param) {
  controls_allowed = 1;
}

static int hyundai_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {

  uint32_t addr;
  if (to_send->RIR & 4) {
    addr = to_send->RIR >> 3;
  } else {
    addr = to_send->RIR >> 21;
  }
  
  // LKA STEER: safety check
  if (addr == 0x340) {
    int rdlr = to_send->RDLR;
    //  Torque Request starts at bit 16 for 11 bits
    int desired_torque = ((rdlr >> 24 & 0x7) << 8) + (rdlr >> 16 & 0xFF);
    uint32_t ts = TIM2->CNT;
    int violation = 0;

    if (controls_allowed) {

      // *** global torque limit check ***
      violation |= max_limit_check(desired_torque, HYUNDAI_STEER_ZERO + HYUNDAI_MAX_STEER,
        HYUNDAI_STEER_ZERO - HYUNDAI_MAX_STEER);

      // *** torque rate limit check ***
      violation |= driver_limit_check(desired_torque, hyundai_desired_torque_last, &hyundai_torque_driver,
        HYUNDAI_MAX_STEER, HYUNDAI_MAX_RATE_UP, HYUNDAI_MAX_RATE_DOWN,
        HYUNDAI_DRIVER_TORQUE_ALLOWANCE, HYUNDAI_DRIVER_TORQUE_FACTOR);

      // used next time
      hyundai_desired_torque_last = desired_torque;

      // *** torque real time rate limit check ***
      violation |= rt_rate_limit_check(desired_torque, hyundai_rt_torque_last, HYUNDAI_MAX_RT_DELTA);

      // every RT_INTERVAL set the new limits
      uint32_t ts_elapsed = get_ts_elapsed(ts, hyundai_ts_last);
      if (ts_elapsed > HYUNDAI_RT_INTERVAL) {
        hyundai_rt_torque_last = desired_torque;
        hyundai_ts_last = ts;
      }
    }

    if (violation) {
      return false;
    }
  }

  return true;
}

static void hyundai_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {

  int bus = (to_push->RDTR >> 4) & 0xF;
  // 832 is lkas cmd. If it is on bus 0, then giraffe switch 1 is high and we want stock
  if ((to_push->RIR>>21) == 832 && (bus == 0)) {
    hyundai_giraffe_switch_1 = 1;
  }
}

static void hyundai_init(int16_t param) {
  controls_allowed = 0;
  hyundai_giraffe_switch_1 = 0;
}

static int hyundai_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  return true;
}

static int hyundai_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {

  // forward camera to car and viceversa, excpet for lkas11 and mdps12
  if ((bus_num == 0 || bus_num == 2) && !hyundai_giraffe_switch_1) {
    int addr = to_fwd->RIR>>21;
    bool is_lkas_msg = (addr == 832 && bus_num == 2) || (addr == 593 && bus_num == 0);
    return is_lkas_msg? -1 : (uint8_t)(~bus_num & 0x2);
  }
  return -1;
}

const safety_hooks hyundai_hooks = {
  .init = hyundai_init,
  .rx = hyundai_rx_hook,
<<<<<<< HEAD
  .tx = nooutput_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .ignition = default_ign_hook,
  .fwd = hyundai_fwd_hook,
};

const safety_hooks hyundai_hooks = {
  .init = hyundai_init,
  .rx = hyundai_rx_hook,
  .tx = nooutput_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .ignition = default_ign_hook,
  .fwd = hyundai_fwd_hook,
};