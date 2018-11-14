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

int hyundai_camera_detected = 0;
int hyundai_camera_bus = 0;
int hyundai_giraffe_switch_2 = 0;          // is giraffe switch 2 high?
int hyundai_cruise_engaged_last = 0;
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
  if (addr == 832) {
    int desired_torque = ((to_send->RDLR >> 16) & 0x7ff) - 1024;
    //  Torque Request starts at bit 16 for 11 bits

    uint32_t ts = TIM2->CNT;
    int violation = 0;

    if (controls_allowed) {

      // *** global torque limit check ***
      violation |= max_limit_check(desired_torque, HYUNDAI_MAX_STEER, -HYUNDAI_MAX_STEER);

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



static int hyundai_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  return true;
}

static int hyundai_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  // forward cam to ccan and viceversa, except lkas cmd
  if ((bus_num == 0 || bus_num == hyundai_camera_bus) && hyundai_giraffe_switch_2) {

    if ((to_fwd->RIR>>21) == 832 && bus_num == hyundai_camera_bus) return -1;
    if (bus_num == 0) return hyundai_camera_bus;
    if (bus_num == hyundai_camera_bus) return 0;
  }
  return -1;
}

static void hyundai_init(int16_t param) {
  controls_allowed = 0;
  hyundai_giraffe_switch_2 = 0;
}

const safety_hooks hyundai_hooks = {
  .init = hyundai_init,
  .rx = hyundai_rx_hook,
  .tx = hyundai_tx_hook,
  .tx_lin = hyundai_tx_lin_hook,
  .ignition = hyundai_ign_hook,
  .fwd = hyundai_fwd_hook,
};

