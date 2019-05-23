from selfdrive.controls.lib.pid import PIController
from common.numpy_fast import interp
from cereal import car

from common.realtime import sec_since_boot

def get_steer_max(CP, v_ego):
  return interp(v_ego, CP.steerMaxBP, CP.steerMaxV)


class LatControl(object):
  def __init__(self, CP):
    self.pid = PIController((CP.steerKpBP, CP.steerKpV),
                            (CP.steerKiBP, CP.steerKiV),
                            k_f=CP.steerKf, pos_limit=1.0)
    self.last_cloudlog_t = 0.0
    self.angle_steers_des = 0.
    self.angle_ff_ratio = 0.0
    self.angle_ff_gain = 1.0
    self.rate_ff_gain = 0.01
    self.angle_ff_bp = [[0.5, 5.0],[0.0, 1.0]]
    
  def reset(self):
    self.pid.reset()
    
  def adjust_angle_gain(self):
    if (self.pid.f > 0) == (self.pid.i > 0) and abs(self.pid.i) >= abs(self.previous_integral):
      self.angle_ff_gain *= 1.0001
    else:
      self.angle_ff_gain *= 0.9999
    self.angle_ff_gain = max(1.0, self.angle_ff_gain)
    self.previous_integral = self.pid.i

  def update(self, active, v_ego, angle_steers, steer_override, CP, VM, path_plan):
    if v_ego < 0.3 or not active:
      output_steer = 0.0
      self.pid.reset()
      self.previous_integral = 0.0
    else:
      self.angle_steers_des = interp(sec_since_boot(), path_plan.mpcTimes, path_plan.mpcAngles)

      steers_max = get_steer_max(CP, v_ego)
      self.pid.pos_limit = steers_max
      self.pid.neg_limit = -steers_max
      steer_feedforward = self.angle_steers_des   # feedforward desired angle
      if CP.steerControlType == car.CarParams.SteerControlType.torque:
        angle_feedforward = steer_feedforward - path_plan.angleOffset
        self.angle_ff_ratio = interp(abs(angle_feedforward), self.angle_ff_bp[0], self.angle_ff_bp[1])
        angle_feedforward *= self.angle_ff_ratio * self.angle_ff_gain
        rate_feedforward = (1.0 - self.angle_ff_ratio) * self.rate_ff_gain * path_plan.rateSteers
        steer_feedforward = v_ego**2 * (rate_feedforward + angle_feedforward)
        
        if v_ego > 10.0:
          if abs(angle_steers) > (self.angle_ff_bp[0][1] / 2.0):
            self.adjust_angle_gain()
          else:
            self.previous_integral = self.pid.i
        
      deadzone = 0.0
      output_steer = self.pid.update(self.angle_steers_des, angle_steers, check_saturation=(v_ego > 10), override=steer_override,
                                     feedforward=steer_feedforward, speed=v_ego, deadzone=deadzone)
    
    self.sat_flag = self.pid.saturated
    return output_steer, float(self.angle_steers_des)    
    
    # ALCA works better with the non-interpolated angle
    #if CP.steerControlType == car.CarParams.SteerControlType.torque:
    #  return output_steer, float(self.angle_steers_des_mpc)
    #else:
    #  return float(self.angle_steers_des_mpc), float(self.angle_steers_des)
