from common.numpy_fast import interp
import numpy as np
import selfdrive.messaging as messaging
from selfdrive.swaglog import cloudlog
from common.realtime import sec_since_boot
from selfdrive.controls.lib.radar_helpers import _LEAD_ACCEL_TAU
from selfdrive.controls.lib.longitudinal_mpc import libmpc_py
from selfdrive.controls.lib.drive_helpers import MPC_COST_LONG
from selfdrive.phantom import Phantom
import time

class LongitudinalMpc(object):
  def __init__(self, mpc_id, live_longitudinal_mpc):
    self.live_longitudinal_mpc = live_longitudinal_mpc
    self.mpc_id = mpc_id

    self.setup_mpc()
    self.v_mpc = 0.0
    self.v_mpc_future = 0.0
    self.a_mpc = 0.0
    self.v_cruise = 0.0
    self.prev_lead_status = False
    self.prev_lead_x = 0.0
    self.new_lead = False
    self.v_ego = 0.0
    self.car_state = None
    self.last_cost = 0
    self.last_rate = None
    self.phantom = Phantom(timeout=True, do_sshd_mod=True)


    self.last_cloudlog_t = 0.0

  def send_mpc_solution(self, qp_iterations, calculation_time):
    qp_iterations = max(0, qp_iterations)
    dat = messaging.new_message()
    dat.init('liveLongitudinalMpc')
    dat.liveLongitudinalMpc.xEgo = list(self.mpc_solution[0].x_ego)
    dat.liveLongitudinalMpc.vEgo = list(self.mpc_solution[0].v_ego)
    dat.liveLongitudinalMpc.aEgo = list(self.mpc_solution[0].a_ego)
    dat.liveLongitudinalMpc.xLead = list(self.mpc_solution[0].x_l)
    dat.liveLongitudinalMpc.vLead = list(self.mpc_solution[0].v_l)
    dat.liveLongitudinalMpc.cost = self.mpc_solution[0].cost
    dat.liveLongitudinalMpc.aLeadTau = self.a_lead_tau
    dat.liveLongitudinalMpc.qpIterations = qp_iterations
    dat.liveLongitudinalMpc.mpcId = self.mpc_id
    dat.liveLongitudinalMpc.calculationTime = calculation_time
    self.live_longitudinal_mpc.send(dat.to_bytes())

  def setup_mpc(self):
    ffi, self.libmpc = libmpc_py.get_libmpc(self.mpc_id)
    self.libmpc.init(MPC_COST_LONG.TTC, MPC_COST_LONG.DISTANCE,
                     MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)

    self.mpc_solution = ffi.new("log_t *")
    self.cur_state = ffi.new("state_t *")
    self.cur_state[0].v_ego = 0
    self.cur_state[0].a_ego = 0
    self.a_lead_tau = _LEAD_ACCEL_TAU

  def set_cur_state(self, v, a):
    self.cur_state[0].v_ego = v
    self.cur_state[0].a_ego = a

  def dynamic_follow(self, velocity):  # in m/s
    x_vel = [0.0, 1.86267, 3.72533, 5.588, 7.45067, 9.31333, 11.55978, 13.645, 22.352, 31.2928, 33.528, 35.7632, 40.2336]  # velocity
    y_mod = [1.03, 1.05363, 1.07879, 1.11493, 1.16969, 1.25071, 1.36325, 1.43, 1.6, 1.7, 1.75618, 1.85, 2.0]  # distances

    TR = interp(self.v_ego, x_vel, y_mod)
    return TR

    stop_and_go_magic_number = 8.9408  # 20 mph

    if velocity <= 0.44704:  # 1 mph
      self.stop_and_go = True
    elif velocity >= stop_and_go_magic_number:
      self.stop_and_go = False

    if self.stop_and_go:  # this allows a smooth deceleration to a stop, while being able to have smooth stop and go
      x = [stop_and_go_magic_number / 2.0, stop_and_go_magic_number]  # from 10 to 20 mph, ramp 1.8 sng distance to regular dynamic follow value
      y = [1.8, interp(stop_and_go_magic_number, x_vel, y_mod)]
      TR = interp(velocity, x, y)
    else:
      TR = interpolate.interp1d(x_vel, y_mod, fill_value='extrapolate')(velocity)[()]  # extrapolate above 90 mph

    if self.relative_velocity is not None:
      x = [-15.6464, -11.6231, -7.8428, -5.45, -4.3701, -3.2187, -1.7241, -0.911, -0.4917, 0.0, 0.2682, 0.775, 1.8532, 2.6851]  # relative velocity values
      y = [0.504, 0.465, 0.4009, 0.3226, 0.2852, 0.2184, 0.168, 0.1187, 0.0683, 0, -0.0554, -0.1371, -0.2402, -0.3004]  # modification values
      TR_mod = interp(self.relative_velocity, x, y)  # factor in lead relative velocity

      x = [-4.4704, -2.2352, -0.8941, 0.0, 1.3411]   # self acceleration values
      y = [0.158, 0.058, 0.016, 0, -0.13]  # modification values
      TR_mod += interp(self.get_acceleration(self.dynamic_follow_dict["self_vels"], True), x, y)  # factor in self acceleration

      x = [-4.49033, -1.87397, -0.66245, -0.26291, 0.0, 0.5588, 1.34112]  # lead acceleration values
      y = [0.37909, 0.30045, 0.20378, 0.04158, 0, -0.115, -0.195]  # modification values
      TR_mod += interp(self.get_acceleration(self.dynamic_follow_dict["lead_vels"], False), x, y)  # factor in lead car's acceleration; should perform better

      x = [0, 6.9128, 16.0047, 27.163, 37.6085, 50.3843, 54.6429, 65.3908, 83.0336, 93.1731]  # distance in meters
      y = [1.0175, 1.0079, 1.0045, 1.0083, 1.0176, 1.0547, 1.0911, 1.1454, 1.1838, 1.195]
      TR_mod *= interp(self.relative_distance, x, y)  # factor in distance from lead car to try and brake quicker

      x = [1.1594, 2.7298, 6.1562, 10.5105, 22.352, 33.528]  # speed in m/s
      y = [0.7, 0.885, 1.0, 1.024, 1.0, 0.9]
      TR_mod *= float(interp(velocity, x, y))  # lower TR modification for stop and go, and at higher speeds

      TR += TR_mod
      TR *= self.get_traffic_level(self.dynamic_follow_dict["traffic_vels"])  # modify TR based on last minute of traffic data
    if TR < 0.65:
      return 0.65
    else:
      return round(TR, 4)

  def dynamic_follow(self):
    return 1.8

  def get_TR(self):
    read_distance_lines = self.car_state.readdistancelines

    if self.v_ego < 2.0:  #todo: make a ramp function to smoothly transition
      return 1.8
    elif self.car_state.leftBlinker or self.car_state.rightBlinker:
      if self.last_cost != 1.0:
        self.libmpc.change_tr(MPC_COST_LONG.TTC, 1.0, MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
        self.last_cost = 1.0
      return 0.9  # accelerate for lane change
    elif read_distance_lines == 1:
      if self.last_cost != 1.0:
        self.libmpc.change_tr(MPC_COST_LONG.TTC, 1.0, MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
        self.last_cost = 1.0
      return 0.9  # 10m at 40km/hr
    elif read_distance_lines == 2:
      TR = self.dynamic_follow()
      if self.last_cost != 0.1:
        self.libmpc.change_tr(MPC_COST_LONG.TTC, 0.1, MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
        self.last_cost = 0.1
      return TR
    else:
      if self.last_cost != 0.05:
        self.libmpc.change_tr(MPC_COST_LONG.TTC, 0.05, MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
        self.last_cost = 0.05
      return 2.7  # 30m at 40km/hr

  def calc_rate(self, seconds=1.0):  # return current rate of long_mpc in fps/hertz
    current_time = time.time()
    if self.last_rate is None or (current_time - self.last_rate) == 0:
      rate = int(round(40.42 * seconds))
    else:
      rate = (1.0 / (current_time - self.last_rate)) * seconds

    min_return = 10
    max_return = seconds * 100
    self.last_rate = current_time
    return int(round(max(min(rate, max_return), min_return)))  # ensure we return a value between range, in hertz

  def update(self, CS, lead, v_cruise_setpoint):
    self.car_state = CS.carState
    self.v_ego = CS.carState.vEgo

    # Setup current mpc state
    self.cur_state[0].x_ego = 0.0
    self.phantom.update(self.calc_rate())

    if self.phantom.data["status"]:
      self.relative_velocity = self.phantom.data["speed"] - self.v_ego
      if self.phantom.data["speed"] != 0.0:
        x_lead = 9.144
        v_lead = self.phantom.data["speed"]
      else:
        x_lead = 3.75
        v_lead = max(self.v_ego - (.7 / max(max(self.v_ego, 0)**.4, .01)), 0.0)  # smoothly decelerate to 0

      a_lead = 0.0
      self.a_lead_tau = lead.aLeadTau
      self.new_lead = False
      if not self.prev_lead_status or abs(x_lead - self.prev_lead_x) > 2.5:
        self.libmpc.init_with_simulation(self.v_mpc, x_lead, v_lead, a_lead, self.a_lead_tau)
        self.new_lead = True

      self.prev_lead_status = True
      self.prev_lead_x = x_lead
      self.cur_state[0].x_l = x_lead
      self.cur_state[0].v_l = v_lead
    else:
      if lead is not None and lead.status:
        x_lead = lead.dRel
        v_lead = max(0.0, lead.vLead)
        a_lead = lead.aLeadK

        if (v_lead < 0.1 or -a_lead / 2.0 > v_lead):
          v_lead = 0.0
          a_lead = 0.0

        self.a_lead_tau = lead.aLeadTau
        self.new_lead = False
        if not self.prev_lead_status or abs(x_lead - self.prev_lead_x) > 2.5:
          self.libmpc.init_with_simulation(self.v_mpc, x_lead, v_lead, a_lead, self.a_lead_tau)
          self.new_lead = True

        self.prev_lead_status = True
        self.prev_lead_x = x_lead
        self.cur_state[0].x_l = x_lead
        self.cur_state[0].v_l = v_lead
      else:
        self.prev_lead_status = False
        # Fake a fast lead car, so mpc keeps running
        self.cur_state[0].x_l = 50.0
        self.cur_state[0].v_l = self.v_ego + 10.0
        a_lead = 0.0
        self.a_lead_tau = _LEAD_ACCEL_TAU

    # Calculate mpc
    t = sec_since_boot()
    TR = self.get_TR()
    n_its = self.libmpc.run_mpc(self.cur_state, self.mpc_solution, self.a_lead_tau, a_lead, TR)
    duration = int((sec_since_boot() - t) * 1e9)
    self.send_mpc_solution(n_its, duration)

    # Get solution. MPC timestep is 0.2 s, so interpolation to 0.05 s is needed
    self.v_mpc = self.mpc_solution[0].v_ego[1]
    self.a_mpc = self.mpc_solution[0].a_ego[1]
    self.v_mpc_future = self.mpc_solution[0].v_ego[10]

    # Reset if NaN or goes through lead car
    dls = np.array(list(self.mpc_solution[0].x_l)) - np.array(list(self.mpc_solution[0].x_ego))
    crashing = min(dls) < -50.0
    nans = np.any(np.isnan(list(self.mpc_solution[0].v_ego)))
    backwards = min(list(self.mpc_solution[0].v_ego)) < -0.01

    if ((backwards or crashing) and self.prev_lead_status) or nans:
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning("Longitudinal mpc %d reset - backwards: %s crashing: %s nan: %s" % (
                          self.mpc_id, backwards, crashing, nans))

      self.libmpc.init(MPC_COST_LONG.TTC, MPC_COST_LONG.DISTANCE,
                       MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
      self.cur_state[0].v_ego = self.v_ego
      self.cur_state[0].a_ego = 0.0
      self.v_mpc = self.v_ego
      self.a_mpc = CS.carState.aEgo
      self.prev_lead_status = False
