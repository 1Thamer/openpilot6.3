from selfdrive.car import limit_steer_rate
from selfdrive.car.hyundai.hyundaican import create_lkas11, create_lkas12, \
                                             create_1191, create_1156, \
                                             create_clu11, learn_checksum
from selfdrive.car.hyundai.values import Buttons
from selfdrive.can.packer import CANPacker


# Steer torque limits

class SteerLimitParams:
  STEER_MAX = 255   # >255 results in frozen torque, >409 results in no torque
  STEER_DELTA_UP = 3
  STEER_DELTA_DOWN = 7
  STEER_DRIVER_ALLOWANCE = 50
  STEER_DRIVER_MULTIPLIER = 2
  STEER_DRIVER_FACTOR = 1
  DIVIDER = 2.0     # Must be > 1.0

class CarController(object):
  def __init__(self, dbc_name, car_fingerprint):
    self.apply_steer_last = 0
    self.car_fingerprint = car_fingerprint

    self.lkas11_cnt = 0
    self.clu11_cnt = 0
    self.cnt = 0
    self.last_resume_cnt = 0

    self.checksum = "NONE"
    self.checksum_learn_cnt = 0

    self.turning_signal_timer = 0
    self.min_steer_speed = 0.
    self.camera_disconnected = False

    self.packer = CANPacker(dbc_name)

  def update(self, enabled, CS, actuators, pcm_cancel_cmd, hud_alert):
    ### Error State Resets ###
    disable_steer = False

    ### Learn Checksum ###

    # Learn Checksum from the Camera
    if self.checksum == "NONE":
      self.checksum = learn_checksum(self.packer, CS.lkas11)
      print ("Discovered Checksum", self.checksum)
      if self.checksum == "NONE" and self.checksum_learn_cnt < 50:
        self.checksum_learn_cnt += 1
        return

    # If MDPS is faulted from bad checksum, then cycle through all Checksums until 1 works
    if CS.steer_error == 1:
      self.camera_disconnected = True
      if self.checksum_learn_cnt > 250:
        self.checksum_learn_cnt = 50
        if self.checksum == "NONE":
          print ("Testing 6B Checksum")
          self.checksum == "6B"
        elif self.checksum == "6B":
          print ("Testing 7B Checksum")
          self.checksum == "7B"
        elif self.checksum == "7B":
          print ("Testing CRC8 Checksum")
          self.checksum == "crc8"
        else:
          self.checksum == "NONE"
      else:
        self.checksum_learn_cnt += 1

    ### Minimum Steer Speed ###

    # Learn Minimum Steer Speed
    if CS.mdps12_flt != 0 and CS.v_ego_raw > 0.:
      if CS.v_ego_raw > self.min_steer_speed:
        self.min_steer_speed = CS.v_ego_raw + 0.1
        print ("Discovered new Min Speed as", self.min_steer_speed)

    # Apply Usage of Minimum Steer Speed
    if CS.v_ego_raw < self.min_steer_speed:
      disable_steer = True


    ### Turning Indicators ###
    if (CS.left_blinker_on == 1 or CS.right_blinker_on == 1):
      self.turning_signal_timer = 100 # Disable for 1.0 Seconds after blinker turned off

    if self.turning_signal_timer > 0:
      disable_steer = True
      self.turning_signal_timer -= 1

    ### Steering Torque ###
    apply_steer = actuators.steer * SteerLimitParams.STEER_MAX
    apply_steer = limit_steer_rate(apply_steer, self.apply_steer_last, CS.steer_torque_driver, SteerLimitParams)

    if not enabled or disable_steer:
      apply_steer = 0

    steer_req = 1 if enabled else 0

    self.apply_steer_last = apply_steer


    ### Generate CAN Messages ###
    can_sends = []

    self.lkas11_cnt = self.cnt % 0x10
    self.clu11_cnt = self.cnt % 0x10

    if self.camera_disconnected:
      if (self.cnt % 10) == 0:
        can_sends.append(create_lkas12())
      if (self.cnt % 50) == 0:
        can_sends.append(create_1191())
      if (self.cnt % 7) == 0:
        can_sends.append(create_1156())

    can_sends.append(create_lkas11(self.packer, self.car_fingerprint, apply_steer, steer_req, self.lkas11_cnt,
                                   enabled, CS.lkas11, hud_alert, (not self.camera_disconnected), self.checksum))

    if pcm_cancel_cmd:
      can_sends.append(create_clu11(self.packer, CS.clu11, Buttons.CANCEL))
    elif CS.stopped and (self.cnt - self.last_resume_cnt) > 5:
      self.last_resume_cnt = self.cnt
      can_sends.append(create_clu11(self.packer, CS.clu11, Buttons.RES_ACCEL))

    self.cnt += 1

    return can_sends
