#!/usr/bin/env python
from cereal import car
import time
from selfdrive.can.parser import CANParser
from selfdrive.car.hyundai.carstate import CarState, get_can_parser
from common.realtime import sec_since_boot

class RadarInterface(object):
  def __init__(self, CP):
    # radar
    self.pts = {}
    self.delay = 0.1
    self.CS = CarState(CP)
    self.cp = get_can_parser(CP)
    self.trigger_msg = self.CS.scc11
    self.updated_messages = set()
    self.no_radar = False

  def update(self, can_strings):
    if self.no_radar:
      ret = car.RadarData.new_message()
      time.sleep(0.05)  # radard runs on RI updates
      return ret

    tm = int(sec_since_boot() * 1e9)
    vls = self.cp.update_strings(tm, can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr =  self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr
  
  

  def _update(self, updated_messages):
    ret = car.RadarData.new_message()
    cpt = self.rcp.vl
    errors = []
    if not self.rcp.can_valid:
      errors.append("canError")
    ret.errors = errors

    valid = self.CS.scc11["ACC_ObjStatus"]
    if valid:
      self.pts.dRel = self.CS.scc11["ACC_ObjDist"]  # from front of car
      self.pts.yRel = self.CS.scc11["ACC_ObjLatPos"]  # in car frame's y axis, left is negative
      self.pts.vRel = self.CS.scc11["ACC_ObjRelSpd"]
      self.pts.aRel = float('nan')
      self.pts.yvRel = float('nan')
      self.pts.measured = True

    ret.points = self.pts.values()
    return ret
