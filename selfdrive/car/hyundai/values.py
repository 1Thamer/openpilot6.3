from cereal import car
from selfdrive.car import dbc_dict

VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert

def get_hud_alerts(visual_alert, audible_alert):
  if visual_alert == VisualAlert.steerRequired:
    return 4 if audible_alert != AudibleAlert.none else 5
  else:
    return 0

class CAR:
  HKG = "HYUNDAI OR KIA"

class Buttons:
  NONE = 0
  RES_ACCEL = 1
  SET_DECEL = 2
  CANCEL = 4

FINGERPRINTS = {
  CAR.HKG: [{
    832: 8
  }],
}

DBC = {
  CAR.HKG: dbc_dict('hyundai_kia_generic', None),
}

STEER_THRESHOLD = 1.5
