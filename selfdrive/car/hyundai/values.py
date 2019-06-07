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
  GENESIS = "GENESIS"                   # Genesis is unique and has issues at this stage if not detected.
  UNKNOWN = "HKG ON EMMERTEX FORK"      # Any Hyundai or Kia Car

class Buttons:
  NONE = 0
  RES_ACCEL = 1
  SET_DECEL = 2
  CANCEL = 4


FINGERPRINTS = {
  CAR.GENESIS: [{
    67: 8, 68: 8, 304: 8, 320: 8, 339: 8, 356: 4, 544: 7, 593: 8, 608: 8, 688: 5, 809: 8, 832: 8, 854: 7, 870: 7, 871: 8, 872: 5, 897: 8, 902: 8, 903: 6, 916: 8, 1024: 2, 1040: 8, 1056: 8, 1057: 8, 1078: 4, 1107: 5, 1136: 8, 1151: 6, 1168: 7, 1170: 8, 1173: 8, 1184: 8, 1265: 4, 1280: 1, 1287: 4, 1292: 8, 1312: 8, 1322: 8, 1331: 8, 1332: 8, 1333: 8, 1334: 8, 1335: 8, 1342: 6, 1345: 8, 1363: 8, 1369: 8, 1370: 8, 1371: 8, 1378: 4, 1384: 5, 1407: 8, 1419: 8, 1427: 6, 1434: 2, 1456: 4
  },
  {
    67: 8, 68: 8, 304: 8, 320: 8, 339: 8, 356: 4, 544: 7, 593: 8, 608: 8, 688: 5, 809: 8, 832: 8, 854: 7, 870: 7, 871: 8, 872: 5, 897: 8, 902: 8, 903: 6, 916: 8, 1024: 2, 1040: 8, 1056: 8, 1057: 8, 1078: 4, 1107: 5, 1136: 8, 1151: 6, 1168: 7, 1170: 8, 1173: 8, 1184: 8, 1265: 4, 1280: 1, 1281: 3, 1287: 4, 1292: 8, 1312: 8, 1322: 8, 1331: 8, 1332: 8, 1333: 8, 1334: 8, 1335: 8, 1345: 8, 1363: 8, 1369: 8, 1370: 8, 1378: 4, 1379: 8, 1384: 5, 1407: 8, 1419: 8, 1427: 6, 1434: 2, 1456: 4
  }],
  CAR.UNKNOWN: [{
    832: 8
  }],
}

CAMERA_MSGS = [832, 1156, 1191, 1342]

# Lane Keep Assist related Features and Limitations
FEATURES = {
  "icon_basic": [CAR.GENESIS],                          # Anything but 2 for LKAS_Icon causes MDPS Fault
  "soft_disable": [CAR.GENESIS],                        # Any steer message below 16.5m/s faults MDPS
}

DBC = {
  CAR.GENESIS: dbc_dict('hyundai_kia_generic', None),
  CAR.UNKNOWN: dbc_dict('hyundai_kia_generic', None),
}

STEER_THRESHOLD = 1.0
