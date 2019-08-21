from common.numpy_fast import interp
import numpy as np
from selfdrive.controls.lib.latcontrol_helpers import model_polyfit, compute_path_pinv
from selfdrive.kegman_conf import kegman_conf

kegman = kegman_conf()

CAMERA_OFFSET = float(kegman.conf['cameraOffset'])  # m from center car to camera


def calc_d_poly(l_poly, r_poly, p_poly, l_prob, r_prob, lane_width):
  # This will improve behaviour when lanes suddenly widen
  lane_width = min(4.0, lane_width)
  l_prob = l_prob * interp(abs(l_poly[3]), [2, 2.5], [1.0, 0.0])
  r_prob = r_prob * interp(abs(r_poly[3]), [2, 2.5], [1.0, 0.0])

  path_from_left_lane = l_poly.copy()
  path_from_left_lane[3] -= lane_width / 2.0
  path_from_right_lane = r_poly.copy()
  path_from_right_lane[3] += lane_width / 2.0

  c_prob = l_prob + r_prob - l_prob * r_prob

  c_poly = (l_prob * path_from_left_lane + r_prob * path_from_right_lane) / (l_prob + r_prob + 0.0001)
  d_poly = (c_prob * c_poly + p_poly) / (c_prob + 1.0)
  return c_poly, d_poly, c_prob


class LanePlanner(object):
  def __init__(self):
    self.l_poly = np.array([0., 0., 0., 0.])
    self.r_poly = np.array([0., 0., 0., 0.])
    self.p_poly = np.array([0., 0., 0., 0.])
    self.d_poly = np.array([0., 0., 0., 0.])
    self.p_poly = np.array([0., 0., 0., 0.])
    self.c_poly = np.array([0., 0., 0., 0.])

    self.lane_width_estimate = 3.7
    self.lane_width_certainty = 1.0
    self.lane_width = 3.7

    self.l_prob = 0.
    self.r_prob = 0.
    self.c_prob = 0.

    self._path_pinv = compute_path_pinv()
    self.x_points = np.arange(50)

  def parse_model(self, md):
    if len(md.leftLane.poly):
      self.l_poly = np.array(md.leftLane.poly)
      self.r_poly = np.array(md.rightLane.poly)
      self.p_poly = np.array(md.path.poly)
    else:
      self.l_poly = model_polyfit(md.leftLane.points, self._path_pinv)  # left line
      self.r_poly = model_polyfit(md.rightLane.points, self._path_pinv)  # right line
      self.p_poly = model_polyfit(md.path.points, self._path_pinv)  # predicted path
    self.l_prob = md.leftLane.prob  # left line prob
    self.r_prob = md.rightLane.prob  # right line prob

  def update_lane(self, controls):
    # only offset left and right lane lines; offsetting p_poly does not make sense
    self.l_poly[3] += CAMERA_OFFSET
    self.r_poly[3] += CAMERA_OFFSET

    # Find current lanewidth
    lr_prob = self.l_prob * self.r_prob
    decay_rate = interp(lr_prob, [0., 0.5], [0.1, 0.5])
    decay_rate *= controls.vEgo / 31.0
    self.lane_width_certainty += 0.05 * decay_rate * (lr_prob - self.lane_width_certainty)
    current_lane_width = abs(self.l_poly[3] - self.r_poly[3])
    self.lane_width_estimate += 0.005 * decay_rate * (current_lane_width - self.lane_width_estimate)
    speed_lane_width = interp(controls.vEgo, [0., 31.], [2.8, 3.5])
    self.lane_width = self.lane_width_certainty * self.lane_width_estimate + \
                      (1 - self.lane_width_certainty) * speed_lane_width

    self.c_poly, self.d_poly, self.c_prob = calc_d_poly(self.l_poly, self.r_poly, self.p_poly, self.l_prob, self.r_prob, self.lane_width)

  def update(self, md, controls):
    self.parse_model(md)
    self.update_lane(controls)
