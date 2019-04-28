import json

class Phantom():
  def __init__(self, up_freq):
    self.frames = 0
    self.update_freq = up_freq
    self.data = None
    self.update()
    self.phantom_file = "/data/phantom.json"

  def update(self):
    self.frames += 1
    if self.frames > self.update_freq:
      self.frames = 0
      try:
        with open(self.phantom_file, "r") as p_f:
          self.data = json.load(p_f)
      except:
        self.data = {"status": False}