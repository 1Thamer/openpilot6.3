import json

class Phantom():
  def __init__(self):
    self.data = None
    self.update()
    self.phantom_file = "/data/phantom.json"
    self.frames = 0

  def update(self):
    self.frames += 1
    if self.frames > 200:
      self.frames = 0
      try:
        with open(self.phantom_file, "r") as p_f:
          self.data = json.load(p_f)
      except:
        self.data = {"status": False}