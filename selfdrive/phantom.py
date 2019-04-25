import json

class Phantom():
  def __init__(self):
    self.data = None
    self.update()
    self.phantom_file = "/data/phantom.json"

  def update(self):
    try:
      with open(self.phantom_file, "r") as p_f:
        self.data = json.load(p_f)
    except:
      self.data = {"status": False}