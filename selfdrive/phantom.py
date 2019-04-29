import time
import json
import threading

def phantom_thread():
  global data
  global high_frequency
  thread_interval = 5  # 12hz
  while True:
    time.sleep(thread_interval)
    start = time.time()
    data = read_phantom()
    if data["status"]:
      if high_frequency:
        thread_interval = 0.02  # 50hz for steering
      else:
        thread_interval = 0.2  # 5hz
    else:
      thread_interval = 5
    thread_interval = max(thread_interval - (time.time() - start), 0)  # rate keeper

def read_phantom():
  try:
    with open(phantom_file, "r") as f:
      return json.load(f)
  except:
    return {"status": False}

high_frequency = False  # set to true from latcontrol, false for long control
data = {"status": False}
phantom_file = "/data/phantom.json"
prev_status = False
threading.Thread(target=phantom_thread).start()