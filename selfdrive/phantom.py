import time
import json
import threading
import selfdrive.kegman_conf as kegman
import subprocess
from common.basedir import BASEDIR


def phantom_thread():
  global data
  global high_frequency
  thread_interval = 5  # .2hz
  thread_start = time.time()
  while True:
    time.sleep(thread_interval)
    start = time.time()
    data = read_phantom()
    if data["status"]:
      thread_start = time.time()
      if high_frequency:
        thread_interval = 0.02  # 50hz for steering
      else:
        thread_interval = 0.2  # 5hz
    else:
      thread_interval = 5
    thread_interval = max(thread_interval - (time.time() - start), 0)  # rate keeper
    if (time.time() - thread_start) > (20*60):  # if car is on for more than 20 minutes without phantom activation, shut down thread
      with open("/data/testing.txt", "a") as f:
        f.write("thread timeout\n")
      data = {"status": False}
      break


def read_phantom():
  tmp = None
  try:
    with open(phantom_file, "r") as f:
      f.seek(0)
      tmp = f.read()
      return json.loads(tmp)
  except Exception,e:
    with open("/data/phantom_error.txt", "a") as f:
      f.write(str(e)+"\n"+tmp+"\n")
    return {"status": False}


def mod_sshd_config():  # this disables dns lookup when connecting to EON to speed up commands from phantom app, reboot required
  sshd_config_file = "/system/comma/usr/etc/ssh/sshd_config"
  result = subprocess.check_call(["mount", "-o", "remount,rw", "/system"])  # mount /system as rw so we can modify sshd_config file
  if result == 0:
    with open(sshd_config_file, "r") as f:
      sshd_config = f.read()
    if "UseDNS no" not in sshd_config:
      if sshd_config[-1:]!="\n":
        use_dns = "\nUseDNS no\n"
      else:
        use_dns = "UseDNS no\n"
      with open(sshd_config_file, "w") as f:
        f.write(sshd_config + use_dns)
      kegman.save({"UseDNS": True})
    subprocess.check_call(["mount", "-o", "remount,ro", "/system"])  # remount system as read only
  else:
    kegman.save({"UseDNS": False})


def start(high_freq=False):
  global high_frequency
  high_frequency = high_freq  # set to true from latcontrol, false for long control
  if BASEDIR == "/data/openpilot":
    if not kegman.get("UseDNS") or kegman.get("UseDNS") is None:
      mod_sshd_config()
    threading.Thread(target=phantom_thread).start()


data = {"status": False}
phantom_file = "/data/phantom.json"
