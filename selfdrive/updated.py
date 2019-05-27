#!/usr/bin/env python

# simple service that waits for network access and tries to update every hour

import zmq
import selfdrive.messaging as messaging
from selfdrive.services import service_list
import time
import subprocess
from selfdrive.swaglog import cloudlog
import selfdrive.kegman_conf as kegman
import os

NICE_LOW_PRIORITY = ["nice", "-n", "19"]
def main(gctx=None):
  context = zmq.Context()
  manager_sock = messaging.sub_sock(context, service_list['managerData'].port)

  while True:
    # try network
    ping_failed = subprocess.call(["ping", "-W", "4", "-c", "1", "8.8.8.8"])
    if ping_failed:
      time.sleep(60)
      continue

    # download application update
    try:
      r = subprocess.check_output(NICE_LOW_PRIORITY + ["git", "fetch"], stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as e:
      cloudlog.event("git fetch failed",
        cmd=e.cmd,
        output=e.output,
        returncode=e.returncode)
      time.sleep(60)
      continue
    cloudlog.info("git fetch success: %s", r)
    if kegman.get("autoUpdate", True):
      try:
        r = subprocess.check_output(NICE_LOW_PRIORITY + ["git", "pull"], stderr=subprocess.STDOUT)
      except subprocess.CalledProcessError as e:
        cloudlog.event("git pull failed",
          cmd=e.cmd,
          output=e.output,
          returncode=e.returncode)
        time.sleep(60)
        continue
      cloudlog.info("git pull success: %s", r)
      msg = messaging.recv_one_or_none(manager_sock)
      if msg:
        if "controlsd" not in msg.managerData.runningProcesses:
          os.system('reboot')

    time.sleep(60*60)

if __name__ == "__main__":
  main()

