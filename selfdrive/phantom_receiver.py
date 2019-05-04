import sys
import zmq
from selfdrive.services import service_list
import selfdrive.messaging as messaging

def broadcast_data(p_status, p_speed, p_angle, p_time):
  p_status = True if p_status == "true" else False
  phantomData_sock = messaging.pub_sock(zmq.Context(), service_list['phantomData'].port)
  data = messaging.new_message()
  data.init('phantomData')
  data.phantomData.status = p_status
  data.phantomData.speed = p_speed
  data.phantomData.angle = p_angle
  data.phantomData.time = p_time
  phantomData_sock.send(data.to_bytes())

if __name__ == "__main__":
  broadcast_data(str(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))