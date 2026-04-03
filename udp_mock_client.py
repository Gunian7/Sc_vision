import socket
import json
import time
import math

UDP_IP = "127.0.0.1" # 发送到本地PlotJuggler
UDP_PORT = 9870
MESSAGE = {
    "cmd_yaw": 0.0,
    "gimbal_yaw": 0.0,
    "find_bool": 1
}

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"开始向 {UDP_IP}:{UDP_PORT} 发送测试数据...")
t = 0
while True:
    MESSAGE["cmd_yaw"] = 30 * math.sin(t)
    MESSAGE["gimbal_yaw"] = 30 * math.sin(t - 0.2)
    MESSAGE["find_bool"] = 1 if math.sin(t) > 0 else 0
    data = json.dumps(MESSAGE)
    sock.sendto(data.encode('utf-8'), (UDP_IP, UDP_PORT))
    time.sleep(0.01) # 100Hz
    t += 0.01

