import socket
import json
import time
import math

# 配置PlotJuggler的UDP接收端口和IP
UDP_IP = "127.0.0.1" 
UDP_PORT = 9870

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print(f"UDP发送端已启动... 正在向 {UDP_IP}:{UDP_PORT} 循环发送测试数据")
print("请在 PlotJuggler 中开启 UDP Server 并监听端口 9870")

t = 0.0
while True:
    try:
        data = {
            "gimbal_yaw": 30.0 * math.sin(t),
            "cmd_yaw": 30.0 * math.sin(t + 0.1),
            "gimbal_pitch": 15.0 * math.cos(t),
            "cmd_pitch": 15.0 * math.cos(t + 0.1),
            "find_bool": 1 if math.sin(t) > 0 else 0
        }
        json_str = json.dumps(data)
        sock.sendto(json_str.encode('utf-8'), (UDP_IP, UDP_PORT))
        
        t += 0.01
        time.sleep(0.01) # 100Hz 发送频率
    except KeyboardInterrupt:
        print("\n停止发送。")
        break
