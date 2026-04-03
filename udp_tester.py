import socket
import json
import threading
import time

IP = "127.0.0.1"
PORT = 9870

def udp_server():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PORT))
    print(f"[服务端] 正在监听 0.0.0.0:{PORT} UDP 端口...")
    while True:
        data, addr = sock.recvfrom(1024)
        try:
            msg = json.loads(data.decode('utf-8'))
            print(f"[服务端] 收到来自 {addr} 的消息: {msg}")
        except Exception as e:
            print(f"[服务端] 收到无法解析的消息: {data}")

def udp_client():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[客户端] 开始向 {IP}:{PORT} 发送测试消息...")
    t = 0
    while True:
        msg = {"test_val": t}
        sock.sendto(json.dumps(msg).encode('utf-8'), (IP, PORT))
        t += 1
        time.sleep(1)

if __name__ == '__main__':
    threading.Thread(target=udp_server, daemon=True).start()
    time.sleep(0.5)
    udp_client()
