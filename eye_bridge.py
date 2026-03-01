import cv2
import socket
import pickle
import struct

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('0.0.0.0', 9999))
server_socket.listen(5)
print("Windows Eye Bridge is LIVE. Waiting for WSL...")

cap = cv2.VideoCapture(0)

while True:
    client_socket, addr = server_socket.accept()
    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret: break
            frame = cv2.resize(frame, (640, 480))
            data = pickle.dumps(frame)
            message = struct.pack("Q", len(data)) + data
            client_socket.sendall(message)
    except Exception as e:
        print(f"Connection closed: {e}")
        client_socket.close()