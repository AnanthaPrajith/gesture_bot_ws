#!/usr/bin/env python
import cv2
import socket
import pickle
import struct
import mediapipe as mp

# Try the new-style import if the old one fails
try:
    from mediapipe.python.solutions import hands as mp_hands
    from mediapipe.python.solutions import drawing_utils as mp_draw
except:
    mp_hands = mp.solutions.hands
    mp_draw = mp.solutions.drawing_utils

hands = mp_hands.Hands(static_image_mode=False, max_num_hands=2, min_detection_confidence=0.7)
mp_draw = mp.solutions.drawing_utils

# 2. Connect to the Windows Bridge
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host_ip = '172.23.112.1'
port = 9999
client_socket.connect((host_ip, port))

data = b""
payload_size = struct.calcsize("Q")

print("WSL is connected to Windows! Hold up your hands...")

while True:
    # Receive frame size
    while len(data) < payload_size:
        packet = client_socket.recv(4096)
        if not packet: break
        data += packet
    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack("Q", packed_msg_size)[0]

    # Receive frame data
    while len(data) < msg_size:
        data += client_socket.recv(4096)
    frame_data = data[:msg_size]
    data = data[msg_size:]
    
    # Decode and Process
    frame = pickle.loads(frame_data)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_frame)

    # Draw landmarks on the frame
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            
    cv2.imshow("WSL Hand Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

client_socket.close()
cv2.destroyAllWindows()