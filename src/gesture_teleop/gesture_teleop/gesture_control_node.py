#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import cv2, socket, pickle, struct, math
import mediapipe as mp

class SimpleKalmanFilter:
    def __init__(self, process_noise=1e-3, measurement_noise=0.05):
        self.q = process_noise
        self.r = measurement_noise
        self.p = 1.0
        self.x = 0.0
        self.k = 0.0

    def update(self, measurement):
        self.p = self.p + self.q
        self.k = self.p / (self.p + self.r)
        self.x = self.x + self.k * (measurement - self.x)
        self.p = (1 - self.k) * self.p
        return self.x

class GestureControlNode(Node):
    def __init__(self):
        super().__init__('gesture_control_node')
        
        self.left_pose_pub = self.create_publisher(Pose, '/left_target_pose', 10)
        self.right_pose_pub = self.create_publisher(Pose, '/right_target_pose', 10)
        self.left_grip_pub = self.create_publisher(Bool, '/left_gripper_cmd', 10)
        self.right_grip_pub = self.create_publisher(Bool, '/right_gripper_cmd', 10)
        
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.7)
        self.mp_draw = mp.solutions.drawing_utils
        
        self.kf_y_l = SimpleKalmanFilter(); self.kf_z_l = SimpleKalmanFilter()
        self.kf_y_r = SimpleKalmanFilter(); self.kf_z_r = SimpleKalmanFilter()
        
        # --- THE GOLDEN ANCHORS ---
        self.base_x = 0.45
        self.base_z = 0.71
        
        # LEFT ARM points outward to the Left (+Y direction)
        self.left_safe_quat = {'w': -0.5, 'x': -0.5, 'y': 0.5, 'z': 0.5}
        
        # RIGHT ARM points outward to the Right (-Y direction)
        # This is a 180-degree mathematical flip of the Left arm!
        self.right_safe_quat = {'w': 0.5, 'x': 0.5, 'y': 0.5, 'z': 0.5}
        
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect(('172.23.112.1', 9999)) # <--- CHECK THIS IP
        self.data = b""
        self.payload_size = struct.calcsize("Q")
        
        self.get_logger().info("--- GESTURE NODE: DUAL ANCHORS ACTIVE ---")
        self.create_timer(0.03, self.main_loop)

    def main_loop(self):
        try:
            while len(self.data) < self.payload_size: self.data += self.client_socket.recv(4096)
            packed_msg_size = self.data[:self.payload_size]
            self.data = self.data[self.payload_size:]
            msg_size = struct.unpack("Q", packed_msg_size)[0]
            while len(self.data) < msg_size: self.data += self.client_socket.recv(4096)
            frame_data = self.data[:msg_size]
            self.data = self.data[msg_size:]
            
            frame = pickle.loads(frame_data)
            results = self.hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

            if results.multi_hand_landmarks and results.multi_handedness:
                for idx, hand_lms in enumerate(results.multi_hand_landmarks):
                    self.mp_draw.draw_landmarks(frame, hand_lms, self.mp_hands.HAND_CONNECTIONS)
                    
                    lm9 = hand_lms.landmark[9]
                    thumb = hand_lms.landmark[4]
                    index = hand_lms.landmark[8]
                    
                    # True Anatomical Hand Detection
                    hand_label = results.multi_handedness[idx].classification[0].label
                    is_left_arm = (hand_label == "Right") 
                    
                    pinch_dist = math.hypot(thumb.x - index.x, thumb.y - index.y)
                    is_pinching = bool(pinch_dist < 0.05)
                    
                    msg = Pose()
                    msg.position.x = self.base_x 
                    grip_msg = Bool()
                    grip_msg.data = is_pinching

                    if is_left_arm:
                        # Left Arm Math & Orientation
                        raw_y = 1.16 + ((0.25 - lm9.x) * 1.0)
                        raw_z = self.base_z + ((0.5 - lm9.y) * 0.8)
                        
                        msg.position.y = self.kf_y_l.update(raw_y)
                        msg.position.z = self.kf_z_l.update(raw_z)
                        
                        msg.orientation.w = self.left_safe_quat['w']
                        msg.orientation.x = self.left_safe_quat['x']
                        msg.orientation.y = self.left_safe_quat['y']
                        msg.orientation.z = self.left_safe_quat['z']
                        
                        self.left_pose_pub.publish(msg)
                        self.left_grip_pub.publish(grip_msg)
                    else:
                        # Right Arm Math & Orientation (Mirrored!)
                        raw_y = -1.16 + ((0.75 - lm9.x) * 1.0)
                        raw_z = self.base_z + ((0.5 - lm9.y) * 0.8)
                        
                        msg.position.y = self.kf_y_r.update(raw_y)
                        msg.position.z = self.kf_z_r.update(raw_z)
                        
                        msg.orientation.w = self.right_safe_quat['w']
                        msg.orientation.x = self.right_safe_quat['x']
                        msg.orientation.y = self.right_safe_quat['y']
                        msg.orientation.z = self.right_safe_quat['z']
                        
                        self.right_pose_pub.publish(msg)
                        self.right_grip_pub.publish(grip_msg)
                        
            cv2.imshow("Dual Arm Control", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error( f"main_loop error: {e}", throttle_duration_sec=3.0)

def main(args=None):
    rclpy.init(args=args)
    node = GestureControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()