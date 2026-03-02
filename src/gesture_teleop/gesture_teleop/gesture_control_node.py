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

def quaternion_multiply(q1, q2):
    w = q1['w']*q2['w'] - q1['x']*q2['x'] - q1['y']*q2['y'] - q1['z']*q2['z']
    x = q1['w']*q2['x'] + q1['x']*q2['w'] + q1['y']*q2['z'] - q1['z']*q2['y']
    y = q1['w']*q2['y'] - q1['x']*q2['z'] + q1['y']*q2['w'] + q1['z']*q2['x']
    z = q1['w']*q2['z'] + q1['x']*q2['y'] - q1['y']*q2['x'] + q1['z']*q2['w']
    return {'w': w, 'x': x, 'y': y, 'z': z}

def make_rotation_z(angle_rad):
    half = angle_rad / 2.0
    return {'w': math.cos(half), 'x': 0.0, 'y': 0.0, 'z': math.sin(half)}

def compute_hand_roll(wrist, middle_mcp):
    dx = middle_mcp.x - wrist.x
    dy = -(middle_mcp.y - wrist.y)  
    return math.atan2(dx, dy) 

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
        
        self.kf_y_l = SimpleKalmanFilter(); self.kf_z_l = SimpleKalmanFilter(); self.kf_roll_l = SimpleKalmanFilter()
        self.kf_y_r = SimpleKalmanFilter(); self.kf_z_r = SimpleKalmanFilter(); self.kf_roll_r = SimpleKalmanFilter()
        
        self.base_x = 0.45
        self.base_z = 0.71
        self.max_roll = math.radians(60)
        self.left_safe_quat = {'w': -0.5, 'x': -0.5, 'y': 0.5, 'z': 0.5}

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
                    wrist = hand_lms.landmark[0]
                    
                    hand_label = results.multi_handedness[idx].classification[0].label
                    is_left_arm = (hand_label == "Right") 
                    
                    pinch_dist = math.hypot(thumb.x - index.x, thumb.y - index.y)
                    is_pinching = bool(pinch_dist < 0.08)
                    
                    raw_roll = compute_hand_roll(wrist, lm9)
                    msg = Pose()
                    msg.position.x = self.base_x 
                    grip_msg = Bool()
                    grip_msg.data = is_pinching

                    if is_left_arm: 
                        raw_y = 1.16 + ((0.25 - lm9.x) * 1.0)
                        raw_z = self.base_z + ((0.5 - lm9.y) * 0.8)
                        
                        msg.position.y = self.kf_y_l.update(raw_y)
                        msg.position.z = self.kf_z_l.update(raw_z)

                        smooth_roll = self.kf_roll_l.update(raw_roll)
                        smooth_roll = max(-self.max_roll, min(self.max_roll, smooth_roll))
                        
                        q_rot   = make_rotation_z(smooth_roll)
                        q_final = quaternion_multiply(self.left_safe_quat, q_rot)

                        msg.orientation.w = q_final['w']
                        msg.orientation.x = q_final['x']
                        msg.orientation.y = q_final['y']
                        msg.orientation.z = q_final['z']
                        
                        self.left_pose_pub.publish(msg)
                        self.left_grip_pub.publish(grip_msg)
                        self._draw_debug(frame, hand_lms, smooth_roll, "L")
                    else:
                        raw_y = -1.16 + ((0.75 - lm9.x) * 1.0)
                        raw_z = self.base_z + ((0.5 - lm9.y) * 0.8)
                        
                        msg.position.y = self.kf_y_r.update(raw_y)
                        msg.position.z = self.kf_z_r.update(raw_z)
                        
                        smooth_roll = self.kf_roll_r.update(raw_roll)
                        smooth_roll = max(-self.max_roll, min(self.max_roll, smooth_roll))

                        q_rot   = make_rotation_z(smooth_roll)
                        q_final = quaternion_multiply(self.right_safe_quat, q_rot)

                        msg.orientation.w = q_final['w']
                        msg.orientation.x = q_final['x']
                        msg.orientation.y = q_final['y']
                        msg.orientation.z = q_final['z']
                        
                        self.right_pose_pub.publish(msg)
                        self.right_grip_pub.publish(grip_msg)
                        self._draw_debug(frame, hand_lms, smooth_roll, "R")
                        
            cv2.imshow("Dual Arm Control", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error( f"main_loop error: {e}", throttle_duration_sec=3.0)

    def _draw_debug(self, frame, hand_lms, roll_rad, label):
        h, w = frame.shape[:2]
        wrist = hand_lms.landmark[0]
        cx    = int(wrist.x * w)
        cy    = int(wrist.y * h)
        roll_deg = math.degrees(roll_rad)
        color    = (0, 255, 0) if label == "L" else (0, 128, 255)
        axes       = (30, 30)
        start_deg  = -90
        end_deg    = int(-90 + roll_deg)
        cv2.ellipse(frame, (cx, cy), axes, 0, start_deg, end_deg, color, 2)
        cv2.putText(
            frame,
            f"{label}: {roll_deg:+.0f}deg",
            (cx - 40, cy - 40),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1
        ) 

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