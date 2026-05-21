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

    def reset(self, value=0.0):
        self.x = value
        self.p = 1.0

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

        self.left_pose_pub  = self.create_publisher(Pose, '/left_target_pose',   10)
        self.right_pose_pub = self.create_publisher(Pose, '/right_target_pose',  10)
        self.left_grip_pub  = self.create_publisher(Bool, '/left_gripper_cmd',   10)
        self.right_grip_pub = self.create_publisher(Bool, '/right_gripper_cmd',  10)

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.7)
        self.mp_draw = mp.solutions.drawing_utils

        self.kf_y_l    = SimpleKalmanFilter()
        self.kf_z_l    = SimpleKalmanFilter()
        self.kf_roll_l = SimpleKalmanFilter()
        self.kf_y_r    = SimpleKalmanFilter()
        self.kf_z_r    = SimpleKalmanFilter()
        self.kf_roll_r = SimpleKalmanFilter()

        self.base_x    = 0.45
        self.base_z    = 0.71
        self.max_roll  = math.radians(60)
        self.left_safe_quat  = {'w': -0.5, 'x': -0.5, 'y':  0.5, 'z': 0.5}
        self.right_safe_quat = {'w':  0.5, 'x':  0.5, 'y':  0.5, 'z': 0.5}

        self.left_anchor_hand   = None
        self.left_anchor_robot  = None
        self.left_last_robot    = (1.16, self.base_z) 

        self.right_anchor_hand  = None
        self.right_anchor_robot = None
        self.right_last_robot   = (-1.16, self.base_z)

        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect(('172.23.112.1', 9999))  # <--- CHECK THIS IP
        self.data = b""
        self.payload_size = struct.calcsize("Q")

        self.get_logger().info("--- GESTURE NODE: DELTA MAPPING ACTIVE ---")
        self.create_timer(0.03, self.main_loop)

    def _compute_delta_pose(self, lm9, anchor_hand, anchor_robot, is_left):

        delta_hand_x = lm9.x - anchor_hand[0]   # camera x delta (rightward +)
        delta_hand_y = lm9.y - anchor_hand[1]   # camera y delta (downward  +)

        robot_y = anchor_robot[0] + (-delta_hand_x * 1.0)
        robot_z = anchor_robot[1] + (-delta_hand_y * 0.8)

        return robot_y, robot_z

    def main_loop(self):
        try:
            while len(self.data) < self.payload_size:
                self.data += self.client_socket.recv(4096)
            packed_msg_size = self.data[:self.payload_size]
            self.data = self.data[self.payload_size:]
            msg_size = struct.unpack("Q", packed_msg_size)[0]
            while len(self.data) < msg_size:
                self.data += self.client_socket.recv(4096)
            frame_data = self.data[:msg_size]
            self.data = self.data[msg_size:]

            frame   = pickle.loads(frame_data)
            results = self.hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            left_seen  = False
            right_seen = False

            if results.multi_hand_landmarks and results.multi_handedness:
                for idx, hand_lms in enumerate(results.multi_hand_landmarks):
                    self.mp_draw.draw_landmarks(frame, hand_lms, self.mp_hands.HAND_CONNECTIONS)

                    lm9   = hand_lms.landmark[9]
                    thumb = hand_lms.landmark[4]
                    index = hand_lms.landmark[8]
                    wrist = hand_lms.landmark[0]

                    hand_label = results.multi_handedness[idx].classification[0].label
                    is_left_arm = (hand_label == "Right")   # mirrored camera

                    pinch_dist  = math.hypot(thumb.x - index.x, thumb.y - index.y)
                    is_pinching = bool(pinch_dist < 0.08)
                    raw_roll    = compute_hand_roll(wrist, lm9)

                    grip_msg      = Bool()
                    grip_msg.data = is_pinching
                    msg           = Pose()
                    msg.position.x = self.base_x

                    if is_left_arm:
                        left_seen = True
                        if self.left_anchor_hand is None:
                            self.left_anchor_hand  = (lm9.x, lm9.y)
                            self.left_anchor_robot = self.left_last_robot
                            self.kf_y_l.reset(self.left_anchor_robot[0])
                            self.kf_z_l.reset(self.left_anchor_robot[1])
                            self.get_logger().info(
                                f"LEFT anchor set: hand=({lm9.x:.3f},{lm9.y:.3f}) "
                                f"robot=({self.left_anchor_robot[0]:.3f},{self.left_anchor_robot[1]:.3f})"
                            )

                        raw_y, raw_z = self._compute_delta_pose(
                            lm9, self.left_anchor_hand, self.left_anchor_robot, is_left=True
                        )

                        smooth_y = self.kf_y_l.update(raw_y)
                        smooth_z = self.kf_z_l.update(raw_z)

                        msg.position.y = smooth_y
                        msg.position.z = smooth_z
                        self.left_last_robot = (smooth_y, smooth_z)   # update for next entry

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
                        right_seen = True
                        if self.right_anchor_hand is None:
                            self.right_anchor_hand  = (lm9.x, lm9.y)
                            self.right_anchor_robot = self.right_last_robot
                            self.kf_y_r.reset(self.right_anchor_robot[0])
                            self.kf_z_r.reset(self.right_anchor_robot[1])
                            self.get_logger().info(
                                f"RIGHT anchor set: hand=({lm9.x:.3f},{lm9.y:.3f}) "
                                f"robot=({self.right_anchor_robot[0]:.3f},{self.right_anchor_robot[1]:.3f})"
                            )

                        raw_y, raw_z = self._compute_delta_pose(
                            lm9, self.right_anchor_hand, self.right_anchor_robot, is_left=False
                        )

                        smooth_y = self.kf_y_r.update(raw_y)
                        smooth_z = self.kf_z_r.update(raw_z)

                        msg.position.y = smooth_y
                        msg.position.z = smooth_z
                        self.right_last_robot = (smooth_y, smooth_z)  # update for next entry

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

            if not left_seen and self.left_anchor_hand is not None:
                self.get_logger().info("LEFT hand lost — anchor cleared.")
                self.left_anchor_hand  = None
                self.left_anchor_robot = None

            if not right_seen and self.right_anchor_hand is not None:
                self.get_logger().info("RIGHT hand lost — anchor cleared.")
                self.right_anchor_hand  = None
                self.right_anchor_robot = None

            cv2.imshow("Dual Arm Control", frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"main_loop error: {e}", throttle_duration_sec=3.0)

    def _draw_debug(self, frame, hand_lms, roll_rad, label):
        h, w   = frame.shape[:2]
        wrist  = hand_lms.landmark[0]
        cx     = int(wrist.x * w)
        cy     = int(wrist.y * h)
        roll_deg  = math.degrees(roll_rad)
        color     = (0, 255, 0) if label == "L" else (0, 128, 255)
        start_deg = -90
        end_deg   = int(-90 + roll_deg)
        cv2.ellipse(frame, (cx, cy), (30, 30), 0, start_deg, end_deg, color, 2)
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