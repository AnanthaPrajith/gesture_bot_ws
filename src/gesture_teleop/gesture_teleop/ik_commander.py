#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState

class IKCommander(Node):
    def __init__(self):
        super().__init__('ik_commander')
        
        self.create_subscription(Pose, '/left_target_pose', self.left_pose_callback, 10)
        self.create_subscription(Pose, '/right_target_pose', self.right_pose_callback, 10)
        self.create_subscription(Bool, '/left_gripper_cmd', self.left_grip_callback, 10)
        self.create_subscription(Bool, '/right_gripper_cmd', self.right_grip_callback, 10)
        
        self.left_grip_closed = False
        self.right_grip_closed = False
        self.master_joint_state = {
        'left_shoulder_pan_joint':   0.0,
        'left_shoulder_lift_joint': -1.5708,
        'left_elbow_joint':          1.5708,
        'left_wrist_1_joint':       -1.5708,
        'left_wrist_2_joint':       -1.5708,   
        'left_wrist_3_joint':        0.0,       
        'right_shoulder_pan_joint':  0.0,
        'right_shoulder_lift_joint':-1.5708,
        'right_elbow_joint':         1.5708,
        'right_wrist_1_joint':      -1.5708,
        'right_wrist_2_joint':       1.5708,   
        'right_wrist_3_joint':       0.0, 
    } 
        
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for MoveIt IK brain...')
            
        self.get_logger().info('--- SILENT COMMANDER LIVE ---')
        self.create_timer(0.05, self.publish_merged_joints)

    def left_grip_callback(self, msg): self.left_grip_closed = msg.data
    def right_grip_callback(self, msg): self.right_grip_closed = msg.data

    def left_pose_callback(self, msg): self.request_ik('left_arm', msg, self.left_grip_closed)
    def right_pose_callback(self, msg): self.request_ik('right_arm', msg, self.right_grip_closed)

    def request_ik(self, group_name, pose_msg, gripper_closed):
        req = GetPositionIK.Request()
        req.ik_request.group_name = group_name
        req.ik_request.timeout.sec = 0
        req.ik_request.timeout.nanosec = 50000000 

        rs = RobotState()
        js = JointState()
        if len(self.master_joint_state) > 0:
            js.name = list(self.master_joint_state.keys())
            js.position = list(self.master_joint_state.values())
        else:
            js.name = ['left_shoulder_pan_joint'] 
            js.position = [0.0]
            
        rs.joint_state = js
        rs.is_diff = True 
        req.ik_request.robot_state = rs
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'world'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose_msg 
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.avoid_collisions = True 
        
        future = self.ik_client.call_async(req)
        future.add_done_callback(lambda f: self.ik_callback(f, group_name, gripper_closed))

    def ik_callback(self, future, group_name, gripper_closed):
        try:
            response = future.result()
            if response.error_code.val == 1: 
                joint_state_msg = response.solution.joint_state
                for i in range(len(joint_state_msg.name)):
                    self.master_joint_state[joint_state_msg.name[i]] = joint_state_msg.position[i]
                
                prefix = 'left_' if group_name == 'left_arm' else 'right_'
                gripper_joint = f'{prefix}robotiq_85_left_knuckle_joint'
                self.master_joint_state[gripper_joint] = 0.8 if gripper_closed else 0.0 
            else:
                self.get_logger().warning(f"[{group_name}] IK FAILED! Error Code: {response.error_code.val}")
        except Exception as e:
            self.get_logger().error(f"IK callback exception: {e}")

    def publish_merged_joints(self):
        all_joints = [
            'left_shoulder_pan_joint', 'left_shoulder_lift_joint', 'left_elbow_joint',
            'left_wrist_1_joint', 'left_wrist_2_joint', 'left_wrist_3_joint',
            'left_robotiq_85_left_knuckle_joint',
            'right_shoulder_pan_joint', 'right_shoulder_lift_joint', 'right_elbow_joint',
            'right_wrist_1_joint', 'right_wrist_2_joint', 'right_wrist_3_joint',
            'right_robotiq_85_left_knuckle_joint'
        ]
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        names = []
        positions = []
        
        for name in all_joints:
            names.append(name)
            if name in self.master_joint_state:
                positions.append(self.master_joint_state[name])
            else:
                positions.append(-1.57 if 'lift' in name or 'wrist_1' in name else 0.0)

        msg.name = names
        msg.position = positions
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IKCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()