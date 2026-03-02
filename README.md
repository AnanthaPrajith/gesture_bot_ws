# gesture_bot_ws
Task given by Anmol Gupta, Sereact, Thank you for the wonderful opportunity that you have give me.
I have implemented the given task using WSL2, ROS2 Humble, Docker and inbuilt laptop's camera.


![Project Screenshot](assets/image2.png)  ![Project Screenshot](assets/image1.png)


# How to setup on Windows
1. Clone the repository for Github.
```bash
git clone https://github.com/Dharnish08/gesture_bot_ws.git
```
2. Now run the python script "eye_bridge.py" on Windows Terminal. This will act as a bridge for camera between windows and WSL2.
```bash
python3 ~/gesture_bot_ws/eye_bridge.py
```
3. After successfully running the above script, now inside WSL2 run the following line of command. This might take few minutes to finish running.
```bash
docker compose build
```
4. After successfully building the docker image, now run the following command. This will open a TMUX split terminals, you can toggle between terminals using **ctrl+B** then → ← ↑ ↓ .
```bash
docker compose run --rm gesture_bot
```
5. Now the rviz window pop-ups. In left side there is **Display**, there change the **Fixed Frame** from **map** to **world**.
6. Then in bottom left you could see **Add**, click and select **MotionPlanning**.
7. In **MotionPlanning** window select **Context**, in **Planning Library** select the unspecified planner to **RRTConnectkConfigDefault**.
8. For better view, you can uncheck the **Query Goal State** in **Planning Request**.

# How to setup on Ubuntu
1. Clone the repository for Github.
```bash
git clone https://github.com/Dharnish08/gesture_bot_ws.git
```
2. Find the camera port 0, 1, 2. select the camera.
```bash
ls/dev/video*
```
3. Update the same in **eye_bridge.py** 
```bash
cap = cv2.VideoCapture(0) # change 0 to your camera /dev/video*
```
4. Now run the python script "eye_bridge.py" on Terminal.
```bash
python3 ~/gesture_bot_ws/eye_bridge.py
```
5. Now open a new terminal, build the docker image. This might take few minutes
```bash
docker compose build
```
6. After successfully building the docker image, now run the following command. This will open a TMUX split terminals, you can toggle between terminals using **ctrl+B** then → ← ↑ ↓ .
```bash
docker compose run --rm gesture_bot
```
7. Now the rviz window pop-ups. In left side there is **Display**, there change the **Fixed Frame** from **map** to **world**.
8. Then in bottom left you could see **Add**, click and select **MotionPlanning**.
9. In **MotionPlanning** window select **Context**, in **Planning Library** select the unspecified planner to **RRTConnectkConfigDefault**.
10. For better view, you can uncheck the **Query Goal State** in **Planning Request**.

# Publisher Subscriber detailed table
| Node | Type | Topic | Message |
|---|---|---|---|
| `gesture_control_node` | Publisher | `/left_target_pose` | `geometry_msgs/Pose` |
| `gesture_control_node` | Publisher | `/right_target_pose` | `geometry_msgs/Pose` |
| `gesture_control_node` | Publisher | `/left_gripper_cmd` | `std_msgs/Bool` |
| `gesture_control_node` | Publisher | `/right_gripper_cmd` | `std_msgs/Bool` |
| `ik_commander` | Subscriber | `/left_target_pose` | `geometry_msgs/Pose` |
| `ik_commander` | Subscriber | `/right_target_pose` | `geometry_msgs/Pose` |
| `ik_commander` | Subscriber | `/left_gripper_cmd` | `std_msgs/Bool` |
| `ik_commander` | Subscriber | `/right_gripper_cmd` | `std_msgs/Bool` |
| `ik_commander` | Publisher | `/joint_states` | `sensor_msgs/JointState` |
| `move_group` | Subscriber | `/joint_states` | `sensor_msgs/JointState` |
| `move_group` | Service Server | `/compute_ik` | `moveit_msgs/GetPositionIK` |
| `ik_commander` | Service Client | `/compute_ik` | `moveit_msgs/GetPositionIK` |
| `robot_state_publisher` | Subscriber | `/joint_states` | `sensor_msgs/JointState` |

# Attention Points
>[!NOTE]
> 1. handled noise using Kalman Filter.
> 2. Robot arms will subscribe to the published messages and reach the joint values within milliseconds.
> 3. one human operator.
> ```bash
> self.hands = self.mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.7)
> ```
> 4. Robots movement is scaled to the hand movement within the camera.
> 5. No sudden arm jump uses seed state, 20Hz join publishing fills gaps smoothly, holds the last valid position.

# Credits
>[!IMPORTANT] 
> 1. Thank you UniversalRobots for your Description Repo
> https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git 
> 2. Thank you PickNikRobotics for your Ros2_robotic_gripper Repo
> https://github.com/PickNikRobotics/ros2_robotiq_gripper.git
> 3. Thank you Willian Woodall for your Serial Repo
> https://github.com/wjwwood/serial.git