# gesture_bot_ws
Task given by Anmol Gupta

Hello Team Sereact, Thank you for the wonderful opportunity that you have give me.
I have implemented the given task using WSL2, ROS2 Humble and inbuilt laptop's camera.

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