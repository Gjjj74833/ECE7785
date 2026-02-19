# Lab 2 – TurtleBot3 Object Follower
CS/ME/ECE/AE/BME 7785 · Georgia Institute of Technology

---

## How this relates to Lab 1

Your Lab 1 `find_object.py` detected an object from a **webcam**.  
Lab 2 takes that same detection logic and wraps it in **two ROS2 nodes**:

| | Lab 1 | Lab 2 |
|---|---|---|
| Image source | Webcam (`cv2.VideoCapture`) | TurtleBot camera (`/image_raw/compressed`) |
| Output | Prints `sx sy` to terminal, shows window | Publishes `Point` to `/object_coord` |
| Robot control | None | `rotate_robot` node reads `/object_coord` → publishes to `/cmd_vel` |

The `HSVRange`, `Calibrator`, `build_mask`, `detect_largest_blob`, `ema_smooth` functions are **identical** to Lab 1.

---

## Folder Structure

```
lab2_repo/                               ← root of your GitHub repo
├── .gitignore
├── README.md
└── team1_object_follower/               ← the ROS2 package
    ├── package.xml                      ← ROS2 metadata
    ├── setup.py                         ← entry points (how ros2 run finds your nodes)
    ├── setup.cfg
    ├── resource/
    │   └── team1_object_follower        ← empty marker file, required by ROS2
    ├── launch/
    │   └── object_follower.launch.py    ← starts both nodes at once
    └── team1_object_follower/           ← Python module (same name as package)
        ├── __init__.py                  ← empty, must exist
        ├── find_object.py               ← Lab 1 logic + ROS2 subscriber/publisher
        └── rotate_robot.py              ← reads /object_coord, drives /cmd_vel
```

---

## Step 1 — Push to GitHub (on your Ubuntu laptop)

```bash
cd lab2_repo
git init
git add .
git commit -m "Lab 2 initial commit"

# Create a new EMPTY repo on github.com, then:
git remote add origin https://github.com/YOUR_USERNAME/lab2_repo.git
git push -u origin main
```

---

## Step 2 — Clone to your Ubuntu laptop workspace

```bash
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/lab2_repo.git

# Build
cd ~/ros2_ws
colcon build --packages-select team1_object_follower
source install/setup.bash
```

---

## Step 3 — Test on your laptop first (before touching the robot)

Your laptop doesn't have the TurtleBot camera, but you can still test that
the nodes start and talk to each other correctly.

**Terminal 1** — start find_object (it will just wait for images):
```bash
source ~/ros2_ws/install/setup.bash
ros2 run team1_object_follower find_object
```

**Terminal 2** — check the topic exists:
```bash
ros2 topic list
# You should see /object_coord in the list
```

**Terminal 3** — start rotate_robot and watch it respond:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run team1_object_follower rotate_robot
```

**Terminal 4** — manually publish a fake coordinate to test rotation logic:
```bash
# Simulate object on the RIGHT (x=250 on a 320-wide frame, center=160)
ros2 topic pub /object_coord geometry_msgs/msg/Point "{x: 250.0, y: 120.0, z: 1000.0}"

# Simulate object on the LEFT
ros2 topic pub /object_coord geometry_msgs/msg/Point "{x: 50.0, y: 120.0, z: 1000.0}"

# Simulate no object found
ros2 topic pub /object_coord geometry_msgs/msg/Point "{x: -1.0, y: -1.0, z: -1.0}"
```

Watch Terminal 3 — you should see `rotate_robot` log which direction it would turn.
The robot won't actually move (no robot connected) but the logic is verified.

---

## Step 4 — Clone to the TurtleBot3

SSH into the robot:
```bash
ssh burger@<ROBOT_IP>
```

On the robot:
```bash
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/lab2_repo.git

cd ~/ros2_ws
colcon build --packages-select team1_object_follower
source install/setup.bash

# Optional: add to .bashrc so you don't need to source every time
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Step 5 — Run on the robot

You need **2 SSH terminals into the robot**.

**Terminal 1** — start the camera + robot bringup:
```bash
ros2 launch turtlebot3_bringup camera_robot.launch.py
```

**Terminal 2** — launch both your nodes:
```bash
ros2 launch team1_object_follower object_follower.launch.py
```

The robot will now rotate toward your object.

---

## Step 6 — View the debug image on your laptop (optional)

Install transport plugin if not already done (on your laptop):
```bash
sudo apt-get install ros-humble-image-transport-plugins
```

```bash
rqt_image_view
# Select topic: /find_object/compressed
# You'll see the live annotated camera feed from the robot
```

---

## Step 7 — Updating code (the git workflow)

```bash
# On your laptop — edit code, then push
git add .
git commit -m "describe your change"
git push

# On the robot — pull and rebuild
cd ~/ros2_ws/src/lab2_repo
git pull
cd ~/ros2_ws
colcon build --packages-select team1_object_follower
source install/setup.bash
```

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `Package not found` | Run `source ~/ros2_ws/install/setup.bash` |
| Robot doesn't turn | Check `/object_coord` is publishing: `ros2 topic echo /object_coord` |
| Object never detected (`x: -1`) | HSV range is wrong — click on the object in the debug window to recalibrate |
| Robot oscillates | Increase `dead_band` to `0.15`, or decrease `angular_speed` to `0.3` |
| Very laggy | Both nodes must run **on the robot**, not your laptop |

---

## Topic Map

```
[camera_robot.launch]
        │  /image_raw/compressed  (CompressedImage)
        ▼
[find_object node]  ──►  /find_object/compressed  (debug image → laptop)
        │  /object_coord  (Point: x=col, y=row, z=area or z=-1 if not found)
        ▼
[rotate_robot node]
        │  /cmd_vel  (Twist: only angular.z, no linear)
        ▼
[robot motors]
```
