# Lab 2 – TurtleBot3 Object Follower
CS/ME/ECE/AE/BME 7785 · Georgia Institute of Technology

---

## Folder Structure

```
lab2_repo/                          ← root of your GitHub repo
├── .gitignore
├── README.md
└── team1_object_follower/          ← ROS2 package (this whole folder is the package)
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/
    │   └── team1_object_follower   ← empty marker file, required by ROS2
    ├── launch/
    │   └── object_follower.launch.py
    └── team1_object_follower/      ← Python module (same name as package)
        ├── __init__.py
        ├── find_object.py          ← detects object, publishes /object_coord
        └── rotate_robot.py         ← reads /object_coord, publishes /cmd_vel
```

---

## Step 1 — Put it on GitHub

On your Ubuntu laptop:

```bash
cd lab2_repo
git init
git add .
git commit -m "Lab 2 initial commit"

# Create a new repo on github.com, then:
git remote add origin https://github.com/YOUR_USERNAME/lab2_object_follower.git
git push -u origin main
```

---

## Step 2 — Clone onto your Ubuntu laptop

```bash
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/lab2_object_follower.git

# This gives you:
# ~/ros2_ws/src/lab2_object_follower/team1_object_follower/
```

Build it:
```bash
cd ~/ros2_ws
colcon build --packages-select team1_object_follower
source install/setup.bash
```

---

## Step 3 — Calibrate HSV for your object (on your laptop first)

Before running on the robot, figure out the correct HSV range for your object.

Run the standalone Lab 1 `find_object.py` on your laptop with your webcam:
```bash
python3 find_object.py
```
- A window opens showing your webcam feed
- **Click on your object** — the terminal prints the HSV lo/hi values
- Note down the numbers (e.g. `lo=(100, 80, 50)  hi=(130, 255, 255)`)

Then open `team1_object_follower/find_object.py` and update `DEFAULT_HSV_RANGES`:
```python
DEFAULT_HSV_RANGES = [
    HSVRange(lo=np.array([100, 80,  50], dtype=np.uint8),   # ← your values
             hi=np.array([130, 255, 255], dtype=np.uint8)),
]
```

Commit and push:
```bash
git add .
git commit -m "Calibrate HSV for my object"
git push
```

---

## Step 4 — Clone onto the TurtleBot3

SSH into the robot:
```bash
ssh burger@<ROBOT_IP>
```

On the robot:
```bash
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/lab2_object_follower.git

cd ~/ros2_ws
colcon build --packages-select team1_object_follower
source install/setup.bash
```

> **Tip:** Add `source ~/ros2_ws/install/setup.bash` to the robot's `~/.bashrc`
> so you don't have to source it every time.

---

## Step 5 — Run it

You need **3 terminals on the robot** (open via SSH from your laptop).

### Terminal 1 — Start the camera
```bash
ros2 launch turtlebot3_bringup camera_robot.launch.py
```

### Terminal 2 — Launch both nodes at once
```bash
ros2 launch team1_object_follower object_follower.launch.py
```

That's it! The robot will now rotate to follow your object.

---

## Step 6 — Watch the debug image on your laptop (optional)

Install the image transport plugin if not already done:
```bash
sudo apt-get install ros-humble-image-transport-plugins
```

Open a terminal on your **laptop** (not the robot):
```bash
# Make sure ROS_DOMAIN_ID matches the robot, then:
rqt_image_view
```
Select topic: `/find_object/compressed`

You'll see the live annotated camera feed showing where the object was detected.

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `Package not found` after build | Run `source ~/ros2_ws/install/setup.bash` |
| Robot doesn't move | Check `/object_coord` is publishing: `ros2 topic echo /object_coord` |
| Object never detected (x=-1 always) | HSV range is wrong — redo calibration |
| Robot oscillates / twitches | Increase `dead_band` to `0.15` or decrease `angular_speed` to `0.3` |
| Very slow to react | Both nodes must run ON the robot, not your laptop |
| Camera topic not found | Run `ros2 topic list` and check camera is up |

---

## Tuning Parameters

Pass overrides to the launch file like this:
```bash
ros2 launch team1_object_follower object_follower.launch.py angular_speed:=0.5 dead_band:=0.12
```

| Parameter | Default | What it does |
|---|---|---|
| `image_width` | `320` | Must match camera resolution |
| `dead_band` | `0.10` | ±10% of frame width = "centered enough" → stop turning |
| `angular_speed` | `0.4` | Base turn speed (rad/s) |
| `max_angular` | `0.8` | Max turn speed cap |
| `proportional` | `True` | Smooth P-control vs bang-bang |
| `min_area` | `900.0` | Min blob size to count as detection |
| `show_debug` | `True` | Publish annotated debug image |
