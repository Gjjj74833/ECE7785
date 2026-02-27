# Lab 3 – Chase Object
CS/ME/ECE/AE/BME 7785 · Georgia Institute of Technology

---

## What this does vs Lab 2

| | Lab 2 | Lab 3 |
|---|---|---|
| Sensors | Camera only | Camera + LIDAR |
| Output | Angle (pixels) | Angle (radians) + Distance (meters) |
| Motion | Rotate only | Rotate AND drive forward/back |
| Control | Bang-bang / P | PD controllers |

---

## Architecture

```
[camera]  /image_raw/compressed
              │
              ▼
      ┌──────────────────┐
      │  detect_object   │  converts pixel → angle (radians)
      └──────────────────┘
              │ /object_angle  (Float32, radians)
              ▼
      ┌──────────────────┐ ◄── /scan  (LaserScan from LIDAR)
      │ get_object_range │  fuses camera angle with LIDAR distance
      └──────────────────┘
              │ /object_range  (Point: x=angle, y=distance, z=valid flag)
              ▼
      ┌──────────────────┐
      │  chase_object    │  dual PD controller
      └──────────────────┘
              │ /cmd_vel  (Twist)
              ▼
         [robot motors]
```

---

## Folder Structure

```
lab3/
├── writeup.md                          ← answers to all 5 questions
└── team1_chase_object/                 ← ROS2 package
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/
    │   └── team1_chase_object
    ├── launch/
    │   └── chase_object.launch.py
    └── team1_chase_object/
        ├── __init__.py
        ├── detect_object.py            ← camera → /object_angle
        ├── get_object_range.py         ← camera + LIDAR → /object_range
        └── chase_object.py             ← PD control → /cmd_vel
```

---

## Setup

```bash
# Copy package to robot or clone from GitHub
scp -r team1_chase_object/ burger@<ROBOT_IP>:~/ros2_ws/src/

# Build on robot
cd ~/ros2_ws
colcon build --packages-select team1_chase_object
source install/setup.bash
```

---

## Running

**Terminal 1** – camera + robot bringup:
```bash
ros2 launch turtlebot3_bringup camera_robot.launch.py
```

**Terminal 2** – all three Lab 3 nodes:
```bash
ros2 launch team1_chase_object chase_object.launch.py
```

---

## Tuning PID Gains

```bash
# Start with defaults, then tune:
ros2 launch team1_chase_object chase_object.launch.py \
    desired_distance:=0.5 \
    ang_kp:=1.5 ang_kd:=0.05 \
    lin_kp:=0.5 lin_kd:=0.1
```

**Tuning guide:**
| Symptom | Fix |
|---|---|
| Robot overshoots angle, oscillates | Increase `ang_kd` or decrease `ang_kp` |
| Robot too slow to face object | Increase `ang_kp` |
| Robot overshoots distance, drives back and forth | Increase `lin_kd` or decrease `lin_kp` |
| Robot stops too far / too close | Adjust `desired_distance` |
| Robot won't stop spinning | `ang_kp` too high — decrease it |

---

## Verify sensor data

```bash
# Check LIDAR is publishing
ros2 topic echo /scan --once

# Check camera angle is being detected
ros2 topic echo /object_angle

# Check fused range output
ros2 topic echo /object_range

# View annotated debug image (on laptop)
rqt_image_view   # select /detect_object/compressed
```
