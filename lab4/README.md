# Lab 4 – Navigate to Goal with Obstacle Avoidance
CS/ME/ECE/AE/BME 7785 · Georgia Institute of Technology

---

## What this does

The robot navigates through 3 waypoints using odometry (dead reckoning), avoiding any unknown obstacle placed in its path.

| Waypoint | Position | Stop tolerance | Stop time |
|---|---|---|---|
| WP1 | (1.5m, 0.0m) | ±10 cm | 10 seconds |
| WP2 | (1.5m, 1.4m) | ±15 cm | 10 seconds |
| WP3 | (0.0m, 1.4m) | ±20 cm | 10 seconds |

---

## Architecture

```
/odom  (nav_msgs/Odometry)
    └──→  go_to_goal  ──→  /cmd_vel  →  Robot motors
              ↑
/obstacle_vector  (geometry_msgs/Point)
    ↑
get_object_range  ←──  /scan  (sensor_msgs/LaserScan)
```

### Nodes

**`get_object_range`** — filters 360° LIDAR scan, clusters beams into objects, publishes nearest obstacle as `Point(x=distance, y=angle, z=valid_flag)` on `/obstacle_vector`.

**`go_to_goal`** — state machine with 4 states:
- `GO_TO_GOAL`: P controllers for heading and linear distance
- `AVOID_OBSTACLE`: turns away from obstacle until clear
- `STOP_AT_GOAL`: stops 10 seconds at each waypoint
- `DONE`: all waypoints reached, robot stops

Uses `RotationScript` logic to zero the odometry at startup so the robot always starts at (0,0) regardless of prior position.

---

## Setup & Run

```bash
# On robot — pull latest code
cd ~/ros2_ws/src/ECE7785
git pull

# Build
cd ~/ros2_ws
colcon build --packages-select team1_navigate_to_goal
source ~/.bashrc
```

**Terminal 1** — hardware bringup:
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

**Terminal 2** — navigate:
```bash
ros2 launch team1_navigate_to_goal navigate_to_goal.launch.py
```

---

## Tuning parameters

```bash
# Tighten obstacle detection distance (default 0.5m)
ros2 launch team1_navigate_to_goal navigate_to_goal.launch.py max_obstacle_dist:=0.45
```

Key values to tune in `go_to_goal.py`:

| Parameter | Default | Effect |
|---|---|---|
| `KP_LINEAR` | 0.4 | Forward speed proportional gain |
| `KP_ANGULAR` | 1.8 | Turning speed proportional gain |
| `MAX_LINEAR` | 0.15 m/s | Hard speed cap |
| `OBSTACLE_STOP_DIST` | 0.40 m | Start avoiding at this range |
| `OBSTACLE_CLEAR_DIST` | 0.60 m | Resume when obstacle beyond this |
| `AVOID_ANGULAR_SPEED` | 0.8 rad/s | Turn speed during avoidance |

---

## Troubleshooting

| Problem | Fix |
|---|---|
| Robot doesn't start at (0,0) | Place robot on floor, restart bringup, then launch your nodes |
| Odometry drifting while stationary | Restart bringup |
| Robot misses waypoints | Check `ros2 topic echo /odom` — verify position is updating |
| Robot hits obstacle | Decrease `OBSTACLE_STOP_DIST` |
| Robot spins forever avoiding | Increase `OBSTACLE_CLEAR_DIST` or adjust `AVOID_ANGULAR_SPEED` |
