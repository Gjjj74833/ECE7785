# Lab 3 Writeup – Chase Object
CS/ME/ECE/AE/BME 7785 · Georgia Institute of Technology

---

## Question 1 – Sampling Time

**What is the sampling time of your system?**

The system has two sensor inputs with different rates:

- **Camera** (`/image_raw/compressed`): ~30 Hz → sampling period ≈ 33 ms
- **LIDAR** (`/scan`): ~5 Hz on the TurtleBot3 LDS-02 → sampling period ≈ 200 ms

The `get_object_range` node fuses these on a 10 Hz timer (100 ms period), which is limited by the slower LIDAR sensor.

**How does sampling time affect theoretical guarantees?**

Continuous-time PID theory assumes infinitely fast sampling. In discrete time, stability guarantees only hold when the sampling frequency is significantly higher than the system's bandwidth (Nyquist criterion). Specifically, the sampling rate must be at least 10–20× faster than the closed-loop bandwidth for the discrete controller to closely approximate the designed continuous controller. If the sampling period approaches the system's time constant, phase lag introduced by the zero-order hold can destabilize an otherwise stable controller.

**Does sampling time impact behavior in practice for this lab?**

In practice for this lab, the sampling time has limited impact because:
1. The robot's mechanical dynamics (inertia, friction) are much slower than 200 ms — the robot itself acts as a low-pass filter.
2. The object being tracked moves slowly relative to the sensor update rate.
3. The onboard motor PID controllers run at a much faster rate and handle the fast dynamics independently.

However, at 5 Hz LIDAR updates, there is noticeable lag when the object moves quickly, which can cause the linear controller to overshoot slightly before correcting.

---

## Question 2 – PID Variant Used

**What variant of PID did you use and why?**

We used **PD control** (Proportional-Derivative) for both the angular and linear controllers.

- **Proportional term (P):** Drives the robot toward the goal proportionally to the error. Larger error → faster response.
- **Derivative term (D):** Adds damping. As the robot approaches the target (error decreasing), D generates a braking force that prevents overshoot and oscillation.
- **Integral term (I): Set to 0** for the following reasons:
  1. The TurtleBot3's onboard motor controllers already include integral action at the motor level. Adding I at the high level creates nested integrators, which can destabilize the system.
  2. Integral windup is a significant risk during large angular errors (e.g., object not visible, robot spinning). During windup, the integrator accumulates a large value that causes the robot to overshoot badly when the object is found again.
  3. As explained in Question 3, P-alone (and P+D) is sufficient to reject the friction disturbances present on this robot.

---

## Question 3 – Steady State with P-Only and Integral Windup

**Why can a P-only controller achieve steady state despite friction?**

In continuous-time systems, a P-only controller cannot reject a constant step disturbance (like friction) because at steady state the error is nonzero — the P output exactly balances the disturbance but does not eliminate the error. This is the standard steady-state error analysis.

However, on the TurtleBot3, the **onboard motor PID controllers** already include integral action. These low-level controllers ensure that even when the high-level command is a constant (nonzero) velocity, the motors continue to accelerate until the commanded velocity is reached. This effectively means friction is already compensated at the motor level before the high-level controller ever sees it. Therefore, from the perspective of the high-level P controller, there is no persistent disturbance to reject — the motor controllers handle it.

**What is integral windup?**

Integral windup occurs when the integrator in a PID controller accumulates a very large value during a period when the controller output is saturated (e.g., at its maximum limit) or when the system is unable to respond (e.g., object not visible). When the saturation condition ends, the large integral term causes a large, sustained overshoot.

**How to avoid integral windup:**

1. **Clamping:** Limit the integral term to a maximum value (`integral_limit` in our `PIDController` class).
2. **Conditional integration (integrator anti-windup):** Only integrate when the controller output is not saturated.
3. **Reset on loss of tracking:** Reset the integrator to 0 whenever the object is lost (implemented in `chase_object.py` when `msg.z < 0`).
4. **Back-calculation:** Feed the difference between actual and saturated output back to reduce the integrator proportionally.

---

## Question 4 – System Instability

**What does it mean for a system to be unstable?**

A linear control system is unstable if its output grows without bound in response to a bounded input, or does not return to equilibrium after a disturbance. Mathematically, instability occurs when the closed-loop poles have positive real parts (in continuous time) or lie outside the unit circle (in discrete time).

**What behaviors will the robot display with an unstable controller?**

- **Unstable angular controller:** The robot will oscillate with growing amplitude — it will overshoot past the object, correct back, overshoot again, each time going further. Eventually it will spin continuously, never settling on the target angle. In the extreme case it will spin at maximum speed indefinitely.

- **Unstable linear controller:** The robot will oscillate back and forth relative to the object with increasing amplitude. It will drive toward the object, overshoot (getting too close), then reverse, overshoot in the other direction (getting too far), and so on, with each oscillation larger than the last. At the extreme, it alternates between full-speed forward and full-speed reverse.

Both instabilities can be triggered by setting gains too high (gain margin exceeded) or by adding too much integral action (phase margin exceeded).

---

## Question 5 – Algorithm for Object Localization

**How do we determine where the object is relative to the robot?**

### Step 1: Camera → Angular Position

The Raspberry Pi Camera v2 has a horizontal field of view of 62.2° at 320×240 resolution.

Given the object's pixel column `cx` in a 320-pixel-wide image:

```
pixel_offset = cx - (image_width / 2)          # pixels from center
rad_per_px   = HFOV_rad / image_width           # = 1.086 rad / 320 = 0.003394 rad/px
angle_camera = pixel_offset × rad_per_px        # radians, + = right of center
```

This gives the bearing of the object relative to the camera's forward axis.

### Step 2: Camera Angle → LIDAR Index

The camera and LIDAR share the same forward axis on the TurtleBot3 Burger, but use opposite sign conventions:

```
Camera: + = right,  − = left
LIDAR:  + = left (CCW),  − = right (CW)   [ROS standard]
```

Therefore:
```
angle_lidar = −angle_camera
```

The LIDAR scan index corresponding to this angle:
```
index = round((angle_lidar − angle_min) / angle_increment)
```

### Step 3: Distance from LIDAR

We take the median of a window of ±3 LIDAR beams around the computed index to reduce noise:

```
distance = median( scan.ranges[index−3 : index+3] )
```

Invalid readings (inf, NaN, out of range_min/range_max) are filtered before taking the median.

### Step 4: PD Control

**Angular controller:**
```
error_angular = angle_camera   (rad)
ω = −Kp_ang × error_angular − Kd_ang × (d/dt error_angular)
```
Negative sign: positive angle (object right) → turn right → negative angular velocity in ROS.

**Linear controller** (only active when `|error_angular| < 15°`):
```
error_linear = distance − desired_distance   (m)
vx = Kp_lin × error_linear + Kd_lin × (d/dt error_linear)
```
Positive error (too far) → drive forward (positive `vx`).
Negative error (too close) → drive backward (negative `vx`).

### Figure

```
         Camera FOV (62.2°)
              ←  31.1°  →
    ┌─────────────────────────┐
    │         Object          │
    │            ●            │
    │         ↑              │
    │   pixel_offset = cx - W/2
    └─────────────────────────┘
              Robot
               ▲
          (forward = 0°)

  LIDAR scan:
  angle=0 → index_center → range = distance to object
```

The robot receives `(angle_camera, distance)` and drives:
- **Angular**: rotate until `angle_camera ≈ 0`
- **Linear**: translate until `distance ≈ desired_distance`
