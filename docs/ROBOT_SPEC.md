# eLLo robot and device specification

The supplied Webots R2025a world is the authoritative model. Coordinates use Webots' **x/y horizontal, z upward** convention. Robot-local +x points forward, +y points left.

| Component | Webots device name | Relevant properties |
|---|---|---|
| Left drive motor | `left_wheel_motor` | RotationalMotor, maxVelocity 8 rad/s, maxTorque 2 N·m |
| Right drive motor | `right_wheel_motor` | Same specification |
| Left wheel encoder | `left_wheel_encoder` | PositionSensor, radians |
| Right wheel encoder | `right_wheel_encoder` | PositionSensor, radians |
| Front rock sensor | `front_obstacle_sensor` | DistanceSensor; higher reading means a closer obstruction within about 0.5 m |
| IR reflectance array | `ir_line_0` through `ir_line_6` | Seven downward DistanceSensors, from robot-left to robot-right, spaced 0.025 m |
| Left floor color camera | `checkpoint_color_left` | 32×32 Camera, local x +0.13 m and y +0.12 m, aimed downward |
| Right floor color camera | `checkpoint_color_right` | 32×32 Camera, local x +0.13 m and y −0.12 m, aimed downward |
| Status light | `rgb_status_led` | RGB LED; color value such as `0x00ff00` for green |

The drive wheel radius is 0.10 m, wheel-center spacing is approximately 0.34 m, and the chassis collision box is 0.35 × 0.26 m. Wheels extend the ground footprint to approximately 0.38 m across. The line is 0.04 m wide; each colored checkpoint is 0.72 × 0.72 m.

The IR emitters are on the front underside at local x +0.13 m, at approximately 0.035 m above the floor. Their simulated `DistanceSensor` infrared response depends on surface reflectance as well as distance. In measured starts, the black line was near 100 and brown ground near 284–291. Colored surfaces have different responses. Calibrate from actual readings and reject invalid sensor values; do not assume a zero or missing return means black.

The two RGB cameras sample the floor beside the center line. They model the downward color-reading function of physical EV3 color sensors but use Webots cameras rather than an exact hardware optical model. Read both sides while following the central line with the IR array. Require stable observations to count a checkpoint; the black line itself should not be classified as a color.

Device initialization for C is demonstrated in `controllers/ello_starter/ello_starter.c`. Other Webots languages expose the same device names through their controller APIs.
