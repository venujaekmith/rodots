# Setup and debugging

**World does not open.** Confirm Webots R2025a and keep the package directory structure. Open the `.wbt` file from the extracted folder.

**No movement.** The starter intentionally keeps motors stopped unless `--motor-smoke` is added to `controllerArgs`. For your own controller, set both motor positions to infinity for velocity control, then set bounded wheel velocities.

**Missing readings.** Enable each PositionSensor, DistanceSensor, and Camera with the world's basic time step before reading it. Check the exact names in `ROBOT_SPEC.md` and in the world scene tree.

**Line detection is unstable.** Calibrate on the black line and on clear brown ground. Inspect all seven readings at once. The color cameras are for colored floor checkpoints; use the IR array for steering. A missing or invalid IR value is not a black-line observation.

**Color detection repeats.** Require several consistent frames from the side cameras and a confirmed exit from a pad before arming another checkpoint event. Keep the publicly announced target sequence separate from motor control.

**Robot spins near a junction.** Log motor commands, encoder travel, all IR readings, and your state transitions. Bound each turn and recovery attempt, then stop safely if the outgoing line cannot be reacquired.

**Rock collision.** Log the front sensor before the robot reaches the rock. Stop with clearance, mark the route unavailable, turn around on the line, and choose another branch at the previous junction.
