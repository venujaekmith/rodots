# Round 01 competition task

## Objective

eLLo starts in the marked START box with GREEN as the first target. Visit **GREEN → RED → BLUE → YELLOW** in that order. All checkpoints are flat, solid-color squares with a continuous black line across them. There are no color clues or markers to decode. After reaching YELLOW, stop with the robot's full ground footprint inside the square.

The START box is a launch area, not the GREEN checkpoint. Detect a checkpoint by stable observations while entering its marked square; do not count the initial START surface.

## Autonomy

The robot must follow the black line, identify junctions, choose routes, detect rock-blocked branches, and recover through its own controller. A checkpoint encountered out of order does not advance the mission. An already completed checkpoint does not count twice. Participants may use wheel encoders, the seven IR sensors, two floor color cameras, the front obstacle sensor, and the RGB LED.

The practice world is visible to teams. Final evaluation may use separate undisclosed worlds with the same device interface. A solution tied to known world coordinates or a fixed route is unlikely to transfer. The robot controller must not use Supervisor ground truth or organizer evaluation data for navigation.

## Evaluation principles

Organizers will observe the physical robot in Webots: valid start, ordered checkpoint entry, final footprint, stopped wheels, elapsed time, rock collisions, and safe obstacle handling. A printed message alone does not prove completion. Partial mission progress can earn partial credit. Exact scoring weights and time limits will be communicated by the organizers; this package does not assert unconfirmed official values.

Use only Webots-supported controller languages and keep your program within the supplied robot's motor and sensor interfaces. Do not modify hidden evaluation worlds or the Webots installation.
