# REIDBOTS: Mission K-391

**A New World. A New Challenge.**  
**Round 01 — The Virtual Expedition — Webots Simulation**

REIDBOTS is the first robotics competition at the University of Colombo School of Computing, organized by the Robotics & Automation Society of UCSC.

Scientists on Earth discover the mysterious planet K-391. An autonomous exploration robot named eLLo is sent to investigate it. During the mission, a system malfunction corrupts its navigation data and causes eLLo to lose contact with its spacecraft. Your task is to help eLLo navigate unfamiliar terrain, overcome obstacles, and complete the expedition.

## Round 01 task

Autonomously follow the thin black line through the branching maze and visit the flat checkpoints in the announced order **GREEN → RED → BLUE → YELLOW**. Rocks block some branches. The robot must find alternate routes when necessary. Finish with eLLo's full ground footprint inside YELLOW and both drive wheels stopped.

The supplied world is a practice arena. Evaluation may use other undisclosed layouts that follow the same device and checkpoint rules. Do not depend on coordinates or a precomputed path from this world.

## Quick start on Windows 11

1. Install **Webots R2025a** from the official Webots distribution.
2. Extract this entire package to a writable folder. Keep `worlds` and `controllers` together.
3. Open `worlds/mission_k391_round01.wbt` in Webots.
4. Open `controllers/ello_starter/ello_starter.py` in Webots' text editor and run. If prompted, reload the world.
5. Start the simulation. The starter prints all sensor readings, sets the LED green, and keeps the motors stopped.
6. Use Webots **Reset** to restore the robot's starting pose between trials.

The starter is a device example. It does not navigate or recognize checkpoints. Replace it with your own controller. You may use any programming language supported by your Webots R2025a installation; the supplied C starter is only one option.

For a very short motor and encoder check, temporarily set the robot's `controllerArgs` in your practice copy to `[ "--motor-smoke" ]`, reload, and run. It drives both wheels at 0.5 rad/s from 0.6 to 1.1 simulation seconds, then stops. Remove the argument for normal development.

## Package contents

- `worlds/mission_k391_round01.wbt`: practice arena and eLLo model.
- `controllers/ello_starter/`: minimal python controller.
- `docs/ROBOT_SPEC.md`: device names, physical dimensions, and sensor behavior.
- `docs/RULES.md`: mission and evaluation principles.
- `docs/SUBMISSION.md`: controller handoff checklist.
- `docs/TROUBLESHOOTING.md`: setup and debugging.

No organizer reference navigation code or private evaluation layouts are included.
