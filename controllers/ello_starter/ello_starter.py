"""eLLo line follower, step 1: follow the thin black line.

No junction, checkpoint or obstacle logic yet. Uses only the 7 downward IR
sensors and the two wheel motors, on Webots R2025a with the standard library.
"""

import math
import sys

from controller import Robot


IR_NAMES = ["ir_line_%d" % i for i in range(7)]
WEIGHTS = (-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0)   # index 0 is the robot's left

CRUISE = 5.0          # rad/s on straights, about 0.5 m/s
CURVE_FLOOR = 2.0     # rad/s on sharp bends
KP = 1.1
KI = 0.02
KD = 0.6
CALIBRATION_STEPS = 25
LOST_LIMIT = 12       # steps without a line before searching


def clamp(value, low, high):
    return max(low, min(high, value))


class LineFollower:
    def __init__(self, robot):
        self.robot = robot
        self.step_ms = int(robot.getBasicTimeStep())

        self.ir = [robot.getDevice(n) for n in IR_NAMES]
        self.left = robot.getDevice("left_wheel_motor")
        self.right = robot.getDevice("right_wheel_motor")
        if any(d is None for d in self.ir + [self.left, self.right]):
            raise RuntimeError("missing line sensor or wheel motor")

        for s in self.ir:
            s.enable(self.step_ms)
        self.left.setPosition(float("inf"))
        self.right.setPosition(float("inf"))
        limits = [m for m in (self.left.getMaxVelocity(), self.right.getMaxVelocity()) if m > 0]
        self.max_vel = min(limits) if limits else 8.0

        self.lo = [0.0] * 7
        self.hi = [0.0] * 7
        self.black_low = True
        self.integral = 0.0
        self.prev = 0.0
        self.last_side = 0.0      # -1 line was on the left, +1 on the right
        self.lost = 0

    def drive(self, left, right):
        self.left.setVelocity(clamp(left, -self.max_vel, self.max_vel))
        self.right.setVelocity(clamp(right, -self.max_vel, self.max_vel))

    def read_ir(self):
        return [float(s.getValue()) for s in self.ir]

    def calibrate(self, samples):
        """Learn polarity: the centre sensors sit on the line at spawn."""
        n = float(len(samples))
        mean = [sum(s[i] for s in samples) / n for i in range(7)]
        centre = (mean[2] + mean[3] + mean[4]) / 3.0
        outer = (mean[0] + mean[1] + mean[5] + mean[6]) / 4.0
        self.black_low = centre <= outer
        low, high = min(centre, outer), max(centre, outer)
        self.lo = [low] * 7
        self.hi = [high] * 7

    def line_bits(self, values):
        bits = []
        for i, v in enumerate(values):
            # Track the black and floor levels so the threshold follows drift.
            self.lo[i] = v if v < self.lo[i] else self.lo[i] + 0.3
            self.hi[i] = v if v > self.hi[i] else self.hi[i] - 0.3
            if self.hi[i] - self.lo[i] < 15.0:
                bits.append(False)
                continue
            mid = 0.5 * (self.lo[i] + self.hi[i])
            bits.append(v <= mid if self.black_low else v >= mid)
        return bits

    def centroid(self, bits):
        count = sum(bits)
        if count == 0:
            return None
        return sum(w for w, b in zip(WEIGHTS, bits) if b) / count

    def run(self):
        self.drive(0.0, 0.0)
        samples = []
        while self.robot.step(self.step_ms) != -1:
            values = self.read_ir()

            if len(samples) < CALIBRATION_STEPS:
                samples.append(values)
                self.drive(0.0, 0.0)
                if len(samples) == CALIBRATION_STEPS:
                    self.calibrate(samples)
                    print("calibrated: %s" % ("black-low" if self.black_low else "black-high"))
                continue

            bits = self.line_bits(values)
            c = self.centroid(bits)

            if c is None:
                # Line lost: sweep toward the side it was last seen on.
                self.lost += 1
                if self.lost > LOST_LIMIT:
                    self.drive(-1.2 * self.last_side, 1.2 * self.last_side)
                else:
                    self.drive(CRUISE * 0.4, CRUISE * 0.4)
                continue

            self.lost = 0
            if c != 0.0:
                self.last_side = -1.0 if c < 0 else 1.0

            # Negative centroid means the line is on the left: turn left,
            # which means slowing the left wheel and speeding up the right.
            error_change = c - self.prev
            self.prev = c
            self.integral = clamp(self.integral + c, -20.0, 20.0)
            turn = KP * c + KI * self.integral + KD * error_change

            blend = min(1.0, abs(c) / 1.5 + 0.4 * abs(error_change))
            speed = CRUISE - (CRUISE - CURVE_FLOOR) * blend
            self.drive(speed + turn, speed - turn)


def main():
    try:
        LineFollower(Robot()).run()
    except RuntimeError as error:
        print("STARTER ERROR: %s" % error, file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())