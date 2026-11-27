"""eLLo line follower with junction scan.

Follows the thin black line. When a junction is found, it moves onto the
crossing centre, turns a full 360 degrees, counts the roads it sees, and
drives down one of them:
  - 3 or more roads: a random road is chosen
  - 2 roads (straight or bend): the road closest to straight ahead
  - 1 road (dead end): that road, which turns the robot around

Uses only the 7 IR line sensors, the two wheel motors and the two wheel
encoders, with the Python standard library and Webots' controller module.
"""

import math
import random
import sys

from controller import Robot


IR_NAMES = ["ir_line_%d" % i for i in range(7)]
WEIGHTS = (-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0)   # index 0 is the robot's left

WHEEL_RADIUS = 0.10
AXLE_LENGTH = 0.34
TWO_PI = 2.0 * math.pi

CRUISE = 5.0                 # rad/s on straights
CURVE_FLOOR = 2.0            # rad/s on sharp bends
APPROACH_SPEED = 2.2
SCAN_SPEED = 2.5             # rad/s wheel command while spinning
TURN_MIN = 0.9
TURN_MAX = 2.2
TURN_GAIN = 2.5
TURN_TOLERANCE = 0.035       # rad, about 2 degrees
KP, KI, KD = 1.1, 0.02, 0.6
CALIBRATION_STEPS = 25
LOST_LIMIT = 12
JUNCTION_CONFIRM_STEPS = 5
JUNCTION_CENTER_OFFSET = 0.11   # m from the first outer-sensor hit to the crossing centre
MIN_ROAD_SAMPLES = 5            # a real road spans about 13 scan samples; shorter sightings are noise


def clamp(value, low, high):
    return max(low, min(high, value))


def angle_wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def find_roads(samples):
    """samples: list of (angle_from_scan_start, line_seen). Returns road centre
    angles in radians, wrapped to (-pi, pi]. A road that crosses the start of
    the scan is joined into one road."""
    intervals = []
    current = None
    for angle, seen in samples:
        if seen:
            if current is None:
                current = [angle, angle, 1]
            else:
                current[1] = angle
                current[2] += 1
        elif current is not None:
            intervals.append(current)
            current = None
    if current is not None:
        intervals.append(current)

    intervals = [iv for iv in intervals if iv[2] >= MIN_ROAD_SAMPLES]

    # The first and last samples are adjacent on the circle, so a road that
    # spans the start of the scan appears as two pieces. Join them.
    if (len(intervals) >= 2 and samples and samples[0][1] and samples[-1][1]
            and intervals[0][0] <= samples[0][0] + 1e-6
            and intervals[-1][1] >= samples[-1][0] - 1e-6):
        first, last = intervals[0], intervals[-1]
        joined = [last[0], first[1] + TWO_PI, first[2] + last[2]]
        intervals = [joined] + intervals[1:-1]

    return [angle_wrap((start + end) / 2.0) for start, end, _ in intervals]


class LineFollower:
    FOLLOW = "FOLLOW"
    APPROACH = "APPROACH"
    SCAN = "SCAN"
    TURN = "TURN"

    def __init__(self, robot):
        self.robot = robot
        self.step_ms = int(robot.getBasicTimeStep())

        self.ir = [robot.getDevice(n) for n in IR_NAMES]
        self.enc_left = robot.getDevice("left_wheel_encoder")
        self.enc_right = robot.getDevice("right_wheel_encoder")
        self.left = robot.getDevice("left_wheel_motor")
        self.right = robot.getDevice("right_wheel_motor")
        devices = self.ir + [self.enc_left, self.enc_right, self.left, self.right]
        if any(d is None for d in devices):
            raise RuntimeError("missing line sensor, encoder or wheel motor")

        for s in self.ir:
            s.enable(self.step_ms)
        self.enc_left.enable(self.step_ms)
        self.enc_right.enable(self.step_ms)
        self.left.setPosition(float("inf"))
        self.right.setPosition(float("inf"))
        limits = [m for m in (self.left.getMaxVelocity(), self.right.getMaxVelocity()) if m > 0]
        self.max_vel = min(limits) if limits else 8.0

        # Odometry from the encoders.
        self.heading = 0.0
        self.travelled = 0.0
        self.last_enc = None

        # Line sensing.
        self.lo = [0.0] * 7
        self.hi = [0.0] * 7
        self.black_low = True
        self.integral = 0.0
        self.prev = 0.0
        self.last_side = 0.0
        self.lost = 0
        self.outer_votes = 0

        # Junction state.
        self.state = self.FOLLOW
        self.state_start = 0.0
        self.target_heading = 0.0
        self.scan_angle = 0.0
        self.scan_samples = []
        self.scan_start_heading = 0.0

    # ---- motion and odometry -------------------------------------------
    def drive(self, left, right):
        self.left.setVelocity(clamp(left, -self.max_vel, self.max_vel))
        self.right.setVelocity(clamp(right, -self.max_vel, self.max_vel))

    def update_odometry(self):
        left = float(self.enc_left.getValue())
        right = float(self.enc_right.getValue())
        if self.last_enc is None:
            self.last_enc = (left, right)
            return 0.0
        dl = (left - self.last_enc[0]) * WHEEL_RADIUS
        dr = (right - self.last_enc[1]) * WHEEL_RADIUS
        self.last_enc = (left, right)
        ds = 0.5 * (dl + dr)
        dth = (dr - dl) / AXLE_LENGTH
        self.heading = angle_wrap(self.heading + dth)
        self.travelled += ds
        return dth

    # ---- line sensing ----------------------------------------------------
    def read_ir(self):
        return [float(s.getValue()) for s in self.ir]

    def calibrate(self, samples):
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

    # ---- behaviour ---------------------------------------------------------
    def follow(self, bits, c):
        if c is None:
            self.lost += 1
            if self.lost > LOST_LIMIT:
                # The line has ended: a dead end. Turn 180 degrees and follow the
                # road back rather than spinning in place and losing alignment.
                print("line ended (dead end); turning around")
                self.drive(0.0, 0.0)
                self.target_heading = angle_wrap(self.heading + math.pi)
                self.state = self.TURN
            else:
                self.drive(CRUISE * 0.4, CRUISE * 0.4)
            return

        self.lost = 0
        if c != 0.0:
            self.last_side = -1.0 if c < 0 else 1.0
        error_change = c - self.prev
        self.prev = c
        self.integral = clamp(self.integral + c, -20.0, 20.0)
        turn = KP * c + KI * self.integral + KD * error_change
        blend = min(1.0, abs(c) / 1.5 + 0.4 * abs(error_change))
        speed = CRUISE - (CRUISE - CURVE_FLOOR) * blend
        self.drive(speed + turn, speed - turn)

    def start_approach(self):
        self.state = self.APPROACH
        self.state_start = self.travelled
        self.prev = 0.0
        self.integral = 0.0

    def start_scan(self):
        self.state = self.SCAN
        self.scan_angle = 0.0
        self.scan_samples = []
        self.scan_start_heading = self.heading

    def choose_road(self, roads):
        """Return the target heading relative to the scan start, or None."""
        if not roads:
            return None
        if len(roads) >= 3:
            return random.choice(roads)
        # One or two roads: take the one closest to straight ahead.
        return min(roads, key=lambda a: abs(a))

    def run(self):
        self.drive(0.0, 0.0)
        samples = []
        while self.robot.step(self.step_ms) != -1:
            dth = self.update_odometry()
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

            if self.state == self.FOLLOW:
                # Junction candidate: an outermost sensor sees the line.
                if bits[0] or bits[6]:
                    self.outer_votes += 1
                else:
                    self.outer_votes = 0
                if self.outer_votes >= JUNCTION_CONFIRM_STEPS:
                    self.outer_votes = 0
                    self.start_approach()
                    print("junction found; moving onto the crossing centre")
                    continue
                self.follow(bits, c)

            elif self.state == self.APPROACH:
                if self.travelled - self.state_start >= JUNCTION_CENTER_OFFSET:
                    self.drive(0.0, 0.0)
                    self.start_scan()
                    print("scanning 360 degrees")
                else:
                    self.drive(APPROACH_SPEED, APPROACH_SPEED)

            elif self.state == self.SCAN:
                self.scan_angle += dth
                self.scan_samples.append((self.scan_angle, any(bits)))
                if self.scan_angle >= TWO_PI:
                    self.drive(0.0, 0.0)
                    roads = find_roads(self.scan_samples)
                    choice = self.choose_road(roads)
                    print("roads found: %d at %s degrees"
                          % (len(roads), [round(math.degrees(r)) for r in roads]))
                    if choice is None:
                        print("no road found; resuming line search")
                        self.start_turn_to(0.0)
                    else:
                        print("chose road at %d degrees" % round(math.degrees(choice)))
                        self.start_turn_to(choice)
                else:
                    self.drive(-SCAN_SPEED, SCAN_SPEED)

            elif self.state == self.TURN:
                error = angle_wrap(self.target_heading - self.heading)
                if abs(error) <= TURN_TOLERANCE:
                    self.drive(0.0, 0.0)
                    self.heading = self.target_heading
                    self.state = self.FOLLOW
                    self.lost = 0
                    self.prev = 0.0
                    self.integral = 0.0
                else:
                    mag = clamp(TURN_GAIN * abs(error), TURN_MIN, TURN_MAX)
                    w = mag if error > 0 else -mag
                    self.drive(-w, w)

    def start_turn_to(self, relative):
        # The scan started at the heading the robot had at the junction.
        self.target_heading = angle_wrap(self.scan_start_heading + relative)
        self.state = self.TURN


def main():
    try:
        random.seed()
        LineFollower(Robot()).run()
    except RuntimeError as error:
        print("STARTER ERROR: %s" % error, file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())