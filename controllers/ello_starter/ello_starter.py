"""eLLo line follower with junction scan.

Follows the thin black line. When a junction is found, it moves onto the
crossing centre, turns a full 360 degrees, counts the roads it sees, and
drives down one of them:
  - 3 or more roads: a random road is chosen
  - 2 roads (straight or bend): the road closest to straight ahead
  - 1 road (dead end): that road, which turns the robot around

Uses only the 7 IR line sensors, the two wheel motors and the two wheel
encoders, with the Python standard library and Webots' controller module.

calibrated: black-low
raw IR at spawn (mean of 25 steps):  292  291  291  100  291  291  291
[t=  0.40] FOLLOW   ir= 292  290  292  100  291  291  291
           bits=0001000 road=0000000 centroid=none colour=no heading=+0.0deg travelled=-0.000
           lo= 227  227  227  100  227  227  227 hi= 292  290  292  290  290  290  290 black_low=True
[t=  0.91] FOLLOW   ir= 292  292  294  100  291  290  292
           bits=0001000 road=0000000 centroid=+0.0 colour=yes heading=+0.0deg travelled=0.248
           lo= 237  237  237  100  237  237  237 hi= 292  292  294  281  292  292  291 black_low=True
[t=  1.42] FOLLOW   ir= 200  199  200  100  200  199  200
           bits=1111111 road=0000000 centroid=+0.0 colour=yes heading=-0.0deg travelled=0.504
           lo= 199  199  200  100  199  199  199 hi= 524  525  524  271  526  525  524 black_low=True
[t=  1.94] FOLLOW   ir= 201  200  200  100  199  200  198
           bits=1111111 road=0000000 centroid=+0.0 colour=yes heading=+0.0deg travelled=0.760
           lo= 105  105  105   99  199  198  198 hi= 515  516  515  262  516  516  514 black_low=True
[t=  2.45] FOLLOW   ir= 198  200  201  100  199  197  201
           bits=1111111 road=0000000 centroid=+0.0 colour=yes heading=-0.0deg travelled=1.015
           lo= 115  115  115  100  199  197  199 hi= 505  506  505  252  507  506  505 black_low=True
[t=  2.96] FOLLOW   ir= 201  201  200   99  200  200  200
           bits=1111111 road=0000000 centroid=+0.0 colour=yes heading=+0.0deg travelled=1.271
           lo= 125  125  125   99  199  200  199 hi= 496  496  495  242  497  496  495 black_low=True
[t=  3.47] FOLLOW   ir= 200  200  201  100  200  200  200
           bits=1111111 road=0000000 centroid=+0.0 colour=yes heading=+0.0deg travelled=1.527
           lo= 134  134  134  100  199  200  199 hi= 486  487  486  233  487  487  486 black_low=True
[t=  3.98] FOLLOW   ir= 200  199  200  100  199  200  200
           bits=1111111 road=0000000 centroid=+0.0 colour=yes heading=-0.0deg travelled=1.783
           lo= 144  144  144  100  198  199  199 hi= 476  477  476  223  478  477  476 black_low=True
[t=  4.50] FOLLOW   ir= 200  200  200  100  200  200  199
           bits=1111111 road=0000000 centroid=+0.0 colour=yes heading=+0.0deg travelled=2.039
           lo= 154  153  154  100  198  199  199 hi= 467  468  467  214  468  468  466 black_low=True
[t=  5.01] FOLLOW   ir= 200  199  200   99  200  199  200
           bits=1111111 road=0000000 centroid=+0.0 colour=yes heading=-0.0deg travelled=2.295
           lo= 163  163  163   99  199  199  199 hi= 457  458  457  204  459  458  457 black_low=True
[t=  5.52] FOLLOW   ir= 200  200  201  100  201  199  199
           bits=1111111 road=0000000 centroid=+0.0 colour=yes heading=+0.0deg travelled=2.551
           lo= 173  173  173  100  200  199  199 hi= 448  448  447  194  449  448  447 black_low=True
[t=  6.03] FOLLOW   ir= 199  200  199  100  199  199  201
           bits=1111111 road=0000000 centroid=+0.0 colour=yes heading=+0.0deg travelled=2.807
           lo= 182  182  182   99  199  199  199 hi= 438  439  438  185  439  439  438 black_low=True
[t=  6.54] FOLLOW   ir= 199  201  199  100  200  201  199
           bits=1111111 road=0000000 centroid=+0.0 colour=yes heading=-0.0deg travelled=3.062
           lo= 192  192  192  100  200  199  199 hi= 428  429  428  175  430  429  428 black_low=True
[t=  7.06] FOLLOW   ir= 200  199  198  100  200  200  200
           bits=1111111 road=0000000 centroid=+0.0 colour=yes heading=+0.0deg travelled=3.318
           lo= 200  198  198  100  198  200  200 hi= 419  419  418  166  420  419  418 black_low=True
[t=  7.57] FOLLOW   ir= 199  200  200  199  199  201  200
           bits=1110111 road=0000000 centroid=+0.0 colour=yes heading=-0.0deg travelled=3.574
           lo= 105  105  106  105  199  199  199 hi= 409  410  409  199  411  410  409 black_low=True
[t=  8.08] FOLLOW   ir= 200  200  200  201  201  199  199
           bits=1110111 road=0000000 centroid=+0.0 colour=yes heading=+0.0deg travelled=3.830
           lo= 115  115  116  115  199  199  199 hi= 399  400  399  201  401  400  399 black_low=True
[t=  8.59] FOLLOW   ir= 199  200  200  201  201  201  199
           bits=1110111 road=0000000 centroid=+0.0 colour=yes heading=-0.0deg travelled=4.086
           lo= 125  124  125  125  199  199  199 hi= 390  391  390  201  391  391  390 black_low=True
[t=  9.10] FOLLOW   ir= 200  200  200  200  200  200  201
           bits=1110111 road=0000000 centroid=+0.0 colour=yes heading=-0.0deg travelled=4.342
           lo= 134  134  135  134  199  200  200 hi= 380  381  380  199  382  381  380 black_low=True
[t=  9.62] FOLLOW   ir= 202  201  199  199  199  200  200
           bits=1110111 road=0000000 centroid=+0.0 colour=yes heading=+0.0deg travelled=4.598
           lo= 144  144  145  144  199  199  198 hi= 371  371  370  200  372  371  370 black_low=True
[t= 10.13] FOLLOW   ir= 199  201  200  200  200  200  200
           bits=1110111 road=0000000 centroid=+0.0 colour=yes heading=-0.0deg travelled=4.853
           lo= 153  153  154  153  200  199  199 hi= 361  362  361  200  363  362  361 black_low=True
[t= 10.64] FOLLOW   ir= 199  199  200  200  199  200  201
           bits=1110111 road=0000000 centroid=+0.0 colour=yes heading=+0.0deg travelled=5.109
           lo= 163  163  164  163  199  200  199 hi= 351  352  351  200  353  352  351 black_low=True
[t= 11.15] FOLLOW   ir= 199  200  199  201  199  200  200
           bits=1110111 road=0000000 centroid=+0.0 colour=yes heading=-0.0deg travelled=5.365
           lo= 173  172  173  173  199  199  199 hi= 342  343  342  201  343  343  342 black_low=True
[t= 11.66] FOLLOW   ir= 201  200  200  200  199  200  199
           bits=1110111 road=0000000 centroid=+0.0 colour=yes heading=-0.0deg travelled=5.621
           lo= 182  182  183  182  199  199  199 hi= 332  333  332  200  334  333  332 black_low=True


"""

import math
import random
import sys

from controller import Camera, Robot


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
BLACK_FRACTION = 0.25           # a road reading must be within this fraction of black


def clamp(value, low, high):
    return max(low, min(high, value))


def angle_wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# Each IR sensor sits at a fixed angle from the axle: sensor i is at lateral
# offset 0.075 - 0.025 i, 0.13 m ahead. When a sensor sees a road, the road's
# direction from the crossing is the heading plus that angle, whatever the
# heading is. Every sensor that sees the same road therefore gives the same
# bearing, however the sightings are split across the sweep.
SENSOR_BEARING = [math.atan2(0.075 - 0.025 * i, 0.13) for i in range(7)]
ROAD_CLUSTER_TOL = math.radians(25)
MIN_ROAD_HITS = 3
DEBUG_PERIOD_S = 0.5            # how often the sensor dump is printed


def find_roads(bearings):
    """bearings: road bearings (radians, any range) from the scan. Groups them
    into roads and returns each road's circular-mean bearing in (-pi, pi]."""
    if not bearings:
        return []
    ordered = sorted(b % TWO_PI for b in bearings)
    clusters = [[ordered[0]]]
    for b in ordered[1:]:
        if b - clusters[-1][-1] <= ROAD_CLUSTER_TOL:
            clusters[-1].append(b)
        else:
            clusters.append([b])
    # The last and first clusters may be one road that wraps past 0 / 2 pi.
    if (len(clusters) >= 2
            and ordered[0] + TWO_PI - clusters[-1][-1] <= ROAD_CLUSTER_TOL):
        clusters[0] = clusters[-1] + clusters[0]
        clusters.pop()
    roads = []
    for cluster in clusters:
        if len(cluster) < MIN_ROAD_HITS:
            continue
        s = sum(math.sin(b) for b in cluster)
        c = sum(math.cos(b) for b in cluster)
        roads.append(angle_wrap(math.atan2(s, c)))
    return roads


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
        self.cam_left = robot.getDevice("checkpoint_color_left")
        self.cam_right = robot.getDevice("checkpoint_color_right")
        devices = (self.ir + [self.enc_left, self.enc_right, self.left, self.right,
                              self.cam_left, self.cam_right])
        if any(d is None for d in devices):
            raise RuntimeError("missing line sensor, encoder or wheel motor")

        for s in self.ir:
            s.enable(self.step_ms)
        self.enc_left.enable(self.step_ms)
        self.enc_right.enable(self.step_ms)
        self.cam_left.enable(self.step_ms)
        self.cam_right.enable(self.step_ms)
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
        self.road_bearings = []
        self.scan_start_heading = 0.0

        # Debug output.
        self.steps = 0
        self.last_debug = -1e9

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

    def road_bits_from(self, values, bits):
        """Stricter than the line bits: a sensor counts as road only when its
        reading is close to the calibrated black level. Mid-grey surfaces (for
        example a coloured pad that reads somewhat dark) fail this test."""
        out = []
        for i, v in enumerate(values):
            span = self.hi[i] - self.lo[i]
            if span < 15.0 or not bits[i]:
                out.append(False)
                continue
            if self.black_low:
                out.append(v <= self.lo[i] + BLACK_FRACTION * span)
            else:
                out.append(v >= self.hi[i] - BLACK_FRACTION * span)
        return out

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

    def coloured_below(self):
        """True when a downward camera sees a saturated colour, i.e. a checkpoint
        pad. Roads are black, so IR readings over a pad are not road sightings."""
        for camera in (self.cam_left, self.cam_right):
            image = camera.getImage()
            if image is None:
                continue
            width, height = camera.getWidth(), camera.getHeight()
            r = g = b = 0.0
            count = 0
            for y in range(height // 2 - height // 8, height // 2 + height // 8):
                for x in range(width // 2 - width // 8, width // 2 + width // 8):
                    r += Camera.imageGetRed(image, width, x, y)
                    g += Camera.imageGetGreen(image, width, x, y)
                    b += Camera.imageGetBlue(image, width, x, y)
                    count += 1
            if count == 0:
                continue
            r, g, b = r / count, g / count, b / count
            top = max(r, g, b)
            if top >= 45 and (top - min(r, g, b)) / top > 0.20:
                return True
        return False

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
        self.road_bearings = []
        self.scan_start_heading = self.heading

    def choose_road(self, roads):
        """Return the target heading relative to the scan start, or None."""
        if not roads:
            return None
        if len(roads) >= 3:
            return random.choice(roads)
        # One or two roads: take the one closest to straight ahead.
        return min(roads, key=lambda a: abs(a))

    def debug(self, values, bits, road_bits, c, on_colour, force=False):
        """Print the sensor state so the run can be checked from the console."""
        t = self.steps * self.step_ms / 1000.0
        if not force and t - self.last_debug < DEBUG_PERIOD_S:
            return
        self.last_debug = t
        print("[t=%6.2f] %-8s ir=%s" % (t, self.state, " ".join("%4d" % v for v in values)))
        print("           bits=%s road=%s centroid=%s colour=%s heading=%+.1fdeg travelled=%.3f"
              % ("".join("1" if b else "0" for b in bits),
                 "".join("1" if b else "0" for b in road_bits),
                 "none" if c is None else "%+.1f" % c,
                 "yes" if on_colour else "no",
                 math.degrees(self.heading), self.travelled))
        print("           lo=%s hi=%s black_low=%s"
              % (" ".join("%4d" % v for v in self.lo),
                 " ".join("%4d" % v for v in self.hi), self.black_low))

    def run(self):
        self.drive(0.0, 0.0)
        samples = []
        while self.robot.step(self.step_ms) != -1:
            self.steps += 1
            dth = self.update_odometry()
            values = self.read_ir()

            if len(samples) < CALIBRATION_STEPS:
                samples.append(values)
                self.drive(0.0, 0.0)
                if len(samples) == CALIBRATION_STEPS:
                    self.calibrate(samples)
                    print("calibrated: %s" % ("black-low" if self.black_low else "black-high"))
                    print("raw IR at spawn (mean of %d steps): %s"
                          % (CALIBRATION_STEPS, " ".join(
                              "%4.0f" % (sum(s[i] for s in samples) / len(samples))
                              for i in range(7))))
                    self.debug(values, self.line_bits(values), [False] * 7, None, False, force=True)
                continue

            bits = self.line_bits(values)
            c = self.centroid(bits)
            # Road sightings: black line only, never over a coloured checkpoint pad.
            on_colour = self.coloured_below()
            road_bits = [False] * 7 if on_colour else self.road_bits_from(values, bits)
            self.debug(values, bits, road_bits, c, on_colour)

            if self.state == self.FOLLOW:
                # Junction candidate: an outermost sensor sees a black road.
                if road_bits[0] or road_bits[6]:
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
                for i, is_road in enumerate(road_bits):
                    if is_road:
                        self.road_bearings.append(
                            self.heading + SENSOR_BEARING[i] - self.scan_start_heading)
                if self.scan_angle >= TWO_PI:
                    self.drive(0.0, 0.0)
                    roads = find_roads(self.road_bearings)
                    print("scan: %d road sightings" % len(self.road_bearings))
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