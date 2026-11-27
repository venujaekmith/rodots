#following lines choosing paths in junctions avoid obstacles. but returning to start point
# make the path selection more inteligent


"""eLLo line follower with junction scan.

Follows the thin black line. When a junction is found, it moves onto the
crossing centre, turns a full 360 degrees, counts the roads it sees, and
drives down one of them:
  - 3 or more roads: an untried road is chosen at random, never the road the
    robot came by
  - 2 roads (straight or bend): the road closest to straight ahead
  - 1 road (dead end): that road, which turns the robot around

Uses only the 7 IR line sensors, the two wheel motors and the two wheel
encoders, with the Python standard library and Webots' controller module.
"""

import math
import random
import sys
from collections import deque

from controller import Camera, Robot


IR_NAMES = ["ir_line_%d" % i for i in range(7)]
WEIGHTS = (-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0)   # index 0 is the robot's left

WHEEL_RADIUS = 0.10
AXLE_LENGTH = 0.34
TWO_PI = 2.0 * math.pi
HALF_PI = math.pi / 2.0

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
LINE_FRACTION = 0.35            # a sensor is on the line when it reads below this fraction
                                # of the way from black to floor (fixed, set at spawn)
MAX_LINE_SENSORS = 4            # more black sensors than this is a dark surface, not a line
SEARCH_SPEED = 1.2              # rad/s wheel command while searching for a lost line
SEARCH_STEPS = 90               # steps to sweep toward the last known side before giving up


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
JUNCTION_MATCH = 0.35           # m: a scan this close to a stored junction is the same junction
OBSTACLE_TRIGGER = 520.0        # front lookup 1000 - 2000*d: 520 is 0.24 m from the sensor
OBSTACLE_BACKUP = 0.10          # m to reverse after an obstacle
RETURN_RADIUS = 0.25            # m: within this of the junction being returned to, odometry
                                # alone is enough to stop on its centre
ARRIVAL_TOL = math.radians(60)  # a scanned road this close to "behind" is the arrival road
DEAD_END_ANGLE = math.radians(135)  # the only road is further round than this: a dead end
UNTRIED, TAKEN, LINK, DEAD_END, BLOCKED = "UNTRIED", "TAKEN", "LINK", "DEAD_END", "BLOCKED"
DIR_NAME = ("E", "N", "W", "S")  # direction index 0..3 = bearing 0, 90, 180, -90 degrees


def dir_of(angle):
    """Quantise an absolute bearing to a cardinal direction index 0..3."""
    return int(round(angle / HALF_PI)) % 4

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
    BACKUP = "BACKUP"
    STOPPED = "STOPPED"

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
        self.front = robot.getDevice("front_obstacle_sensor")
        if self.front is None:
            raise RuntimeError("missing front_obstacle_sensor")
        self.front.enable(self.step_ms)
        self.front_votes = deque(maxlen=3)
        self.front_value = 0.0
        self.x = 0.0
        self.y = 0.0

        # Junction memory. Each junction records its position and, for each
        # cardinal road, its state: UNTRIED, TAKEN, LINK (to another junction),
        # DEAD_END or BLOCKED.
        self.junctions = []
        self.pending = None             # (junction, direction) of the road being driven
        self.return_to = None           # junction being driven back to after a closed road
        self.scan_junction = None       # junction a forced (odometry) scan belongs to
        self.backup_goal = 0.0

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
        self.black_ref = 0.0
        self.floor_ref = 0.0
        self.threshold = 0.0
        self.integral = 0.0
        self.prev = 0.0
        self.last_side = 0.0
        self.travel_heading = 0.0       # heading when the line was last centred
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
        mid = self.heading + 0.5 * dth
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.heading = angle_wrap(self.heading + dth)
        self.travelled += ds
        return dth

    # ---- line sensing ----------------------------------------------------
    def read_ir(self):
        return [float(s.getValue()) for s in self.ir]

    def calibrate(self, samples):
        """Fix the black and floor references from the spawn samples. The line
        sensor reads the darkest value; most sensors read the floor. The threshold
        is set once, so it cannot drift over a long run."""
        readings = sorted(v for s in samples for v in s)
        self.black_ref = readings[0]
        self.floor_ref = readings[len(readings) // 2]
        self.threshold = self.black_ref + LINE_FRACTION * (self.floor_ref - self.black_ref)

    def line_bits(self, values):
        """A sensor sees the black line when it reads at or below the threshold."""
        return [v <= self.threshold for v in values]

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
        if count == 0 or count > MAX_LINE_SENSORS:
            return None
        return sum(w for w, b in zip(WEIGHTS, bits) if b) / count

    # ---- behaviour ---------------------------------------------------------
    def follow(self, bits, c):
        if c is None:
            self.lost += 1
            if self.lost <= LOST_LIMIT:
                self.drive(CRUISE * 0.4, CRUISE * 0.4)
            elif self.lost <= LOST_LIMIT + SEARCH_STEPS:
                # Sweep toward the side the line was last seen on. A positive
                # side means the line was on the right, so turn right.
                side = self.last_side or 1.0
                self.drive(SEARCH_SPEED * side, -SEARCH_SPEED * side)
            else:
                print("line not found after search; turning around")
                self.close_pending(DEAD_END)
                self.drive(0.0, 0.0)
                # Turn back to the heading the robot had on the line, not its current
                # heading, which the search sweep may have disturbed.
                self.target_heading = angle_wrap(self.travel_heading + math.pi)
                self.state = self.TURN
            return

        self.lost = 0
        if abs(c) < 0.3:
            self.travel_heading = self.heading
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

    def read_front(self):
        """Median of the last three front readings, so one noisy step never triggers."""
        value = float(self.front.getValue())
        if not math.isfinite(value):
            value = 0.0
        self.front_votes.append(value)
        ordered = sorted(self.front_votes)
        return ordered[len(ordered) // 2]

    def junction_index(self):
        """Index of the junction at the current position. A known junction is
        matched by its nearest stored position and the odometry is snapped to
        it, so drift cannot split one junction into two."""
        best, best_dist = None, JUNCTION_MATCH
        for i, j in enumerate(self.junctions):
            dist = math.hypot(j["x"] - self.x, j["y"] - self.y)
            if dist <= best_dist:
                best, best_dist = i, dist
        if best is not None:
            self.x, self.y = self.junctions[best]["x"], self.junctions[best]["y"]
            return best
        self.junctions.append({"x": self.x, "y": self.y, "roads": {}})
        return len(self.junctions) - 1

    def start_obstacle_escape(self):
        """An obstacle is ahead on this road: stop, reverse, and block the road at
        the junction it starts from, so every later visit avoids it."""
        self.drive(0.0, 0.0)
        print("obstacle ahead (front %.0f): reversing and blocking this road"
              % self.front_value)
        self.close_pending(BLOCKED)
        self.backup_goal = self.travelled - OBSTACLE_BACKUP
        # Turn to face back along the line, toward the junction.
        self.target_heading = angle_wrap(self.travel_heading + math.pi)
        self.state = self.BACKUP

    def close_pending(self, state):
        """The road being driven has ended in a dead end or an obstacle. If it was
        a known link, both ends are blocked so no route can cross it."""
        if self.pending is None:
            return
        j, d = self.pending
        old = self.junctions[j]["roads"].get(d, {})
        new = {"state": state}
        if "to" in old:
            new["to"], new["back"] = old["to"], old["back"]
            far = self.junctions[old["to"]]["roads"].get(old["back"])
            if far is not None:
                self.junctions[old["to"]]["roads"][old["back"]] = {
                    "state": state, "to": j, "back": d}
        self.junctions[j]["roads"][d] = new
        print("memory: junction %d road %s is %s" % (j, DIR_NAME[d], state))
        self.pending = None
        # The robot now drives back to junction j and must scan there again.
        self.return_to = j

    def reached_return_junction(self):
        """True when the robot, driving back after a closed road, is on or past the
        centre of the junction that road started from. This catches the case where
        the road was closed so near the junction that the line sensors are already
        beyond it and would never detect it, which sent the robot straight through
        and back down the road it first arrived by."""
        if self.return_to is None:
            return False
        j = self.junctions[self.return_to]
        dx, dy = j["x"] - self.x, j["y"] - self.y
        if math.hypot(dx, dy) > RETURN_RADIUS:
            return False
        return dx * math.cos(self.heading) + dy * math.sin(self.heading) <= 0.0

    def arrival_road(self, scanned):
        """The road the robot physically arrived by: the scanned road closest to
        directly behind it. This stays correct when the road from the previous
        junction had bends, or when the robot is coming back from an obstacle."""
        back = angle_wrap(self.scan_start_heading + math.pi)

        def gap(d):
            return abs(angle_wrap(d * HALF_PI - back))

        if scanned:
            best = min(scanned, key=gap)
            if gap(best) <= ARRIVAL_TOL:
                return best
        return dir_of(back)

    def link_arrival(self, here, back):
        """The road that was driven from a known junction has reached this one."""
        if self.pending is None:
            return
        src, d = self.pending
        self.pending = None
        if src == here and back == d:
            # The robot came back down the road it left by: that road goes nowhere.
            self.junctions[here]["roads"][d] = {"state": DEAD_END}
            print("memory: junction %d road %s is %s" % (here, DIR_NAME[d], DEAD_END))
            return
        self.junctions[src]["roads"][d] = {"state": LINK, "to": here, "back": back}
        self.junctions[here]["roads"][back] = {"state": LINK, "to": src, "back": d}
        print("memory: junction %d road %s links to junction %d"
              % (src, DIR_NAME[d], here))

    def note_roads(self, here, dirs):
        roads = self.junctions[here]["roads"]
        for d in dirs:
            if d not in roads:
                roads[d] = {"state": UNTRIED}

    def describe(self, here):
        j = self.junctions[here]
        parts = []
        for d in range(4):
            r = j["roads"].get(d)
            if r is None:
                continue
            if r["state"] == LINK:
                parts.append("%s=LINK->%d" % (DIR_NAME[d], r["to"]))
            else:
                parts.append("%s=%s" % (DIR_NAME[d], r["state"]))
        return "junction %d at (%.2f, %.2f): %s" % (here, j["x"], j["y"], " ".join(parts))

    def route_to_untried(self, here, avoid):
        """Shortest route over known links to a junction that has an untried road.
        The road `avoid` is not used as the first step. Returns (first_direction,
        target_junction, list of junctions on the route) or None."""
        parent = {here: None}
        queue = deque([here])
        while queue:
            j = queue.popleft()
            if j != here and any(r["state"] == UNTRIED
                                 for r in self.junctions[j]["roads"].values()):
                path = [j]
                node = j
                while parent[node][0] != here:
                    node = parent[node][0]
                    path.append(node)
                path.append(here)
                path.reverse()
                return parent[path[1]][1], j, path
            for d, r in self.junctions[j]["roads"].items():
                if j == here and d == avoid:
                    continue
                if r["state"] == LINK and r["to"] not in parent:
                    parent[r["to"]] = (j, d)
                    queue.append(r["to"])
        return None

    def choose_next(self, here, scanned, came_from):
        """Pick the next road. Never the road the robot came by, and never a road
        already taken, linked, dead-ended or blocked. An untried road here is
        chosen at random. Otherwise the robot routes to the nearest junction with
        an untried road, avoiding the arrival road. The arrival road is used only
        when no other route exists."""
        roads = self.junctions[here]["roads"]
        untried = [d for d in scanned
                   if d != came_from and roads.get(d, {}).get("state") == UNTRIED]
        if untried:
            return random.choice(untried), "untried road"

        route = self.route_to_untried(here, came_from)
        if route is not None:
            first, target, path = route
            print("route to junction %d through junctions %s"
                  % (target, " -> ".join(str(p) for p in path)))
            return first, "route over known links to an untried road"
        route = self.route_to_untried(here, None)
        if route is not None:
            first, target, path = route
            print("no other route: retracing through junctions %s"
                  % " -> ".join(str(p) for p in path))
            return first, "no other route: retracing the arrival road"
        return None

    def start_scan(self, junction=None):
        self.scan_junction = junction
        self.state = self.SCAN
        self.scan_angle = 0.0
        self.road_bearings = []
        self.scan_start_heading = self.heading

    def debug(self, values, bits, road_bits, c, on_colour, force=False):
        """Print the sensor state so the run can be checked from the console."""
        t = self.steps * self.step_ms / 1000.0
        if not force and t - self.last_debug < DEBUG_PERIOD_S:
            return
        self.last_debug = t
        print("[t=%6.2f] %-8s ir=%s" % (t, self.state, " ".join("%4d" % v for v in values)))
        print("           front=%.0f bits=%s road=%s centroid=%s colour=%s heading=%+.1fdeg travelled=%.3f"
              % (self.front_value, "".join("1" if b else "0" for b in bits),
                 "".join("1" if b else "0" for b in road_bits),
                 "none" if c is None else "%+.1f" % c,
                 "yes" if on_colour else "no",
                 math.degrees(self.heading), self.travelled))
        print("           threshold=%.0f (black %.0f, floor %.0f)"
              % (self.threshold, self.black_ref, self.floor_ref))

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
                    print("calibrated: threshold %.0f (black %.0f, floor %.0f)"
                          % (self.threshold, self.black_ref, self.floor_ref))
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
            road_bits = [False] * 7 if on_colour else bits
            self.front_value = self.read_front()
            self.debug(values, bits, road_bits, c, on_colour)

            if self.state == self.FOLLOW:
                if self.front_value >= OBSTACLE_TRIGGER:
                    self.start_obstacle_escape()
                    continue
                if self.reached_return_junction():
                    self.drive(0.0, 0.0)
                    self.outer_votes = 0
                    print("back at junction %d; scanning 360 degrees" % self.return_to)
                    self.start_scan(self.return_to)
                    continue
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
                    scanned = sorted({dir_of(self.scan_start_heading + r) for r in roads})
                    forced = self.scan_junction
                    self.scan_junction = None
                    if forced is None and len(scanned) <= 2:
                        # A bend or a straight stretch, not a junction: follow the road
                        # closest to straight ahead without recording anything.
                        if roads:
                            ahead = min(roads, key=abs)
                            if abs(ahead) > DEAD_END_ANGLE:
                                # The only road is the one behind: close the road
                                # being driven so it is never chosen again.
                                print("dead end; turning around")
                                self.close_pending(DEAD_END)
                            else:
                                print("bend or straight; following the road closest to ahead")
                            self.start_turn_to(ahead)
                        else:
                            self.start_turn_to(0.0)
                        continue

                    if forced is not None:
                        # Back at a known junction by odometry: use its stored roads
                        # as well, in case the scan was slightly off centre.
                        here = forced
                        self.x, self.y = self.junctions[here]["x"], self.junctions[here]["y"]
                        scanned = sorted(set(scanned) | set(self.junctions[here]["roads"]))
                    else:
                        here = self.junction_index()
                    self.return_to = None
                    # The road the robot came by is the one physically behind it.
                    came_from = self.arrival_road(scanned)
                    self.link_arrival(here, came_from)
                    roads_here = self.junctions[here]["roads"]
                    # The came-from road is never offered as a choice. Mark it TAKEN
                    # unless it is already a link, dead end or block, which must be kept.
                    if roads_here.get(came_from, {}).get("state", UNTRIED) == UNTRIED:
                        roads_here[came_from] = {"state": TAKEN}
                    self.note_roads(here, [d for d in scanned if d != came_from])
                    print(self.describe(here))

                    choice = self.choose_next(here, scanned, came_from)
                    if choice is None:
                        print("maze explored: no untried road is reachable; stopping")
                        self.state = self.STOPPED
                    else:
                        d, reason = choice
                        print("chose road %s at junction %d (%s)" % (DIR_NAME[d], here, reason))
                        # Only mark a road TAKEN if it was untried. A link must keep its
                        # destination, or the map loses the connection.
                        if roads_here.get(d, {}).get("state") == UNTRIED:
                            roads_here[d] = {"state": TAKEN}
                        self.pending = (here, d)
                        self.start_turn_to(d * HALF_PI - self.scan_start_heading)
                else:
                    self.drive(-SCAN_SPEED, SCAN_SPEED)

            elif self.state == self.BACKUP:
                if self.travelled <= self.backup_goal:
                    self.drive(0.0, 0.0)
                    self.state = self.TURN
                else:
                    self.drive(-APPROACH_SPEED, -APPROACH_SPEED)

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

            else:
                self.drive(0.0, 0.0)

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