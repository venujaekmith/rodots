"""eLLo autonomous controller for REIDBOTS Mission K-391, Round 01.

Webots R2025a, Python 3, standard library plus the Webots ``controller`` module
only. No hardcoded maps, coordinates or world-specific branches.

Layers (see README / spec section 5):
  1. RingBufferFilter, SensorSuite      adaptive IR normalisation, RGB pad vote,
                                         median obstacle detector
  2. TopologicalGraph                   spatial-hashed nodes, DFS decision stack,
                                         A* sprint and nearest-frontier Dijkstra
  3. Odometry, LinePID, turn helpers    differential odometry with heading snap,
                                         curvature-adaptive PID, closed-loop turns
  4. ElloMissionController              state machine, checkpoint sequence, undo
                                         and backtrack, parking, telemetry

Device names and geometry were checked against mission_k391_round01.wbt:
wheel radius 0.10 m, axle 0.34 m, IR index 0 on the robot's left, checkpoint
cameras 0.13 m ahead of the axle, front distance sensor 0.19 m ahead with a
lookup of 1000 at 0 m falling to 0 at 0.5 m.
"""

import heapq
import math
import sys
from collections import Counter, deque
from enum import Enum, auto

from controller import Camera, Robot


# ============================================================================
# Hardware contract
# ============================================================================
REQUIRED_DEVICES = (
    ["ir_line_%d" % i for i in range(7)]
    + [
        "left_wheel_motor",
        "right_wheel_motor",
        "left_wheel_encoder",
        "right_wheel_encoder",
        "front_obstacle_sensor",
        "checkpoint_color_left",
        "checkpoint_color_right",
        "rgb_status_led",
    ]
)

WHEEL_RADIUS = 0.10                 # m
AXLE_LENGTH = 0.34                  # m, wheel anchors at y = +/-0.17
ROBOT_HALF_LENGTH = 0.175           # m, chassis, bumper and rear caster from axle
CAM_FORWARD = 0.13                  # m, checkpoint cameras ahead of axle
IR_WEIGHTS = (-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0)   # index 0 = robot left
IR_COUNT = 7
PAD_HALF_SIZE = 0.36                # m, 0.72 m checkpoint squares

# Front sensor lookup in the world file: 1000 at 0 m, 0 at 0.5 m.
FRONT_VALUE_AT_ZERO = 1000.0
FRONT_VALUE_SLOPE = 2000.0          # per metre
ROCK_STOP_DISTANCE = 0.24           # m from sensor, leaves about 0.22 m clear
ROCK_TRIGGER = FRONT_VALUE_AT_ZERO - FRONT_VALUE_SLOPE * ROCK_STOP_DISTANCE

TIME_LIMIT_S = 1200.0
HALF_PI = math.pi / 2.0


# ============================================================================
# Tuning
# ============================================================================
CALIBRATION_STEPS = 25
CRUISE_SPEED = 5.0                  # rad/s, about 0.5 m/s
CURVE_SPEED = 2.5                   # rad/s on sharp curves
APPROACH_SPEED = 2.2                # rad/s near junctions and pads
SEARCH_SPEED = 1.2                  # rad/s for sweeps and slow reacquisition
BACKUP_SPEED = 1.2                  # rad/s when reversing before a pivot
TURN_MAX = 2.2                      # rad/s wheel command cap for pivots
TURN_MIN = 0.9                      # rad/s, friction-breaking floor
TURN_GAIN = 2.5
TURN_TOLERANCE = 0.035              # rad, about 2 degrees
KP, KI, KD = 1.1, 0.02, 0.6
INTEGRAL_CLAMP = 20.0
IR_RELAX = 0.3                      # IR bound drift per step, sensor units
IR_MIN_CONTRAST = 15.0
LINE_LOST_LIMIT = 12                # steps before bounded recovery
REACQUIRE_LIMIT = 260               # steps for a pivot to find the line
RECOVER_RIGHT_STEPS = 60
RECOVER_LEFT_STEPS = 120
JUNCTION_CONFIRM_STEPS = 3
JUNCTION_CENTER_OFFSET = 0.11       # m from trigger until axle is on the crossing
JUNCTION_PROBE = 0.10               # m beyond the crossing for the straight check
JUNCTION_WINDOW = JUNCTION_CENTER_OFFSET + JUNCTION_PROBE
BACKUP_AT_JUNCTION = JUNCTION_PROBE # reverse so the pivot sits on the crossing
ROCK_BACKUP = 0.10                  # m reverse before the U-turn after a rock
DEADEND_BACKUP = 0.04               # m reverse before the U-turn at a dead end
NODE_MERGE_RADIUS = 0.25            # m
NODE_ARRIVE_RADIUS = 0.06           # m, arrival test while backtracking
RETURN_EXTRA = 0.30                 # m, backtrack watchdog margin
GREEN_SPAWN_EXCLUSION = 0.70        # m, ignores the GREEN START pad at spawn
PAD_HALF_SIZE = 0.36                # m, half of a 0.72 m checkpoint square
# Park margin is sized for heading error at first sight: the side-mounted
# cameras (0.12 m off-axis) can be up to ~0.04 m ahead of the centre-line estimate
# when the robot is still turning into the pad. The 0.72 m square leaves room.
PARK_MARGIN = 0.10                  # m
PARK_TRAVEL = CAM_FORWARD + ROBOT_HALF_LENGTH + PARK_MARGIN
LOG_PERIOD_S = 2.0


# ============================================================================
# Enums and constants
# ============================================================================
class State(Enum):
    CALIBRATE = auto()
    FOLLOW = auto()
    JUNCTION = auto()
    BACKUP = auto()
    TURN = auto()
    REACQUIRE = auto()
    RECOVER = auto()
    PARK = auto()
    DONE = auto()
    SAFE_STOP = auto()


class PadColor(Enum):
    NONE = 0
    GREEN = 1
    RED = 2
    BLUE = 3
    YELLOW = 4


CHECKPOINT_ORDER = (PadColor.GREEN, PadColor.RED, PadColor.BLUE, PadColor.YELLOW)
LED_RGB = {
    PadColor.NONE: 0x000000,
    PadColor.GREEN: 0x00FF00,
    PadColor.RED: 0xFF0000,
    PadColor.BLUE: 0x0000FF,
    PadColor.YELLOW: 0xFFFF00,
}

# Exit states on a node's four cardinal directions.
ABSENT, UNEXPLORED, TRAVERSING, LINKED, BLOCKED = range(5)
EXIT_NAMES = ("ABSENT", "UNEXPLORED", "TRAVERSING", "LINKED", "BLOCKED")


# ============================================================================
# Small helpers
# ============================================================================
def clamp(value, low, high):
    return max(low, min(high, value))


def angle_wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def classify_rgb(r, g, b):
    """Map a mean RGB sample to a checkpoint colour, or NONE."""
    maximum = max(r, g, b)
    minimum = min(r, g, b)
    if maximum < 45:
        return PadColor.NONE
    rn, gn, bn = r / maximum, g / maximum, b / maximum
    saturation = (maximum - minimum) / max(maximum, 1.0)
    if saturation < 0.20:
        return PadColor.NONE
    if rn > 0.78 and gn > 0.55 and bn < 0.48 and maximum >= 125 and abs(rn - gn) < 0.42:
        return PadColor.YELLOW
    if rn > 0.72 and maximum >= 150 and rn > gn * 1.18 and rn > bn * 1.25:
        return PadColor.RED
    if gn > 0.68 and maximum >= 125 and gn > rn * 1.12 and gn > bn * 1.12:
        return PadColor.GREEN
    if bn > 0.62 and maximum >= 125 and bn > rn * 1.08 and bn > gn * 1.08:
        return PadColor.BLUE
    return PadColor.NONE


# ============================================================================
# 1. Sensor filters and sensor suite
# ============================================================================
class RingBufferFilter:
    """Fixed-length sample window. Push is O(1); median and majority are O(k log k)
    with k at most a few samples, so they stay constant-time in practice."""

    def __init__(self, size):
        self._buf = deque(maxlen=size)

    def push(self, value):
        self._buf.append(value)
        return self

    def median(self, default=0.0):
        if not self._buf:
            return default
        ordered = sorted(self._buf)
        return ordered[len(ordered) // 2]

    def majority(self, default):
        if not self._buf:
            return default
        value, count = Counter(self._buf).most_common(1)[0]
        return value if count * 2 > len(self._buf) else default


class SensorSuite:
    """Owns every Webots device. Consumers get normalised values, not raw ones."""

    def __init__(self, robot, step_ms):
        devices = {name: robot.getDevice(name) for name in REQUIRED_DEVICES}
        missing = [name for name, device in devices.items() if device is None]
        if missing:
            raise RuntimeError("missing Webots device(s): " + ", ".join(missing))

        self.ir = [devices["ir_line_%d" % i] for i in range(IR_COUNT)]
        self.enc_left = devices["left_wheel_encoder"]
        self.enc_right = devices["right_wheel_encoder"]
        self.front = devices["front_obstacle_sensor"]
        self.cam_left = devices["checkpoint_color_left"]
        self.cam_right = devices["checkpoint_color_right"]
        self.led = devices["rgb_status_led"]
        self.mot_left = devices["left_wheel_motor"]
        self.mot_right = devices["right_wheel_motor"]

        for sensor in self.ir:
            sensor.enable(step_ms)
        self.enc_left.enable(step_ms)
        self.enc_right.enable(step_ms)
        self.front.enable(step_ms)
        self.cam_left.enable(step_ms)
        self.cam_right.enable(step_ms)

        self.mot_left.setPosition(float("inf"))
        self.mot_right.setPosition(float("inf"))
        # Read the real motor limit instead of hardcoding it.
        limits = [m for m in (self.mot_left.getMaxVelocity(),
                              self.mot_right.getMaxVelocity()) if m and m > 0]
        self.max_vel = min(limits) if limits else 8.0
        self.stop()

        self.lo = [0.0] * IR_COUNT
        self.hi = [0.0] * IR_COUNT
        self.black_low = True
        self.last_ir = [0.0] * IR_COUNT
        self.front_votes = RingBufferFilter(3)
        self.pad_votes = RingBufferFilter(3)
        self.pad_color = PadColor.NONE
        self.raw_pad = PadColor.NONE

    # ---- actuation -------------------------------------------------------
    def drive(self, left, right):
        self.mot_left.setVelocity(clamp(left, -self.max_vel, self.max_vel))
        self.mot_right.setVelocity(clamp(right, -self.max_vel, self.max_vel))

    def stop(self):
        self.mot_left.setVelocity(0.0)
        self.mot_right.setVelocity(0.0)

    def set_led(self, color):
        self.led.set(LED_RGB.get(color, 0))

    # ---- odometry input --------------------------------------------------
    def read_encoders(self):
        left = float(self.enc_left.getValue())
        right = float(self.enc_right.getValue())
        return (left if math.isfinite(left) else 0.0,
                right if math.isfinite(right) else 0.0)

    # ---- IR line sensors -------------------------------------------------
    def read_ir(self):
        values = []
        for i, sensor in enumerate(self.ir):
            value = float(sensor.getValue())
            if not math.isfinite(value):
                value = self.last_ir[i]
            values.append(value)
            self.last_ir[i] = value
        return values

    def calibrate_ir(self, samples):
        """Learn line polarity from a stationary sample on the spawn track.

        The spawn sits on the first track segment, so the central sensors see the
        black line and the outer ones see the floor. Whichever group reads lower
        decides whether black is low or high. Ties default to black-low, which
        matches the world's IR lookup table."""
        n = float(len(samples))
        mean = [sum(s[i] for s in samples) / n for i in range(IR_COUNT)]
        centre = (mean[2] + mean[3] + mean[4]) / 3.0
        outer = (mean[0] + mean[1] + mean[5] + mean[6]) / 4.0
        self.black_low = centre <= outer
        # Seed each sensor's bounds with the measured black and white levels so the
        # contrast is non-zero even when a sensor read a constant value at spawn.
        low, high = min(centre, outer), max(centre, outer)
        self.lo = [low] * IR_COUNT
        self.hi = [high] * IR_COUNT
        return centre, outer

    def update_ir(self, values):
        """Adaptive per-sensor thresholding. Bounds relax toward the current
        reading so the threshold follows slow drift in the floor and line levels."""
        bits = []
        for i, v in enumerate(values):
            if v < self.lo[i]:
                self.lo[i] = v
            else:
                self.lo[i] += IR_RELAX
            if v > self.hi[i]:
                self.hi[i] = v
            else:
                self.hi[i] -= IR_RELAX
            contrast = self.hi[i] - self.lo[i]
            if contrast < IR_MIN_CONTRAST:
                bits.append(False)
                continue
            threshold = 0.5 * (self.lo[i] + self.hi[i])
            is_black = (v <= threshold) if self.black_low else (v >= threshold)
            bits.append(is_black)
        return bits

    def line(self, values):
        """Return (bits, weighted centroid, seen). A negative centroid means the
        line is on the robot's left."""
        bits = self.update_ir(values)
        count = sum(bits)
        if count == 0:
            return bits, 0.0, False
        centroid = sum(w for w, b in zip(IR_WEIGHTS, bits) if b) / count
        return bits, centroid, True

    # ---- front distance sensor -------------------------------------------
    def read_front(self):
        value = float(self.front.getValue())
        if not math.isfinite(value):
            value = 0.0
        return self.front_votes.push(value).median()

    # ---- downward RGB cameras --------------------------------------------
    def _camera_mean(self, camera):
        image = camera.getImage()
        if image is None:
            return None
        width = camera.getWidth()
        height = camera.getHeight()
        mx = max(1, width // 8)
        my = max(1, height // 8)
        r = g = b = 0.0
        count = 0
        for y in range(height // 2 - my, height // 2 + my):
            for x in range(width // 2 - mx, width // 2 + mx):
                if 0 <= x < width and 0 <= y < height:
                    r += Camera.imageGetRed(image, width, x, y)
                    g += Camera.imageGetGreen(image, width, x, y)
                    b += Camera.imageGetBlue(image, width, x, y)
                    count += 1
        if count == 0:
            return None
        return r / count, g / count, b / count

    def read_pad(self):
        """Classify each camera, take the first non-NONE reading, and majority-vote
        the last three frames so one noisy frame never flips the pad state."""
        raw = PadColor.NONE
        for camera in (self.cam_left, self.cam_right):
            mean = self._camera_mean(camera)
            if mean is not None:
                raw = classify_rgb(*mean)
                if raw != PadColor.NONE:
                    break
        self.raw_pad = raw
        self.pad_color = self.pad_votes.push(raw).majority(PadColor.NONE)
        return self.pad_color


# ============================================================================
# 2. Topological graph, DFS decision stack, planners
# ============================================================================
class Node:
    __slots__ = ("nid", "x", "y", "state", "link")

    def __init__(self, nid, x, y):
        self.nid = nid
        self.x = x
        self.y = y
        self.state = [ABSENT] * 4        # per cardinal exit
        self.link = [None] * 4           # (neighbour id, cost m, neighbour exit dir)


class DecisionFrame:
    __slots__ = ("node_id", "entry_dir", "chosen_exit_dir", "edge_distance")

    def __init__(self, node_id, entry_dir, chosen_exit_dir, edge_distance=None):
        self.node_id = node_id
        self.entry_dir = entry_dir
        self.chosen_exit_dir = chosen_exit_dir
        self.edge_distance = edge_distance

    def __repr__(self):
        return "Frame(node=%d, entry=%d, exit=%d)" % (
            self.node_id, self.entry_dir, self.chosen_exit_dir)


class TopologicalGraph:
    """Graph of junctions and checkpoints.

    Nodes live in a spatial hash whose cell size equals the merge radius, so a
    lookup inspects at most nine buckets. Edges store both end directions,
    which is needed because a curved edge leaves one node and arrives at the
    other on different absolute headings.

    The decision stack is a LIFO of the exploration choices made so far. A
    failed branch pops its own frame, so an undo never touches an unrelated
    decision.
    """

    def __init__(self, merge_radius):
        self.radius = merge_radius
        self.nodes = []
        self.buckets = {}
        self.checkpoints = {}            # PadColor -> node id
        self.decision_stack = []

    # ---- spatial hash ----------------------------------------------------
    def _key(self, x, y):
        return (math.floor(x / self.radius), math.floor(y / self.radius))

    def nearest(self, x, y):
        kx, ky = self._key(x, y)
        best, best_d = None, self.radius
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for nid in self.buckets.get((kx + dx, ky + dy), ()):
                    node = self.nodes[nid]
                    d = math.hypot(node.x - x, node.y - y)
                    if d < best_d:
                        best, best_d = nid, d
        return best

    def create(self, x, y):
        nid = len(self.nodes)
        self.nodes.append(Node(nid, x, y))
        self.buckets.setdefault(self._key(x, y), []).append(nid)
        return nid

    # ---- edges -----------------------------------------------------------
    def link(self, a, ea, b, eb, cost):
        na, nb = self.nodes[a], self.nodes[b]
        na.state[ea] = LINKED
        na.link[ea] = (b, cost, eb)
        nb.state[eb] = LINKED
        nb.link[eb] = (a, cost, ea)

    def block(self, a, ea):
        na = self.nodes[a]
        if na.state[ea] == LINKED and na.link[ea] is not None:
            b, _, eb = na.link[ea]
            nb = self.nodes[b]
            nb.state[eb] = BLOCKED
            nb.link[eb] = None
        na.state[ea] = BLOCKED
        na.link[ea] = None

    def split(self, a, ea, along, x, y, heading_dir):
        """Insert a node at (x, y), a distance `along` down edge (a, ea).

        Used when a checkpoint is confirmed mid-corridor. If the edge was
        already linked, both halves are relinked; otherwise the forward half
        is marked as being traversed."""
        p = self.create(x, y)
        back = (heading_dir + 2) % 4
        node_a = self.nodes[a]
        if node_a.state[ea] == LINKED and node_a.link[ea] is not None:
            b, cost, eb = node_a.link[ea]
            self.link(a, ea, p, back, along)
            self.link(p, heading_dir, b, eb, max(cost - along, 0.0))
        else:
            self.link(a, ea, p, back, along)
            self.nodes[p].state[heading_dir] = TRAVERSING
        return p

    # ---- decision stack --------------------------------------------------
    def push_decision(self, node_id, entry_dir, chosen_exit_dir):
        frame = DecisionFrame(node_id, entry_dir, chosen_exit_dir)
        self.decision_stack.append(frame)
        return frame

    def pop_failed(self, node_id, exit_dir):
        """Remove the frame for a failed branch. Usually the top; searched
        from the top otherwise so mismatches are tolerated."""
        for i in range(len(self.decision_stack) - 1, -1, -1):
            frame = self.decision_stack[i]
            if frame.node_id == node_id and frame.chosen_exit_dir == exit_dir:
                return self.decision_stack.pop(i)
        return None

    # ---- planners --------------------------------------------------------
    def shortest(self, src, is_goal, heuristic):
        """Best-first search over LINKED edges. With a zero heuristic this is
        Dijkstra; with Euclidean distance it is A*. The heuristic is scaled by
        0.9 so measured odometry error cannot make it inadmissible.

        Returns (found, first_exit_dir). first_exit_dir is None when src
        itself satisfies the goal."""
        g = {src: 0.0}
        first = {src: None}
        closed = set()
        tie = 0
        heap = [(heuristic(src), tie, src)]
        while heap:
            _, _, u = heapq.heappop(heap)
            if u in closed:
                continue
            closed.add(u)
            if is_goal(u):
                return True, first[u]
            node = self.nodes[u]
            for e in range(4):
                if node.state[e] != LINKED or node.link[e] is None:
                    continue
                v, cost, _ = node.link[e]
                candidate = g[u] + cost
                if candidate < g.get(v, math.inf):
                    g[v] = candidate
                    first[v] = e if u == src else first[u]
                    tie += 1
                    heapq.heappush(heap, (candidate + heuristic(v), tie, v))
        return False, None


# ============================================================================
# 3. Motion: odometry, PID, pivots
# ============================================================================
class Odometry:
    """Differential odometry from wheel encoders, with heading snap and gentle
    position blending on revisited junctions to cancel long-run drift."""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.h = 0.0
        self.total = 0.0                 # signed distance driven since spawn
        self._last = None

    def update(self, enc_left, enc_right):
        if self._last is None:
            self._last = (enc_left, enc_right)
            return 0.0
        dl = (enc_left - self._last[0]) * WHEEL_RADIUS
        dr = (enc_right - self._last[1]) * WHEEL_RADIUS
        self._last = (enc_left, enc_right)
        ds = 0.5 * (dl + dr)
        dth = (dr - dl) / AXLE_LENGTH
        mid = self.h + 0.5 * dth
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.h = angle_wrap(self.h + dth)
        self.total += ds
        return ds

    def heading_dir(self):
        return int(round(self.h / HALF_PI)) % 4

    def distance_to(self, x, y):
        return math.hypot(self.x - x, self.y - y)

    def distance_from_origin(self):
        return math.hypot(self.x, self.y)

    def nudge(self, dx, dy, alpha=0.35):
        self.x += alpha * dx
        self.y += alpha * dy

    def gentle_snap(self, gain=0.02):
        """Pull heading toward the nearest cardinal axis. Track segments are
        axis-aligned, so this removes slow yaw drift on straights."""
        nearest = round(self.h / HALF_PI) * HALF_PI
        self.h = angle_wrap(self.h + gain * angle_wrap(nearest - self.h))

    def snap_heading(self, target):
        self.h = angle_wrap(target)


class LinePID:
    """Weighted-centroid PID. Output is a differential term added to the left
    wheel and subtracted from the right."""

    def __init__(self):
        self.integral = 0.0
        self.prev = 0.0
        self.last_delta = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev = 0.0
        self.last_delta = 0.0

    def step(self, centroid):
        self.integral = clamp(self.integral + centroid, -INTEGRAL_CLAMP, INTEGRAL_CLAMP)
        self.last_delta = centroid - self.prev
        self.prev = centroid
        return KP * centroid + KI * self.integral + KD * self.last_delta


# ============================================================================
# 4. Mission controller
# ============================================================================
class ElloMissionController:
    def __init__(self, robot):
        self.robot = robot
        self.step_ms = int(robot.getBasicTimeStep())
        self.dt = self.step_ms / 1000.0
        self.sensors = SensorSuite(robot, self.step_ms)
        self.odom = Odometry()
        self.graph = TopologicalGraph(NODE_MERGE_RADIUS)
        self.pid = LinePID()

        self.state = State.CALIBRATE
        self.steps = 0
        self.t = 0.0
        self.last_log = -1e9
        self.smoke = "--motor-smoke" in sys.argv

        # Sensor snapshot for this step.
        self.ir_values = [0.0] * IR_COUNT
        self.bits = [False] * IR_COUNT
        self.centroid = 0.0
        self.line_seen = False
        self.outer_seen = False
        self.center_seen = False
        self.front_value = 0.0
        self.pad_color = PadColor.NONE
        self.last_pad = PadColor.NONE

        # Checkpoint sequence.
        self.target_index = 0
        self.first_yellow_total = None
        self.spawn_exclusion_active = True

        # Edge being traversed: the node it left and the exit it used.
        spawn = self.graph.create(0.0, 0.0)
        self.graph.nodes[spawn].state[0] = TRAVERSING
        self.edge_from = spawn
        self.edge_dir = 0
        self.edge_dist = 0.0

        # Junction probe.
        self.junction_votes = 0
        self.j_start = 0.0
        self.j_left = False
        self.j_right = False
        self.j_fwd = False
        self.j_snapshot = None
        self.pending_backup = False

        # Turns, backups and sweeps.
        self.turn_target = 0.0
        self.spin_sign = 1
        self.backup_goal = 0.0
        self.reacquire_steps = 0
        self.recover_steps = 0
        self.line_lost = 0
        self.travel_heading = 0.0        # heading when the line was last centred
        self.calib_samples = []

        # Undo / backtrack.
        self.returning = False
        self.return_node = None
        self.return_len = 0.0
        self.return_start = None

        # A rock can be sensed while the robot is still on a parent edge, because
        # the sensor looks past the junction. Such blocks are remembered so they
        # can be reopened for one bounded retry if nothing else is reachable.
        self.rock_blocks = []
        self.rock_retries = 0

    # ------------------------------------------------------------------
    # Logging and utilities
    # ------------------------------------------------------------------
    def log(self, message):
        print("[%8.2fs] %s" % (self.t, message), flush=True)

    def current_target(self):
        if self.target_index < len(CHECKPOINT_ORDER):
            return CHECKPOINT_ORDER[self.target_index]
        return None

    def set_state(self, state):
        if state == State.FOLLOW:
            self.pid.reset()
            self.line_lost = 0
        self.state = state

    def enter_safe_stop(self, reason):
        self.sensors.stop()
        self.log("SAFE STOP: " + reason)
        self.set_state(State.SAFE_STOP)

    def drive(self, left, right):
        self.sensors.drive(left, right)

    def follow_line(self, cruise):
        """Curvature-adaptive PID: slow down in proportion to the line's offset
        and how quickly that offset is changing."""
        turn = self.pid.step(self.centroid)
        floor = min(CURVE_SPEED, cruise)
        blend = min(1.0, abs(self.centroid) / 1.5 + 0.4 * abs(self.pid.last_delta))
        speed = cruise - (cruise - floor) * blend
        self.drive(speed + turn, speed - turn)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def run(self):
        self.log("eLLo online. max_velocity=%.2f rad/s, time step=%d ms"
                 % (self.sensors.max_vel, self.step_ms))
        self.sensors.set_led(CHECKPOINT_ORDER[0])
        self.sensors.stop()
        if self.smoke:
            return self.run_motor_smoke()

        while self.robot.step(self.step_ms) != -1:
            self.steps += 1
            self.t = self.steps * self.dt
            self.read_sensors()
            self.update_checkpoints()

            if self.t >= TIME_LIMIT_S and self.state not in (State.DONE, State.SAFE_STOP):
                self.enter_safe_stop("time limit reached")

            self.dispatch()
            self.telemetry()

            if self.state in (State.DONE, State.SAFE_STOP):
                self.sensors.stop()
                break

        self.sensors.stop()
        return 0

    def run_motor_smoke(self):
        """Optional motor and encoder check, enabled with controllerArgs
        ["--motor-smoke"]. Drives both wheels at 0.5 rad/s from 0.6 to 1.1 s."""
        while self.robot.step(self.step_ms) != -1:
            self.steps += 1
            self.t = self.steps * self.dt
            if 0.6 <= self.t < 1.1:
                self.sensors.drive(0.5, 0.5)
            else:
                self.sensors.stop()
            if self.t >= 1.2:
                break
        self.sensors.stop()
        self.log("MOTOR SMOKE COMPLETE")
        return 0

    def read_sensors(self):
        enc_left, enc_right = self.sensors.read_encoders()
        ds = self.odom.update(enc_left, enc_right)
        if self.edge_from is not None:
            self.edge_dist += ds

        self.ir_values = self.sensors.read_ir()
        self.bits, self.centroid, self.line_seen = self.sensors.line(self.ir_values)
        # Only the outermost sensors (0 and 6, 0.075 m out) mark a side branch.
        # Sensors 1 and 5 sit 0.05 m out and fire on ordinary tracking offset.
        self.outer_seen = self.bits[0] or self.bits[6]
        self.center_seen = self.bits[2] or self.bits[3] or self.bits[4]
        self.front_value = self.sensors.read_front()
        self.pad_color = self.sensors.read_pad()

    def update_checkpoints(self):
        if (self.sensors.raw_pad == PadColor.YELLOW and self.target_index == 3
                and self.first_yellow_total is None):
            self.first_yellow_total = self.odom.total
        if self.pad_color != self.last_pad:
            self.last_pad = self.pad_color
            if self.pad_color != PadColor.NONE:
                self.on_checkpoint(self.pad_color)

    def on_checkpoint(self, color):
        if color == PadColor.GREEN and self.spawn_exclusion_active:
            if self.odom.distance_from_origin() < GREEN_SPAWN_EXCLUSION:
                self.log("ignoring the GREEN START pad near spawn")
                return
        self.spawn_exclusion_active = False
        self.register_pad(color)

        if color == CHECKPOINT_ORDER[self.target_index]:
            if color == PadColor.YELLOW:
                self.begin_park()
                return
            self.target_index += 1
            self.sensors.set_led(CHECKPOINT_ORDER[self.target_index])
            self.log("CHECKPOINT %s confirmed; next target %s"
                     % (color.name, CHECKPOINT_ORDER[self.target_index].name))
        else:
            self.log("out-of-order checkpoint %s saved to memory (target still %s)"
                     % (color.name, self.current_target().name))

    def register_pad(self, color):
        """Place the checkpoint at the pad centre.

        The colour is confirmed when the cameras first see the pad, which is at
        the near edge, so the centre lies CAM_FORWARD + PAD_HALF_SIZE ahead of
        the axle. A pad that coincides with a known node reuses it; otherwise
        the current edge is split there. The edge tracker then measures from the
        pad centre, so the axle sits at a negative distance from it."""
        heading = self.odom.heading_dir()
        ahead = CAM_FORWARD + PAD_HALF_SIZE
        cx = self.odom.x + ahead * math.cos(self.odom.h)
        cy = self.odom.y + ahead * math.sin(self.odom.h)
        nid = self.graph.nearest(cx, cy)
        if nid is None and self.edge_from is not None:
            along = self.edge_dist + ahead
            nid = self.graph.split(self.edge_from, self.edge_dir, along, cx, cy, heading)
        if nid is None:
            self.log("pad %s seen with no graph context; progress still counts"
                     % color.name)
            return
        self.graph.checkpoints[color] = nid
        self.edge_from = nid
        self.edge_dir = heading
        self.edge_dist = -ahead

    def begin_park(self):
        if self.first_yellow_total is None:
            self.first_yellow_total = self.odom.total - 0.05
        self.sensors.set_led(PadColor.YELLOW)
        self.log("YELLOW confirmed as target. Parking so the whole footprint sits inside.")
        self.set_state(State.PARK)

    # ------------------------------------------------------------------
    # Dispatch
    # ------------------------------------------------------------------
    def dispatch(self):
        s = self.state
        if s == State.CALIBRATE:
            self.do_calibrate()
        elif s == State.FOLLOW:
            self.do_follow()
        elif s == State.JUNCTION:
            self.do_junction()
        elif s == State.BACKUP:
            self.do_backup()
        elif s == State.TURN:
            self.do_turn()
        elif s == State.REACQUIRE:
            self.do_reacquire()
        elif s == State.RECOVER:
            self.do_recover()
        elif s == State.PARK:
            self.do_park()
        elif s in (State.DONE, State.SAFE_STOP):
            self.sensors.stop()

    def do_calibrate(self):
        self.sensors.stop()
        self.calib_samples.append(list(self.ir_values))
        if len(self.calib_samples) < CALIBRATION_STEPS:
            return
        centre, outer = self.sensors.calibrate_ir(self.calib_samples)
        polarity = "black-low" if self.sensors.black_low else "black-high"
        if abs(centre - outer) < 4.0:
            self.log("IR polarity uncertain (centre %.1f, outer %.1f); defaulting to %s"
                     % (centre, outer, polarity))
        else:
            self.log("IR calibrated: %s (centre %.1f, outer %.1f)" % (polarity, centre, outer))
        self.log("front sensor baseline reading %.1f" % self.front_value)
        self.set_state(State.FOLLOW)

    # ---- FOLLOW ------------------------------------------------------------
    def do_follow(self):
        if self.front_value >= ROCK_TRIGGER:
            self.log("rock ahead (front reading %.0f)" % self.front_value)
            self.on_failure("rock", ROCK_BACKUP)
            return

        if self.returning and self.return_arrived():
            self.sensors.stop()
            self.returning = False
            self.log("backtracked to node %d" % self.return_node)
            self.decide(self.return_node)
            return

        # Junction candidate: outer sensors active while not on a colour pad.
        if self.pad_color == PadColor.NONE and self.outer_seen:
            self.junction_votes += 1
        else:
            self.junction_votes = 0
        if self.junction_votes >= JUNCTION_CONFIRM_STEPS:
            self.begin_junction()
            return

        if self.line_seen:
            self.line_lost = 0
            if abs(self.centroid) < 0.3:
                self.travel_heading = self.odom.h
            if abs(self.centroid) < 0.2 and not self.returning:
                self.odom.gentle_snap()
            self.follow_line(CRUISE_SPEED)
        else:
            self.line_lost += 1
            if self.line_lost >= LINE_LOST_LIMIT:
                self.log("line lost; bounded recovery")
                self.begin_recover()
            else:
                self.drive(APPROACH_SPEED, APPROACH_SPEED)

    def return_arrived(self):
        node = self.graph.nodes[self.return_node]
        if self.odom.distance_to(node.x, node.y) <= NODE_ARRIVE_RADIUS:
            return True
        return (self.return_start is not None
                and self.odom.total - self.return_start > self.return_len + RETURN_EXTRA)

    # ---- JUNCTION ----------------------------------------------------------
    def begin_junction(self):
        self.junction_votes = 0
        self.j_start = self.odom.total
        self.j_left = self.j_right = self.j_fwd = False
        self.j_snapshot = None
        self.set_state(State.JUNCTION)

    def do_junction(self):
        travel = self.odom.total - self.j_start

        # Forward presence is checked only near the crossing, so a pivoting PID
        # steering into a side leg does not look like a straight continuation.
        if travel <= JUNCTION_CENTER_OFFSET + 0.02 and self.center_seen:
            self.j_fwd = True
        if self.bits[0]:
            self.j_left = True
        if self.bits[6]:
            self.j_right = True

        if self.j_snapshot is None and travel >= JUNCTION_CENTER_OFFSET:
            self.j_snapshot = (self.odom.x, self.odom.y, self.odom.heading_dir(),
                               self.edge_dist - (travel - JUNCTION_CENTER_OFFSET))

        if travel >= JUNCTION_WINDOW:
            self.sensors.stop()
            self.evaluate_junction()
            return

        if self.line_seen:
            self.follow_line(APPROACH_SPEED)
        else:
            self.drive(APPROACH_SPEED, APPROACH_SPEED)

    def evaluate_junction(self):
        x, y, heading, edge_len = self.j_snapshot
        outs = []
        if self.j_fwd:
            outs.append(heading)
        if self.j_left:
            outs.append((heading + 1) % 4)
        if self.j_right:
            outs.append((heading + 3) % 4)

        existing = self.graph.nearest(x, y)
        if existing is None and len(outs) < 2:
            # A simple bend or a spur, not a graph node. Keep following the line.
            self.log("bend at (%.2f, %.2f); not a graph node" % (x, y))
            self.set_state(State.FOLLOW)
            return

        if existing is None:
            nid = self.graph.create(x, y)
        else:
            nid = existing
            node = self.graph.nodes[nid]
            # Correct drift against the stored position, using the snapshot
            # where the axle crossed the junction centre.
            self.odom.nudge(node.x - self.j_snapshot[0], node.y - self.j_snapshot[1])
            self.log("revisited node %d; drift corrected" % nid)

        if self.edge_from is not None and self.edge_from != nid:
            self.graph.link(self.edge_from, self.edge_dir, nid, (heading + 2) % 4, edge_len)

        node = self.graph.nodes[nid]
        for d in outs:
            if node.state[d] == ABSENT:
                node.state[d] = UNEXPLORED

        if self.returning and nid == self.return_node:
            self.returning = False
        self.pending_backup = True
        self.log("junction node %d: exits %s, entry %s"
                 % (nid, [EXIT_NAMES[node.state[d]] for d in range(4)], heading))
        self.decide(nid)

    # ---- DECIDE (planner and DFS push) -------------------------------------
    def decide(self, nid):
        graph = self.graph
        node = graph.nodes[nid]
        here = self.odom.heading_dir()
        chosen = None
        exploring = False
        reason = ""

        goal = self.current_target()
        if goal is not None and goal in graph.checkpoints:
            dst = graph.checkpoints[goal]
            if dst != nid:
                dn = graph.nodes[dst]
                found, first = graph.shortest(
                    nid,
                    lambda u: u == dst,
                    lambda u: 0.9 * math.hypot(graph.nodes[u].x - dn.x, graph.nodes[u].y - dn.y))
                if found and first is not None:
                    chosen = first
                    reason = "A* sprint to %s at node %d" % (goal.name, dst)

        if chosen is None and UNEXPLORED in node.state:
            for rel in (0, 1, 3, 2):          # straight, left, right, back
                d = (here + rel) % 4
                if node.state[d] == UNEXPLORED:
                    chosen = d
                    exploring = True
                    reason = "explore new branch"
                    break

        if chosen is None:
            found, first = graph.shortest(
                nid,
                lambda u: UNEXPLORED in graph.nodes[u].state,
                lambda u: 0.0)
            if found and first is not None:
                chosen = first
                reason = "nearest frontier"

        if chosen is None and self.rock_blocks and self.rock_retries < 2:
            self.rock_retries += 1
            for bnid, bexit in self.rock_blocks:
                if graph.nodes[bnid].state[bexit] == BLOCKED:
                    graph.nodes[bnid].state[bexit] = UNEXPLORED
            self.log("nothing else reachable; reopening %d rock-blocked exit(s) for retry %d"
                     % (len(self.rock_blocks), self.rock_retries))
            self.rock_blocks = []
            self.decide(nid)
            return

        if chosen is None:
            self.enter_safe_stop("no unexplored exit and no reachable checkpoint at node %d" % nid)
            return

        if exploring:
            node.state[chosen] = TRAVERSING
            graph.push_decision(nid, (here + 2) % 4, chosen)
        self.log("decision at node %d -> exit %d (%s); stack depth %d"
                 % (nid, chosen, reason, len(graph.decision_stack)))
        self.begin_hop(nid, chosen)

    def begin_hop(self, nid, chosen):
        self.edge_from = nid
        self.edge_dir = chosen
        self.edge_dist = 0.0
        backup = BACKUP_AT_JUNCTION if self.pending_backup else 0.0
        self.pending_backup = False
        if chosen == self.odom.heading_dir():
            self.set_state(State.FOLLOW)
            return
        target = self.odom.h + angle_wrap(chosen * HALF_PI - self.odom.h)
        self.begin_turn(target, backup)

    # ---- BACKUP / TURN / REACQUIRE -----------------------------------------
    def begin_turn(self, target_heading, backup_dist=0.0):
        self.turn_target = angle_wrap(target_heading)
        self.pid.reset()
        if backup_dist > 1e-6:
            self.backup_goal = self.odom.total - backup_dist
            self.set_state(State.BACKUP)
        else:
            self.set_state(State.TURN)

    def do_backup(self):
        if self.odom.total <= self.backup_goal:
            self.sensors.stop()
            self.set_state(State.TURN)
        else:
            self.drive(-BACKUP_SPEED, -BACKUP_SPEED)

    def do_turn(self):
        err = angle_wrap(self.turn_target - self.odom.h)
        if abs(err) <= TURN_TOLERANCE:
            self.sensors.stop()
            self.odom.snap_heading(self.turn_target)
            self.reacquire_steps = 0
            if self.returning and self.return_start is None:
                self.return_start = self.odom.total
            self.set_state(State.REACQUIRE)
            return
        mag = clamp(TURN_GAIN * abs(err), TURN_MIN, TURN_MAX)
        w = mag if err > 0 else -mag
        self.spin_sign = 1 if err > 0 else -1
        self.drive(-w, w)

    def do_reacquire(self):
        if self.center_seen:
            self.sensors.stop()
            self.set_state(State.FOLLOW)
            return
        self.reacquire_steps += 1
        if self.reacquire_steps > REACQUIRE_LIMIT:
            self.log("line not found after pivot; bounded recovery")
            self.begin_recover()
            return
        self.drive(-SEARCH_SPEED * self.spin_sign, SEARCH_SPEED * self.spin_sign)

    # ---- RECOVER -----------------------------------------------------------
    def begin_recover(self):
        self.recover_steps = 0
        self.sensors.stop()
        self.set_state(State.RECOVER)

    def do_recover(self):
        if self.line_seen:
            self.sensors.stop()
            self.set_state(State.FOLLOW)
            return
        s = self.recover_steps
        self.recover_steps += 1
        if s < RECOVER_RIGHT_STEPS:
            self.drive(SEARCH_SPEED, -SEARCH_SPEED)
        elif s < RECOVER_RIGHT_STEPS + RECOVER_LEFT_STEPS:
            self.drive(-SEARCH_SPEED, SEARCH_SPEED)
        else:
            self.sensors.stop()
            if self.returning:
                self.enter_safe_stop("line lost while backtracking")
            else:
                self.on_failure("dead end", DEADEND_BACKUP)

    # ---- Undo and backtrack ------------------------------------------------
    def on_failure(self, reason, backup_dist):
        """A branch failed. Pop its frame, block its exit at the parent node,
        then reverse and return to that node."""
        self.sensors.stop()
        node_id = self.edge_from
        if self.returning or node_id is None:
            self.enter_safe_stop("%s with no undo target" % reason)
            return
        exit_dir = self.edge_dir
        travelled = self.edge_dist
        frame = self.graph.pop_failed(node_id, exit_dir)
        if frame is not None:
            frame.edge_distance = travelled
        self.graph.block(node_id, exit_dir)
        if reason == "rock":
            self.rock_blocks.append((node_id, exit_dir))
        self.log("UNDO (%s): node %d exit %d blocked after %.2f m; stack depth %d"
                 % (reason, node_id, exit_dir, travelled, len(self.graph.decision_stack)))

        self.returning = True
        self.return_node = node_id
        self.return_len = max(0.0, travelled - backup_dist)
        self.return_start = None
        self.edge_from = None
        # Turn to the cardinal opposite the last well-centred travel heading, not
        # the current heading, which a recovery sweep may have disturbed.
        travel_card = round(self.travel_heading / HALF_PI) * HALF_PI
        self.begin_turn(travel_card + math.pi, backup_dist)

    # ---- PARK --------------------------------------------------------------
    def do_park(self):
        if self.first_yellow_total is None:
            self.first_yellow_total = self.odom.total
        if self.odom.total - self.first_yellow_total >= PARK_TRAVEL:
            self.sensors.stop()
            self.sensors.set_led(PadColor.YELLOW)
            print("MISSION COMPLETE", flush=True)
            self.log("parked: footprint inside YELLOW; nodes=%d, checkpoints=%s"
                     % (len(self.graph.nodes),
                        {k.name: v for k, v in self.graph.checkpoints.items()}))
            self.set_state(State.DONE)
            return
        if self.line_seen:
            self.follow_line(APPROACH_SPEED)
        else:
            self.drive(APPROACH_SPEED, APPROACH_SPEED)

    # ---- Telemetry ---------------------------------------------------------
    def telemetry(self):
        if self.t - self.last_log < LOG_PERIOD_S:
            return
        self.last_log = self.t
        target = self.current_target()
        self.log("state=%s target=%s pos=(%+.2f,%+.2f) hdg=%+.0fdeg nodes=%d stack=%d "
                 "front=%.0f pad=%s ir=%s"
                 % (self.state.name, target.name if target else "-",
                    self.odom.x, self.odom.y, math.degrees(self.odom.h),
                    len(self.graph.nodes), len(self.graph.decision_stack),
                    self.front_value, self.pad_color.name,
                    "".join("1" if b else "0" for b in self.bits)))


# ============================================================================
# Entry point
# ============================================================================
def main():
    try:
        robot = Robot()
        return ElloMissionController(robot).run()
    except RuntimeError as error:
        print("STARTER ERROR: " + str(error), file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())