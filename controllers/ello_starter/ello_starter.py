"""eLLo Graph-Based Maze Solver & Ordered Checkpoint Controller.

Coordinate convention:
- Initial heading = 0.0 rad = North (+x axis), in meters.
- Left turn (+pi/2 rad) = West (+y axis); Right turn (-pi/2 rad) = East (-y axis).
- Moving 5.0 m straight North places a node at (x=5.0, y=0.0).

Graph model:
- JUNCTION nodes: topological decision vertices (start, 3/4-way intersections, dead ends)
  with (x, y) coordinates and loop-closure snapping to prevent duplicate path traversal.
- TURN nodes: geometric 90-degree corner waypoints (x, y) logged along edges without
  wasting time on 360-degree scans.
- Edges: track state (UNTRIED, EXPLORED, BLOCKED) and intermediate TURN waypoints.
"""

from collections import deque
import math
import sys

from controller import Camera, Robot


# --- Hardware & Geometry Constants (ROBOT_SPEC.md) ---
IR_NAMES = [f"ir_line_{i}" for i in range(7)]
IR_WEIGHTS = (-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0)  # index 0 = robot-left (+y)
SENSOR_BEARING = [math.atan2(0.075 - 0.025 * i, 0.13) for i in range(7)]

WHEEL_RADIUS = 0.10          # m
AXLE_LENGTH = 0.34           # m
SENSOR_FORWARD_OFFSET = 0.13 # m: IR array & cameras sit +0.13 m ahead of wheel axle
TWO_PI = 2.0 * math.pi
HALF_PI = math.pi / 2.0

# --- Speeds & PID Control ---
CRUISE_SPEED = 5.4           # rad/s on straights
CURVE_MIN_SPEED = 2.1        # rad/s on tight curves
APPROACH_SPEED = 2.6         # rad/s while centering axle over intersection
SCAN_SPEED = 2.8             # rad/s during 360-degree junction scan
TURN_MIN_SPEED = 1.0
TURN_MAX_SPEED = 2.6
TURN_GAIN = 2.8
TURN_TOLERANCE = 0.032       # rad (~1.8 deg)

KP, KI, KD = 1.15, 0.015, 0.65

# --- Line & Sensor Thresholds ---
CALIBRATION_STEPS = 20
LINE_FRACTION = 0.36
MAX_LINE_SENSORS = 4
LOST_GRACE_STEPS = 10
SEARCH_SWEEP_STEPS = 85
SEARCH_SPEED = 1.4

# --- Junction, Turn & Graph Loop-Closure Parameters ---
OUTER_CONFIRM_STEPS = 4
ROAD_CLUSTER_TOL = math.radians(24)
MIN_ROAD_HITS = 3
ANGLE_MATCH_TOL = math.radians(28)
JUNCTION_MERGE_RADIUS = 0.45  # m: snap odometry to existing junction within this radius
TURN_MERGE_RADIUS = 0.35      # m: deduplicate TURN waypoint nodes

# --- Two-Stage Obstacle Detection ---
OBSTACLE_PRE_ENTRY = 460.0    # front sensor threshold before committing to an edge
OBSTACLE_MID_PATH = 530.0     # front sensor threshold while following a corridor (~0.23 m)

# --- Heuristic Edge Weights & Start-Direction Resistance ---
WEIGHT_LEFT = 1.35            # higher priority to Left turns
WEIGHT_RIGHT = 1.30           # higher priority to Right turns
WEIGHT_STRAIGHT = 1.00        # lower priority to Straight
WEIGHT_REVERSE = 0.15         # avoid U-turns unless backtracking via BFS
START_RESISTANCE_MAX = 0.85   # max penalty for heading back toward (0, 0)
RESISTANCE_RAMP_DIST = 12.0   # m of travel to reach full start-direction resistance
RESISTANCE_RAMP_TIME = 45.0   # s of mission time to reach full start-direction resistance

# --- Checkpoints & Parking (RULES.md) ---
CHECKPOINTS = ("GREEN", "RED", "BLUE", "YELLOW")
LED_COLORS = {
    "GREEN": 0x00FF00,
    "RED": 0xFF0000,
    "BLUE": 0x0000FF,
    "YELLOW": 0xFFFF00,
    None: 0x000000,
}
PAD_CONFIRM_FRAMES = 3
PAD_EXIT_FRAMES = 4
START_BOX_RADIUS = 0.85       # m around (0, 0) treated as launch area, not GREEN checkpoint
BLINK_HALF_STEPS = 10
BLINK_CYCLES = 3
YELLOW_CENTER_DRIVE = 0.36    # m: centers 0.35x0.38 m robot inside 0.72x0.72 m YELLOW pad

# --- Edge States ---
UNTRIED = "UNTRIED"
EXPLORED = "EXPLORED"
BLOCKED = "BLOCKED"


def clamp(val, low, high):
    return max(low, min(high, val))


def angle_wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_diff(a, b):
    return abs(angle_wrap(a - b))


def classify_rgb(r, g, b):
    """Classify downward camera mean RGB into checkpoint name or None."""
    top = max(r, g, b)
    low = min(r, g, b)
    if top < 45.0 or (top - low) / top <= 0.20:
        return None
    rn, gn, bn = r / top, g / top, b / top
    if rn > 0.78 and gn > 0.55 and bn < 0.48 and top >= 125.0 and abs(rn - gn) < 0.42:
        return "YELLOW"
    if rn > 0.72 and top >= 150.0 and rn > gn * 1.18 and rn > bn * 1.25:
        return "RED"
    if gn > 0.68 and top >= 125.0 and gn > rn * 1.12 and gn > bn * 1.12:
        return "GREEN"
    if bn > 0.62 and top >= 125.0 and bn > rn * 1.08 and bn > gn * 1.08:
        return "BLUE"
    return None


def cluster_road_bearings(bearings):
    """Cluster raw relative road sightings into distinct branch bearings in (-pi, pi]."""
    if not bearings:
        return []
    ordered = sorted(b % TWO_PI for b in bearings)
    clusters = [[ordered[0]]]
    for b in ordered[1:]:
        if b - clusters[-1][-1] <= ROAD_CLUSTER_TOL:
            clusters[-1].append(b)
        else:
            clusters.append([b])
    if len(clusters) >= 2 and (ordered[0] + TWO_PI - clusters[-1][-1]) <= ROAD_CLUSTER_TOL:
        clusters[0] = clusters[-1] + clusters[0]
        clusters.pop()

    roads = []
    for cl in clusters:
        if len(cl) < MIN_ROAD_HITS:
            continue
        s = sum(math.sin(b) for b in cl)
        c = sum(math.cos(b) for b in cl)
        roads.append(angle_wrap(math.atan2(s, c)))
    return roads


class MazeGraph:
    """Topological + geometric graph storing JUNCTION and TURN nodes."""

    def __init__(self):
        # Nodes dict: id -> {"id": int, "type": "JUNCTION"|"TURN", "x": float, "y": float, "branches": [...]}
        self.nodes = {}
        self.next_id = 0
        # Checkpoint memory: color -> {"node_id": int, "branch_idx": int, "x": float, "y": float}
        self.checkpoints = {}

    def add_junction(self, x, y, branch_bearings):
        """Find existing JUNCTION within JUNCTION_MERGE_RADIUS or create a new one.
        Returns (node_id, snapped_x, snapped_y, is_new)."""
        best_id = None
        best_dist = JUNCTION_MERGE_RADIUS
        for nid, node in self.nodes.items():
            if node["type"] != "JUNCTION":
                continue
            dist = math.hypot(node["x"] - x, node["y"] - y)
            if dist <= best_dist:
                best_id = nid
                best_dist = dist

        if best_id is not None:
            node = self.nodes[best_id]
            # Merge any newly discovered bearings not already registered on this junction
            for b in branch_bearings:
                if self.match_branch(best_id, b) is None:
                    node["branches"].append({
                        "bearing": angle_wrap(b),
                        "state": UNTRIED,
                        "to_node": None,
                        "to_branch": None,
                        "turns": [],
                        "length": 0.0,
                    })
            return best_id, node["x"], node["y"], False

        nid = self.next_id
        self.next_id += 1
        branches = []
        for b in branch_bearings:
            branches.append({
                "bearing": angle_wrap(b),
                "state": UNTRIED,
                "to_node": None,
                "to_branch": None,
                "turns": [],
                "length": 0.0,
            })
        self.nodes[nid] = {
            "id": nid,
            "type": "JUNCTION",
            "x": x,
            "y": y,
            "branches": branches,
        }
        return nid, x, y, True

    def add_turn_node(self, x, y):
        """Register a TURN waypoint node (or return an existing one nearby)."""
        for nid, node in self.nodes.items():
            if node["type"] == "TURN" and math.hypot(node["x"] - x, node["y"] - y) <= TURN_MERGE_RADIUS:
                return nid
        nid = self.next_id
        self.next_id += 1
        self.nodes[nid] = {
            "id": nid,
            "type": "TURN",
            "x": x,
            "y": y,
            "branches": [],
        }
        return nid

    def match_branch(self, node_id, bearing):
        """Return index of branch on node_id matching absolute `bearing`, or None."""
        node = self.nodes[node_id]
        best_idx = None
        best_err = ANGLE_MATCH_TOL
        for idx, br in enumerate(node["branches"]):
            err = angle_diff(br["bearing"], bearing)
            if err <= best_err:
                best_err = err
                best_idx = idx
        return best_idx

    def link_corridor(self, src_id, src_br, dst_id, dst_br, turn_ids, length):
        """Connect two junction branches bidirectionally and mark both EXPLORED."""
        b_src = self.nodes[src_id]["branches"][src_br]
        b_dst = self.nodes[dst_id]["branches"][dst_br]

        if b_src["state"] != BLOCKED:
            b_src["state"] = EXPLORED
        b_src["to_node"] = dst_id
        b_src["to_branch"] = dst_br
        b_src["turns"] = list(turn_ids)
        b_src["length"] = length

        if b_dst["state"] != BLOCKED:
            b_dst["state"] = EXPLORED
        b_dst["to_node"] = src_id
        b_dst["to_branch"] = src_br
        b_dst["turns"] = list(reversed(turn_ids))
        b_dst["length"] = length

    def block_branch(self, node_id, branch_idx):
        """Mark a branch (and its paired reverse link, if known) as BLOCKED."""
        br = self.nodes[node_id]["branches"][branch_idx]
        br["state"] = BLOCKED
        if br["to_node"] is not None and br["to_branch"] is not None:
            far_br = self.nodes[br["to_node"]]["branches"][br["to_branch"]]
            far_br["state"] = BLOCKED

    def bfs_to_target(self, start_id, goal_predicate):
        """Shortest path over non-blocked edges from start_id to any junction satisfying
        goal_predicate(node_id). Returns (first_branch_idx, target_node_id) or None."""
        queue = deque([start_id])
        parent = {start_id: None}  # node_id -> (prev_node_id, via_branch_idx)

        while queue:
            curr = queue.popleft()
            if goal_predicate(curr):
                if curr == start_id:
                    return None, start_id
                step = curr
                while parent[step][0] != start_id:
                    step = parent[step][0]
                return parent[step][1], curr

            for idx, br in enumerate(self.nodes[curr]["branches"]):
                if br["state"] == BLOCKED or br["to_node"] is None:
                    continue
                nxt = br["to_node"]
                if nxt not in parent:
                    parent[nxt] = (curr, idx)
                    queue.append(nxt)
        return None


class ElloMazeController:
    # Finite-state machine states
    FOLLOW = "FOLLOW"
    APPROACH = "APPROACH"
    SCAN = "SCAN"
    TURN = "TURN"
    CHECK_ENTRY = "CHECK_ENTRY"
    ABORT_TURN = "ABORT_TURN"
    FINISH = "FINISH"
    STOPPED = "STOPPED"

    def __init__(self, robot):
        self.robot = robot
        self.step_ms = int(robot.getBasicTimeStep())

        # Devices
        self.left_motor = robot.getDevice("left_wheel_motor")
        self.right_motor = robot.getDevice("right_wheel_motor")
        self.left_enc = robot.getDevice("left_wheel_encoder")
        self.right_enc = robot.getDevice("right_wheel_encoder")
        self.front_sensor = robot.getDevice("front_obstacle_sensor")
        self.cam_left = robot.getDevice("checkpoint_color_left")
        self.cam_right = robot.getDevice("checkpoint_color_right")
        self.led = robot.getDevice("rgb_status_led")
        self.ir = [robot.getDevice(name) for name in IR_NAMES]

        required = [
            self.left_motor, self.right_motor, self.left_enc, self.right_enc,
            self.front_sensor, self.cam_left, self.cam_right, self.led, *self.ir
        ]
        if any(d is None for d in required):
            raise RuntimeError("Missing required eLLo device(s).")

        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        limits = [m for m in (self.left_motor.getMaxVelocity(), self.right_motor.getMaxVelocity()) if m > 0]
        self.max_vel = min(limits) if limits else 8.0

        self.left_enc.enable(self.step_ms)
        self.right_enc.enable(self.step_ms)
        self.front_sensor.enable(self.step_ms)
        self.cam_left.enable(self.step_ms)
        self.cam_right.enable(self.step_ms)
        for s in self.ir:
            s.enable(self.step_ms)

        # Pose & Odometry (North = 0.0 rad = +x)
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.travelled = 0.0
        self.last_enc = None
        self.front_buf = deque(maxlen=3)
        self.front_val = 0.0

        # IR Calibration
        self.black_ref = 100.0
        self.ground_ref = 285.0
        self.ir_threshold = 165.0
        self.ground_calibrated = False

        # PID Line Following
        self.integral = 0.0
        self.prev_err = 0.0
        self.last_side = 1.0
        self.lost_steps = 0
        self.outer_votes = 0
        self.outer_left_seen = False
        self.outer_right_seen = False

        # Graph & Navigation State
        self.graph = MazeGraph()
        # Register START node 0 at (0, 0) with initial North branch (0.0 rad)
        start_id, _, _, _ = self.graph.add_junction(0.0, 0.0, [0.0])
        self.graph.nodes[start_id]["branches"][0]["state"] = EXPLORED
        self.active_edge = (start_id, 0)   # (origin_junction_id, origin_branch_idx)
        self.edge_start_dist = 0.0
        self.edge_turns = []               # TURN node IDs recorded along current edge
        self.returning_from_rock = False   # True while line-following back after an obstacle

        # State Machine Registers
        self.state = self.FOLLOW
        self.approach_start_dist = 0.0
        self.arrival_heading = 0.0
        self.scan_start_heading = 0.0
        self.scan_accum = 0.0
        self.scan_bearings = []
        self.target_heading = 0.0
        self.pending_junction = None
        self.pending_branch = None
        self.entry_check_frames = 0

        # Checkpoints & LED
        self.target_idx = 0
        self.pad_votes = deque(maxlen=PAD_CONFIRM_FRAMES)
        self.active_pad = None
        self.pad_clear_count = 0
        self.blink_steps_left = 0
        self.blink_timer = 0
        self.blink_color = None
        self.finish_target_dist = 0.0
        self.steps = 0

    # --- Motion & Odometry ---
    def set_wheels(self, left_v, right_v):
        self.left_motor.setVelocity(clamp(left_v, -self.max_vel, self.max_vel))
        self.right_motor.setVelocity(clamp(right_v, -self.max_vel, self.max_vel))

    def update_odometry(self):
        l_enc = float(self.left_enc.getValue())
        r_enc = float(self.right_enc.getValue())
        if self.last_enc is None:
            self.last_enc = (l_enc, r_enc)
            return 0.0
        dl = (l_enc - self.last_enc[0]) * WHEEL_RADIUS
        dr = (r_enc - self.last_enc[1]) * WHEEL_RADIUS
        self.last_enc = (l_enc, r_enc)

        ds = 0.5 * (dl + dr)
        dth = (dr - dl) / AXLE_LENGTH
        mid_heading = self.heading + 0.5 * dth
        self.x += ds * math.cos(mid_heading)
        self.y += ds * math.sin(mid_heading)
        self.heading = angle_wrap(self.heading + dth)
        self.travelled += abs(ds)
        return dth

    # --- Sensors & Calibration ---
    def read_ir_valid(self):
        """Read all 7 IR sensors, rejecting NaN/inf/zero glitches."""
        vals = []
        for s in self.ir:
            v = float(s.getValue())
            if not math.isfinite(v) or v <= 1.0:
                vals.append(float("nan"))
            else:
                vals.append(v)
        return vals

    def calibrate_initial(self, samples):
        valid = sorted(v for row in samples for v in row if math.isfinite(v))
        if len(valid) >= 7:
            self.black_ref = valid[0]
            self.ground_ref = valid[len(valid) // 2]
            self.ir_threshold = self.black_ref + LINE_FRACTION * (self.ground_ref - self.black_ref)

    def refine_ground_calibration(self, vals, bits, on_pad):
        """Once outside the START box and not on a colored pad, adapt ground_ref to brown floor."""
        if self.ground_calibrated or on_pad:
            return
        if math.hypot(self.x, self.y) < START_BOX_RADIUS:
            return
        if sum(bits) in (1, 2):
            off_line = [v for v, b in zip(vals, bits) if not b and math.isfinite(v)]
            if len(off_line) >= 4:
                med_floor = sorted(off_line)[len(off_line) // 2]
                self.ground_ref = med_floor
                self.ir_threshold = self.black_ref + LINE_FRACTION * (self.ground_ref - self.black_ref)
                self.ground_calibrated = True

    def get_line_bits(self, vals):
        return [math.isfinite(v) and (v <= self.ir_threshold) for v in vals]

    def read_front_median(self):
        v = float(self.front_sensor.getValue())
        if not math.isfinite(v):
            v = 0.0
        self.front_buf.append(v)
        s = sorted(self.front_buf)
        return s[len(s) // 2]

    def detect_pad_color(self):
        """Return confirmed checkpoint color under either downward camera, or None."""
        for cam in (self.cam_left, self.cam_right):
            img = cam.getImage()
            if img is None:
                continue
            w, h = cam.getWidth(), cam.getHeight()
            r = g = b = 0.0
            cnt = 0
            for y in range(h // 2 - h // 8, h // 2 + h // 8):
                for x in range(w // 2 - w // 8, w // 2 + w // 8):
                    r += Camera.imageGetRed(img, w, x, y)
                    g += Camera.imageGetGreen(img, w, x, y)
                    b += Camera.imageGetBlue(img, w, x, y)
                    cnt += 1
            if cnt > 0:
                color = classify_rgb(r / cnt, g / cnt, b / cnt)
                if color is not None:
                    return color
        return None

    # --- Heuristic Scoring & Graph Routing ---
    def score_branch(self, node_x, node_y, arrival_heading, branch_bearing):
        """Score an UNTRIED branch:
        - Higher preference for Left and Right turns over Straight.
        - Increasing resistance against branches pointing back toward the start (0, 0).
        """
        rel = angle_wrap(branch_bearing - arrival_heading)
        if abs(rel - HALF_PI) <= math.radians(35):
            base_weight = WEIGHT_LEFT
        elif abs(rel + HALF_PI) <= math.radians(35):
            base_weight = WEIGHT_RIGHT
        elif abs(rel) <= math.radians(35):
            base_weight = WEIGHT_STRAIGHT
        else:
            base_weight = WEIGHT_REVERSE

        # Unit vector pointing from current junction back toward the start origin (0, 0)
        dist_origin = math.hypot(node_x, node_y)
        if dist_origin > 0.35:
            to_start_x = -node_x / dist_origin
            to_start_y = -node_y / dist_origin
        else:
            # Near origin, starting heading is North (+x), so start direction is South (-x)
            to_start_x, to_start_y = -1.0, 0.0

        br_x = math.cos(branch_bearing)
        br_y = math.sin(branch_bearing)
        alignment_to_start = max(0.0, br_x * to_start_x + br_y * to_start_y)

        # Ramp factor grows from 0.15 early on up to 1.0 as time and distance increase
        elapsed_s = (self.steps * self.step_ms) / 1000.0
        ramp = clamp(
            0.15 + 0.85 * max(self.travelled / RESISTANCE_RAMP_DIST, elapsed_s / RESISTANCE_RAMP_TIME),
            0.15,
            1.0,
        )
        start_resistance = ramp * START_RESISTANCE_MAX * alignment_to_start
        return base_weight - start_resistance

    def select_outgoing_branch(self, node_id, arrival_heading):
        """Choose next branch at `node_id` using:
        1) BFS to already-spotted active target checkpoint if mapped.
        2) Heuristic-weighted UNTRIED branch (Left/Right preferred, start-direction resisted).
        3) BFS shortest unblocked backtrack to the nearest junction with an UNTRIED branch.
        """
        node = self.graph.nodes[node_id]
        branches = node["branches"]

        # 1. If active target checkpoint was already seen elsewhere in the graph, route to it
        target_color = CHECKPOINTS[self.target_idx] if self.target_idx < len(CHECKPOINTS) else None
        if target_color and target_color in self.graph.checkpoints:
            cp = self.graph.checkpoints[target_color]
            if cp["node_id"] == node_id:
                br_idx = cp["branch_idx"]
                if branches[br_idx]["state"] != BLOCKED:
                    return br_idx, f"direct to mapped {target_color} checkpoint"
            else:
                route = self.graph.bfs_to_target(node_id, lambda nid: nid == cp["node_id"])
                if route is not None and route[0] is not None:
                    return route[0], f"BFS route to {target_color} at node {cp['node_id']}"

        # 2. Evaluate all UNTRIED outgoing branches at this junction
        untried = []
        for idx, br in enumerate(branches):
            if br["state"] == UNTRIED:
                sc = self.score_branch(node["x"], node["y"], arrival_heading, br["bearing"])
                untried.append((sc, idx))

        if untried:
            untried.sort(reverse=True)
            best_idx = untried[0][1]
            return best_idx, f"heuristic UNTRIED (score={untried[0][0]:.2f})"

        # 3. All local branches are EXPLORED or BLOCKED -> BFS backtrack to nearest UNTRIED node
        route = self.graph.bfs_to_target(
            node_id,
            lambda nid: any(b["state"] == UNTRIED for b in self.graph.nodes[nid]["branches"]),
        )
        if route is not None and route[0] is not None:
            return route[0], f"BFS backtrack toward node {route[1]}"

        return None, "no reachable untried branches remaining"

    # --- PID Line Following ---
    def follow_line(self, bits, on_pad):
        count = sum(bits)
        if count == 0 or count > MAX_LINE_SENSORS:
            self.lost_steps += 1
            if self.lost_steps <= LOST_GRACE_STEPS:
                self.set_wheels(CRUISE_SPEED * 0.45, CRUISE_SPEED * 0.45)
            elif self.lost_steps <= LOST_GRACE_STEPS + SEARCH_SWEEP_STEPS:
                self.set_wheels(SEARCH_SPEED * self.last_side, -SEARCH_SPEED * self.last_side)
            else:
                # Reached a dead end on the line -> treat as degree-1 junction and turn around
                self.handle_dead_end()
            return

        self.lost_steps = 0
        err = sum(w for w, b in zip(IR_WEIGHTS, bits) if b) / count
        if err != 0.0:
            self.last_side = -1.0 if err < 0 else 1.0

        d_err = err - self.prev_err
        self.prev_err = err
        self.integral = clamp(self.integral + err, -15.0, 15.0)
        turn = KP * err + KI * self.integral + KD * d_err

        curve_factor = min(1.0, abs(err) / 1.5 + 0.35 * abs(d_err))
        speed = CRUISE_SPEED - (CRUISE_SPEED - CURVE_MIN_SPEED) * curve_factor
        self.set_wheels(speed + turn, speed - turn)

    def handle_dead_end(self):
        """Treat line termination as a degree-1 JUNCTION node and turn 180° back."""
        self.set_wheels(0.0, 0.0)
        back_bearing = angle_wrap(self.heading + math.pi)
        nid, sx, sy, _ = self.graph.add_junction(self.x, self.y, [back_bearing])
        self.x, self.y = sx, sy
        in_idx = self.graph.match_branch(nid, back_bearing)

        if self.active_edge is not None and not self.returning_from_rock and in_idx is not None:
            src_id, src_br = self.active_edge
            corridor_len = self.travelled - self.edge_start_dist
            self.graph.link_corridor(src_id, src_br, nid, in_idx, self.edge_turns, corridor_len)

        self.returning_from_rock = False
        self.active_edge = (nid, in_idx) if in_idx is not None else None
        self.edge_start_dist = self.travelled
        self.edge_turns = []
        self.target_heading = back_bearing
        self.state = self.TURN
        print(f"Dead end mapped as JUNCTION {nid} at ({self.x:.2f}, {self.y:.2f}); turning around.")

    # --- Checkpoints & LED ---
    def update_led(self):
        if self.blink_steps_left > 0:
            self.blink_timer -= 1
            if self.blink_timer <= 0:
                lit = (self.blink_steps_left % 2 == 0)
                self.led.set(LED_COLORS[self.blink_color if lit else None])
                self.blink_steps_left -= 1
                self.blink_timer = BLINK_HALF_STEPS
            return
        target = CHECKPOINTS[self.target_idx] if self.target_idx < len(CHECKPOINTS) else None
        self.led.set(LED_COLORS[target])

    def process_checkpoint(self, color):
        if color == "GREEN" and math.hypot(self.x, self.y) < START_BOX_RADIUS:
            return  # Ignore launch START pad

        if color not in self.graph.checkpoints and self.active_edge is not None:
            self.graph.checkpoints[color] = {
                "node_id": self.active_edge[0],
                "branch_idx": self.active_edge[1],
                "x": self.x,
                "y": self.y,
            }
            print(f"Mapped checkpoint {color} at ({self.x:.2f}, {self.y:.2f}) via node {self.active_edge[0]}")

        expected = CHECKPOINTS[self.target_idx] if self.target_idx < len(CHECKPOINTS) else None
        if color == expected:
            self.target_idx += 1
            self.blink_color = color
            self.blink_steps_left = BLINK_CYCLES * 2
            self.blink_timer = 0
            print(f"CHECKPOINT {color} CONFIRMED ({self.target_idx}/{len(CHECKPOINTS)})")
            if color == "YELLOW":
                self.finish_target_dist = self.travelled + YELLOW_CENTER_DRIVE
                self.state = self.FINISH
                print(f"Centering full footprint inside YELLOW ({YELLOW_CENTER_DRIVE:.2f} m)...")

    # --- Main Control Loop ---
    def run(self):
        cal_samples = []
        self.update_led()

        while self.robot.step(self.step_ms) != -1:
            self.steps += 1
            dth = self.update_odometry()
            vals = self.read_ir_valid()

            # Initial IR calibration
            if len(cal_samples) < CALIBRATION_STEPS:
                cal_samples.append(vals)
                self.set_wheels(0.0, 0.0)
                if len(cal_samples) == CALIBRATION_STEPS:
                    self.calibrate_initial(cal_samples)
                continue

            # Camera checkpoint detection with exit hysteresis
            raw_pad = self.detect_pad_color()
            self.pad_votes.append(raw_pad)
            if (
                len(self.pad_votes) == PAD_CONFIRM_FRAMES
                and self.pad_votes[0] is not None
                and all(v == self.pad_votes[0] for v in self.pad_votes)
            ):
                confirmed = self.pad_votes[0]
                self.pad_clear_count = 0
                if confirmed != self.active_pad:
                    self.active_pad = confirmed
                    self.process_checkpoint(confirmed)
            elif raw_pad is None:
                self.pad_clear_count += 1
                if self.pad_clear_count >= PAD_EXIT_FRAMES:
                    self.active_pad = None

            self.update_led()
            on_pad = (raw_pad is not None) or (self.active_pad is not None)
            bits = self.get_line_bits(vals)
            self.refine_ground_calibration(vals, bits, on_pad)
            self.front_val = self.read_front_median()

            # --- State Machine ---
            if self.state == self.FOLLOW:
                # Stage 2 Mid-Path Obstacle Detection
                if self.front_val >= OBSTACLE_MID_PATH:
                    self.set_wheels(0.0, 0.0)
                    if self.active_edge is not None:
                        src_id, src_br = self.active_edge
                        self.graph.block_branch(src_id, src_br)
                        print(f"Obstacle ahead ({self.front_val:.0f}): blocked edge from node {src_id}; turning 180° to backtrack.")
                    self.returning_from_rock = True
                    self.target_heading = angle_wrap(self.heading + math.pi)
                    self.state = self.ABORT_TURN
                    continue

                # Check for outer IR trigger (possible TURN corner or JUNCTION)
                if not on_pad and (bits[0] or bits[6]):
                    self.outer_votes += 1
                    self.outer_left_seen = self.outer_left_seen or bits[0]
                    self.outer_right_seen = self.outer_right_seen or bits[6]
                else:
                    self.outer_votes = 0
                    self.outer_left_seen = False
                    self.outer_right_seen = False

                if self.outer_votes >= OUTER_CONFIRM_STEPS:
                    self.outer_votes = 0
                    self.approach_start_dist = self.travelled
                    self.arrival_heading = self.heading
                    self.state = self.APPROACH
                    continue

                self.follow_line(bits, on_pad)

            elif self.state == self.APPROACH:
                # Advance +0.13 m so the wheel axle sits directly over the intersection center
                if self.travelled - self.approach_start_dist >= SENSOR_FORWARD_OFFSET:
                    self.set_wheels(0.0, 0.0)
                    straight_continues = any(bits[2:5])
                    both_sides = self.outer_left_seen and self.outer_right_seen

                    # Fast Corner Optimization: 90° TURN node (no straight line & only 1 side branch)
                    if not straight_continues and not both_sides:
                        turn_id = self.graph.add_turn_node(self.x, self.y)
                        if not self.edge_turns or self.edge_turns[-1] != turn_id:
                            self.edge_turns.append(turn_id)
                        turn_dir = HALF_PI if self.outer_left_seen else -HALF_PI
                        self.target_heading = angle_wrap(self.arrival_heading + turn_dir)
                        self.outer_left_seen = False
                        self.outer_right_seen = False
                        self.state = self.TURN
                    else:
                        # True multi-way JUNCTION -> run 360° scan
                        self.outer_left_seen = False
                        self.outer_right_seen = False
                        self.scan_start_heading = self.heading
                        self.scan_accum = 0.0
                        self.scan_bearings = []
                        self.state = self.SCAN
                else:
                    self.set_wheels(APPROACH_SPEED, APPROACH_SPEED)

            elif self.state == self.SCAN:
                self.scan_accum += abs(dth)
                if not on_pad:
                    for i, active in enumerate(bits):
                        if active:
                            self.scan_bearings.append(
                                self.heading + SENSOR_BEARING[i] - self.scan_start_heading
                            )

                if self.scan_accum >= TWO_PI:
                    self.set_wheels(0.0, 0.0)
                    rel_roads = cluster_road_bearings(self.scan_bearings)
                    abs_roads = [angle_wrap(self.scan_start_heading + r) for r in rel_roads]

                    # Ensure entry road behind robot is represented
                    entry_bearing = angle_wrap(self.arrival_heading + math.pi)
                    if not any(angle_diff(b, entry_bearing) <= ANGLE_MATCH_TOL for b in abs_roads):
                        abs_roads.append(entry_bearing)

                    nid, sx, sy, is_new = self.graph.add_junction(self.x, self.y, abs_roads)
                    self.x, self.y = sx, sy
                    in_br = self.graph.match_branch(nid, entry_bearing)

                    # Link incoming corridor edge if we arrived from another junction
                    if self.active_edge is not None and not self.returning_from_rock and in_br is not None:
                        src_id, src_br = self.active_edge
                        if src_id != nid or src_br != in_br:
                            corridor_len = self.travelled - self.edge_start_dist
                            self.graph.link_corridor(
                                src_id, src_br, nid, in_br, self.edge_turns, corridor_len
                            )
                    elif in_br is not None and self.graph.nodes[nid]["branches"][in_br]["state"] == UNTRIED:
                        self.graph.nodes[nid]["branches"][in_br]["state"] = EXPLORED

                    self.returning_from_rock = False
                    print(f"JUNCTION {nid} ({'NEW' if is_new else 'KNOWN'}) at ({self.x:.2f}, {self.y:.2f}) "
                          f"with {len(self.graph.nodes[nid]['branches'])} branches")

                    self.pending_junction = nid
                    self.choose_and_turn_at_junction(nid)
                else:
                    self.set_wheels(-SCAN_SPEED, SCAN_SPEED)

            elif self.state == self.TURN:
                err = angle_wrap(self.target_heading - self.heading)
                if abs(err) <= TURN_TOLERANCE:
                    self.set_wheels(0.0, 0.0)
                    self.heading = self.target_heading
                    self.prev_err = 0.0
                    self.integral = 0.0
                    self.lost_steps = 0
                    if self.pending_junction is not None and self.pending_branch is not None:
                        # Stage 1 Pre-Entry Obstacle Check before committing to the edge
                        self.entry_check_frames = 3
                        self.state = self.CHECK_ENTRY
                    else:
                        self.state = self.FOLLOW
                else:
                    w = clamp(TURN_GAIN * abs(err), TURN_MIN_SPEED, TURN_MAX_SPEED)
                    w = w if err > 0 else -w
                    self.set_wheels(-w, w)

            elif self.state == self.CHECK_ENTRY:
                self.set_wheels(0.0, 0.0)
                self.entry_check_frames -= 1
                if self.entry_check_frames <= 0:
                    nid = self.pending_junction
                    br_idx = self.pending_branch
                    if self.front_val >= OBSTACLE_PRE_ENTRY:
                        print(f"Pre-entry check: obstacle on node {nid} branch {br_idx} "
                              f"(front={self.front_val:.0f}); marking BLOCKED.")
                        self.graph.block_branch(nid, br_idx)
                        self.choose_and_turn_at_junction(nid)
                    else:
                        # Path clear -> commit to edge
                        if self.graph.nodes[nid]["branches"][br_idx]["state"] == UNTRIED:
                            self.graph.nodes[nid]["branches"][br_idx]["state"] = EXPLORED
                        self.active_edge = (nid, br_idx)
                        self.edge_start_dist = self.travelled
                        self.edge_turns = []
                        self.pending_junction = None
                        self.pending_branch = None
                        self.state = self.FOLLOW

            elif self.state == self.ABORT_TURN:
                err = angle_wrap(self.target_heading - self.heading)
                if abs(err) <= TURN_TOLERANCE:
                    self.set_wheels(0.0, 0.0)
                    self.heading = self.target_heading
                    self.prev_err = 0.0
                    self.integral = 0.0
                    self.lost_steps = 0
                    self.state = self.FOLLOW
                else:
                    w = clamp(TURN_GAIN * abs(err), TURN_MIN_SPEED, TURN_MAX_SPEED)
                    w = w if err > 0 else -w
                    self.set_wheels(-w, w)

            elif self.state == self.FINISH:
                if self.front_val >= OBSTACLE_MID_PATH or self.travelled >= self.finish_target_dist:
                    self.set_wheels(0.0, 0.0)
                    self.led.set(LED_COLORS["YELLOW"])
                    print(f"MISSION COMPLETE at ({self.x:.2f}, {self.y:.2f}); wheels stopped.")
                    self.state = self.STOPPED
                else:
                    self.follow_line(bits, True)

            else:  # STOPPED
                self.set_wheels(0.0, 0.0)

    def choose_and_turn_at_junction(self, nid):
        choice, reason = self.select_outgoing_branch(nid, self.arrival_heading)
        if choice is None:
            print(f"Exploration exhausted at node {nid}: {reason}")
            self.pending_junction = None
            self.pending_branch = None
            self.state = self.STOPPED
            return

        self.pending_branch = choice
        br = self.graph.nodes[nid]["branches"][choice]
        print(f"Node {nid}: selected branch {choice} (bearing {math.degrees(br['bearing']):+.1f} deg) -> {reason}")
        self.target_heading = br["bearing"]
        self.state = self.TURN


def main():
    try:
        ElloMazeController(Robot()).run()
    except RuntimeError as exc:
        print(f"CONTROLLER ERROR: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())