"""Sensor-only line-following, junction, checkpoint, and rock controller."""

from controller import Camera, Robot
from enum import Enum, auto
import math
import sys
from typing import Dict, Iterable, List, Optional, Set, Tuple


TIME_STEP_MS = 16
WHEEL_RADIUS = 0.10
AXLE_LENGTH = 0.34
MAX_VEL = 8.0
BASE_SPEED = 2.65
CURVE_SPEED = 1.55
APPROACH_SPEED = 1.05
TURN_SPEED = 1.30
RECOVERY_SPEED = 0.70

IR_BLACK_IS_HIGH = False
IR_CALIBRATION_STEPS = 100
JUNCTION_CONFIRM_STEPS = 4
PAD_CONFIRM_STEPS = 5
TURN_TOLERANCE = 0.055
APPROACH_DISTANCE = 0.060
JUNCTION_MERGE_DISTANCE = 0.22
LINE_LOST_LIMIT = 18
ROCK_THRESHOLD = 280.0
ROCK_CONFIRM_STEPS = 4
ROCK_REVERSE_DISTANCE = 0.12
YELLOW_ADVANCE_DISTANCE = 0.34
RUN_LIMIT_SECONDS = 600.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def angle_wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class State(Enum):
    CALIBRATE = auto()
    FOLLOW = auto()
    APPROACH_JUNCTION = auto()
    DECIDE = auto()
    TURN = auto()
    ON_PAD = auto()
    ROCK_REVERSE = auto()
    ROCK_TURN = auto()
    ROCK_BACKTRACK = auto()
    ROCK_REALIGN = auto()
    FINAL_APPROACH = auto()
    RECOVER = auto()
    DONE = auto()
    SAFE_STOP = auto()


class PadColor(Enum):
    NONE = "NONE"
    GREEN = "GREEN"
    RED = "RED"
    BLUE = "BLUE"
    YELLOW = "YELLOW"


class Sensors:
    def __init__(self, robot: Robot) -> None:
        self.step = int(robot.getBasicTimeStep())
        self.ir = [robot.getDevice("ir_line_%d" % i) for i in range(7)]
        self.left_encoder = robot.getDevice("left_wheel_encoder")
        self.right_encoder = robot.getDevice("right_wheel_encoder")
        self.front = robot.getDevice("front_obstacle_sensor")
        self.left_camera = robot.getDevice("checkpoint_color_left")
        self.right_camera = robot.getDevice("checkpoint_color_right")
        self.led = robot.getDevice("rgb_status_led")
        self.left_motor = robot.getDevice("left_wheel_motor")
        self.right_motor = robot.getDevice("right_wheel_motor")

        devices = (self.ir + [self.left_encoder, self.right_encoder, self.front,
                              self.left_camera, self.right_camera, self.led,
                              self.left_motor, self.right_motor])
        if any(device is None for device in devices):
            raise RuntimeError("A required Webots device is missing")

        for sensor in self.ir:
            sensor.enable(self.step)
        self.left_encoder.enable(self.step)
        self.right_encoder.enable(self.step)
        self.front.enable(self.step)
        self.left_camera.enable(self.step)
        self.right_camera.enable(self.step)
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.stop()

        self.ir_low = [float("inf")] * 7
        self.ir_high = [float("-inf")] * 7
        self.ir_threshold = [192.0] * 7
        self.common_low = float("inf")
        self.common_high = float("-inf")

    def stop(self) -> None:
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    def set_motors(self, left: float, right: float) -> None:
        self.left_motor.setVelocity(clamp(left, -MAX_VEL, MAX_VEL))
        self.right_motor.setVelocity(clamp(right, -MAX_VEL, MAX_VEL))

    def calibrate(self) -> None:
        for i, sensor in enumerate(self.ir):
            value = float(sensor.getValue())
            if 0.0 <= value <= 1001.0:
                self.common_low = min(self.common_low, value)
                self.common_high = max(self.common_high, value)
                self.ir_low[i] = min(self.ir_low[i], value)
                self.ir_high[i] = max(self.ir_high[i], value)
                if self.ir_high[i] > self.ir_low[i]:
                    self.ir_threshold[i] = (
                        self.ir_low[i] + 0.50 * (self.ir_high[i] - self.ir_low[i])
                    )

    def line_pattern(self) -> Tuple[List[bool], float, bool]:
        bits: List[bool] = []
        weighted = 0.0
        weight_sum = 0.0
        positions = (-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0)
        for i, sensor in enumerate(self.ir):
            value = float(sensor.getValue())
            if not math.isfinite(value) or value < 0.0 or value > 1001.0:
                bits.append(False)
                continue
            black = (value >= self.ir_threshold[i]) if IR_BLACK_IS_HIGH else (
                value <= self.ir_threshold[i])
            bits.append(black)
            if black:
                weighted += positions[i]
                weight_sum += 1.0
        if weight_sum:
            position = weighted / weight_sum
            # Positive means the line is to the robot's left.
            position = -position
        else:
            position = 0.0
        return bits, position, weight_sum > 0.0

    def front_value(self) -> float:
        value = float(self.front.getValue())
        return value if math.isfinite(value) else 0.0

    def camera_rgb(self, camera) -> Tuple[float, float, float]:
        image = camera.getImage()
        if image is None:
            return (0.0, 0.0, 0.0)
        width = camera.getWidth()
        height = camera.getHeight()
        margin_x = max(1, width // 8)
        margin_y = max(1, height // 8)
        red = green = blue = 0.0
        count = 0
        for y in range(height // 2 - margin_y, height // 2 + margin_y):
            for x in range(width // 2 - margin_x, width // 2 + margin_x):
                if 0 <= x < width and 0 <= y < height:
                    red += Camera.imageGetRed(image, width, x, y)
                    green += Camera.imageGetGreen(image, width, x, y)
                    blue += Camera.imageGetBlue(image, width, x, y)
                    count += 1
        if count == 0:
            return (0.0, 0.0, 0.0)
        return (red / count, green / count, blue / count)

    def classify_camera(self, camera) -> PadColor:
        r, g, b = self.camera_rgb(camera)
        maximum = max(r, g, b)
        minimum = min(r, g, b)
        if maximum < 45:
            return PadColor.NONE
        rn = r / maximum
        gn = g / maximum
        bn = b / maximum
        saturation = (maximum - minimum) / max(maximum, 1.0)
        if saturation < 0.20:
            return PadColor.NONE
        if rn > 0.78 and gn > 0.55 and bn < 0.48:
            if maximum >= 125 and abs(rn - gn) < 0.42:
                return PadColor.YELLOW
        if rn > 0.72 and maximum >= 150 and rn > gn * 1.18 and rn > bn * 1.25:
            return PadColor.RED
        if gn > 0.68 and maximum >= 125 and gn > rn * 1.12 and gn > bn * 1.12:
            return PadColor.GREEN
        if bn > 0.62 and maximum >= 125 and bn > rn * 1.08 and bn > gn * 1.08:
            return PadColor.BLUE
        return PadColor.NONE

    def camera_color(self) -> PadColor:
        left = self.classify_camera(self.left_camera)
        right = self.classify_camera(self.right_camera)
        if left == right and left != PadColor.NONE:
            return left
        if left != PadColor.NONE:
            return left
        if right != PadColor.NONE:
            return right
        return PadColor.NONE

    def set_led(self, color: PadColor) -> None:
        values = {
            PadColor.GREEN: 0x00ff00,
            PadColor.RED: 0xff0000,
            PadColor.BLUE: 0x0000ff,
            PadColor.YELLOW: 0xffff00,
        }
        self.led.set(values.get(color, 0x000000))


class Odometry:
    def __init__(self, sensors: Sensors) -> None:
        left = sensors.left_encoder.getValue()
        right = sensors.right_encoder.getValue()
        self.left_encoder = left if math.isfinite(left) else 0.0
        self.right_encoder = right if math.isfinite(right) else 0.0
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0

    def reset_encoders(self, sensors: Sensors) -> None:
        left = sensors.left_encoder.getValue()
        right = sensors.right_encoder.getValue()
        self.left_encoder = left if math.isfinite(left) else 0.0
        self.right_encoder = right if math.isfinite(right) else 0.0

    def update(self, sensors: Sensors) -> float:
        left = sensors.left_encoder.getValue()
        right = sensors.right_encoder.getValue()
        if not math.isfinite(left) or not math.isfinite(right):
            return 0.0
        dl = (left - self.left_encoder) * WHEEL_RADIUS
        dr = (right - self.right_encoder) * WHEEL_RADIUS
        self.left_encoder = left
        self.right_encoder = right
        distance = (dl + dr) * 0.5
        delta_heading = (dr - dl) / AXLE_LENGTH
        self.heading = angle_wrap(self.heading + delta_heading)
        self.x += distance * math.cos(self.heading - delta_heading * 0.5)
        self.y += distance * math.sin(self.heading - delta_heading * 0.5)
        return distance

    def encoder_distance(self, start_left: float, start_right: float,
                         sensors: Sensors) -> float:
        left = sensors.left_encoder.getValue()
        right = sensors.right_encoder.getValue()
        if not math.isfinite(left) or not math.isfinite(right):
            return 0.0
        dl = (left - start_left) * WHEEL_RADIUS
        dr = (right - start_right) * WHEEL_RADIUS
        return (dl + dr) * 0.5


class MissionController:
    def __init__(self, robot: Robot) -> None:
        self.robot = robot
        self.step = int(robot.getBasicTimeStep())
        self.sensors = Sensors(robot)
        self.odom = Odometry(self.sensors)

        self.state = State.CALIBRATE
        self.start_time = robot.getTime()
        self.calibrate_steps = 0

        self.current_target = PadColor.GREEN
        self.confirmed = {PadColor.GREEN: False, PadColor.RED: False,
                          PadColor.BLUE: False, PadColor.YELLOW: False}
        self.color_candidate = PadColor.NONE
        self.pad_stable_steps = 0
        self.pad_left = False

        self.junction_confirm_steps = 0
        self.line_lost_steps = 0
        self.rock_confirm = 0

        self.target_heading = 0.0
        self.move_start_left = 0.0
        self.move_start_right = 0.0
        self.move_distance = 0.0
        self.turn_label = ""

        self.nodes: List[Tuple[float, float]] = [(0.0, 0.0)]
        self.current_node = 0
        self.visited: Dict[int, Set[int]] = {0: set()}
        self.blocked: Dict[int, Set[int]] = {0: set()}
        self.dist_since_node = 0.0
        self.rock_origin_node = 0
        self.rock_origin_distance = 0.0
        self.rock_origin_heading = 0.0

        self.recovery_phase = 0
        self.recovery_steps = 0
        self.loop_watch: List[Tuple[int, int, str]] = []

        self.sensors.set_led(PadColor.GREEN)

    def log(self, message: str) -> None:
        print("[MISSION] " + message, flush=True)

    def set_state(self, state: State) -> None:
        if state != self.state:
            self.log("state %s -> %s" % (self.state.name, state.name))
            self.state = state

    def stop(self) -> None:
        self.sensors.stop()

    def begin_distance(self, distance: float) -> None:
        self.move_start_left = self.sensors.left_encoder.getValue()
        self.move_start_right = self.sensors.right_encoder.getValue()
        self.move_distance = distance

    def traveled(self) -> float:
        return abs(self.odom.encoder_distance(
            self.move_start_left, self.move_start_right, self.sensors))

    def begin_turn(self, delta: float, label: str) -> None:
        self.target_heading = angle_wrap(self.odom.heading + delta)
        self.turn_label = label
        self.set_state(State.TURN)

    def motor_forward(self, speed: float) -> None:
        _, position, seen = self.sensors.line_pattern()
        if not seen:
            self.sensors.set_motors(speed * 0.55, speed * 0.55)
            return
        if abs(position) > 1.5:
            speed = min(speed, CURVE_SPEED)
        correction = 0.78 * position
        left = clamp(speed - correction, -MAX_VEL, speed)
        right = clamp(speed + correction, -MAX_VEL, speed)
        self.sensors.set_motors(left, right)

    def classify_junction(self) -> Tuple[str, bool, bool, bool]:
        bits, _, seen = self.sensors.line_pattern()
        left = bits[0] or bits[1]
        right = bits[5] or bits[6]
        forward = bits[2] or bits[3] or bits[4]
        count = sum(bits)
        if not seen or count == 0:
            kind = "DEAD_END"
        elif left and right and forward:
            kind = "CROSSROAD"
        elif left and right:
            kind = "T_JUNCTION"
        elif left and forward:
            kind = "LEFT_AND_STRAIGHT"
        elif right and forward:
            kind = "RIGHT_AND_STRAIGHT"
        elif left:
            kind = "LEFT_ONLY"
        elif right:
            kind = "RIGHT_ONLY"
        else:
            kind = "STRAIGHT"
        return kind, left, forward, right

    def merge_node(self) -> int:
        for node_id, (x, y) in enumerate(self.nodes):
            if math.hypot(self.odom.x - x, self.odom.y - y) <= JUNCTION_MERGE_DISTANCE:
                self.nodes[node_id] = (
                    0.75 * x + 0.25 * self.odom.x,
                    0.75 * y + 0.25 * self.odom.y,
                )
                self.visited.setdefault(node_id, set())
                self.blocked.setdefault(node_id, set())
                return node_id
        node_id = len(self.nodes)
        self.nodes.append((self.odom.x, self.odom.y))
        self.visited[node_id] = set()
        self.blocked[node_id] = set()
        return node_id

    def quantized_heading(self, heading: Optional[float] = None) -> int:
        heading = self.odom.heading if heading is None else heading
        return int(round(angle_wrap(heading) / (math.pi / 2.0)))

    def pad_confirmed(self) -> Optional[PadColor]:
        color = self.sensors.camera_color()
        if self.robot.getTime() - self.start_time < 2.0:
            self.pad_stable_steps = 0
            self.color_candidate = PadColor.NONE
            return None
        if color != PadColor.NONE:
            if color == self.color_candidate:
                self.pad_stable_steps += 1
            else:
                self.color_candidate = color
                self.pad_stable_steps = 1
        else:
            self.pad_stable_steps = 0
            self.color_candidate = PadColor.NONE
        if self.pad_stable_steps >= PAD_CONFIRM_STEPS:
            self.log("camera stable: " + self.color_candidate.name)
            return self.color_candidate
        return None

    def on_pad_confirmed(self, color: PadColor) -> None:
        if color != PadColor.YELLOW and color != self.current_target:
            self.log("out-of-order/done pad: %s" % color.name)
            return
        if self.confirmed[color]:
            self.log("duplicate pad: %s" % color.name)
            return

        self.confirmed[color] = True
        self.log("confirmed pad: %s" % color.name)
        if color == PadColor.GREEN:
            self.current_target = PadColor.RED
            self.sensors.set_led(PadColor.RED)
        elif color == PadColor.RED:
            self.current_target = PadColor.BLUE
            self.sensors.set_led(PadColor.BLUE)
        elif color == PadColor.BLUE:
            self.current_target = PadColor.YELLOW
            self.sensors.set_led(PadColor.YELLOW)
        elif color == PadColor.YELLOW:
            self.begin_distance(YELLOW_ADVANCE_DISTANCE)
            self.set_state(State.FINAL_APPROACH)

    def choose_exit(self, left: bool, forward: bool, right: bool,
                    kind: str) -> Optional[float]:
        exits = {
            "FORWARD": forward,
            "LEFT": left,
            "RIGHT": right,
        }
        visited = self.visited.setdefault(self.current_node, set())
        blocked = self.blocked.setdefault(self.current_node, set())

        for name in ("FORWARD", "LEFT", "RIGHT"):
            if not exits[name]:
                continue
            delta = {"FORWARD": 0.0, "LEFT": math.pi / 2.0,
                     "RIGHT": -math.pi / 2.0}[name]
            absolute = self.quantized_heading(self.odom.heading + delta)
            if absolute not in visited and absolute not in blocked:
                visited.add(absolute)
                return delta

        for name in ("FORWARD", "LEFT", "RIGHT"):
            if not exits[name]:
                continue
            delta = {"FORWARD": 0.0, "LEFT": math.pi / 2.0,
                     "RIGHT": -math.pi / 2.0}[name]
            absolute = self.quantized_heading(self.odom.heading + delta)
            if absolute not in blocked:
                visited.add(absolute)
                return delta
        return None

    def detect_rock(self) -> bool:
        if self.sensors.front_value() >= ROCK_THRESHOLD:
            self.rock_confirm += 1
        else:
            self.rock_confirm = 0
        return self.rock_confirm >= ROCK_CONFIRM_STEPS

    def begin_rock_recovery(self) -> None:
        self.stop()
        self.rock_origin_node = self.current_node
        self.rock_origin_distance = max(0.05, self.dist_since_node)
        self.rock_origin_heading = self.odom.heading
        self.begin_distance(ROCK_REVERSE_DISTANCE)
        self.set_state(State.ROCK_REVERSE)

    def reverse_encoder(self) -> bool:
        traveled = self.traveled()
        if traveled >= self.move_distance:
            self.stop()
            return True
        speed = APPROACH_SPEED
        self.sensors.set_motors(-speed, -speed)
        return False

    def run(self) -> int:
        while self.robot.step(self.step) != -1:
            if self.robot.getTime() - self.start_time >= RUN_LIMIT_SECONDS:
                self.log("run limit reached")
                self.set_state(State.SAFE_STOP)

            distance = self.odom.update(self.sensors)

            if self.state not in (State.CALIBRATE, State.ROCK_REVERSE,
                                  State.ROCK_TURN, State.ROCK_BACKTRACK,
                                  State.ROCK_REALIGN, State.FINAL_APPROACH, State.DONE,
                                  State.SAFE_STOP):
                if self.detect_rock():
                    self.begin_rock_recovery()

            if self.state == State.CALIBRATE:
                self.sensors.calibrate()
                self.sensors.set_motors(0.45, 0.45)
                self.calibrate_steps += 1
                if self.calibrate_steps >= IR_CALIBRATION_STEPS:
                    self.odom.reset_encoders(self.sensors)
                    if math.isfinite(self.sensors.common_low) and math.isfinite(
                            self.sensors.common_high):
                        common = (
                            self.sensors.common_low
                            + 0.50 * (
                                self.sensors.common_high - self.sensors.common_low
                            )
                        )
                        self.sensors.ir_threshold = [common] * 7
                    self.log("IR threshold: %.1f" % self.sensors.ir_threshold[0])
                    self.set_state(State.FOLLOW)

            elif self.state == State.FOLLOW:
                self.dist_since_node += abs(distance)
                bits, _, seen = self.sensors.line_pattern()
                if seen:
                    self.line_lost_steps = 0
                else:
                    self.line_lost_steps += 1

                color = self.sensors.camera_color()
                if color != PadColor.NONE:
                    if not self.pad_left:
                        confirmed = self.pad_confirmed()
                        if confirmed is not None:
                            self.stop()
                            self.pad_left = True
                            self.on_pad_confirmed(confirmed)
                            if self.state != State.FINAL_APPROACH:
                                self.begin_distance(0.08)
                                self.set_state(State.ON_PAD)
                            continue
                else:
                    if self.pad_left:
                        self.pad_left = False
                        self.pad_stable_steps = 0
                        self.color_candidate = PadColor.NONE

                count = sum(bits)
                left = bits[0] or bits[1]
                right = bits[5] or bits[6]
                forward = bits[2] or bits[3] or bits[4]
                potential = ((left and forward) or (right and forward) or
                             (left and right) or count >= 6)
                if potential:
                    self.junction_confirm_steps += 1
                else:
                    self.junction_confirm_steps = 0
                if self.junction_confirm_steps >= JUNCTION_CONFIRM_STEPS:
                    self.stop()
                    self.begin_distance(APPROACH_DISTANCE)
                    self.set_state(State.APPROACH_JUNCTION)
                    self.junction_confirm_steps = 0
                    continue

                if self.line_lost_steps >= LINE_LOST_LIMIT:
                    self.log("line lost; starting bounded recovery")
                    self.stop()
                    self.recovery_phase = 0
                    self.recovery_steps = 0
                    self.set_state(State.RECOVER)
                else:
                    self.motor_forward(BASE_SPEED)

            elif self.state == State.APPROACH_JUNCTION:
                if self.traveled() < APPROACH_DISTANCE:
                    self.motor_forward(APPROACH_SPEED)
                else:
                    self.stop()
                    kind, left, forward, right = self.classify_junction()
                    self.log("junction: " + kind)
                    self.current_node = self.merge_node()
                    self.dist_since_node = 0.0
                    if kind == "DEAD_END":
                        blocked = self.blocked.setdefault(self.current_node, set())
                        blocked.add(self.quantized_heading())
                        self.begin_turn(math.pi, "dead-end U-turn")
                    else:
                        self.set_state(State.DECIDE)

            elif self.state == State.DECIDE:
                kind, left, forward, right = self.classify_junction()
                if kind == "DEAD_END":
                    blocked = self.blocked.setdefault(self.current_node, set())
                    blocked.add(self.quantized_heading())
                    self.begin_turn(math.pi, "dead-end U-turn")
                    continue
                delta = self.choose_exit(left, forward, right, kind)
                if delta is None:
                    self.log("all exits exhausted; U-turn")
                    self.begin_turn(math.pi, "backtrack")
                elif abs(delta) < 0.2:
                    self.dist_since_node = 0.0
                    self.set_state(State.FOLLOW)
                else:
                    name = "left" if delta > 0.0 else "right"
                    self.begin_turn(delta, name)

            elif self.state == State.TURN:
                error = angle_wrap(self.target_heading - self.odom.heading)
                if abs(error) <= TURN_TOLERANCE:
                    self.stop()
                    self.dist_since_node = 0.0
                    self.log("turn complete: " + self.turn_label)
                    self.set_state(State.FOLLOW)
                else:
                    correction = clamp(1.8 * error, -TURN_SPEED, TURN_SPEED)
                    self.sensors.set_motors(-correction, correction)

            elif self.state == State.ON_PAD:
                if self.traveled() >= self.move_distance:
                    self.stop()
                    self.set_state(State.FOLLOW)
                else:
                    self.motor_forward(APPROACH_SPEED)

            elif self.state == State.ROCK_REVERSE:
                if self.reverse_encoder():
                    self.begin_turn(math.pi, "rock U-turn")
                    self.set_state(State.ROCK_TURN)

            elif self.state == State.ROCK_TURN:
                error = angle_wrap(self.target_heading - self.odom.heading)
                if abs(error) <= TURN_TOLERANCE:
                    self.stop()
                    self.begin_distance(max(0.10, self.rock_origin_distance))
                    self.current_node = self.rock_origin_node
                    blocked = self.blocked.setdefault(self.current_node, set())
                    blocked.add(self.quantized_heading(self.rock_origin_heading))
                    self.set_state(State.ROCK_BACKTRACK)
                else:
                    correction = clamp(1.8 * error, -TURN_SPEED, TURN_SPEED)
                    self.sensors.set_motors(-correction, correction)

            elif self.state == State.ROCK_BACKTRACK:
                if self.traveled() >= self.move_distance:
                    self.stop()
                    self.current_node = self.rock_origin_node
                    self.dist_since_node = 0.0
                    self.begin_turn(math.pi, "rock realign")
                    self.set_state(State.ROCK_REALIGN)
                else:
                    _, _, seen = self.sensors.line_pattern()
                    if seen:
                        self.motor_forward(APPROACH_SPEED)
                    else:
                        self.line_lost_steps += 1
                        if self.line_lost_steps >= LINE_LOST_LIMIT:
                            self.set_state(State.SAFE_STOP)
                        else:
                            self.sensors.set_motors(
                                RECOVERY_SPEED * 0.55, RECOVERY_SPEED * 0.55)

            elif self.state == State.ROCK_REALIGN:
                error = angle_wrap(self.target_heading - self.odom.heading)
                if abs(error) <= TURN_TOLERANCE:
                    self.stop()
                    self.dist_since_node = 0.0
                    self.set_state(State.DECIDE)
                else:
                    correction = clamp(1.8 * error, -TURN_SPEED, TURN_SPEED)
                    self.sensors.set_motors(-correction, correction)

            elif self.state == State.FINAL_APPROACH:
                if self.traveled() >= YELLOW_ADVANCE_DISTANCE:
                    self.stop()
                    print("MISSION COMPLETE", flush=True)
                    self.sensors.set_led(PadColor.YELLOW)
                    self.set_state(State.DONE)
                else:
                    self.motor_forward(APPROACH_SPEED)

            elif self.state == State.RECOVER:
                # A short right sweep, then a longer left sweep, with hard limits.
                if self.recovery_phase == 0:
                    self.sensors.set_motors(RECOVERY_SPEED, -RECOVERY_SPEED)
                    self.recovery_steps += 1
                    if self.recovery_steps >= 24:
                        self.recovery_phase = 1
                        self.recovery_steps = 0
                elif self.recovery_phase == 1:
                    self.sensors.set_motors(-RECOVERY_SPEED, RECOVERY_SPEED)
                    self.recovery_steps += 1
                    if self.recovery_steps >= 48:
                        self.recovery_phase = 2
                        self.recovery_steps = 0
                else:
                    self.stop()
                    self.set_state(State.SAFE_STOP)
                _, _, seen = self.sensors.line_pattern()
                if seen:
                    self.line_lost_steps = 0
                    self.stop()
                    self.set_state(State.FOLLOW)

            elif self.state in (State.DONE, State.SAFE_STOP):
                self.stop()

            # Watchdog against an actual physical loop rather than wall-clock only.
            key = (int(round(self.odom.x * 2.0)),
                   int(round(self.odom.y * 2.0)),
                   self.state.name)
            self.loop_watch.append(key)
            if len(self.loop_watch) > 400:
                self.loop_watch.pop(0)
            if self.loop_watch.count(key) >= 1000:
                self.log("bounded no-progress watchdog")
                self.set_state(State.SAFE_STOP)

        self.stop()
        return 0


def main() -> int:
    try:
        robot = Robot()
        return MissionController(robot).run()
    except RuntimeError as error:
        print("STARTER ERROR: " + str(error), file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
