import math
import time
from collections import deque
from marlin import Robot, Thruster, Servo, IMU, RadioBeacon

# Brain port definitions
# Each port can only be used ONCE
IMU_PORT = "S6"          # IMU cable is in S6
RADIOBEACON_PORT = "S1"  # radio receiver cable is in S1
LEFT = "M1"        # left drive motor
RIGHT = "M7"       # right drive motor
LEFT_HAND = "M2"   # left hand servo
RIGHT_HAND = "M5"  # right hand servo

# Left hand angles (left top button)
# The left servo is mounted mirrored, so it counts DOWN to move outward
# (the right hand counts UP). Angles can't go below 0.
LEFT_HAND_OPEN_ANGLE = 0    # default, when the robot starts (near the body)
LEFT_HAND_CLOSED_ANGLE = 120    # after pressing the button (moves outward)
# Right hand angles (right top button)
RIGHT_HAND_OPEN_ANGLE = 0      # default, when the robot starts
RIGHT_HAND_CLOSED_ANGLE = 120  # after pressing the button
# How fast the hand servos move, in degrees per second (bigger = faster)
HAND_SPEED = 180

# Turning speed for each direction (1.0 = normal). Lower the faster side
# so both directions turn at the same speed.
TURN_LEFT_SPEED = 1.0
TURN_RIGHT_SPEED = 2.0   # right turns twice as strong (was 1.5)

# Backward speed boost (1.0 = normal, 1.5 = 50% stronger). Thrusters push
# weaker in reverse, so this makes backward closer to forward.
# Full stick backward is already the maximum (-100) and can't go higher.
BACKWARD_SPEED = 3.0   # was 2.0: full backward power at one-third stick


# =====================================================================
# AUTONOMOUS SETTINGS - drive STRAIGHT to the Centerline, then STRAIGHT back
# =====================================================================
# No turning, no searching: the robot drives straight forward from wherever
# it is placed. The Beacon is always ON the Centerline, so if it is in front
# of the robot, the basket catches it when the robot reaches the line. Then
# the robot drives straight back to the Starting Area.
#   => PLACE THE ROBOT IN LINE WITH THE BEACON (and not in front of the
#      Anchorage in the middle of the pool - its posts are in the way).
# The gyro (IMU) keeps it driving straight; the Beacon signal is only logged
# (and used to notice if the Beacon falls out on the way back).
#
# The robot has no GPS, so it ESTIMATES where it is ("dead reckoning"):
#   heading = from the gyro (IMU), in degrees. 0 = facing the Centerline,
#             positive = turned LEFT.
#   x       = metres forward from the start, toward the Centerline.
#   y       = metres to the LEFT of the middle of the pool's 2 m width.
# Distance comes from "speed x time", so MAX_SPEED must be measured.
# Every value marked CALIBRATE has to be measured in a real pool.

AUTON_TIME = 20.0     # the Autonomous Period is 20 seconds

# Where you place the robot across the Starting Area, in metres LEFT of the
# pool's middle (negative = right). Only used for the log - the robot drives
# straight from wherever you put it.
START_Y = 0.5
# Distance from the robot's start position to the Centerline (pool is about
# 2.93 m long, so the Centerline is ~1.46 m from the wall; robot centre
# starts ~0.2 m from the wall).
CENTERLINE_X = 1.25

MAX_SPEED = 0.40      # CALIBRATE: metres/second at duty 100 (see notes)
GYRO_SIGN = -1.0      # FLIPPED: log showed the heading going the wrong way (was 1.0)
GYRO_AXIS = "z"       # CALIBRATE: "z" if the IMU lies flat; "x" or "y" if it is mounted on its side
HEADING_KP = 1.5      # steering effort per degree of heading error (keeps it straight)
MAX_TURN_DUTY = 80    # strongest steering/turning effort in autonomous
TURN_IN_PLACE_ANGLE = 45  # if off by more than this, stop and spin first

BASKET_OFFSET = 0.2   # CALIBRATE: metres from the robot's centre to the front of the basket
APPROACH_X = CENTERLINE_X - BASKET_OFFSET + 0.05  # robot centre stops here (basket 5 cm past the line)
FORWARD_DUTY = 60     # speed going out to the Centerline

# Search on the way out: compare the Beacon signal every SAMPLE_TIME seconds
#   stronger           -> keep going forward
#   weaker             -> turn RIGHT by TURN_STEP
#     still weaker     -> turn LEFT back to the old direction, go straight
#     stronger         -> keep the new direction
#   in range           -> stop and go home
CAPTURE_RSSI = -97.0  # CALIBRATE: "in range" (your range: about -95 near, -110 far)
SAMPLE_TIME = 1.0     # CALIBRATE: seconds of straight driving between comparisons
RSSI_DROP = 2.0       # CALIBRATE: dB weaker that counts as "weaker" (ignores noise)
TURN_STEP = 30        # degrees to turn right when the signal gets weaker
MAX_SEARCH_TURN = 90  # never search more than this far from straight ahead
ALIGNED_ANGLE = 10    # only compare the signal once the robot faces its new direction
CENTERLINE_LIMIT = CENTERLINE_X + 0.1  # never go further than this (robot centre)
RSSI_AVERAGE = 4      # readings averaged for the live rssi value
SETTLE_TIME = 0.5     # stop briefly so the Beacon stops bobbing before we turn

# Going home - while carrying the Beacon: NEVER reverse (it would fall out of
# the basket) and turn gently so it isn't flung out.
CARRY_TURN_DUTY = 70  # CALIBRATE: turning effort with the Beacon (was 30: far too slow)
RETURN_ARC_DUTY = 30  # CALIBRATE: keep moving forward this much while turning round
                      # (an arc turns faster than spinning, and keeps the Beacon pushed in)
RETURN_DUTY = 80      # speed on the way home (pushing the Beacon)
HOME_X = 0.1          # stop here: well inside the Starting Area
WAYPOINT_SPACING = 0.25   # Nav records a route point every this many metres
# Lost Beacon check: if the signal gets this much weaker than when we picked it
# up, for LOST_CONFIRM_TIME, the Beacon fell out: go back out and try again.
LOST_DROP = 8.0       # CALIBRATE: dB (your range is about -95 near to -110 far)
LOST_CONFIRM_TIME = 0.5

# Hands are not used in autonomous. Hold them tucked in so they don't block
# the basket (if not told anything, the servos sit at 90 degrees).
# CALIBRATE: pick angles that keep the hands clear of the basket opening.
AUTO_HANDS_POSITION = (LEFT_HAND_OPEN_ANGLE, RIGHT_HAND_OPEN_ANGLE)

# Gyro "drift" measured while the robot sits still (set in initialize)
gyro_bias = 0.0


robot = Robot()
robot.configure(
    sensors={IMU_PORT: IMU, RADIOBEACON_PORT: RadioBeacon},
    effectors={
        LEFT: Thruster,
        RIGHT: Thruster,
        LEFT_HAND: Servo,
        RIGHT_HAND: Servo,
    },
)


# Buttons for the hands (press once to close, press again to open).
# Valid names: UP, DOWN, LEFT, RIGHT, X, Y, A, B, LEFT_TRIGGER, RIGHT_TRIGGER
LEFT_TOP_NAMES = ["LEFT_TRIGGER"]    # left hand button: left top (shoulder) button
RIGHT_TOP_NAMES = ["RIGHT_TRIGGER"]  # right hand button: right top (shoulder) button


def pressed(robot, names):
    for name in names:
        try:
            if robot.controller.button(name).is_down:
                print("pressed:", name)
                return True
        except Exception:
            pass
    return False


def initialize(robot):
    time.sleep(0.5)
    print("Battery:", robot.battery_voltage, "V")

    # Set hand servo speed
    robot.servo(LEFT_HAND).set_speed(HAND_SPEED)
    robot.servo(RIGHT_HAND).set_speed(HAND_SPEED)

    # The robot is sitting still before the match, so measure gyro drift now
    calibrate_gyro(robot)


# ---------------------------------------------------------------------
# Autonomous helpers
# ---------------------------------------------------------------------

def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_angle(deg):
    """Keep an angle between -180 and 180 degrees."""
    return (deg + 180.0) % 360.0 - 180.0


def yaw_rate(reading):
    """Turning speed (deg/s) from the gyro axis set in GYRO_AXIS."""
    return getattr(reading, "gyro_" + GYRO_AXIS)


def calibrate_gyro(robot, seconds=1.0):
    """Average the turning gyro while still. That average is drift we subtract later."""
    global gyro_bias
    samples = []
    last_ts = None
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        reading = robot.imu(IMU_PORT).read()
        if reading.ok and reading.timestamp != last_ts:
            samples.append(yaw_rate(reading))
            last_ts = reading.timestamp
        time.sleep(0.02)
    if samples:
        gyro_bias = sum(samples) / len(samples)
        print("Gyro bias:", round(gyro_bias, 3), "deg/s from", len(samples), "samples")
    else:
        print("WARNING: no IMU readings - gyro not calibrated")


def set_hands(robot, angles):
    robot.servo(LEFT_HAND).angle(angles[0])
    robot.servo(RIGHT_HAND).angle(angles[1])


class Nav:
    """Tracks where the robot thinks it is, records the route, and steers."""

    def __init__(self, robot):
        self.robot = robot
        self.x = 0.0
        self.y = START_Y
        self.heading = 0.0
        self.forward = 0.0           # forward duty we are commanding now
        self.last_time = time.monotonic()
        self.last_imu_ts = None
        self.rssi_values = deque(maxlen=RSSI_AVERAGE)
        self.last_rx_count = None
        self.waypoints = [(self.x, self.y)]  # the recorded route, start first
        self.new_rssi = None

    def update(self):
        """Call once per loop: update heading, position, route and beacon signal."""
        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now

        # Heading: add up how fast we are turning x how long (only new readings)
        imu = self.robot.imu(IMU_PORT).read()
        if imu.ok and imu.timestamp != self.last_imu_ts:
            if self.last_imu_ts is not None:
                imu_dt = min((imu.timestamp - self.last_imu_ts) / 1000.0, 0.1)
                self.heading += GYRO_SIGN * (yaw_rate(imu) - gyro_bias) * imu_dt
            self.last_imu_ts = imu.timestamp

        # Position: speed x time in the direction we are facing
        speed = self.forward / 100.0 * MAX_SPEED
        rad = math.radians(self.heading)
        self.x += speed * math.cos(rad) * dt
        self.y += speed * math.sin(rad) * dt

        # Record a route point every WAYPOINT_SPACING metres
        last_x, last_y = self.waypoints[-1]
        if math.hypot(self.x - last_x, self.y - last_y) >= WAYPOINT_SPACING:
            self.waypoints.append((self.x, self.y))

        # Beacon signal: only use NEW messages (rx_count goes up for each one)
        self.new_rssi = None  # the new raw reading this loop, if one arrived
        beacon = self.robot.radiobeacon(RADIOBEACON_PORT).read()
        if beacon.ok and beacon.rx_count != self.last_rx_count:
            self.rssi_values.append(beacon.rssi)
            self.last_rx_count = beacon.rx_count
            self.new_rssi = beacon.rssi

    def rssi(self):
        """Average of the latest beacon readings, or None if nothing heard yet."""
        if not self.rssi_values:
            return None
        return sum(self.rssi_values) / len(self.rssi_values)

    def drive(self, forward, target_heading, max_turn=MAX_TURN_DUTY, arc_forward=0):
        """Drive forward while steering toward target_heading (degrees).

        max_turn limits how hard it turns (use CARRY_TURN_DUTY with the Beacon).
        arc_forward: forward speed to keep while making a big turn (0 = spin
        on the spot; more than 0 = turn in an arc).
        """
        error = wrap_angle(target_heading - self.heading)
        if abs(error) > TURN_IN_PLACE_ANGLE:
            forward = arc_forward  # too far off: turn first (spin, or arc)
        # error > 0 means we must turn LEFT, and in this robot positive turn = right
        turn = clamp(-HEADING_KP * error, -max_turn, max_turn)
        if turn > 0:
            turn *= TURN_RIGHT_SPEED
        else:
            turn *= TURN_LEFT_SPEED
        self.forward = forward
        self.robot.thruster(LEFT).set_duty(forward + turn)
        self.robot.thruster(RIGHT).set_duty(forward - turn)

    def stop(self):
        self.forward = 0
        self.robot.thruster(LEFT).set_duty(0)
        self.robot.thruster(RIGHT).set_duty(0)

    def route_length_home(self):
        """Length of the recorded route from here back to the start."""
        return self.path_length(list(reversed(self.waypoints)))

    def path_length(self, points):
        """Distance from here through each point in the list, in order."""
        total = 0.0
        px, py = self.x, self.y
        for wx, wy in points:
            total += math.hypot(px - wx, py - wy)
            px, py = wx, wy
        return total


# ---------------------------------------------------------------------
# Autonomous: FORWARD (search on the way) -> SETTLE -> RETURN -> DONE
#   No time limits. If the Beacon falls out on the way home: RETURN -> FORWARD.
# ---------------------------------------------------------------------

def autonomous(robot):
    start = time.monotonic()
    nav = Nav(robot)
    set_hands(robot, AUTO_HANDS_POSITION)  # keep hands out of the basket's way

    has_beacon = False
    pickup_rssi = None    # signal when the Beacon came in range
    lost_since = None

    # search state (on the way out)
    base_heading = 0.0    # the "straight" direction we are keeping
    search_heading = 0.0  # where we are steering right now
    mode = "straight"     # "straight" or "right" (trying a right turn)
    ref_rssi = None       # strongest signal on the current straight line
    turn_rssi = None      # signal when we decided to try the right turn
    last_sample_time = start

    state = "FORWARD"
    state_start = start
    last_log = start
    print(f"[auto] start, x=0 y={START_Y}: forward; weaker -> right {TURN_STEP} deg;"
          f" still weaker -> back left; in range (>= {CAPTURE_RSSI}) -> go home")

    while robot.running:
        now = time.monotonic()
        elapsed = now - start
        nav.update()
        rssi = nav.rssi()
        new_state = state

        if state == "FORWARD":
            if rssi is not None and rssi >= CAPTURE_RSSI:
                # In range: the Beacon is at the basket - go home
                nav.stop()
                pickup_rssi = rssi
                print(f"[auto] Beacon IN RANGE (rssi={round(rssi, 1)}) - going home")
                new_state = "SETTLE"
            elif nav.x >= CENTERLINE_LIMIT:
                # Reached the Centerline without getting in range
                nav.stop()
                shown = None if rssi is None else round(rssi, 1)
                print(f"[auto] reached the Centerline but the Beacon is not in range"
                      f" (rssi={shown}) - going home")
                new_state = "RETURN"
            else:
                aligned = abs(wrap_angle(search_heading - nav.heading)) < ALIGNED_ANGLE
                if not aligned:
                    last_sample_time = now  # still turning: don't judge the signal yet
                elif rssi is not None and now - last_sample_time >= SAMPLE_TIME:
                    last_sample_time = now
                    if mode == "straight":
                        if ref_rssi is None or rssi > ref_rssi:
                            ref_rssi = rssi              # stronger: keep forward
                        elif rssi < ref_rssi - RSSI_DROP:
                            # weaker: try turning right
                            turn_rssi = rssi
                            search_heading = max(base_heading - TURN_STEP, -MAX_SEARCH_TURN)
                            mode = "right"
                            print(f"[auto] weaker ({round(ref_rssi, 1)} -> {round(rssi, 1)})"
                                  f" - TURN RIGHT to {round(search_heading)} deg")
                    else:  # mode == "right"
                        if rssi < turn_rssi:
                            # still weaker: turn left back to the old direction
                            print(f"[auto] still weaker ({round(turn_rssi, 1)} ->"
                                  f" {round(rssi, 1)}) - TURN LEFT back to"
                                  f" {round(base_heading)} deg, going straight")
                            search_heading = base_heading
                        else:
                            # stronger after the right turn: keep the new direction
                            print(f"[auto] stronger after turning right ({round(turn_rssi, 1)}"
                                  f" -> {round(rssi, 1)}) - keeping {round(search_heading)} deg")
                            base_heading = search_heading
                        mode = "straight"
                        ref_rssi = rssi
                nav.drive(FORWARD_DUTY, search_heading)

        elif state == "SETTLE":
            nav.stop()
            if now - state_start >= SETTLE_TIME:
                has_beacon = True
                new_state = "RETURN"

        elif state == "RETURN":
            # Lost Beacon check: signal much weaker than when it came in range
            if (has_beacon and pickup_rssi is not None and rssi is not None
                    and rssi < pickup_rssi - LOST_DROP):
                if lost_since is None:
                    lost_since = now
                elif now - lost_since >= LOST_CONFIRM_TIME:
                    print(f"[auto] Beacon fell out (rssi {round(pickup_rssi, 1)} ->"
                          f" {round(rssi, 1)}) - going back out for it")
                    has_beacon = False
                    lost_since = None
                    base_heading = search_heading = 0.0
                    mode, ref_rssi = "straight", None
                    new_state = "FORWARD"
            else:
                lost_since = None

            if new_state == "RETURN":
                # Turn round in an arc, then straight back (heading 180)
                if nav.x <= HOME_X:
                    nav.stop()
                    new_state = "DONE"
                else:
                    max_turn = CARRY_TURN_DUTY if has_beacon else MAX_TURN_DUTY
                    nav.drive(RETURN_DUTY, 180.0, max_turn, arc_forward=RETURN_ARC_DUTY)

        else:  # DONE
            nav.stop()

        # Safety: be stopped before the period ends
        if elapsed > AUTON_TIME - 0.3:
            nav.stop()

        # ---- State change: print it ----
        if new_state != state:
            print(f"[auto] {elapsed:4.1f}s {state} -> {new_state}")
            state = new_state
            state_start = now
            if state == "DONE":
                print("[auto] back in the Starting Area!")

        # ---- Log twice a second so you can tune the numbers ----
        if now - last_log >= 0.5:
            last_log = now
            shown = None if rssi is None else round(rssi, 1)
            print(f"[auto] t={elapsed:4.1f} {state:7s} x={nav.x:5.2f} y={nav.y:5.2f} "
                  f"hdg={nav.heading:6.1f} rssi={shown}")

    nav.stop()


def driver(robot):
    # Hands start open. Each press of a top button flips that hand
    # (open -> closed -> open ...)
    left_closed = False
    right_closed = False
    left_was_pressed = False
    right_was_pressed = False

    while robot.running:
        # Left stick up (ONE) = forward, right stick down (THREE) = backward,
        # right stick left/right (FOUR) = turn. All centered = stop.
        left_stick = robot.controller.joystick("ONE").value
        right_stick = robot.controller.joystick("THREE").value
        forward = max(left_stick, 0) + min(right_stick, 0)
        turn = robot.controller.joystick("FOUR").value

        # Ignore small stick movements so a slight wobble isn't a turn
        DEADZONE = 0.2
        if abs(forward) < DEADZONE:
            forward = 0
        if abs(turn) < DEADZONE:
            turn = 0

        # Make backward stronger (never past full power, -100)
        if forward < 0:
            forward = max(forward * BACKWARD_SPEED, -100)

        # Positive turn = right, negative turn = left
        if turn > 0:
            turn *= TURN_RIGHT_SPEED
        else:
            turn *= TURN_LEFT_SPEED

        left = forward + turn
        right = forward - turn

        # Scale down so mixing never exceeds the stick's max value
        total = abs(forward) + abs(turn)
        if total > 0:
            scale = max(abs(forward), abs(turn)) / total
            left *= scale
            right *= scale

        print("forward:", forward, "turn:", turn, "left:", left, "right:", right)

        robot.thruster(LEFT).set_duty(left)
        robot.thruster(RIGHT).set_duty(right)

        # Press a top button once to close that hand, press again to open.
        # Only flip at the moment the button goes down, so holding it
        # doesn't make the hand flap open and closed

        # Left hand: left top button
        left_is_pressed = pressed(robot, LEFT_TOP_NAMES)
        if left_is_pressed and not left_was_pressed:
            left_closed = not left_closed
        left_was_pressed = left_is_pressed
        if left_closed:
            robot.servo(LEFT_HAND).angle(LEFT_HAND_CLOSED_ANGLE)
        else:
            robot.servo(LEFT_HAND).angle(LEFT_HAND_OPEN_ANGLE)

        # Right hand: right top button
        right_is_pressed = pressed(robot, RIGHT_TOP_NAMES)
        if right_is_pressed and not right_was_pressed:
            right_closed = not right_closed
        right_was_pressed = right_is_pressed
        if right_closed:
            robot.servo(RIGHT_HAND).angle(RIGHT_HAND_CLOSED_ANGLE)
        else:
            robot.servo(RIGHT_HAND).angle(RIGHT_HAND_OPEN_ANGLE)

        # print IMU readings
        imu_reading = robot.imu(IMU_PORT).read()
        if imu_reading.ok:
            print(imu_reading)

        # print RadioBeacon readings
        radiobeacon_reading = robot.radiobeacon(RADIOBEACON_PORT).read()
        if radiobeacon_reading.ok:
            print(radiobeacon_reading)


robot.run(
    initialize=initialize,
    autonomous=autonomous,
    driver=driver,
)
