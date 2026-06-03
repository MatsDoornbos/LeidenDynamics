import time
import curses
import sys
import json
import os
import RPi.GPIO as GPIO
from picamera2 import Picamera2
from pupil_apriltags import Detector
import cv2

# --- Pin definitions (BCM) ---
MOTOR_YAW_PINS    = [6, 13, 19, 26]
MOTOR_PITCH_PINS  = [12, 16, 20, 21]
BUTTON_PIN = 1
RELAY_PIN = 25
IR_HOME_PIN = 24

# --- Motor constants ---
STEP_DELAY = 0.002

HALF_STEP_SEQUENCE = [
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
    [1, 0, 0, 1],
]

YAW_LEFT  =  1
YAW_RIGHT = -1
PITCH_UP  = 1
PITCH_DOWN = -1

# Scanning constants
STEPS_PER_DEGREE = 4096 / 360

SCAN_LEFT_DEGREES = 70
SCAN_RIGHT_DEGREES = 140
SCAN_LEFT_STEPS = int(SCAN_LEFT_DEGREES * STEPS_PER_DEGREE)
SCAN_RIGHT_STEPS = int(SCAN_RIGHT_DEGREES * STEPS_PER_DEGREE)
SCAN_STEP_SIZE = 30

# --- Camera constants ---
IMG_W, IMG_H = 1280, 720
CX = IMG_W // 2

CAMERA_PARAMS = (940, 940, 640, 360)
TAG_SIZE = 0.058

# Centering parameters
DEAD_ZONE = 30
STEPS_PER_CORRECTION = 15

# Pitch movement constants
PRE_HOME_BACKOFF_STEPS = 300

# Calibration
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_FILE = os.path.join(SCRIPT_DIR, "pitch_calibration.json")
MAX_FIRE_STEPS = 5000   # safety clamp for the pitch motor (just past the new calibration max)


#   "descending"     - higher step count, STEEPER arc (drops into pots from above; default)
#   "ascending"      - lower step count, FLATTER arc (would hit pot fronts; rarely useful)
#   "closer_to_max"  - whichever is closer to peak range
PITCH_PREFERENCE = "descending"

# aim offset to land in pot
AIM_OFFSET_CM = 5.0

# =====================================================================
# Pitch calibration
# =====================================================================

def load_calibration(path):
    """Load calibration data from JSON. Raises on missing or invalid file.
    Prints warnings if the data looks suspiciously incomplete."""
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"Calibration file not found: {path}\n"
            f"Run water_turret_calibrate.py first to create it."
        )
    with open(path) as f:
        data = json.load(f)
    measurements = data.get("measurements", [])
    if len(measurements) < 2:
        raise ValueError(
            f"Calibration file has only {len(measurements)} measurement(s); "
            f"need at least 2 to interpolate."
        )

    # Sanity checks: catch the case where someone reran calibration and
    # accidentally saved just the new measurements instead of merging them
    # with the previous session.
    pts = sorted([(m["steps_from_home"], m["distance_cm"]) for m in measurements])
    max_idx = max(range(len(pts)), key=lambda i: pts[i][1])
    peak_steps, peak_dist = pts[max_idx]

    warnings = []
    if len(measurements) < 5:
        warnings.append(
            f"Only {len(measurements)} measurements — interpolation will be coarse."
        )
    if max_idx == 0:
        warnings.append(
            f"Peak distance ({peak_dist:.0f} cm) is at the LOWEST step count "
            f"({peak_steps}). No ascending-side data present — the curve looks "
            f"one-sided. Did you forget to merge calibration sessions?"
        )
    if max_idx == len(pts) - 1:
        warnings.append(
            f"Peak distance ({peak_dist:.0f} cm) is at the HIGHEST step count "
            f"({peak_steps}). No descending-side data present — the curve looks "
            f"one-sided. Did you forget to merge calibration sessions?"
        )

    if warnings:
        print("WARNING: calibration data may be incomplete:", file=sys.stderr)
        for w in warnings:
            print(f"  - {w}", file=sys.stderr)
        print(f"  File: {path}", file=sys.stderr)
        print(file=sys.stderr)

    return {
        "measurements": measurements,
        "fire_duration_s": float(data.get("fire_duration_s", 2.0)),
        "pressure_kg": data.get("pressure_kg"),
        "nozzle_height_cm": data.get("nozzle_height_cm"),
    }


def pitch_steps_for_distance(distance_cm, measurements,
                             prefer=PITCH_PREFERENCE):
    """
    Distances above the peak are clamped to the peak position.
    Distances below the minimum reachable are clamped to the closest
    measured point.
    """
    if not measurements:
        raise ValueError("No calibration measurements available")

    pts = sorted(
        [(m["steps_from_home"], m["distance_cm"]) for m in measurements],
        key=lambda p: p[0],
    )
    max_idx = max(range(len(pts)), key=lambda i: pts[i][1])
    max_steps, max_distance = pts[max_idx]

    if distance_cm >= max_distance:
        return max_steps

    ascending  = pts[: max_idx + 1]   # distance grows with steps
    descending = pts[max_idx:]        # distance falls with steps

    def interp(target, points):
        distances = [p[1] for p in points]
        if target < min(distances) or target > max(distances):
            return None
        for i in range(len(points) - 1):
            s1, d1 = points[i]
            s2, d2 = points[i + 1]
            if (d1 <= target <= d2) or (d2 <= target <= d1):
                if d1 == d2:
                    return int(round((s1 + s2) / 2))
                t = (target - d1) / (d2 - d1)
                return int(round(s1 + t * (s2 - s1)))
        return None

    asc = interp(distance_cm, ascending)
    desc = interp(distance_cm, descending)

    if asc is None and desc is None:
        # Below minimum reachable; pick the calibration point closest
        # to the requested distance.
        return min(pts, key=lambda p: abs(p[1] - distance_cm))[0]
    if asc is None:
        return desc
    if desc is None:
        return asc

    if prefer == "ascending":
        return asc
    if prefer == "closer_to_max":
        return asc if abs(asc - max_steps) < abs(desc - max_steps) else desc
    return desc


# =====================================================================
# Low-level functions
# =====================================================================

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    for p in MOTOR_YAW_PINS + MOTOR_PITCH_PINS:
        GPIO.setup(p, GPIO.OUT)
        GPIO.output(p, 0)
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RELAY_PIN, GPIO.OUT)
    GPIO.output(RELAY_PIN, GPIO.HIGH)
    GPIO.setup(IR_HOME_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)


def step_motor(pins, steps, direction=1, on_step=None):

    seq = HALF_STEP_SEQUENCE if direction == 1 else list(reversed(HALF_STEP_SEQUENCE))
    for i in range(steps):
        pattern = seq[i % len(seq)]
        for pin, val in zip(pins, pattern):
            GPIO.output(pin, val)
        time.sleep(STEP_DELAY)
        if on_step is not None and on_step(i + 1):
            for p in pins:
                GPIO.output(p, 0)
            return i + 1
    for p in pins:
        GPIO.output(p, 0)
    return steps


def de_energize_all():
    for p in MOTOR_YAW_PINS + MOTOR_PITCH_PINS:
        GPIO.output(p, 0)


def button_pressed():
    return GPIO.input(BUTTON_PIN) == GPIO.LOW


def ir_home_detected():
    return GPIO.input(IR_HOME_PIN) == GPIO.HIGH


def fire_relay(duration):
    GPIO.output(RELAY_PIN, GPIO.LOW)
    time.sleep(duration)
    GPIO.output(RELAY_PIN, GPIO.HIGH)


def relay_off():
    GPIO.output(RELAY_PIN, GPIO.HIGH)


def status(stdscr, msg):
    stdscr.move(2, 0)
    stdscr.clrtoeol()
    stdscr.addstr(2, 0, msg[:curses.COLS - 1] if curses.COLS else msg)
    stdscr.refresh()


def detail(stdscr, msg):
    stdscr.move(3, 0)
    stdscr.clrtoeol()
    stdscr.addstr(3, 0, msg[:curses.COLS - 1] if curses.COLS else msg)
    stdscr.refresh()


def check_quit(stdscr):
    key = stdscr.getch()
    if key != -1 and chr(key).lower() == 'q':
        return True
    return False


def move_yaw_to(current_pos, target_pos):
    diff = target_pos - current_pos
    if diff > 0:
        step_motor(MOTOR_YAW_PINS, diff, YAW_RIGHT)
    elif diff < 0:
        step_motor(MOTOR_YAW_PINS, abs(diff), YAW_LEFT)
    return target_pos


# =====================================================================
# High-level functions
# =====================================================================

def home_pitch(stdscr):
    while True:
        if check_quit(stdscr):
            return False
        if button_pressed():
            de_energize_all()
            time.sleep(0.3)
            return True
        step_motor(MOTOR_PITCH_PINS, 10, direction=PITCH_UP)


def home_yaw(stdscr, current_pos):
    """
    Return to horizontal home using the IR sensor.

    Sensor wins over odometry

    Returns (True, 0) on success, (False, current_pos) on failure.
    """
    if ir_home_detected():
        status(stdscr, "Already at home (sensor confirms).")
        return True, 0

    # Direction toward home from our position estimate
    if current_pos >= 0:
        direction = YAW_LEFT
        travel = current_pos
    else:
        direction = YAW_RIGHT
        travel = -current_pos

    # Generous travel: estimate plus 50% margin or 500 steps, whichever
    # is larger. This covers significant odometry drift without spinning
    # forever if the sensor is dead.
    margin = max(500, int(travel * 0.5))
    max_travel = travel + margin

    state = {'found': False, 'quit': False}

    def poll(_step):
        if ir_home_detected():
            state['found'] = True
            return True
        key = stdscr.getch()
        if key != -1:
            try:
                if chr(key).lower() == 'q':
                    state['quit'] = True
                    return True
            except ValueError:
                pass
        return False

    dir_name = "LEFT" if direction == YAW_LEFT else "RIGHT"
    status(stdscr, f"Returning home {dir_name} (up to {max_travel} steps, "
                   f"polling sensor every step)...")
    actual = step_motor(MOTOR_YAW_PINS, max_travel, direction, on_step=poll)
    if direction == YAW_RIGHT:
        current_pos += actual
    else:
        current_pos -= actual

    if state['quit']:
        return False, current_pos
    if state['found']:
        status(stdscr, f"Home found after {actual} steps.")
        return True, 0

    # Didn't find it in the expected direction. Reverse and sweep.
    other = YAW_RIGHT if direction == YAW_LEFT else YAW_LEFT
    other_name = "RIGHT" if other == YAW_RIGHT else "LEFT"
    sweep = max_travel + 500
    status(stdscr, f"Not found — sweeping {other_name} (up to {sweep} steps)...")

    state['found'] = False
    actual = step_motor(MOTOR_YAW_PINS, sweep, other, on_step=poll)
    if other == YAW_RIGHT:
        current_pos += actual
    else:
        current_pos -= actual

    if state['quit']:
        return False, current_pos
    if state['found']:
        status(stdscr, f"Home found on reverse sweep ({actual} steps).")
        return True, 0

    status(stdscr, "ERROR: IR home sensor never triggered. Yaw position is unreliable.")
    return False, current_pos


def scan_environment(stdscr, picam, detector):
    detections_log = {}

    status(stdscr, f"Scan: Moving left {SCAN_LEFT_DEGREES} degrees...")
    step_motor(MOTOR_YAW_PINS, SCAN_LEFT_STEPS, YAW_LEFT)
    current_pos = -SCAN_LEFT_STEPS
    time.sleep(0.3)

    status(stdscr, f"Scan: Sweeping right {SCAN_RIGHT_DEGREES} degrees...")
    steps_swept = 0

    while steps_swept < SCAN_RIGHT_STEPS:
        if check_quit(stdscr):
            return None, current_pos

        frame = picam.capture_array()
        frame = cv2.flip(frame, -1)
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        detections = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=CAMERA_PARAMS,
            tag_size=TAG_SIZE
        )

        for d in detections:
            tid = d.tag_id
            dist = d.pose_t[2][0]
            if tid not in detections_log:
                detections_log[tid] = []
            detections_log[tid].append((current_pos, dist))

        tag_ids_here = [d.tag_id for d in detections]
        found_so_far = len(detections_log)
        deg_from_start = current_pos / STEPS_PER_DEGREE
        detail(stdscr, f"Pos:{deg_from_start:+.0f}deg  Here:{tag_ids_here}  Found:{found_so_far}")

        actual_step = min(SCAN_STEP_SIZE, SCAN_RIGHT_STEPS - steps_swept)
        step_motor(MOTOR_YAW_PINS, actual_step, YAW_RIGHT)
        current_pos += actual_step
        steps_swept += actual_step

    targets = []
    for tag_id, observations in detections_log.items():
        positions = sorted([p for p, d in observations])
        distances = [d for p, d in observations]
        median_pos = positions[len(positions) // 2]
        avg_dist = sum(distances) / len(distances)
        targets.append({
            'tag_id': tag_id,
            'step_pos': median_pos,
            'distance': avg_dist,
        })

    return targets, current_pos


def nearest_neighbor_order(targets, start_pos):
    remaining = list(targets)
    ordered = []
    pos = start_pos

    while remaining:
        nearest_idx = 0
        nearest_dist = abs(remaining[0]['step_pos'] - pos)
        for i, t in enumerate(remaining):
            d = abs(t['step_pos'] - pos)
            if d < nearest_dist:
                nearest_dist = d
                nearest_idx = i

        chosen = remaining.pop(nearest_idx)
        ordered.append(chosen)
        pos = chosen['step_pos']

    return ordered


def center_on_tag(stdscr, picam, detector, current_pos, expected_tag_id):
    centered_count = 0
    tag_distance = None
    attempts = 0
    max_attempts = 200

    while attempts < max_attempts:
        attempts += 1
        if check_quit(stdscr):
            return False, current_pos, None

        frame = picam.capture_array()
        frame = cv2.flip(frame, -1)
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        detections = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=CAMERA_PARAMS,
            tag_size=TAG_SIZE
        )

        # Filter to the specific tag we're trying to engage. Two tags can
        # share the camera frame; without this filter we'd lock onto
        # whichever one happens to be detections[0], which is often not
        # the target we just slewed yaw to.
        matching = [d for d in detections if d.tag_id == expected_tag_id]

        if not matching:
            detail(stdscr, f"Tag {expected_tag_id} not in view — waiting...")
            centered_count = 0
            time.sleep(0.1)
            continue

        tag = matching[0]
        tx = tag.center[0]
        dx = tx - CX
        tag_distance = tag.pose_t[2][0]

        if abs(dx) <= DEAD_ZONE:
            centered_count += 1
            detail(stdscr, f"CENTERED ({dx:+.0f}px) [{centered_count}/5] d={tag_distance:.2f}m")
            if centered_count >= 5:
                return True, current_pos, tag_distance
            time.sleep(0.05)
            continue

        centered_count = 0

        if dx > 0:
            step_motor(MOTOR_YAW_PINS, STEPS_PER_CORRECTION, YAW_RIGHT)
            current_pos += STEPS_PER_CORRECTION
        else:
            step_motor(MOTOR_YAW_PINS, STEPS_PER_CORRECTION, YAW_LEFT)
            current_pos -= STEPS_PER_CORRECTION

        detail(stdscr, f"Centering: offset {dx:+.0f}px | pos={current_pos}")

    return False, current_pos, tag_distance


def fire_sequence(stdscr, target_distance_cm, calibration):
    """
    Pitch to the calibrated angle for the target distance, fire for the
    calibrated duration, then return to home.
    """
    measurements = calibration["measurements"]
    fire_duration = calibration["fire_duration_s"]

    fire_steps = pitch_steps_for_distance(target_distance_cm, measurements)
    # Safety clamp — never command a position outside the soft range
    fire_steps = max(0, min(fire_steps, MAX_FIRE_STEPS))

    status(stdscr, "Preparing: vertical home...")
    step_motor(MOTOR_PITCH_PINS, PRE_HOME_BACKOFF_STEPS, direction=PITCH_DOWN)
    de_energize_all()
    time.sleep(0.2)
    if not home_pitch(stdscr):
        return False

    status(stdscr, f"Pitching to {fire_steps} steps "
                   f"for target at {target_distance_cm:.0f} cm...")
    step_motor(MOTOR_PITCH_PINS, fire_steps, direction=PITCH_DOWN)
    de_energize_all()
    time.sleep(0.5)

    status(stdscr, f"FIRING for {fire_duration:.1f}s!")
    fire_relay(fire_duration)
    time.sleep(0.5)

    status(stdscr, "Returning to vertical home...")
    step_motor(MOTOR_PITCH_PINS, PRE_HOME_BACKOFF_STEPS, direction=PITCH_DOWN)
    de_energize_all()
    time.sleep(0.2)
    if not home_pitch(stdscr):
        return False

    return True


# =====================================================================
# Main
# =====================================================================

def main(stdscr, calibration):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.clear()

    setup_gpio()

    picam = Picamera2()
    config = picam.create_video_configuration(main={"size": (IMG_W, IMG_H), "format": "RGB888"})
    picam.configure(config)
    picam.start()
    time.sleep(1)

    detector = Detector(families="tag36h11")
    current_pos = 0

    try:
        n_measurements = len(calibration['measurements'])
        stdscr.addstr(0, 0,
            f"=== WATER TURRET === ({n_measurements} cal pts, "
            f"fire={calibration['fire_duration_s']}s) (Q to quit)")
        stdscr.refresh()

        # ==========================================================
        # PHASE 1: Initial home (both axes)
        # ==========================================================
        status(stdscr, "Phase 1: Vertical home...")
        step_motor(MOTOR_PITCH_PINS, PRE_HOME_BACKOFF_STEPS, direction=PITCH_DOWN)
        de_energize_all()
        time.sleep(0.2)
        if not home_pitch(stdscr):
            return

        status(stdscr, "Phase 1: Verifying horizontal home (IR sensor)...")
        if not ir_home_detected():
            status(stdscr, "ERROR: Not at home! Manually center the turret so the IR sensor")
            detail(stdscr, "triggers, then restart. Press Q to quit.")
            while not check_quit(stdscr):
                time.sleep(0.1)
            return
        status(stdscr, "Phase 1: At horizontal home.")
        current_pos = 0
        time.sleep(0.5)

        # ==========================================================
        # PHASE 2: Scan environment
        # ==========================================================
        status(stdscr, "Phase 2: Scanning...")
        time.sleep(0.5)

        targets, current_pos = scan_environment(stdscr, picam, detector)
        if targets is None:
            return

        if len(targets) == 0:
            status(stdscr, "No tags found. Returning home...")
            success, current_pos = home_yaw(stdscr, current_pos)
            status(stdscr, "Home. No targets. Press Q.")
            while not check_quit(stdscr):
                time.sleep(0.1)
            return

        targets = nearest_neighbor_order(targets, current_pos)

        tag_summary = ", ".join([
            f"ID{t['tag_id']}({t['step_pos']/STEPS_PER_DEGREE:+.0f}deg)"
            for t in targets
        ])
        status(stdscr, f"Found {len(targets)}: {tag_summary}")
        detail(stdscr, "Starting engagement...")
        time.sleep(2)

        # ==========================================================
        # PHASE 3: Engage each target
        # ==========================================================
        for i, target in enumerate(targets):
            deg = target['step_pos'] / STEPS_PER_DEGREE
            status(stdscr, f"[{i+1}/{len(targets)}] Tag {target['tag_id']} "
                           f"({deg:+.0f}deg, ~{target['distance']:.2f}m)")

            current_pos = move_yaw_to(current_pos, target['step_pos'])
            time.sleep(0.3)

            detail(stdscr, "Fine centering...")
            success, current_pos, distance_m = center_on_tag(
                stdscr, picam, detector, current_pos, target['tag_id']
            )

            if not success:
                detail(stdscr, f"Could not center on tag {target['tag_id']} — skipping")
                time.sleep(1)
                continue

            # Tag pose distance is in meters; calibration is in cm.
            # Add AIM_OFFSET_CM so we shoot into the pot (which extends
            # behind the tag), not at the tag itself.
            tag_distance_cm = distance_m * 100.0
            target_distance_cm = tag_distance_cm + AIM_OFFSET_CM

            status(stdscr, f"Locked: tag {target['tag_id']} at "
                           f"{tag_distance_cm:.0f} cm "
                           f"(aim {target_distance_cm:.0f} cm)")

            if not fire_sequence(stdscr, target_distance_cm, calibration):
                return

            status(stdscr, f"Tag {target['tag_id']} done! ({i+1}/{len(targets)})")
            detail(stdscr, "")
            time.sleep(1)

        # ==========================================================
        # PHASE 4: Return to home
        # ==========================================================
        success, current_pos = home_yaw(stdscr, current_pos)
        if not success:
            return

        status(stdscr, "HOME. All targets engaged. Press Q.")
        while not check_quit(stdscr):
            time.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        relay_off()
        de_energize_all()
        picam.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    # Load calibration before entering curses, so errors are visible
    try:
        calibration = load_calibration(CALIBRATION_FILE)
    except Exception as e:
        print(f"ERROR loading calibration: {e}", file=sys.stderr)
        sys.exit(1)

    n = len(calibration['measurements'])
    print(f"Calibration loaded: {n} points, "
          f"pressure={calibration['pressure_kg']} kg, "
          f"fire={calibration['fire_duration_s']}s")

    curses.wrapper(main, calibration)