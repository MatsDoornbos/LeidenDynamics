#!/usr/bin/env python3
"""
WATER TURRET PITCH CALIBRATION
==============================

Calibrates the relationship between pitch motor position (steps from home)
and water trajectory landing distance, at a fixed input pressure.

Nothing happens automatically — every motor move and every fire requires
a deliberate keypress.

PHYSICAL SETUP
--------------
- Apply 9 kg of pressure to the syringe (constant pressure).
- The nozzle is 24 cm above the table the water lands on.
- Have a measuring tape ready.
- Make sure the water lands somewhere safe (not on electronics!).

WORKFLOW
--------
1. Apply 9 kg pressure on the syringe.
2. Run this script.
3. Press 'h' to home the pitch motor (it moves UP until the limit switch).
4. Use u/d (or i/k for fine, U/D for coarse) to move the pitch motor.
5. Press 'f' to fire (2-second water release).
6. Walk over and measure the horizontal distance from the nozzle to where
   the water lands.
7. Come back and press 'r'. Type the distance in cm and press Enter.
8. Repeat for different pitch positions.
9. Press 's' to save (or 'q' to save and quit).

CONTROLS
--------
  h        Home pitch motor  (DO THIS FIRST)
  u / d    Pitch up / down   50 steps
  U / D    Pitch up / down  200 steps   (Shift + u / d)
  i / k    Pitch up / down   10 steps   (fine)
  f        FIRE  (2-second relay activation)
  r        Record measurement at current pitch position
  v        View all recorded measurements
  z        Undo last measurement
  s        Save calibration to file
  q        Quit (saves first if there are unsaved measurements)

OUTPUT FILE
-----------
Calibration data is written to: pitch_calibration.json (in the directory
you ran the script from). Format:
  {
    "pressure_kg": 9.0,
    "nozzle_height_cm": 24,
    "fire_duration_s": 2.0,
    "saved_at": "...",
    "measurements": [
      { "steps_from_home": 1200,
        "distance_cm": 45.0,
        "notes": "...",
        "timestamp": "..." },
      ...
    ]
  }

If a file with that name already exists when you save, it is renamed to
pitch_calibration.json.bak so you don't lose previous data.
"""

import time
import curses
import json
import os
from datetime import datetime

import RPi.GPIO as GPIO


# --- Pin definitions (must match the main turret script) ---
MOTOR_PITCH_PINS = [12, 16, 20, 21]
BUTTON_PIN = 1     # pitch limit switch
RELAY_PIN = 25     # water valve relay (active LOW)

# --- Motor stepping constants (same as main script) ---
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

PITCH_UP = 1
PITCH_DOWN = -1

# --- Calibration constants ---
PRESSURE_KG = 9.0
NOZZLE_HEIGHT_CM = 24
FIRE_DURATION_S = 1.5
PRE_HOME_BACKOFF_STEPS = 300   # match main script

# Step increments
STEP_INC_FINE = 10
STEP_INC_NORMAL = 50
STEP_INC_LARGE = 200

# Soft limit: how far below home the pitch is allowed to go.
# Adjust if your mechanism allows more, but be careful — going past the
# physical end of travel can damage the motor or the gearing.
MAX_DOWN_STEPS = 5000

# Output file (relative to current directory)
CALIBRATION_FILE = "pitch_calibration.json"


# ==============================================================
# GPIO and motor functions
# ==============================================================

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    for p in MOTOR_PITCH_PINS:
        GPIO.setup(p, GPIO.OUT)
        GPIO.output(p, 0)
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RELAY_PIN, GPIO.OUT)
    GPIO.output(RELAY_PIN, GPIO.HIGH)  # relay OFF (active-low)


def step_motor(pins, steps, direction=1):
    seq = HALF_STEP_SEQUENCE if direction == 1 else list(reversed(HALF_STEP_SEQUENCE))
    for i in range(steps):
        pattern = seq[i % len(seq)]
        for pin, val in zip(pins, pattern):
            GPIO.output(pin, val)
        time.sleep(STEP_DELAY)
    for p in pins:
        GPIO.output(p, 0)


def de_energize_pitch():
    for p in MOTOR_PITCH_PINS:
        GPIO.output(p, 0)


def button_pressed():
    return GPIO.input(BUTTON_PIN) == GPIO.LOW


def fire_relay(duration):
    GPIO.output(RELAY_PIN, GPIO.LOW)   # ON
    time.sleep(duration)
    GPIO.output(RELAY_PIN, GPIO.HIGH)  # OFF


def relay_off():
    GPIO.output(RELAY_PIN, GPIO.HIGH)


# ==============================================================
# State
# ==============================================================

class CalibrationState:
    def __init__(self):
        self.is_homed = False
        self.pitch_steps = 0          # steps from home; DOWN is positive
        self.measurements = []
        self.unsaved_changes = False
        self.last_status = "Press 'h' to home the pitch motor."
        self.last_detail = ""

    def add_measurement(self, distance_cm, notes=""):
        self.measurements.append({
            "steps_from_home": self.pitch_steps,
            "distance_cm": distance_cm,
            "notes": notes,
            "timestamp": datetime.now().isoformat(timespec='seconds'),
        })
        self.unsaved_changes = True

    def undo_last(self):
        if not self.measurements:
            return None
        self.unsaved_changes = True
        return self.measurements.pop()


# ==============================================================
# UI helpers
# ==============================================================

def safe_addstr(stdscr, y, x, s):
    """Write a string without crashing if it would go off the screen."""
    h, w = stdscr.getmaxyx()
    if y >= h or x >= w:
        return
    s = s[: max(0, w - x - 1)]
    try:
        stdscr.addstr(y, x, s)
    except curses.error:
        pass


def redraw(stdscr, state):
    stdscr.erase()
    h, w = stdscr.getmaxyx()

    safe_addstr(stdscr, 0, 0, "=== WATER TURRET PITCH CALIBRATION ===")
    safe_addstr(stdscr, 1, 0,
        f"Pressure: {PRESSURE_KG} kg | Nozzle height: {NOZZLE_HEIGHT_CM} cm | "
        f"Fire duration: {FIRE_DURATION_S} s")

    homed_str = "YES" if state.is_homed else "NO  (press 'h' first!)"
    unsaved_mark = "*" if state.unsaved_changes else " "
    safe_addstr(stdscr, 3, 0, f"Homed:           {homed_str}")
    safe_addstr(stdscr, 4, 0, f"Pitch from home: {state.pitch_steps} steps "
                              f"(DOWN is positive)")
    safe_addstr(stdscr, 5, 0, f"Measurements:    {len(state.measurements)} "
                              f"{unsaved_mark}   (* = unsaved)")

    safe_addstr(stdscr, 7, 0, "CONTROLS:")
    safe_addstr(stdscr, 8,  2, "h        Home pitch motor (DO FIRST)")
    safe_addstr(stdscr, 9,  2, "u / d    Pitch up / down   50 steps")
    safe_addstr(stdscr, 10, 2, "U / D    Pitch up / down  200 steps")
    safe_addstr(stdscr, 11, 2, "i / k    Pitch up / down   10 steps (fine)")
    safe_addstr(stdscr, 12, 2, "f        FIRE  (2-second water release)")
    safe_addstr(stdscr, 13, 2, "r        Record measurement at current pitch")
    safe_addstr(stdscr, 14, 2, "v        View all recorded measurements")
    safe_addstr(stdscr, 15, 2, "z        Undo last measurement")
    safe_addstr(stdscr, 16, 2, "s        Save calibration to file")
    safe_addstr(stdscr, 17, 2, "q        Quit (saves first if unsaved)")

    safe_addstr(stdscr, h - 3, 0, f"Status: {state.last_status}")
    safe_addstr(stdscr, h - 2, 0, f"Detail: {state.last_detail}")
    stdscr.refresh()


def prompt_input(stdscr, prompt, max_len=30):
    """
    Switch to blocking, echoed input mode and ask the user for a value.
    Returns the entered string (stripped) or empty string if cancelled.
    """
    h, _ = stdscr.getmaxyx()
    prompt_y = max(0, h - 5)
    curses.echo()
    curses.curs_set(1)
    stdscr.nodelay(False)
    try:
        stdscr.move(prompt_y, 0)
        stdscr.clrtoeol()
        safe_addstr(stdscr, prompt_y, 0, prompt)
        stdscr.refresh()
        try:
            value = stdscr.getstr(prompt_y, len(prompt), max_len)
        except Exception:
            return ""
        try:
            return value.decode("utf-8").strip()
        except Exception:
            return ""
    finally:
        curses.noecho()
        curses.curs_set(0)
        stdscr.nodelay(True)


# ==============================================================
# Actions
# ==============================================================

def home_pitch(stdscr, state):
    """
    Home pitch by stepping DOWN by a small backoff, then UP until the
    limit switch is pressed. Aborts on 'q'.
    """
    state.last_status = "Homing: backing off, then moving UP slowly..."
    state.last_detail = "Press 'q' to abort."
    redraw(stdscr, state)

    # Backoff downward to ensure we approach home from below
    step_motor(MOTOR_PITCH_PINS, PRE_HOME_BACKOFF_STEPS, direction=PITCH_DOWN)
    de_energize_pitch()
    time.sleep(0.2)

    # Move up in chunks, checking the limit switch and 'q' between chunks
    while True:
        if button_pressed():
            de_energize_pitch()
            time.sleep(0.2)
            state.is_homed = True
            state.pitch_steps = 0
            state.last_status = "Homed! Pitch is at the top (limit switch)."
            state.last_detail = ""
            return True

        key = stdscr.getch()
        if key != -1:
            try:
                if chr(key).lower() == 'q':
                    de_energize_pitch()
                    state.is_homed = False
                    state.last_status = "Homing aborted by user."
                    state.last_detail = "Pitch position is now UNKNOWN. Press 'h' to retry."
                    return False
            except ValueError:
                pass

        step_motor(MOTOR_PITCH_PINS, 10, direction=PITCH_UP)


def adjust_pitch(stdscr, state, requested_steps, direction):
    """
    Move pitch with soft limits. Going up is bounded by home (0 steps).
    Going down is bounded by MAX_DOWN_STEPS.
    """
    if direction == PITCH_UP:
        max_steps = state.pitch_steps  # cannot go above home
        if max_steps <= 0:
            state.last_detail = "At home — cannot go further up."
            return False
        steps = min(requested_steps, max_steps)
    else:
        room = MAX_DOWN_STEPS - state.pitch_steps
        if room <= 0:
            state.last_detail = (f"At soft limit ({MAX_DOWN_STEPS} steps) — "
                                 f"cannot go further down.")
            return False
        steps = min(requested_steps, room)

    step_motor(MOTOR_PITCH_PINS, steps, direction=direction)
    de_energize_pitch()
    if direction == PITCH_UP:
        state.pitch_steps -= steps
    else:
        state.pitch_steps += steps

    arrow = "UP" if direction == PITCH_UP else "DOWN"
    state.last_detail = (f"Moved {steps} steps {arrow}. "
                         f"Now at {state.pitch_steps} steps from home.")
    return True


def fire(stdscr, state):
    state.last_status = f"FIRING for {FIRE_DURATION_S} s..."
    state.last_detail = "Hold steady. Watch where the water lands."
    redraw(stdscr, state)
    fire_relay(FIRE_DURATION_S)
    state.last_status = "Fire complete. Measure the distance, then press 'r' to record."
    state.last_detail = ""


def record_measurement(stdscr, state):
    distance_str = prompt_input(
        stdscr,
        f"Distance in cm at {state.pitch_steps} steps (empty to cancel): ",
        max_len=15,
    )
    if not distance_str:
        state.last_status = "Recording cancelled (no distance entered)."
        return

    try:
        distance_cm = float(distance_str)
    except ValueError:
        state.last_status = f"Could not parse '{distance_str}' as a number. Not recorded."
        return

    notes = prompt_input(stdscr, "Optional notes (Enter to skip): ", max_len=80)
    state.add_measurement(distance_cm, notes)
    state.last_status = (f"Recorded: {state.pitch_steps} steps -> "
                         f"{distance_cm:.1f} cm. "
                         f"Total: {len(state.measurements)} (unsaved).")
    state.last_detail = ""


def undo_last(state):
    removed = state.undo_last()
    if removed is None:
        state.last_status = "Nothing to undo (no measurements yet)."
        return
    state.last_status = (f"Removed last measurement: "
                         f"{removed['steps_from_home']} steps -> "
                         f"{removed['distance_cm']:.1f} cm. "
                         f"Now {len(state.measurements)} remain.")


def view_measurements(stdscr, state):
    stdscr.nodelay(False)
    try:
        stdscr.erase()
        h, w = stdscr.getmaxyx()

        safe_addstr(stdscr, 0, 0,
                    f"=== RECORDED MEASUREMENTS ({len(state.measurements)}) ===")
        safe_addstr(stdscr, 1, 0, "(sorted by steps from home)")
        safe_addstr(stdscr, 3, 0, f"{'#':>3}  {'steps':>6}  {'cm':>7}  notes")
        safe_addstr(stdscr, 4, 0, "-" * (w - 1))

        sorted_ms = sorted(state.measurements, key=lambda m: m["steps_from_home"])
        max_lines = max(1, h - 7)
        for i, m in enumerate(sorted_ms[:max_lines]):
            line = (f"{i+1:>3}  {m['steps_from_home']:>6}  "
                    f"{m['distance_cm']:>7.1f}  {m.get('notes','')}")
            safe_addstr(stdscr, 5 + i, 0, line)

        if len(sorted_ms) > max_lines:
            safe_addstr(stdscr, h - 3, 0,
                        f"... and {len(sorted_ms) - max_lines} more "
                        f"(screen too small to show all)")
        safe_addstr(stdscr, h - 2, 0, "Press any key to return.")
        stdscr.refresh()
        stdscr.getch()
    finally:
        stdscr.nodelay(True)


def save_calibration(state):
    payload = {
        "pressure_kg": PRESSURE_KG,
        "nozzle_height_cm": NOZZLE_HEIGHT_CM,
        "fire_duration_s": FIRE_DURATION_S,
        "saved_at": datetime.now().isoformat(timespec='seconds'),
        "measurements": state.measurements,
    }
    if os.path.exists(CALIBRATION_FILE):
        try:
            os.replace(CALIBRATION_FILE, CALIBRATION_FILE + ".bak")
        except OSError:
            pass
    with open(CALIBRATION_FILE, "w") as f:
        json.dump(payload, f, indent=2)
    state.unsaved_changes = False


# ==============================================================
# Main loop
# ==============================================================

def main(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.clear()

    setup_gpio()
    state = CalibrationState()

    try:
        redraw(stdscr, state)

        while True:
            key = stdscr.getch()
            if key == -1:
                time.sleep(0.05)
                continue

            try:
                ch = chr(key)
            except ValueError:
                continue

            if ch == 'q':
                if state.unsaved_changes and state.measurements:
                    save_calibration(state)
                    state.last_status = (
                        f"Saved {len(state.measurements)} measurements "
                        f"to {CALIBRATION_FILE}. Goodbye.")
                    redraw(stdscr, state)
                    time.sleep(0.6)
                break

            elif ch == 'h':
                home_pitch(stdscr, state)

            elif ch in ('u', 'd', 'U', 'D', 'i', 'k'):
                if not state.is_homed:
                    state.last_status = "Refused: not homed yet. Press 'h' first."
                else:
                    if ch == 'u':
                        adjust_pitch(stdscr, state, STEP_INC_NORMAL, PITCH_UP)
                    elif ch == 'd':
                        adjust_pitch(stdscr, state, STEP_INC_NORMAL, PITCH_DOWN)
                    elif ch == 'U':
                        adjust_pitch(stdscr, state, STEP_INC_LARGE, PITCH_UP)
                    elif ch == 'D':
                        adjust_pitch(stdscr, state, STEP_INC_LARGE, PITCH_DOWN)
                    elif ch == 'i':
                        adjust_pitch(stdscr, state, STEP_INC_FINE, PITCH_UP)
                    elif ch == 'k':
                        adjust_pitch(stdscr, state, STEP_INC_FINE, PITCH_DOWN)
                    state.last_status = f"At {state.pitch_steps} steps from home."

            elif ch == 'f':
                if not state.is_homed:
                    state.last_status = "Refused: not homed. Press 'h' first."
                else:
                    fire(stdscr, state)

            elif ch == 'r':
                if not state.is_homed:
                    state.last_status = ("Refused: not homed. Cannot record "
                                         "without a known position.")
                else:
                    record_measurement(stdscr, state)

            elif ch == 'v':
                view_measurements(stdscr, state)

            elif ch == 'z':
                undo_last(state)

            elif ch == 's':
                if not state.measurements:
                    state.last_status = "Nothing to save (no measurements)."
                else:
                    save_calibration(state)
                    state.last_status = (
                        f"Saved {len(state.measurements)} measurements "
                        f"to {CALIBRATION_FILE}.")

            redraw(stdscr, state)

    except KeyboardInterrupt:
        pass
    finally:
        relay_off()
        de_energize_pitch()
        GPIO.cleanup()


if __name__ == "__main__":
    curses.wrapper(main)