# Leiden Dynamics Smart Sprinkler System

**Team:** Mats Doornbos, Justin Gruijters, Lucas Idsinga, Benard Wanyande
**Institution:** Leiden University (MSc Robotics)

**Availability Notice:** This repository and all associated files, code, and documentation will remain publicly available and accessible until at least August 31st, 2026.

---

## Repository Contents

*   `Turret_Demo.py`: The main autonomous execution script for scanning, targeting, and firing.
*   `Turret_Calibration.py`: The script used to generate the required trajectory lookup table.
*   `Reloading_Water.py`: A utility script for manually opening and closing the water relay.
*   `/Designs/`: Directory containing CAD models (STEP files) and assembly diagrams for the 3D-printed turret and A-frame fluid system.
*   `SmartSprinkler.mp4`: Video demonstration of the final working system.

## System Requirements

### Hardware Configuration
The software is written specifically for a Raspberry Pi connected to the custom 3D-printed turret and passive-pressure syringe system. Ensure your wiring matches the following configuration for data and power.

**GPIO Pinout (BCM):**
*   **Yaw (Pan) Stepper Motor:** Pins 6, 13, 19, 26
*   **Pitch (Tilt) Stepper Motor:** Pins 12, 16, 20, 21
*   **Pitch Limit Switch (Vertical Home):** Pin 1 (Wired to Ground)
*   **Yaw IR Sensor (Horizontal Home):** Pin 24
*   **Valve Relay:** Pin 25

**Power Supply Wiring:**
*   **5V Power Supply:** Connected directly to the motor power pins to drive the two stepper motors.
*   **12V Power Supply:** Connected via the relay module to provide power to the solenoid valve.
*   **Solenoid Valve:** Wired as a separate circuit, receiving 12V power only when the relay is opened by the Raspberry Pi.

The physical fluid system requires a 500ml syringe filled with water, pressurized by a 9kg weight placed on the plunger.

### Software Dependencies
The system runs on Raspberry Pi OS. You must install the following Python libraries before executing the code:

```bash
sudo apt-get update
sudo apt-get install python3-opencv
pip install RPi.GPIO picamera2 pupil-apriltags gpiozero
```

## Instructions for Use

The system operates strictly via the terminal using the `curses` library. Do not run these scripts inside an IDE (like Thonny), as the keyboard inputs will not register correctly. 

### Step 1: Manual Relay Control (Optional)
If you need to prime the water lines or empty the syringe without moving the motors, run the manual relay script:
```bash
python3 Reloading_Water.py
```
*   Press `o` to open the valve.
*   Press `p` to close the valve.
*   Press `q` to quit.

### Step 2: System Calibration (Required)
The main targeting script cannot calculate water trajectory without calibration data. You must generate a `pitch_calibration.json` file.

1. Ensure the syringe is filled and the 9kg weight is applied.
2. Run the calibration script:
```bash
python3 Turret_Calibration.py
```
3. Press `h` to home the pitch motor against the limit switch.
4. Use `u` and `d` (or `U` and `D` for larger steps) to adjust the pitch angle.
5. Press `f` to fire a burst of water.
6. Measure the horizontal distance from the nozzle to where the water landed in centimeters.
7. Press `r` and type the measured distance to record the data point.
8. Repeat steps 4 through 7 for various pitch angles to create a robust data curve.
9. Press `s` to save the data to `pitch_calibration.json`, then press `q` to quit.

### Step 3: Autonomous Execution
Once `pitch_calibration.json` exists in the same directory, the system can run autonomously.

1. Place AprilTags belonging to the `tag36h11` family on the target plant pots.
2. Run the main targeting script:
```bash
python3 Turret_Demo.py
```
3. The robot will automatically:
   * Home the pitch motor against the limit switch.
   * Home the yaw motor using the IR sensor.
   * Scan 140 degrees to locate all visible AprilTags.
   * Center the camera on each tag.
   * Calculate the required pitch angle using the distance data from the calibration file.
   * Trigger the solenoid valve to fire water into the pot.
   * Return to the home position and await further instruction.
4. Press `q` at any time to safely terminate the program and de-energize the motors and valve.

### Configuration
If the water lands too far in front of or behind the plant pot, open `Turret_Demo.py` in a text editor and adjust the `AIM_OFFSET_CM` variable (default is `5.0`). This value adds a fixed distance to the target calculation to ensure the water lands inside the depth of the pot rather than directly on the front-mounted AprilTag.