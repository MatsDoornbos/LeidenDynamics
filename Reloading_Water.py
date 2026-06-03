"""
relay_trigger.py — simple keyboard relay control for the waterturret.

    'o'  open  (energize the relay)
    'p'  close (de-energize the relay)
    'q'  quit  (also Ctrl+C)

Run with the venv active: `python relay_trigger.py`
"""

from gpiozero import OutputDevice
import sys
import termios
import tty

# --- CONFIGURATION ---------------------------------------------------------

RELAY_PIN = 25       # BCM pin number where the relay's IN is wired

# Most cheap blue relay modules are ACTIVE LOW (relay energizes when the
# GPIO is pulled LOW). Leave False for those. Only set True if your specific
# module is the high-trigger kind.
ACTIVE_HIGH = False

# --------------------------------------------------------------------------

relay = OutputDevice(RELAY_PIN, active_high=ACTIVE_HIGH, initial_value=False)

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)

try:
    # Raw mode: read each keystroke instantly, no Enter required.
    tty.setraw(fd)

    sys.stdout.write("'o' open  |  'p' close  |  'q' quit\r\n")
    sys.stdout.flush()

    while True:
        ch = sys.stdin.read(1)

        if ch == 'q' or ch == '\x03':   # 'q' or Ctrl+C
            break
        elif ch == 'o':
            relay.on()
            sys.stdout.write("OPEN\r\n")
            sys.stdout.flush()
        elif ch == 'p':
            relay.off()
            sys.stdout.write("CLOSED\r\n")
            sys.stdout.flush()

finally:
    relay.off()
    relay.close()
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    sys.stdout.write("\r\nDone.\r\n")
    sys.stdout.flush()