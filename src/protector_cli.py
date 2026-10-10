"""
Voltage Protector - Python serial controller
--------------------------------------------
Talks to the Arduino sketch through a virtual COM port pair (VSPE):

    Proteus COMPIM  <->  COM1 ==VSPE pair== COM2  <->  this script

Install:   pip install pyserial
Run:       python protector_cli.py            (interactive: type commands)
           python protector_cli.py --demo     (runs the 8 test commands)

Change PORT below to the VSPE port that Proteus is NOT using.
"""

import sys
import time
import serial

# ---------------------------- SETTINGS -------------------------------------
PORT = "COM2"          # the other end of the VSPE pair (COMPIM uses COM1)
BAUD = 9600            # must match Serial.begin(9600) and the COMPIM baud
BOOT_WAIT = 10         # max seconds to wait for "READY" after opening port
IDLE_TIMEOUT = 0.8     # a reply is finished after this many silent seconds
MAX_REPLY_TIME = 6     # never wait longer than this for one reply

DEMO_COMMANDS = [
    "GET STATUS",
    "SET OVP 255",
    "SET UVP 170",
    "SET OCP 12",
    "SET DELAY 5",
    "RELAY OFF",
    "RELAY ON",
    "GET LOG",
]


# ---------------------------- SERIAL HELPERS --------------------------------
def open_port():
    """Open the COM port. Proteus must be running and VSPE device started."""
    try:
        ser = serial.Serial(PORT, BAUD, bytesize=8, parity="N", stopbits=1,
                            timeout=0.1)
    except serial.SerialException as e:
        sys.exit(f"Cannot open {PORT}: {e}\n"
                 "Check: VSPE pair is started, Proteus is not using this port, "
                 "no other program (Serial Monitor) has it open.")
    return ser


def wait_for_boot(ser):
    """
    Wait until the sketch prints READY (end of the 1 s splash screen).
    If the sketch has no READY line, fall back to a fixed 3 s wait.
    Anything received before that is discarded (boot junk).
    """
    print(f"Waiting for board on {PORT} ...")
    end = time.time() + BOOT_WAIT
    while time.time() < end:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            print(f"  < {line}")
        if line == "READY":
            ser.reset_input_buffer()
            return
    time.sleep(3)                      # no READY seen: just wait a bit
    ser.reset_input_buffer()


def read_reply(ser):
    """Collect lines until the board stays quiet for IDLE_TIMEOUT seconds."""
    lines = []
    start = last = time.time()
    while time.time() - start < MAX_REPLY_TIME:
        raw = ser.readline()
        if raw:
            text = raw.decode(errors="ignore").strip()
            if text:
                lines.append(text)
                last = time.time()
                if text == "END":          # GET LOG finished
                    break
        elif lines and time.time() - last > IDLE_TIMEOUT:
            break
    return lines


def stream_view(ser):
    """Print every incoming line until Ctrl+C, then turn the stream off."""
    try:
        while True:
            raw = ser.readline()
            if raw:
                text = raw.decode(errors="ignore").strip()
                if text:
                    print(f"  < {text}")
    except KeyboardInterrupt:
        pass
    ser.write(b"STREAM OFF\n")
    time.sleep(0.5)
    ser.reset_input_buffer()
    print("\n  (stream stopped)")


def send(ser, cmd, retry=False):
    """Send one command (with newline) and print the reply."""
    ser.reset_input_buffer()
    ser.write((cmd.strip() + "\n").encode("ascii"))
    print(f"> {cmd}")

    # STREAM ON: keep printing live until Ctrl+C
    if cmd.strip().upper() == "STREAM ON":
        print("  (streaming - press Ctrl+C to stop)")
        stream_view(ser)
        return []

    lines = read_reply(ser)

    # First command after boot sometimes fails: try once more
    if retry and any(l.startswith("ERR CMD") for l in lines):
        print("  (first command rejected, retrying once)")
        time.sleep(0.5)
        return send(ser, cmd)

    if not lines:
        print("  (no reply - is the simulation running?)")
    for l in lines:
        print(f"  < {l}")
    return lines

# ---------------------------- MODES -----------------------------------------
def run_demo(ser):
    for i, cmd in enumerate(DEMO_COMMANDS):
        send(ser, cmd, retry=(i == 0))
        time.sleep(0.3)


def run_interactive(ser):
    print("Type a command (e.g. GET STATUS). Type 'quit' to exit.")
    print("Tip: after STREAM ON, status lines print every 500 ms; "
          "send STREAM OFF to stop.")
    first = True
    while True:
        try:
            cmd = input("cmd> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if cmd.lower() in ("quit", "exit", "q"):
            break
        if cmd:
            send(ser, cmd, retry=first)
            first = False


def main():
    ser = open_port()
    try:
        wait_for_boot(ser)
        if "--demo" in sys.argv:
            run_demo(ser)
        else:
            run_interactive(ser)
    finally:
        ser.close()
        print("Port closed.")


if __name__ == "__main__":
    main()
