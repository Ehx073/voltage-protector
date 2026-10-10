"""
Voltage Protector - Python serial controller
--------------------------------------------
    Proteus COMPIM  <->  COM1 ==VSPE pair== COM2  <->  this script

Install:   pip install pyserial
Run:       python protector_cli.py            (interactive)
           python protector_cli.py --demo     (runs the 8 test commands)
"""

import sys
import time
import queue
import threading
import serial

# ---------------------------- SETTINGS -------------------------------------
PORT = "COM2"
BAUD = 9600
BOOT_WAIT = 10         # max seconds to wait for "READY"
IDLE_TIMEOUT = 0.8     # a reply is finished after this many silent seconds
MAX_REPLY_TIME = 6     # never wait longer than this for one reply

DEMO_COMMANDS = [
    "GET STATUS", "SET OVP 255", "SET UVP 170", "SET OCP 12",
    "SET DELAY 5", "RELAY OFF", "RELAY ON", "GET LOG",
]

rx_queue = queue.Queue()   # reply lines from the board (events are not queued)


# ---------------------------- SERIAL HELPERS --------------------------------
def open_port():
    try:
        return serial.Serial(PORT, BAUD, bytesize=8, parity="N", stopbits=1,
                             timeout=0.1)
    except serial.SerialException as e:
        sys.exit(f"Cannot open {PORT}: {e}\n"
                 "Check: VSPE pair is started, Proteus is not using this port, "
                 "no other program (Serial Monitor) has it open.")


def wait_for_boot(ser):
    print(f"Waiting for board on {PORT} ...")
    end = time.time() + BOOT_WAIT
    while time.time() < end:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            print(f"  < {line}")
        if line == "READY":
            ser.reset_input_buffer()
            return
    time.sleep(3)
    ser.reset_input_buffer()


def drain_queue():
    while not rx_queue.empty():
        try:
            rx_queue.get_nowait()
        except queue.Empty:
            break


def listener(ser, stop):
    """Background reader: assembles complete lines, prints TRIP events at once,
    queues all other lines for the command replies."""
    buf = ""
    while not stop.is_set():
        try:
            data = ser.read(ser.in_waiting or 1)
        except (serial.SerialException, OSError):
            break
        if not data:
            continue
        buf += data.decode(errors="ignore")
        while "\n" in buf:                      # only act on complete lines
            line, buf = buf.split("\n", 1)
            text = line.strip()
            if not text:
                continue
            if '"event"' in text:
                print(f"\n  !! {text}\ncmd> ", end="", flush=True)
            else:
                rx_queue.put(text)


def read_reply():
    """Collect lines until the board stays quiet for IDLE_TIMEOUT seconds."""
    lines = []
    start = last = time.time()
    while time.time() - start < MAX_REPLY_TIME:
        try:
            text = rx_queue.get(timeout=0.1)
        except queue.Empty:
            if lines and time.time() - last > IDLE_TIMEOUT:
                break
            continue
        lines.append(text)
        last = time.time()
        if text == "END":
            break
    return lines


def stream_view(ser):
    """Print every incoming line until Ctrl+C, then turn the stream off."""
    try:
        while True:
            try:
                print(f"  < {rx_queue.get(timeout=0.2)}")
            except queue.Empty:
                pass
    except KeyboardInterrupt:
        pass
    ser.write(b"STREAM OFF\n")
    time.sleep(0.5)
    drain_queue()
    print("\n  (stream stopped)")


def send(ser, cmd, retry=False):
    drain_queue()
    ser.write((cmd.strip() + "\n").encode("ascii"))
    print(f"> {cmd}")

    if cmd.strip().upper() == "STREAM ON":
        print("  (streaming - press Ctrl+C to stop)")
        stream_view(ser)
        return []

    lines = read_reply()

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
    print("Trip events print automatically as '!!' lines. "
          "After STREAM ON, press Ctrl+C to stop the stream.")
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
    stop = threading.Event()
    try:
        wait_for_boot(ser)
        threading.Thread(target=listener, args=(ser, stop), daemon=True).start()
        if "--demo" in sys.argv:
            run_demo(ser)
        else:
            run_interactive(ser)
    finally:
        stop.set()
        time.sleep(0.2)
        ser.close()
        print("Port closed.")


if __name__ == "__main__":
    main()