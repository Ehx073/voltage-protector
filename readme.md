# Microcontroller-Based Voltage Protector

FSMB Recruitment 2026, Phase 1 - Embedded System Engineer
Author: Ehsan Mahmud Sourav

An Arduino Uno based mains voltage/current protector, simulated in **Proteus 8 Professional**. It disconnects the load through a relay on surge, over-voltage, under-voltage, short circuit and overload. Settings can be changed on the device (3 buttons + LCD) or from a computer over serial (Python tool). **Simulation only, nothing is connected to real mains.**

## Repository structure

```
src/                   firmware (voltage_protector.ino) + PC tool (protector_cli.py)
design_document.pdf    one-page project report
schematics/            schematic / block diagram
test_evidence/         screenshots, logs, timing captures
project_files/         Proteus simulation project
readme.md              this file
```

## Pin map (Arduino Uno)

| Function | Pin |
|---|---|
| Voltage sense (biased AC) | A0 |
| Current sense (0.05 ohm shunt, biased AC) | A1 |
| Relay | D8 (HIGH = load connected) |
| Green LED (NORMAL) | D9 |
| Red LED (WAITING / FAULT) | D10 |
| Buzzer (FAULT) | D11 |
| Reset button (to GND) | D12 |
| LCD 16x2 RS, E, D4-D7 | D2, D3, D4, D5, D6, D7 |
| Menu SELECT / UP / DOWN (to GND) | A4 / A2 / A3 |
| Serial (to COMPIM) | D1 (TX), D0 (RX), 9600 baud |

## How to run

**Needs:** Proteus 8 Professional, VSPE (virtual serial port emulator), Python 3 with `pyserial`.

1. Open the Proteus project in `project_files/`.
2. Load the compiled firmware into the Arduino: build `src/voltage_protector.ino` in the Arduino IDE (Verify), then point the Arduino component in Proteus to the generated `.hex` file.
3. In VSPE create a **Pair** device with COM1 <-> COM2 and start it.
4. In Proteus the COMPIM component uses **COM1, 9600 baud, 8N1**. Wiring is straight-through: Arduino TX (D1) to COMPIM TXD, Arduino RX (D0) to COMPIM RXD. Do not connect a Virtual Terminal at the same time.
5. Start the simulation.
6. Install and run the PC tool (it uses COM2; edit `PORT` at the top of the script if needed):

```
pip install pyserial
python src/protector_cli.py            # interactive: type commands
python src/protector_cli.py --demo     # runs 8 test commands
```

The sketch prints `READY` when its 1 s splash screen ends; the tool waits for it.

## Serial commands (9600 baud, newline terminated)

| Command | Meaning |
|---|---|
| `GET STATUS` | one JSON line: voltage, current, power, state, cause |
| `STREAM ON` / `STREAM OFF` | status every 500 ms (Ctrl+C in the tool stops it) |
| `SET OVP <230-280>` | over-voltage limit, V |
| `SET UVP <100-210>` | under-voltage limit, V |
| `SET OCP <1-25>` | overload current limit, A |
| `SET DELAY <1-600>` | reconnect delay, s |
| `RELAY ON` / `RELAY OFF` | clear latch / manual disconnect |
| `GET LOG` / `CLEAR LOG` | last 5 trips |

Out-of-range values reply `ERR RANGE`; unknown commands reply `ERR CMD`. Every trip also sends a JSON `TRIP` event automatically.

## On-device menu

SELECT opens the menu and steps OVP -> UVP -> OCP -> DELAY -> save and exit. UP/DOWN change the value (OVP/UVP 1 V, OCP 0.5 A, DELAY 1 s, or 10 s when held for more than 2 s). It saves and exits after 10 s without a key. Protection keeps running while the menu is open.

## Protections

| Fault | Trigger | Response | Recovery |
|---|---|---|---|
| Surge | > 270 V RMS or > 400 V peak | instant | automatic |
| Over-voltage | > OVP (default 250 V) | 5 cycles | automatic |
| Under-voltage | < UVP (default 180 V) | 25 cycles | automatic |
| Short circuit | > 30 A | instant | latched |
| Heavy overload | > 15 A | 5 cycles | latched |
| Overload | > OCP (default 10 A) | inverse-time | latched |

Automatic faults reconnect after 190-240 V is stable for the reconnect delay. Latched faults need the reset button or `RELAY ON`.

## Known limitations

See the "not done or not good enough yet" section of `design_document.pdf`. In short: first command right after boot may return `ERR CMD` (unconfirmed), reconnect window is hard-coded, menu changes save only on exit, `String` class used in the parser, calibration constants not verified against a true-RMS meter, and a real product needs isolation, a fuse and a correctly rated relay.

## Warning

Please Do not use multiple voltage source simultaneously. These sources are only for creating demo fault. Please turn off one source then state goes to normal and then connect to another source
