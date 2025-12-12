# nmea_server – ERLE Brain 3 / Raspberry Pi NMEA 0183 Sensor Hub

This project turns a DietPi-based Raspberry Pi + ERLE Brain 3–style HAT into a
small NMEA 0183 sensor hub for OpenCPN (or any NMEA-aware client).

It reads:

- **GPS** on `/dev/ttyS0` @ **38400 baud** (NMEA sentences pass-through)
- **Compass** QMC5883L on **I²C 0x0d** → NMEA `$HCHDG` (magnetic heading)
- **ADC** ADS1015 on **I²C 0x48** → NMEA `XDR` for battery **voltage** and **current**
- **Servo driver** PCA9685 on **I²C 0x40/0x70** → rudder/servo outputs controlled over TCP

All NMEA is broadcast over **TCP port 10110**, which OpenCPN can consume
directly as a “Network / TCP” connection.

This README assumes the code lives in `/home/dietpi/nmea_server`
on a DietPi system, but paths/usernames are easy to adjust.

---

## Hardware assumptions

- DietPi / Raspberry Pi with:

  - `/dev/ttyS0` wired to the GPS module (38400 baud, NMEA output)
  - I²C bus 1 enabled and `i2cdetect -y 1` shows at least:
    - `0x0d` → QMC5883L magnetometer
    - `0x40` and `0x70` → PCA9685 PWM/servo driver
    - `0x48` → ADS1015 ADC

- Serial console disabled on `/dev/ttyS0` so the GPS has exclusive use of it.
- I²C enabled in `dietpi-config`.

---

## Directory layout

All project files live in one folder, e.g. `/home/dietpi/nmea_server`:

```text
nmea_server/
├── nmea_server.py         # main async NMEA server
├── compass_calibrate.py   # helper to calibrate the QMC5883L
├── requirements.txt       # Python dependencies (for venv)
├── smbus.py               # shim so py_qmc5883l can use smbus2 in the venv
├── nmea-server.service    # example systemd unit
├── .gitignore             # ignores .venv, __pycache__, etc.
└── .venv/                 # your Python virtualenv (ignored by git)
```

You’ll create the Python virtualenv in `./.venv` (ignored by git).

---

## Installation (DietPi / Raspberry Pi)

### 1. OS packages

Install the basics:

```bash
sudo apt update
sudo apt install -y \
  python3 python3-venv python3-pip \
  python3-smbus build-essential python3-dev git
```

Give your user access to serial + I²C (assuming user `dietpi`):

```bash
sudo usermod -aG dialout,i2c dietpi
# log out and log back in so group membership updates
```

### 2. Get the code and set up the virtualenv

If you’re using git:

```bash
cd /home/dietpi
git clone <your-github-url> nmea_server
cd nmea_server
```

Or copy/unzip the project into `/home/dietpi/nmea_server` yourself, then:

```bash
cd /home/dietpi/nmea_server

python3 -m venv .venv

./.venv/bin/pip install --upgrade pip
./.venv/bin/pip install -r requirements.txt
```

> `requirements.txt` includes `smbus2` and uses a `git+https://...` URL to
> install `py-qmc5883l` directly from GitHub.

### 3. First-run test

From `/home/dietpi/nmea_server`:

```bash
./.venv/bin/python nmea_server.py
```

You should see logs like:

```text
Compass (QMC5883L) initialized on I2C bus 1, addr 0x0d
PCA9685 servo driver initialized at 0x40 on bus 1
ADS1015 ADC initialized at 0x48
NMEA server listening on: ('0.0.0.0', 10110)
GPS: opened /dev/ttyS0 @ 38400
```

In another terminal on the Pi, check the TCP stream locally:

```bash
nc 127.0.0.1 10110
```

You should see raw NMEA scrolling:

```text
$GNRMC,...
$GNGGA,...
$HCHDG,123.45,,,,*hh
$IIXDR,V,12.34,V,BATV*hh
$IIXDR,C,23.10,A,BATI*hh
...
```

Press `Ctrl-C` in the first terminal to stop the script after testing.

---

## Running as a systemd service

The repo includes an example unit file `nmea-server.service`. Copy it into
`/etc/systemd/system` and enable it:

```bash
sudo cp /home/dietpi/nmea_server/nmea-server.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now nmea-server.service
```

Check status and logs:

```bash
sudo systemctl status nmea-server.service
sudo journalctl -u nmea-server.service -f
```

Sample logs when things are healthy:

```text
[INFO] Compass (QMC5883L) initialized on I2C bus 1, addr 0x0d
[INFO] PCA9685 servo driver initialized at 0x40 on bus 1
[INFO] ADS1015 ADC initialized at 0x48
[INFO] NMEA server listening on: ('0.0.0.0', 10110)
[INFO] GPS: opened /dev/ttyS0 @ 38400
[INFO] GPS: sent 548 NMEA sentences in last 30s; last='$GNRMC,...'
[INFO] Compass: raw=(...), corrected=...
[INFO] ADC: V=12.45 V  I=2.13 A
```

When a client connects, you’ll see:

```text
[INFO] Client connected: ('192.168.x.x', 54321)
```

---

## OpenCPN configuration

In OpenCPN (laptop/tablet on the same LAN):

1. **Options → Connections → Add**
2. **Type:** Network
3. **Protocol:** TCP
4. **Address:** IP address of the Pi (e.g. `192.168.1.50`)
5. **DataPort:** `10110`
6. Ensure “Input” is checked

Enable the NMEA Debug Window in OpenCPN if you want to see the exact sentences.

> **Important:** The server binds to `0.0.0.0:10110`, so any host on the same
> network can connect. If you open this port to the wider internet, consider
> using a VPN or firewall rules; the protocol is plain NMEA over TCP with no
> authentication.

---

## NMEA sentences produced

### GPS

All `$GP...` / `$GN...` sentences are passed through unchanged from
`/dev/ttyS0` at 38400 baud. Typical examples:

- `$GNRMC` – Recommended Minimum data (position, SOG, COG)
- `$GNGGA` – Fix data (lat, lon, alt)

### Compass (heading)

The QMC5883L compass produces:

- `$HCHDG,<heading>,,,,*hh`

Where:

- `heading` is in **degrees magnetic** (`0–360`)
- The Pi **does not** apply declination by default; OpenCPN’s WMM handles this.

If you ever want the Pi to apply declination itself, you can set
`COMPASS_DECLINATION_DEG` in `nmea_server.py`, but for OpenCPN it’s better to
leave that as `None` and let OpenCPN do it.

### ADC (battery voltage/current)

Two `XDR` sentences are emitted every `ADS_POLL_INTERVAL` seconds:

- `$IIXDR,V,<volts>,V,BATV*hh`  – battery voltage
- `$IIXDR,C,<amps>,A,BATI*hh`  – battery current

Scaling is controlled by:

```python
ADS_VOLT_SCALE = 11.0  # ADC volts -> real battery volts
ADS_CURR_SCALE = 1.0   # ADC volts -> Amps
```

You should tune those to match your actual voltage divider and shunt/amp.

---

## Servo control over TCP (PCA9685)

The PCA9685 driver is initialized on I²C address `0x40` on bus 1 and driven at
50 Hz (standard hobby servo frequency). The code exposes a small TCP command
protocol **on the same port 10110** alongside NMEA.

Commands are simple ASCII lines that do **not** start with `$` (so they’re not
NMEA sentences) and are ignored by NMEA clients:

### Command format

Send a line:

```text
!SRV,<channel>,<angle_deg>\n
```

Where:

- `channel` is the PCA9685 channel `0..15`
- `angle_deg` is a float `0..180` (angles outside this range are clamped)

Example (from another Linux box on the LAN):

```bash
printf '!SRV,0,90\n' | nc <PI_IP> 10110
```

This will move channel 0 to ~90° using the mapping:

```python
SERVO_MIN_US = 1000  # 0°
SERVO_MAX_US = 2000  # 180°
```

You can tune those in `nmea_server.py` for your specific servos.

> **Power note:** PCA9685 `V+` for servos must come from a proper 5–6 V supply
> with common ground to the Pi. Do **not** attempt to drive multiple servos
> directly from the Pi’s 5 V pin.

---

## Compass calibration

Because sensor mounting, local magnetic environment, and wiring can distort the
magnetometer, `nmea_server.py` supports a 2D calibration (hard-iron offset +
scale) for X and Y.

### 1. Run the calibration script

Stop the NMEA service while calibrating:

```bash
sudo systemctl stop nmea-server.service
cd /home/dietpi/nmea_server
./.venv/bin/python compass_calibrate.py
```

For ~60 seconds:

- Slowly rotate the board/boat through all headings
- Draw big circles and lazy figure-8s so the sensor sees all directions

At the end you’ll see output like:

```text
Suggested nmea_server.py settings:
  COMPASS_USE_CALIB = True
  COMPASS_X_OFFSET = 1098.500
  COMPASS_Y_OFFSET = 1246.000
  COMPASS_X_SCALE  = 0.929910
  COMPASS_Y_SCALE  = 1.081517
```

(Those numbers are from one specific sensor; you should re-run calibration for
your hardware and copy your own values into `nmea_server.py`.)

### 2. Paste values into `nmea_server.py`

At the top of `nmea_server.py` set:

```python
COMPASS_USE_CALIB = True
COMPASS_X_OFFSET = 1098.500
COMPASS_Y_OFFSET = 1246.000
COMPASS_X_SCALE  = 0.929910
COMPASS_Y_SCALE  = 1.081517
```

### 3. Direction sanity check

After restarting the service:

```bash
sudo systemctl restart nmea-server.service
sudo journalctl -u nmea-server.service -f
```

You’ll see logs like:

```text
Compass: raw=(..., ...) corrected=NNN.NN°
```

- Point the bow roughly **north** → corrected heading should be near 0°.
- Turn the bow **to the right** (clockwise, toward east) → heading should
  increase toward 90°.
- Turn **left** (counter-clockwise, toward west) → heading should decrease
  toward 270°.

If right/left feel swapped, toggle:

```python
COMPASS_REVERSE = False  # instead of True
```

If everything is consistent but off by a small constant, tweak:

```python
COMPASS_OFFSET_DEG = <small positive/negative trim>
```

until it matches a real compass / phone.

---

## ADC scaling & tuning

Voltage and current readings depend on your resistor divider (for voltage) and
your shunt + amplifier (for current).

Given:

- `V_adc` = voltage at ADC channel
- `V_batt` = real battery voltage
- `I` = current

You configure:

```python
ADS_VOLT_SCALE = V_batt / V_adc
ADS_CURR_SCALE = I / V_adc
```

### Example

If:

- Your resistor divider gives 2.0 V at ADC when battery is 12.0 V → `V_batt/V_adc = 6`
- Your shunt/amp gives 0.1 V per Amp at the ADC pin → `I/V_adc = 10`

Then:

```python
ADS_VOLT_SCALE = 6.0
ADS_CURR_SCALE = 10.0
```

After changing these, restart the service and compare against a multimeter or
clamp meter.

---

## Logging & troubleshooting

### View logs

Follow NMEA server logs live:

```bash
sudo journalctl -u nmea-server.service -f
```

You’ll see periodic summaries like:

- `GPS: sent ... NMEA sentences in last 30s; last='...'`
- `Compass: raw=(...), corrected=...`
- `ADC: V=... I=...`
- Servo commands like `[servo] command: channel=0 angle=90.0`

### Test from the Pi

Check GPS device manually:

```bash
sudo stty -F /dev/ttyS0 38400 -echo -icanon
sudo cat /dev/ttyS0
```

You should see `$GNRMC`, `$GNGGA`, etc.

Check TCP stream locally:

```bash
nc 127.0.0.1 10110
```

If you see NMEA here but not on your laptop, it’s a network/firewall issue.

### Common issues

- **No GPS data in OpenCPN**
  - Confirm `/dev/ttyS0` shows NMEA with `cat` as above.
  - Make sure the serial console is disabled on `/dev/ttyS0` via `dietpi-config`.
  - Check OpenCPN connection IP/port (must be `<Pi_IP>:10110`, TCP).

- **No `$HCHDG` sentences**
  - `sudo i2cdetect -y 1` should show `0x0d`.
  - Check for compass-related warnings in the logs.
  - Re-run calibration if needed.

- **PCA9685 init errors**
  - `sudo i2cdetect -y 1` should show `0x40` and `0x70`.
  - Ensure PCA9685 is powered (3.3V logic, 5–6 V on V+ as per board).

- **ADC values stuck or nonsense**
  - `sudo i2cdetect -y 1` should show `0x48`.
  - Verify wiring and scaling values in `nmea_server.py`.

---

## Git usage

From `/home/dietpi/nmea_server`:

```bash
git init
git add .
git commit -m "Initial nmea_server for ERLE Brain 3"
git branch -M main
git remote add origin <your-github-url>
git push -u origin main
```

You now have:

- A self-contained NMEA sensor hub on the Pi
- A GitHub repo with clear setup docs
- A calibration flow for the compass
- Logging and TCP access suitable for OpenCPN, custom dashboards, or your
  future autopilot logic.

---

## License

Pick whatever you like (MIT is a solid default). This code is simple and meant
to be hacked on for personal boating projects.
