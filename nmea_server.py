#!/usr/bin/env python3
"""
nmea_server.py

Async NMEA 0183 sensor hub for a Raspberry Pi + ERLE Brain 3 HAT.
Features:

- Reads GPS NMEA from /dev/ttyS0 @ 38400 baud and rebroadcasts it over TCP.
- Reads heading from a QMC5883L magnetometer on I²C (0x0d) and outputs $HCHDG.
- Reads battery voltage/current from an ADS1015 ADC on I²C (0x48) and outputs XDR.
- Accepts simple servo commands for a PCA9685 PWM driver on I²C (0x40).

All NMEA goes out on TCP port 10110, suitable for OpenCPN.
"""

import asyncio
import logging
import math
import time

import serial  # pyserial

# Try compass library
try:
    import py_qmc5883l
    HAVE_QMC = True
except ImportError:
    py_qmc5883l = None
    HAVE_QMC = False

# Try PCA9685 servo driver
try:
    import Adafruit_PCA9685
    HAVE_PCA9685 = True
except ImportError:
    Adafruit_PCA9685 = None
    HAVE_PCA9685 = False

# Try ADS1015 / ADS1115 ADC driver
try:
    import Adafruit_ADS1x15
    HAVE_ADS = True
except ImportError:
    Adafruit_ADS1x15 = None
    HAVE_ADS = False

# --- CONFIG -----------------------------------------------------------------

NMEA_PORT = 10110                   # TCP port to serve NMEA on
GPS_DEVICE = "/dev/ttyS0"           # Serial device for GPS
GPS_BAUD = 38400                    # GPS baud you found

# Compass (QMC5883L at 0x0d). We output *magnetic* heading and let OpenCPN
# handle declination using its own WMM model.
COMPASS_DECLINATION_DEG = None      # leave None; OpenCPN will handle WMM
COMPASS_OFFSET_DEG = 0.0            # extra yaw trim after calibration
COMPASS_REVERSE = True              # flip direction if right/left feel swapped

# 2D magnetometer calibration (values below are example values from one sensor
# and should be re-generated with compass_calibrate.py for each installation).
COMPASS_USE_CALIB = True
COMPASS_X_OFFSET = 1098.500
COMPASS_Y_OFFSET = 1246.000
COMPASS_X_SCALE  = 0.929910
COMPASS_Y_SCALE  = 1.081517

# Servo (PCA9685 at 0x40 / 0x70)
SERVO_I2C_ADDRESS = 0x40
SERVO_FREQ_HZ = 50                  # good for hobby servos
SERVO_MIN_US = 1000                 # pulse width at 0 deg
SERVO_MAX_US = 2000                 # pulse width at 180 deg

# ADC (ADS1015 at 0x48)
ADS_I2C_ADDRESS = 0x48
ADS_GAIN = 1                        # +/- 4.096V full scale
ADS_FULL_SCALE_V = 4.096            # volts at full-scale for GAIN=1 on ADS1015
ADS_VOLT_CHANNEL = 0                # A0: battery voltage divider input
ADS_VOLT_SCALE = 11.0               # multiply measured volts by this (tune for your divider)
ADS_CURR_CHANNEL = 1                # A1: shunt amplifier output
ADS_CURR_SCALE = 1.0                # amps per volt at ADC (tune for your shunt/amp)
ADS_POLL_INTERVAL = 1.0             # seconds between ADC reads

# Logging: how often to emit “we’re alive” summaries from each task
LOG_INTERVAL_SEC = 30.0

# ---------------------------------------------------------------------------

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger("nmea_server")

clients = set()


def make_nmea(body: str) -> str:
    """Build a full NMEA sentence with checksum and CRLF from the body
    (body should NOT include '$' or checksum)."""
    checksum = 0
    for ch in body.encode("ascii", errors="ignore"):
        checksum ^= ch
    return f"${body}*{checksum:02X}\r\n"


async def broadcast(line: str) -> None:
    """Send a pre-built NMEA line (with CRLF) to all connected clients."""
    if not clients:
        return

    dead = set()
    data = line.encode("ascii", errors="ignore")
    for w in clients:
        try:
            w.write(data)
            await w.drain()
        except Exception:
            dead.add(w)

    for w in dead:
        try:
            w.close()
        except Exception:
            pass
        clients.discard(w)


# --- GPS / UART ------------------------------------------------------------

async def gps_task():
    """Read raw NMEA from the GPS UART and rebroadcast it as-is."""
    while True:
        try:
            with serial.Serial(GPS_DEVICE, GPS_BAUD, timeout=1) as ser:
                log.info("GPS: opened %s @ %d", GPS_DEVICE, GPS_BAUD)
                last_log = time.time()
                sent_count = 0
                last_text = ""
                while True:
                    raw = ser.readline()
                    if not raw:
                        await asyncio.sleep(0.05)
                        continue

                    text = raw.decode("ascii", errors="ignore").strip()
                    if not text:
                        await asyncio.sleep(0)
                        continue
                    if not text.startswith("$"):
                        # ignore any non-NMEA chatter/binary
                        await asyncio.sleep(0)
                        continue

                    line = text + "\r\n"  # ensure CRLF
                    await broadcast(line)

                    sent_count += 1
                    last_text = text
                    now = time.time()
                    if now - last_log >= LOG_INTERVAL_SEC:
                        log.info(
                            "GPS: sent %d NMEA sentences in last %ds; last='%s...'",
                            sent_count,
                            int(now - last_log),
                            (last_text[:60] if last_text else ""),
                        )
                        sent_count = 0
                        last_log = now

                    # Always yield so other tasks (TCP server, compass, ADC) run
                    await asyncio.sleep(0)

        except Exception as e:
            log.warning("[gps_task] serial error: %r - retrying in 2s", e)
            await asyncio.sleep(2.0)


# --- COMPASS / QMC5883L ----------------------------------------------------

COMPASS_OK = False
compass = None

if HAVE_QMC:
    try:
        compass = py_qmc5883l.QMC5883L()   # I2C bus 1, address 0x0d
        COMPASS_OK = True
        log.info("Compass (QMC5883L) initialized on I2C bus 1, addr 0x0d")
    except Exception as e:
        log.warning("Compass init failed: %r", e)
        COMPASS_OK = False
else:
    log.info("py_qmc5883l not installed; compass disabled.")


async def imu_task():
    """Poll QMC5883L, apply calibration, send NMEA HDG sentences."""
    if not COMPASS_OK or compass is None:
        log.info("Compass task: disabled (no compass available)")
        while True:
            await asyncio.sleep(5)

    last_log = time.time()
    last_heading = None
    last_raw = None

    while True:
        try:
            # QMC driver returns [x, y] or (x, y)
            m = compass.get_magnet()
            if not isinstance(m, (list, tuple)) or len(m) < 2:
                await asyncio.sleep(0.2)
                continue

            mx, my = float(m[0]), float(m[1])
            last_raw = (mx, my)

            # Apply calibration: hard-iron offset + scale
            if COMPASS_USE_CALIB:
                mx = (mx - COMPASS_X_OFFSET) * COMPASS_X_SCALE
                my = (my - COMPASS_Y_OFFSET) * COMPASS_Y_SCALE

            # Heading from calibrated X/Y
            # 0° = +X (North-ish), angle increases CCW; wrap 0–360
            heading = math.degrees(math.atan2(my, mx))
            heading = (heading + 360.0) % 360.0

            # If right/left feel swapped, mirror it
            if COMPASS_REVERSE:
                heading = (-heading) % 360.0

            # Extra trim + declination (if ever used)
            if COMPASS_OFFSET_DEG:
                heading = (heading + COMPASS_OFFSET_DEG) % 360.0
            if COMPASS_DECLINATION_DEG is not None:
                heading = (heading + COMPASS_DECLINATION_DEG) % 360.0

            body = f"HCHDG,{heading:06.2f},,,,"
            line = make_nmea(body)
            await broadcast(line)

            last_heading = heading

        except Exception as e:
            log.warning("[imu_task] compass error: %r", e)

        now = time.time()
        if (
            now - last_log >= LOG_INTERVAL_SEC
            and last_heading is not None
            and last_raw is not None
        ):
            log.info(
                "Compass: raw=(%.1f,%.1f) corrected=%.2f°",
                last_raw[0], last_raw[1], last_heading,
            )
            last_log = now

        await asyncio.sleep(0.2)  # ~5 Hz


# --- SERVO / PCA9685 -------------------------------------------------------

pca = None
if HAVE_PCA9685:
    try:
        # Force I2C bus 1 instead of auto-detect
        pca = Adafruit_PCA9685.PCA9685(address=SERVO_I2C_ADDRESS, busnum=1)
        pca.set_pwm_freq(SERVO_FREQ_HZ)
        log.info(
            "PCA9685 servo driver initialized at 0x%02X on bus 1",
            SERVO_I2C_ADDRESS,
        )
    except Exception as e:
        log.warning("PCA9685 init failed: %r", e)
        pca = None
else:
    log.info("Adafruit_PCA9685 not installed; servo driver disabled.")


def us_to_ticks(us: float) -> int:
    """Convert microseconds of pulse width to PCA9685 ticks (0–4095)."""
    period_us = 1_000_000.0 / SERVO_FREQ_HZ  # e.g. 20_000 us at 50 Hz
    ticks = int(us * 4096.0 / period_us)
    return max(0, min(4095, ticks))


def set_servo_angle(channel: int, angle_deg: float):
    """Set servo angle (0–180 deg) on given PCA9685 channel."""
    if pca is None:
        log.debug("[servo] PCA9685 not available")
        return
    if not (0 <= channel <= 15):
        log.warning("[servo] invalid channel %s", channel)
        return
    angle_deg = max(0.0, min(180.0, angle_deg))
    pulse_us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * (angle_deg / 180.0)
    ticks = us_to_ticks(pulse_us)
    pca.set_pwm(channel, 0, ticks)
    log.info(
        "[servo] ch=%d angle=%.1fdeg pulse=%dus ticks=%d",
        channel,
        angle_deg,
        int(pulse_us),
        ticks,
    )


def handle_servo_command(cmd: str):
    """Parse a simple servo command: !SRV,<channel>,<angle_deg>"""
    if not cmd.startswith("!SRV"):
        return
    parts = cmd.split(",")
    if len(parts) != 3:
        log.warning("[servo] bad command format: %r", cmd)
        return
    try:
        ch = int(parts[1])
        angle = float(parts[2])
    except ValueError:
        log.warning("[servo] invalid number(s) in: %r", cmd)
        return
    log.info("[servo] command: channel=%d angle=%.1f", ch, angle)
    set_servo_angle(ch, angle)


# --- ADC / ADS1015 ---------------------------------------------------------

adc = None
if HAVE_ADS:
    try:
        adc = Adafruit_ADS1x15.ADS1015(address=ADS_I2C_ADDRESS, busnum=1)
        log.info("ADS1015 ADC initialized at 0x%02X", ADS_I2C_ADDRESS)
    except Exception as e:
        log.warning("ADS1015 init failed: %r", e)
        adc = None
else:
    log.info("Adafruit_ADS1x15 not installed; ADC disabled.")


def ads_raw_to_volts(raw: int) -> float:
    """Convert ADS1015 raw value to volts at the ADC pin for current GAIN."""
    # ADS1015 is 12-bit, -2048..2047 range for +/- ADS_FULL_SCALE_V
    return (raw * ADS_FULL_SCALE_V) / 2048.0


async def ads_task():
    """Poll ADS1015 for voltage/current and send NMEA XDR sentences."""
    if adc is None:
        log.info("ADC task: disabled (no ADS1015 available)")
        while True:
            await asyncio.sleep(10)

    last_log = time.time()
    last_v = None
    last_i = None

    while True:
        try:
            raw_v = adc.read_adc(ADS_VOLT_CHANNEL, gain=ADS_GAIN)
            raw_i = adc.read_adc(ADS_CURR_CHANNEL, gain=ADS_GAIN)

            volts_v = ads_raw_to_volts(raw_v)
            volts_i = ads_raw_to_volts(raw_i)

            batt_v = volts_v * ADS_VOLT_SCALE
            batt_i = volts_i * ADS_CURR_SCALE

            last_v = batt_v
            last_i = batt_i

            body_v = f"IIXDR,V,{batt_v:0.2f},V,BATV"
            body_i = f"IIXDR,C,{batt_i:0.2f},A,BATI"

            await broadcast(make_nmea(body_v))
            await broadcast(make_nmea(body_i))
        except Exception as e:
            log.warning("[ads_task] ADC error: %r", e)

        now = time.time()
        if (
            now - last_log >= LOG_INTERVAL_SEC
            and last_v is not None
            and last_i is not None
        ):
            log.info("ADC: V=%.2f V  I=%.2f A", last_v, last_i)
            last_log = now

        await asyncio.sleep(ADS_POLL_INTERVAL)


# --- TCP SERVER ------------------------------------------------------------

async def handle_client(reader: asyncio.StreamReader,
                        writer: asyncio.StreamWriter):
    addr = writer.get_extra_info("peername")
    log.info("Client connected: %s", (addr,))
    clients.add(writer)
    buffer = b""
    try:
        while True:
            data = await reader.read(1024)
            if not data:
                break
            buffer += data
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                text = line.decode("ascii", errors="ignore").strip()
                if text.startswith("!SRV"):
                    handle_servo_command(text)
    finally:
        log.info("Client disconnected: %s", (addr,))
        clients.discard(writer)
        try:
            writer.close()
            await writer.wait_closed()
        except Exception:
            pass


async def main():
    server = await asyncio.start_server(
        handle_client, "0.0.0.0", NMEA_PORT
    )
    addrs = ", ".join(str(sock.getsockname()) for sock in server.sockets)
    log.info("NMEA server listening on: %s", addrs)
    async with server:
        await asyncio.gather(
            server.serve_forever(),
            gps_task(),
            imu_task(),
            ads_task(),
        )


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        log.info("Exiting.")
