#!/usr/bin/env python3
"""
Smart Helmet + Fall & Helmet-Removal Detection System
=====================================================
ENGG1101 Project — Raspberry Pi 5

Components & GPIO Pin Map:
--------------------------
  HC-SR04 Ultrasonic  : Trig = GPIO23, Echo = GPIO24
  DHT22 (Temp/Humid)  : GPIO21
  MPU6050 IMU (I2C)   : SDA = GPIO2, SCL = GPIO3  (addr 0x68)
  VL53L0X ToF (I2C)   : SDA = GPIO2, SCL = GPIO3  (addr 0x29)
  BH1750 Light (I2C)  : SDA = GPIO2, SCL = GPIO3  (addr 0x23)
  SSD1306 OLED (I2C)  : SDA = GPIO2, SCL = GPIO3  (addr 0x3C)
  Touch Sensor         : GPIO18
  IR Sensor            : GPIO20
  KY-006 Buzzer        : GPIO17
"""

import time
import math
import threading
import struct
import signal
import sys
import os
import json
from datetime import datetime, timedelta, timezone

# ─── GPIO (works on Raspberry Pi 5 via lgpio / gpiozero) ───
from gpiozero import DigitalOutputDevice, DigitalInputDevice, TonalBuzzer
from gpiozero.tones import Tone
import board
import busio

# ─── I2C sensor libraries ───
import adafruit_ssd1306          # OLED display
import adafruit_vl53l0x          # ToF distance sensor
import adafruit_dht              # DHT22 temperature / humidity

from PIL import Image, ImageDraw, ImageFont

# ═══════════════════════════════════════════════════════════════
#  CONFIGURATION — tweak thresholds here
# ═══════════════════════════════════════════════════════════════

# --- GPIO Pin Assignments ---
PIN_TRIG        = 23      # HC-SR04 trigger
PIN_ECHO        = 24      # HC-SR04 echo
PIN_DHT22       = 21      # DHT22 data
PIN_TOUCH       = 18      # Touch sensor (helmet-on detection, active-low)
PIN_IR          = 20      # IR proximity sensor (digital out)
PIN_BUZZER      = 17      # KY-006 passive buzzer

# --- I2C Addresses ---
ADDR_MPU6050    = 0x68
ADDR_VL53L0X    = 0x29    # handled by library
ADDR_BH1750     = 0x23
ADDR_OLED       = 0x3C    # handled by library

# --- Fall Detection (MPU6050) ---
FREEFALL_THRESHOLD_G    = 0.4     # below this total-g → free-fall
IMPACT_THRESHOLD_G      = 3.0     # above this total-g → hard impact
FALL_CONFIRM_MS         = 150     # free-fall must last at least this long
GYRO_TUMBLE_THRESHOLD   = 250.0   # deg/s — rapid rotation = tumbling

# --- Helmet Removal Detection (Touch + IR) ---
HELMET_REMOVAL_SECONDS  = 3       # touch must be absent this long before helmet is marked off

# --- Proximity / Collision Warning ---
ULTRASONIC_WARN_CM      = 100     # warn if obstacle < 100 cm
ULTRASONIC_DANGER_CM    = 40      # danger if obstacle < 40 cm
TOF_WARN_MM             = 600     # VL53L0X warn threshold
TOF_DANGER_MM           = 250     # VL53L0X danger threshold
IR_REAR_DETECT_ACTIVE_LOW = True  # rear IR sensor is active-low when an object is detected

# --- Heat Stress (DHT22) ---
HEAT_INDEX_CAUTION      = 27.0    # °C — caution
HEAT_INDEX_DANGER       = 32.0    # °C — danger, take a break
HEAT_INDEX_EXTREME      = 40.0    # °C — extreme danger

# --- Light Level (BH1750) ---
LOW_LIGHT_LUX           = 50      # below this → poor visibility warning

# --- Timing ---
MAIN_LOOP_INTERVAL      = 0.3     # seconds between sensor reads
OLED_REFRESH_INTERVAL   = 0.5     # seconds between display updates
BUZZER_SHORT_BEEP       = 0.1
BUZZER_LONG_BEEP        = 0.5

ALERT_HISTORY_FILE      = os.path.join(os.path.dirname(__file__), "alert_history.json")
ALERT_HISTORY_DAYS      = 7


# ═══════════════════════════════════════════════════════════════
#  SENSOR DRIVERS
# ═══════════════════════════════════════════════════════════════

# -------- I2C bus (shared) --------
i2c = busio.I2C(board.SCL, board.SDA)


class SensorUnavailableError(Exception):
    """Raised when a sensor should be disabled after repeated hardware failures."""


# ===================== MPU6050 (raw I2C) =====================
class MPU6050:
    """Minimal driver for MPU6050 accelerometer + gyroscope."""

    PWR_MGMT_1   = 0x6B
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H  = 0x43
    ACCEL_SCALE  = 16384.0   # ±2 g default
    GYRO_SCALE   = 131.0     # ±250 °/s default

    def __init__(self, i2c_bus, address=ADDR_MPU6050):
        self.address = address
        self.i2c = i2c_bus
        # Wake the sensor (clear sleep bit)
        self._write_byte(self.PWR_MGMT_1, 0x00)
        time.sleep(0.1)

    # --- low-level helpers ---
    def _write_byte(self, reg, value):
        while not self.i2c.try_lock():
            pass
        try:
            self.i2c.writeto(self.address, bytes([reg, value]))
        finally:
            self.i2c.unlock()

    def _read_bytes(self, reg, count):
        buf = bytearray(count)
        while not self.i2c.try_lock():
            pass
        try:
            self.i2c.writeto_then_readfrom(self.address, bytes([reg]), buf)
        finally:
            self.i2c.unlock()
        return buf

    # --- high-level reads ---
    def read_accel(self):
        """Return (ax, ay, az) in g."""
        raw = self._read_bytes(self.ACCEL_XOUT_H, 6)
        ax = struct.unpack(">h", raw[0:2])[0] / self.ACCEL_SCALE
        ay = struct.unpack(">h", raw[2:4])[0] / self.ACCEL_SCALE
        az = struct.unpack(">h", raw[4:6])[0] / self.ACCEL_SCALE
        return ax, ay, az

    def read_gyro(self):
        """Return (gx, gy, gz) in °/s."""
        raw = self._read_bytes(self.GYRO_XOUT_H, 6)
        gx = struct.unpack(">h", raw[0:2])[0] / self.GYRO_SCALE
        gy = struct.unpack(">h", raw[2:4])[0] / self.GYRO_SCALE
        gz = struct.unpack(">h", raw[4:6])[0] / self.GYRO_SCALE
        return gx, gy, gz

    def total_g(self):
        ax, ay, az = self.read_accel()
        return math.sqrt(ax*ax + ay*ay + az*az)

    def total_gyro(self):
        gx, gy, gz = self.read_gyro()
        return math.sqrt(gx*gx + gy*gy + gz*gz)


# ===================== BH1750 Light Sensor (raw I2C) =====================
class BH1750:
    """Minimal driver for BH1750 ambient light sensor."""

    POWER_ON       = 0x01
    CONTINUOUS_H   = 0x10   # 1 lx resolution, 120 ms

    def __init__(self, i2c_bus, address=ADDR_BH1750):
        self.address = address
        self.i2c = i2c_bus
        self._command(self.POWER_ON)
        self._command(self.CONTINUOUS_H)
        time.sleep(0.18)  # first measurement ready

    def _command(self, cmd):
        while not self.i2c.try_lock():
            pass
        try:
            self.i2c.writeto(self.address, bytes([cmd]))
        finally:
            self.i2c.unlock()

    def read_lux(self):
        buf = bytearray(2)
        while not self.i2c.try_lock():
            pass
        try:
            self.i2c.readfrom_into(self.address, buf)
        finally:
            self.i2c.unlock()
        raw = (buf[0] << 8) | buf[1]
        return raw / 1.2


# ===================== HC-SR04 Ultrasonic =====================
class HCSR04:
    """Ultrasonic distance sensor using gpiozero for Pi 5 compatibility."""

    def __init__(self, trig_pin, echo_pin):
        self.trig = DigitalOutputDevice(trig_pin)
        self.echo = DigitalInputDevice(echo_pin)

    def read_distance_cm(self, timeout=0.04):
        """Return distance in cm, or -1 on timeout."""
        self.trig.off()
        time.sleep(0.00002)
        self.trig.on()
        time.sleep(0.00001)
        self.trig.off()

        # Wait for echo to go HIGH
        start_wait = time.monotonic()
        while not self.echo.is_active:
            if (time.monotonic() - start_wait) > timeout:
                return -1

        pulse_start = time.monotonic()

        # Wait for echo to go LOW
        while self.echo.is_active:
            if (time.monotonic() - pulse_start) > timeout:
                return -1

        pulse_end = time.monotonic()
        duration = pulse_end - pulse_start
        distance = (duration * 34300) / 2   # speed of sound cm/s
        return round(distance, 1)

    def close(self):
        self.trig.close()
        self.echo.close()


# ═══════════════════════════════════════════════════════════════
#  HEAT INDEX CALCULATION
# ═══════════════════════════════════════════════════════════════

def compute_heat_index(temp_c, humidity):
    """
    Compute the heat index (feels-like temperature) in °C.
    Uses the Rothfusz regression equation (converted from °F formula).
    """
    T = temp_c * 9.0 / 5.0 + 32.0
    RH = humidity

    HI = 0.5 * (T + 61.0 + (T - 68.0) * 1.2 + RH * 0.094)

    if HI >= 80:
        HI = (-42.379
               + 2.04901523 * T
               + 10.14333127 * RH
               - 0.22475541 * T * RH
               - 0.00683783 * T * T
               - 0.05481717 * RH * RH
               + 0.00122874 * T * T * RH
               + 0.00085282 * T * RH * RH
               - 0.00000199 * T * T * RH * RH)

        if RH < 13 and 80 < T < 112:
            HI -= ((13 - RH) / 4) * math.sqrt((17 - abs(T - 95)) / 17)
        elif RH > 85 and 80 < T < 87:
            HI += ((RH - 85) / 10) * ((87 - T) / 5)

    return round((HI - 32) * 5.0 / 9.0, 1)


# ═══════════════════════════════════════════════════════════════
#  ALERT / BUZZER MANAGER
# ═══════════════════════════════════════════════════════════════
class AlertManager:
    """Manages PWM buzzer patterns to avoid overlapping tones."""

    PATTERN_NONE        = 0
    PATTERN_WARN        = 1   # short beep
    PATTERN_DANGER      = 2   # rapid double beep
    PATTERN_FALL        = 3   # fast repeated alarm
    PATTERN_HELMET_OFF  = 4   # repeating long tone

    def __init__(self, buzzer_pin):
        self.buzzer = TonalBuzzer(buzzer_pin, octaves=3)
        self._current = self.PATTERN_NONE
        self._running = True
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def set_pattern(self, pattern):
        with self._lock:
            self._current = pattern

    def _play_tone(self, note="A5", duration=0.2):
        try:
            self.buzzer.play(Tone(note))
            time.sleep(duration)
        finally:
            self.buzzer.stop()

    def _silence(self, duration):
        self.buzzer.stop()
        time.sleep(duration)

    def _loop(self):
        while self._running:
            with self._lock:
                p = self._current

            if p == self.PATTERN_NONE:
                self._silence(0.1)

            elif p == self.PATTERN_WARN:
                self._play_tone("A5", BUZZER_SHORT_BEEP)
                self._silence(0.9)

            elif p == self.PATTERN_DANGER:
                self._play_tone("B6", BUZZER_SHORT_BEEP)
                self._silence(0.08)
                self._play_tone("E6", BUZZER_SHORT_BEEP)
                self._silence(0.55)

            elif p == self.PATTERN_FALL:
                self._play_tone("B6", 0.5)
                self._silence(0.04)
                self._play_tone("E6", 0.5)
                self._silence(0.04)

            elif p == self.PATTERN_HELMET_OFF:
                self._play_tone("B6", BUZZER_LONG_BEEP)
                self._silence(0.5)

    def stop(self):
        self._running = False
        self.buzzer.stop()
        self.buzzer.close()


# ═══════════════════════════════════════════════════════════════
#  OLED DISPLAY MANAGER
# ═══════════════════════════════════════════════════════════════

class OLEDDisplay:
    """Handles the 128x64 SSD1306 OLED display."""

    WIDTH  = 128
    HEIGHT = 64

    def __init__(self, i2c_bus):
        self.oled = adafruit_ssd1306.SSD1306_I2C(self.WIDTH, self.HEIGHT, i2c_bus, addr=ADDR_OLED)
        self.oled.fill(0)
        self.oled.show()
        self.font = ImageFont.load_default()

    def update(self, lines: list[str]):
        """Draw up to 8 lines of text on the display."""
        image = Image.new("1", (self.WIDTH, self.HEIGHT))
        draw = ImageDraw.Draw(image)
        y = 0
        for line in lines[:8]:
            draw.text((0, y), line, font=self.font, fill=255)
            y += 8
        self.oled.image(image)
        self.oled.show()

    def show_alert(self, title, message):
        """Flash a big centred alert."""
        image = Image.new("1", (self.WIDTH, self.HEIGHT))
        draw = ImageDraw.Draw(image)
        draw.rectangle([0, 0, self.WIDTH - 1, self.HEIGHT - 1], outline=255)
        draw.rectangle([2, 2, self.WIDTH - 3, self.HEIGHT - 3], outline=255)
        draw.text((8, 10), title, font=self.font, fill=255)
        draw.text((8, 30), message, font=self.font, fill=255)
        self.oled.image(image)
        self.oled.show()

    def clear(self):
        self.oled.fill(0)
        self.oled.show()


# ═══════════════════════════════════════════════════════════════
#  FALL DETECTION ENGINE
# ═══════════════════════════════════════════════════════════════

class FallDetector:
    """
    Two-phase fall detection:
      1) Free-fall phase  – total g drops below FREEFALL_THRESHOLD_G
      2) Impact phase     – total g spikes above IMPACT_THRESHOLD_G
    Also detects rapid tumbling via gyroscope.
    """

    def __init__(self, imu: MPU6050):
        self.imu = imu
        self._in_freefall = False
        self._freefall_start = 0.0
        self.fall_detected = False

    def update(self):
        g = self.imu.total_g()
        gyro = self.imu.total_gyro()
        now = time.monotonic()

        # Phase 1: detect free-fall
        if g < FREEFALL_THRESHOLD_G:
            if not self._in_freefall:
                self._in_freefall = True
                self._freefall_start = now
        else:
            # Phase 2: check for impact after free-fall
            if self._in_freefall:
                freefall_duration = (now - self._freefall_start) * 1000  # ms
                if freefall_duration >= FALL_CONFIRM_MS and g > IMPACT_THRESHOLD_G:
                    self.fall_detected = True
                self._in_freefall = False

        # Also flag tumbling
        if gyro > GYRO_TUMBLE_THRESHOLD and g > IMPACT_THRESHOLD_G:
            self.fall_detected = True

        return self.fall_detected

    def reset(self):
        self.fall_detected = False
        self._in_freefall = False


# ═══════════════════════════════════════════════════════════════
#  HELMET-REMOVAL DETECTION (Touch Sensor + IR Sensor)
# ═══════════════════════════════════════════════════════════════

class HelmetRemovalDetector:
    """
    Helmet-on/off detection using the touch sensor only.

      Touch sensor (GPIO18) — mounted inside the helmet.
      This wiring is treated as active-low:
        LOW = head contact, HIGH = no contact.

    Rear IR sensor (GPIO20) is mounted on the back of the helmet and is used
    only for rear-proximity detection / telemetry, not helmet-on detection.

    If touch contact is absent continuously for
    HELMET_REMOVAL_SECONDS, the helmet is considered removed.
    """

    def __init__(self, touch_pin, ir_pin):
        # Touch sensor wired active-low: LOW when touching head
        self.touch = DigitalInputDevice(touch_pin, pull_up=True)
        # Rear IR sensor for rear obstacle / presence detection
        self.ir = DigitalInputDevice(ir_pin, pull_up=True)

        self._no_head_since = None
        self.helmet_removed = False

    def _head_present(self):
        """Returns True if the touch sensor detects head contact."""
        touch_active = not self.touch.is_active   # LOW = head touching
        return touch_active

    def rear_object_detected(self):
        """Returns True when the rear IR sensor sees an object behind the helmet."""
        if IR_REAR_DETECT_ACTIVE_LOW:
            return not self.ir.is_active
        return self.ir.is_active

    def update(self):
        now = time.monotonic()

        if self._head_present():
            # At least one sensor sees the head — helmet is on
            self._no_head_since = None
            self.helmet_removed = False
        else:
            # Neither sensor sees the head
            if self._no_head_since is None:
                self._no_head_since = now
            elif (now - self._no_head_since) >= HELMET_REMOVAL_SECONDS:
                self.helmet_removed = True

        return self.helmet_removed

    def close(self):
        self.touch.close()
        self.ir.close()


# ═══════════════════════════════════════════════════════════════
#  MAIN APPLICATION
# ═══════════════════════════════════════════════════════════════

class SmartHelmet:
    """Top-level controller that ties every subsystem together."""

    def __init__(self):
        print("[INIT] Smart Helmet System starting...")

        # ---- I2C Sensors ----
        print("  -> MPU6050 IMU")
        self.imu = MPU6050(i2c)

        print("  -> VL53L0X ToF")
        print("  !! VL53L0X disabled by configuration")
        self.tof = None

        print("  -> BH1750 Light")
        try:
            self.light = BH1750(i2c)
        except Exception as e:
            print(f"  !! BH1750 unavailable: {e}")
            self.light = None

        print("  -> SSD1306 OLED")
        try:
            self.oled = OLEDDisplay(i2c)
        except Exception as e:
            print(f"  !! OLED unavailable: {e}")
            self.oled = None

        # ---- GPIO Sensors ----
        print("  -> HC-SR04 Ultrasonic")
        try:
            self.ultrasonic = HCSR04(PIN_TRIG, PIN_ECHO)
        except Exception as e:
            print(f"  !! Ultrasonic unavailable: {e}")
            self.ultrasonic = None

        print("  -> DHT22 Temp/Humidity")
        try:
            self.dht = adafruit_dht.DHT22(getattr(board, f"D{PIN_DHT22}"))
        except Exception as e:
            print(f"  !! DHT22 unavailable: {e}")
            self.dht = None

        print("  -> Touch Sensor + rear IR sensor")
        try:
            self.helmet_det = HelmetRemovalDetector(PIN_TOUCH, PIN_IR)
        except Exception as e:
            print(f"  !! Helmet sensors unavailable: {e}")
            self.helmet_det = None

        # ---- Output ----
        print("  -> Buzzer")
        try:
            self.alerts = AlertManager(PIN_BUZZER)
        except Exception as e:
            print(f"  !! Buzzer unavailable: {e}")
            self.alerts = None

        # ---- Detection Engines ----
        self.fall_det = FallDetector(self.imu) if self.imu is not None else None

        # ---- State ----
        self.running = True
        self.last_temp = None
        self.last_hum = None
        self.last_heat_index = None
        self.last_lux = None
        self.last_us_dist = None
        self.last_tof_dist = None
        self.alert_level = "OK"  # OK | WARN | DANGER | FALL | HELMET_OFF
        self._last_logged_alert_level = None
        self._error_counts = {
            "imu": 0,
            "tof": 0,
            "light": 0,
            "ultrasonic": 0,
            "dht": 0,
            "helmet": 0,
            "oled": 0,
        }

        print("[INIT] All systems ready.\n")

    def _record_sensor_error(self, sensor_name, error):
        count = self._error_counts.get(sensor_name, 0) + 1
        self._error_counts[sensor_name] = count
        print(f"[WARN] {sensor_name} error #{count}: {error}", flush=True)
        if count >= 5:
            raise SensorUnavailableError(sensor_name)

    def _reset_sensor_error(self, sensor_name):
        self._error_counts[sensor_name] = 0

    def _disable_sensor(self, sensor_name, error=None):
        if error is not None:
            print(f"[WARN] disabling {sensor_name}: {error}", flush=True)
        if sensor_name == "tof":
            self.tof = None
            self.last_tof_dist = None
        elif sensor_name == "light":
            self.light = None
            self.last_lux = None
        elif sensor_name == "ultrasonic":
            self.ultrasonic = None
            self.last_us_dist = None
        elif sensor_name == "dht":
            self.dht = None
            self.last_temp = None
            self.last_hum = None
            self.last_heat_index = None
        elif sensor_name == "helmet":
            self.helmet_det = None
        elif sensor_name == "oled":
            self.oled = None

    # ---- sensor read helpers (with error handling) ----

    def _read_dht(self):
        if self.dht is None:
            return
        try:
            import signal
            def _dht_timeout(signum, frame):
                raise TimeoutError("DHT22 read timeout")
            old = signal.signal(signal.SIGALRM, _dht_timeout)
            signal.alarm(1)  # 1 second timeout
            try:
                t = self.dht.temperature
                h = self.dht.humidity
                signal.alarm(0)
                signal.signal(signal.SIGALRM, old)
                self._reset_sensor_error("dht")
                if t is not None and h is not None:
                    self.last_temp = round(t, 1)
                    self.last_hum = round(h, 1)
                    self.last_heat_index = compute_heat_index(t, h)
            except (TimeoutError, Exception) as e:
                signal.alarm(0)
                signal.signal(signal.SIGALRM, old)
                self._record_sensor_error("dht", e)
        except SensorUnavailableError as e:
            self._disable_sensor("dht", e)
        except Exception:
            pass  # DHT22 unavailable — skip

    def _read_ultrasonic(self):
        if self.ultrasonic is None:
            return
        try:
            d = self.ultrasonic.read_distance_cm()
            self._reset_sensor_error("ultrasonic")
            if d > 0:
                self.last_us_dist = d
        except Exception as e:
            try:
                self._record_sensor_error("ultrasonic", e)
            except SensorUnavailableError as disable_exc:
                self._disable_sensor("ultrasonic", disable_exc)

    def _read_tof(self):
        if self.tof is None:
            return
        try:
            self.last_tof_dist = self.tof.range  # mm
            self._reset_sensor_error("tof")
        except RuntimeError as e:
            try:
                self._record_sensor_error("tof", e)
            except SensorUnavailableError as disable_exc:
                self._disable_sensor("tof", disable_exc)

    def _read_light(self):
        if self.light is None:
            return
        try:
            self.last_lux = round(self.light.read_lux(), 1)
            self._reset_sensor_error("light")
        except Exception as e:
            try:
                self._record_sensor_error("light", e)
            except SensorUnavailableError as disable_exc:
                self._disable_sensor("light", disable_exc)

    # ---- alert history ----

    def _load_alert_history(self):
        try:
            with open(ALERT_HISTORY_FILE, "r") as f:
                rows = json.load(f)
            if isinstance(rows, list):
                return rows
        except Exception:
            pass
        return []

    def _save_alert_history(self, rows):
        try:
            with open(ALERT_HISTORY_FILE, "w") as f:
                json.dump(rows, f)
                f.flush()
                os.fsync(f.fileno())
        except Exception as e:
            print(f"[WARN] could not save alert history: {e}", flush=True)

    def _record_alert_history(self):
        level = self.alert_level
        if level in ("OK", "HELMET OFF"):
            self._last_logged_alert_level = level
            return
        if level == self._last_logged_alert_level:
            return

        category = None
        if level == "!! FALL !!":
            category = "fall"
        elif level.startswith("HEAT"):
            category = "heat"
        elif level in ("PROX WARN", "PROX DANGER", "TOF WARN", "TOF DANGER", "REAR WARN"):
            category = "prox"

        self._last_logged_alert_level = level
        if category is None:
            return

        today = datetime.now(timezone.utc).date().isoformat()
        rows = self._load_alert_history()
        by_day = {row.get("date"): row for row in rows if isinstance(row, dict) and row.get("date")}

        if today not in by_day:
            by_day[today] = {
                "date": today,
                "day": datetime.now().strftime("%a"),
                "fall": 0,
                "heat": 0,
                "prox": 0,
            }

        by_day[today][category] = int(by_day[today].get(category, 0)) + 1
        by_day[today]["day"] = datetime.now().strftime("%a")

        cutoff = datetime.now(timezone.utc).date() - timedelta(days=ALERT_HISTORY_DAYS - 1)
        kept = []
        for date_str, row in by_day.items():
            try:
                row_date = datetime.fromisoformat(date_str).date()
            except Exception:
                continue
            if row_date >= cutoff:
                kept.append(row)

        kept.sort(key=lambda row: row.get("date", ""))
        self._save_alert_history(kept)

    # ---- decision logic ----

    def _evaluate_alerts(self):
        """Determine the highest-priority alert."""
        # Helmet off means the helmet is not being worn, so silence alerts.
        priority = AlertManager.PATTERN_NONE
        self.alert_level = "OK"

        try:
            helmet_removed = self.helmet_det is not None and self.helmet_det.update()
            if self.helmet_det is not None:
                self._reset_sensor_error("helmet")
        except SensorUnavailableError as e:
            self._disable_sensor("helmet", e)
            helmet_removed = False
        except Exception as e:
            self._record_sensor_error("helmet", e)
            helmet_removed = False

        if helmet_removed:
            self.alert_level = "HELMET OFF"
            if self.alerts is not None:
                self.alerts.set_pattern(AlertManager.PATTERN_NONE)
            return

        # Priority while worn: FALL > DANGER > WARN > OK
        try:
            if self.fall_det is not None and self.fall_det.update():
                self.alert_level = "!! FALL !!"
                priority = AlertManager.PATTERN_FALL
        except Exception as e:
            self._record_sensor_error("imu", e)

        # Proximity danger (ultrasonic)
        if self.last_us_dist is not None:
            if self.last_us_dist < ULTRASONIC_DANGER_CM and priority < AlertManager.PATTERN_DANGER:
                self.alert_level = "PROX DANGER"
                priority = AlertManager.PATTERN_DANGER
            elif self.last_us_dist < ULTRASONIC_WARN_CM and priority < AlertManager.PATTERN_WARN:
                self.alert_level = "PROX WARN"
                priority = AlertManager.PATTERN_WARN

        # Proximity danger (ToF)
        if self.last_tof_dist is not None:
            if self.last_tof_dist < TOF_DANGER_MM and priority < AlertManager.PATTERN_DANGER:
                self.alert_level = "TOF DANGER"
                priority = AlertManager.PATTERN_DANGER
            elif self.last_tof_dist < TOF_WARN_MM and priority < AlertManager.PATTERN_WARN:
                self.alert_level = "TOF WARN"
                priority = AlertManager.PATTERN_WARN

        # Rear proximity (IR sensor on back of helmet)
        if self.helmet_det is not None and self.helmet_det.rear_object_detected():
            if priority < AlertManager.PATTERN_WARN:
                self.alert_level = "REAR WARN"
                priority = AlertManager.PATTERN_WARN

        # Heat stress
        if self.last_heat_index is not None:
            if self.last_heat_index >= HEAT_INDEX_EXTREME and priority < AlertManager.PATTERN_DANGER:
                self.alert_level = "HEAT EXTREME"
                priority = AlertManager.PATTERN_DANGER
            elif self.last_heat_index >= HEAT_INDEX_DANGER and priority < AlertManager.PATTERN_DANGER:
                self.alert_level = "HEAT DANGER"
                priority = AlertManager.PATTERN_DANGER
            elif self.last_heat_index >= HEAT_INDEX_CAUTION and priority < AlertManager.PATTERN_WARN:
                self.alert_level = "HEAT CAUTION"
                priority = AlertManager.PATTERN_WARN

        # Low light
        if self.last_lux is not None:
            if self.last_lux < LOW_LIGHT_LUX and priority < AlertManager.PATTERN_WARN:
                self.alert_level = "LOW LIGHT"
                priority = AlertManager.PATTERN_WARN

        if self.alerts is not None:
            self.alerts.set_pattern(priority)

        self._record_alert_history()

    # ---- display ----

    def _update_display(self):
        """Compose and push status lines to the OLED."""
        if self.oled is None:
            return

        try:
            us_str = f"{self.last_us_dist:.1f} cm" if self.last_us_dist is not None else "--"

            if self.helmet_det is not None:
                helmet_worn = not self.helmet_det.helmet_removed
                helmet_str = "WORN" if helmet_worn else "NOT WORN"
            else:
                helmet_str = "UNKNOWN"

            lines = [
                "ULTRASONIC",
                us_str,
                "HELMET",
                helmet_str,
            ]

            self.oled.update(lines)
            self._reset_sensor_error("oled")
        except Exception as e:
            try:
                self._record_sensor_error("oled", e)
            except SensorUnavailableError as disable_exc:
                self._disable_sensor("oled", disable_exc)

    # ---- console log (for debugging over SSH) ----

    def _print_status(self):
        import sys
        try:
            if self.imu is not None:
                ax, ay, az = self.imu.read_accel()
                gx, gy, gz = self.imu.read_gyro()
                self._reset_sensor_error("imu")
            else:
                ax = ay = az = gx = gy = gz = 0
        except Exception as e:
            self._record_sensor_error("imu", e)
            ax = ay = az = gx = gy = gz = 0
        if self.helmet_det is not None:
            touch = "Y" if not self.helmet_det.touch.is_active else "N"
            ir = "Y" if self.helmet_det.rear_object_detected() else "N"
            helmet_state = 'ON' if not self.helmet_det.helmet_removed else 'OFF'
        else:
            touch = ir = '-'
            helmet_state = '--'
        status_line = (
            f"[{self.alert_level:^14s}] "
            f"A({ax:+.2f},{ay:+.2f},{az:+.2f})g  "
            f"G({gx:+.1f},{gy:+.1f},{gz:+.1f})d/s  "
            f"US={self.last_us_dist}cm  ToF={self.last_tof_dist}mm  "
            f"T={self.last_temp}C  H={self.last_hum}%  HI={self.last_heat_index}C  "
            f"Lux={self.last_lux}  "
            f"Touch={touch}  IR={ir}  "
            f"Helmet={helmet_state}"
        )
        print(status_line, flush=True)
        sys.stdout.flush()

    def _write_api_state(self):
        """Write latest sensor snapshot to shared JSON file for helmet_api.py."""
        state_file = "/tmp/helmet_api_state.json"
        try:
            if self.imu is not None:
                total_g    = self.imu.total_g()
                total_gyro = self.imu.total_gyro()
                ax, ay, az = self.imu.read_accel()
                gx, gy, gz = self.imu.read_gyro()
                self._reset_sensor_error("imu")
            else:
                total_g = total_gyro = 0
                ax = ay = az = gx = gy = gz = 0
        except Exception as e:
            self._record_sensor_error("imu", e)
            total_g = total_gyro = 0
            ax = ay = az = gx = gy = gz = 0
        if self.helmet_det is not None:
            touch_active = not self.helmet_det.touch.is_active
            ir_detecting = self.helmet_det.rear_object_detected()
            helmet_on = not self.helmet_det.helmet_removed
        else:
            touch_active = False
            ir_detecting = False
            helmet_on = False
        data = {
            "accel":         {"x": round(ax, 3), "y": round(ay, 3), "z": round(az, 3)},
            "gyro":          {"x": round(gx, 2), "y": round(gy, 2), "z": round(gz, 2)},
            "totalG":        round(total_g, 3),
            "totalGyro":     round(total_gyro, 2),
            "fallDetected":  self.fall_det.fall_detected if self.fall_det is not None else False,
            "temperature":   self.last_temp,
            "humidity":      self.last_hum,
            "heatIndex":     self.last_heat_index,
            "usDist":        self.last_us_dist,
            "tofDist":       self.last_tof_dist,
            "lux":           round(self.last_lux, 1) if self.last_lux else None,
            "touchActive":   touch_active,
            "irDetecting":   ir_detecting,
            "helmetOn":      helmet_on,
            "alertLevel":    self.alert_level,
            "timestamp":     self._now_iso(),
        }
        try:
            with open(state_file, "w") as f:
                json.dump(data, f)
                f.flush()
                os.fsync(f.fileno())
        except Exception:
            pass

    def _now_iso(self):
        from datetime import datetime, timezone
        return datetime.now(timezone.utc).isoformat()

    # ---- main loop ----

    def run(self):
        print("==========================================")
        print("    SMART HELMET SYSTEM — RUNNING")
        print("    Press Ctrl+C to stop")
        print("==========================================\n")

        last_oled = 0

        while self.running:
            try:
                # 1. Read all sensors
                self._read_dht()
                self._read_ultrasonic()
                self._read_tof()
                self._read_light()

                # 2. Evaluate alert level
                self._evaluate_alerts()

                # 3. Update OLED (throttled)
                now = time.monotonic()
                if (now - last_oled) >= OLED_REFRESH_INTERVAL:
                    self._update_display()
                    last_oled = now

                # 4. Console output (debug)
                self._print_status()

                # 5. Write shared API state file (for helmet_api.py)
                self._write_api_state()

                time.sleep(MAIN_LOOP_INTERVAL)

            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"[ERROR] {e}")
                time.sleep(1)

        self.shutdown()

    def shutdown(self):
        print("\n[SHUTDOWN] Cleaning up...")
        self.running = False
        if self.alerts is not None:
            self.alerts.stop()
        if self.oled is not None:
            self.oled.clear()
        if self.ultrasonic is not None:
            self.ultrasonic.close()
        if self.helmet_det is not None:
            self.helmet_det.close()
        try:
            if self.dht is not None:
                self.dht.exit()
        except Exception:
            pass
        print("[SHUTDOWN] Done. Stay safe!")


# ═══════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════════

def main():
    helmet = SmartHelmet()

    # Graceful shutdown on SIGTERM (e.g., systemd stop)
    def _signal_handler(sig, frame):
        helmet.running = False
    signal.signal(signal.SIGTERM, _signal_handler)

    helmet.run()


if __name__ == "__main__":
    main()
