"""
ORBIT -- Rocket Ground Station
A PySide6 desktop ground-station dashboard for an ESP32 USB serial receiver.

Single self-contained file (the local-only command selection lives in the
`command_state` class below).

Install:
    py -m pip install PySide6 pyserial pyqtgraph
"""

import sys
import json
import csv
import time
import math
from datetime import datetime
from collections import deque

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont, QColor
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QComboBox, QFrame, QStackedWidget,
    QTableWidget, QTableWidgetItem, QFileDialog, QMessageBox, QPlainTextEdit,
    QHeaderView, QSplitter, QAbstractItemView, QLineEdit,
    QFormLayout, QScrollArea
)

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

try:
    import pyqtgraph as pg
except ImportError:
    pg = None

# =====================================================================
# Local-only operator command selection (built into this file)
# =====================================================================
class command_state:
    """
    Command selection and opcode table; sending is done by GroundStation.send_command.
    Opcodes mirror the flight-software enum cmd_opcode_t exactly.

    Read the current selection through the class (never copy it), so you always see the
    latest value:   command_state.selected_command / command_state.selected_opcode
    Python values are NOT shared with C++; a future C++ integration needs its own mechanism.
    """
    COMMANDS = {
        "CMD_NONE":            {"opcode": 0x00, "description": "No command."},
        "CMD_START_TELEMETRY": {"opcode": 0x01, "description": "Start telemetry transmission."},
        "CMD_STOP_TELEMETRY":  {"opcode": 0x02, "description": "Stop telemetry transmission."},
        "CMD_CALIBRATE_BARO":  {"opcode": 0x03, "description": "Calibrate barometer."},
        "CMD_CALIBRATE_GYRO":  {"opcode": 0x04, "description": "Calibrate gyroscope."},
        "CMD_CALIBRATE_ACCEL": {"opcode": 0x05, "description": "Calibrate accelerometer."},
        "CMD_ENTER_TEST":      {"opcode": 0x06, "description": "Enter test mode."},
        "CMD_ENTER_LAUNCHPAD": {"opcode": 0x07, "description": "Enter launch-pad state."},
        "CMD_SWITCH_CHANNEL":  {"opcode": 0x08,
                                "description": "Switch radio channel: MAIN (argument 0) or BACKUP (argument 1)."},
        "CMD_TRIGGER_SEP":     {"opcode": 0x09, "description": "Trigger separation mechanism."},
        "CMD_TOGGLE_PAYLOAD":  {"opcode": 0x0A, "description": "Toggle payload."},
        "CMD_TOGGLE_PARACHUTE": {"opcode": 0x0B, "description": "Toggle parachute mechanism."},
    }
    COMMANDS_TRANSMITTED = True         # commands are sent through the bridge (see send_command)

    # Current selection (initial: CMD_NONE / 0x00). Overwritten by set_command().
    selected_command = "CMD_NONE"
    selected_opcode = 0x00

    @classmethod
    def set_command(cls, command_name):
        """
        Validate and, if valid, overwrite selected_command/selected_opcode. Surrounding
        whitespace and letter case are ignored. Returns (True, canonical_name) or
        (False, error_message); on failure the previous valid selection is untouched.
        """
        if not isinstance(command_name, str):
            return False, "Command must be text."
        name = command_name.strip().upper()
        if not name:
            return False, "Empty input. Type a command name such as CMD_START_TELEMETRY."
        entry = cls.COMMANDS.get(name)
        if entry is None:
            shown = command_name.strip()
            if len(shown) > 40:
                shown = shown[:40] + "..."
            return False, f"Unknown command '{shown}'. Selection unchanged. See the legend for valid names."
        cls.selected_command = name
        cls.selected_opcode = entry["opcode"]
        return True, name

    @classmethod
    def opcode_hex(cls, opcode=None):
        """Format an opcode as 0xNN (defaults to the current selection)."""
        if opcode is None:
            opcode = cls.selected_opcode
        return f"0x{opcode:02X}"

    @classmethod
    def reset(cls):
        cls.selected_command = "CMD_NONE"
        cls.selected_opcode = 0x00


# =====================================================================
# Six-byte telecommand packet (explicit byte encoding, no struct packing)
#   [0] 0xAA sync  [1] team LSB  [2] team MSB  [3] opcode  [4] argument
#   [5] XOR of bytes 0..4
# =====================================================================
TC_SYNC = 0xAA
TC_TEAM_NUMBER = 7              # sent little-endian as 0x07 0x00


def encode_command(opcode, argument=0):
    """Return the six-byte telecommand packet as a bytes object."""
    for name, value in (("opcode", opcode), ("argument", argument)):
        if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 0xFF:
            raise ValueError(f"{name} must be an integer from 0 to 255, got {value!r}")
    packet = bytearray([
        TC_SYNC,
        TC_TEAM_NUMBER & 0xFF,
        (TC_TEAM_NUMBER >> 8) & 0xFF,
        opcode,
        argument,
    ])
    checksum = 0
    for byte in packet:
        checksum ^= byte
    packet.append(checksum)
    return bytes(packet)

def decode_command(packet):
    """Validate a six-byte telecommand packet and return (opcode, argument).

    Raises ValueError saying what is wrong. This checks packets in the same order
    the rocket firmware will: length, sync, team number, checksum."""
    packet = bytes(packet)
    if len(packet) != 6:
        raise ValueError(f"expected 6 bytes, got {len(packet)}")
    if packet[0] != TC_SYNC:
        raise ValueError(f"bad sync byte 0x{packet[0]:02X}")
    team = packet[1] | (packet[2] << 8)
    if team != TC_TEAM_NUMBER:
        raise ValueError(f"wrong team number {team}")
    expected = 0
    for byte in packet[:5]:
        expected ^= byte
    if packet[5] != expected:
        raise ValueError(f"bad checksum 0x{packet[5]:02X}, expected 0x{expected:02X}")
    return packet[3], packet[4]


# ---- Telecommand uplink settings ----
TOGGLE_OPCODES = (0x0A, 0x0B)             # payload / parachute: the argument byte is a sequence number
CHANNEL_ARGS = {"MAIN": 0, "BACKUP": 1}   # CMD_SWITCH_CHANNEL argument
TC_REPEATS = 3                            # transmissions per request (the rocket is deaf while it transmits)
TC_TICK_MS = 400                          # spacing between repeats
TC_ACK_WAIT_TICKS = 5                     # ticks to wait for an ACK after the last repeat (2 s)


# ---------- Palette ----------
BG = "#0A0F1A"
SIDEBAR = "#0D1524"
PANEL = "#111C2D"
PANEL_ALT = "#162338"
BORDER = "#26374D"
TEXT = "#EAF1FA"
MUTED = "#91A4BC"
CYAN = "#4ADBE8"
GREEN = "#42D392"
AMBER = "#F4BD62"
RED = "#FF657A"
BLUE = "#729BFF"


# =====================================================================
# Protocol definitions
# =====================================================================

# Flight-software state enum (fsw_state_t). ST_COUNT (7) is a sentinel, NOT a valid state.
FSW_STATES = {
    0: ("ST_BOOT", "Flight computer booting."),
    1: ("ST_TEST_MODE", "Test/diagnostics mode."),
    2: ("ST_LAUNCH_PAD", "Waiting on the launch pad."),
    3: ("ST_ASCENT", "Ascent."),
    4: ("ST_PAYLOAD_SEP", "Payload-separation state."),
    5: ("ST_DESCENT", "Descent."),
    6: ("ST_IMPACT", "Impact/landing detected."),
}
FSW_ST_COUNT = 7  # ST_COUNT sentinel value

# GUI-level mission phases (exact human-readable names). APOGEE and LANDED are GUI
# labels/events, NOT firmware enum values.
PHASES = ["LAUNCH_PAD", "ASCENT", "APOGEE", "PRIMARY DESCENT", "LANDED"]

# Firmware state used by the SIMULATION for each GUI phase. APOGEE has no dedicated
# firmware state, so the simulator keeps reporting ST_ASCENT during the apogee window.
SIM_PHASE_TO_STATE = {
    "LAUNCH_PAD": 2,        # ST_LAUNCH_PAD
    "ASCENT": 3,            # ST_ASCENT
    "APOGEE": 3,            # phase/event only -> still ST_ASCENT
    "PRIMARY DESCENT": 5,   # ST_DESCENT
    "LANDED": 6,            # ST_IMPACT
}

# For LIVE data without a "gui_phase" key: only unambiguous firmware states are mapped.
# APOGEE is never inferred from live firmware state.
LIVE_STATE_TO_PHASE = {2: "LAUNCH_PAD", 3: "ASCENT", 5: "PRIMARY DESCENT", 6: "LANDED"}

# (field name, C type, unit / note, GUI location, card title) -- mirrors telemetry_packet_t
TELEMETRY_FIELDS = [
    ("team_number", "uint16_t", "team ID", "Telemetry page", "Team number"),
    ("mission_time", "uint32_t", "s since power-on", "Overview + Telemetry", "Mission time"),
    ("packet_count", "uint16_t", "packets", "Overview + Radio page", "Packet count"),
    ("altitude", "float", "m AGL (barometric)", "Main altitude graph", "Barometric altitude"),
    ("pressure", "float", "Pa", "Overview + Telemetry", "Pressure"),
    ("temperature", "float", "degC", "Overview + Telemetry", "Temperature"),
    ("voltage", "float", "V (firmware sends a 0.0 placeholder for now)", "Telemetry page", "Voltage"),
    ("gnss_time", "uint32_t", "HHMMSS", "GPS panel + Telemetry", "GNSS time"),
    ("latitude", "float", "unit not specified in struct", "GPS panel + Telemetry", "Latitude"),
    ("longitude", "float", "unit not specified in struct", "GPS panel + Telemetry", "Longitude"),
    ("gnss_altitude", "float", "m MSL (GNSS, not AGL)", "Secondary GNSS graph", "GNSS altitude"),
    ("gnss_sats", "uint8_t", "satellites tracked", "GPS panel + Telemetry", "GNSS satellites"),
    ("accel_x", "float", "unit not specified by firmware", "Overview + Telemetry", "Accel X"),
    ("accel_y", "float", "unit not specified by firmware", "Overview + Telemetry", "Accel Y"),
    ("accel_z", "float", "unit not specified by firmware", "Overview + Telemetry", "Accel Z"),
    ("gyro_spin_rate", "float", "deg/s", "Overview + Telemetry", "Gyro spin rate"),
    ("flight_state", "uint8_t", "fsw_state_t as a byte", "Overview + Telemetry", "Flight state"),
    ("checksum", "uint8_t", "intended XOR of bytes 0..57 (NOT verified)", "Telemetry page", "Checksum"),
]
FIELD_INFO = {f[0]: f for f in TELEMETRY_FIELDS}

# Older key names accepted for backward compatibility -> canonical struct field name.
LEGACY_ALIASES = {
    "altitude_m": "altitude", "baro_altitude_m": "altitude",
    "pressure_pa": "pressure", "temperature_c": "temperature",
    "lat": "latitude", "latitude_deg": "latitude",
    "lon": "longitude", "longitude_deg": "longitude",
    "gps_altitude_m": "gnss_altitude", "gps_alt_m": "gnss_altitude",
    "satellites": "gnss_sats", "satellites_used": "gnss_sats",
    "packet_id": "packet_count",
    "phase": "gui_phase", "flight_phase": "gui_phase",
    "rssi": "rssi_dbm", "snr": "snr_db",
}


def normalize_packet(data):
    """Return a copy of `data` using canonical field names (legacy aliases renamed)."""
    out = dict(data)
    for old, new in LEGACY_ALIASES.items():
        if old in out:
            value = out.pop(old)
            out.setdefault(new, value)
    return out


def parse_telemetry_line(line):
    """
    Modular live-input parser: one JSON object per line -> dict with canonical names.
    Raises ValueError on anything that is not a JSON object. To support a different
    receiver output format later, replace only this function.
    """
    try:
        data = json.loads(line)
    except Exception as exc:
        raise ValueError(f"not valid JSON: {exc}")
    if not isinstance(data, dict):
        raise ValueError("JSON root is not an object")
    return normalize_packet(data)


# Rocket flight_state text (fsw_state_name) -> fsw_state_t number used by the GUI
FSW_STATE_BY_NAME = {"BOOT": 0, "TEST_MODE": 1, "LAUNCH_PAD": 2, "ASCENT": 3,
                     "PAYLOAD_SEP": 4, "DESCENT": 5, "IMPACT": 6}

# The 17 ASCII telemetry columns in the order the rocket sends them: (column, GUI key, kind)
ASCII_FIELDS = [
    ("TEAM_ID", "team_number", "text"),
    ("TIME_S", "mission_time", "int"),
    ("PACKET_COUNT", "packet_count", "int"),
    ("ALTITUDE_M", "altitude", "float"),
    ("PRESSURE_PA", "pressure", "float"),
    ("TEMP_C", "temperature", "float"),
    ("VOLTAGE_V", "voltage", "float"),
    ("GNSS_TIME", "gnss_time", "int"),
    ("GNSS_LAT", "latitude", "float"),
    ("GNSS_LON", "longitude", "float"),
    ("GNSS_ALT_M", "gnss_altitude", "float"),
    ("GNSS_SATS", "gnss_sats", "int"),
    ("ACCEL_X", "accel_x", "float"),
    ("ACCEL_Y", "accel_y", "float"),
    ("ACCEL_Z", "accel_z", "float"),
    ("GYRO_SPIN_DPS", "gyro_spin_rate", "float"),
    ("FLIGHT_STATE", "flight_state", "state"),
]
ASCII_COLUMNS = [f[0] for f in ASCII_FIELDS]


def parse_ascii_telemetry(line):
    """Parse one rocket telemetry line (17 comma-separated ASCII fields).
    Returns a dict with GUI keys plus "_raw_fields" (the 17 text fields as received).
    Raises ValueError for anything malformed."""
    fields = [f.strip() for f in line.split(",")]
    if len(fields) != len(ASCII_FIELDS):
        raise ValueError(f"expected {len(ASCII_FIELDS)} fields, got {len(fields)}")
    data = {}
    for text, (column, key, kind) in zip(fields, ASCII_FIELDS):
        try:
            if kind == "int":
                data[key] = int(text)
            elif kind == "float":
                value = float(text)
                if not math.isfinite(value):
                    raise ValueError("not finite")
                data[key] = value
            elif kind == "state":
                if text == "":
                    raise ValueError("empty")
                data[key] = FSW_STATE_BY_NAME.get(text.upper(), text)   # unknown name -> shown INVALID
            else:
                if text == "":
                    raise ValueError("empty")
                data[key] = text
        except ValueError:
            raise ValueError(f"{column} is not valid: {text!r}")
    data["_raw_fields"] = fields
    return data


def describe_flight_state(value):
    """Return (label, is_valid) for a received flight_state value."""
    if value is None:
        return "—", True
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return f"INVALID ({value!r})", False
    if isinstance(value, float):
        if not value.is_integer():
            return f"INVALID ({value})", False
        value = int(value)
    if value in FSW_STATES:
        return f"{FSW_STATES[value][0]} ({value})", True
    if value == FSW_ST_COUNT:
        return f"ST_COUNT ({value}) · INVALID SENTINEL", False
    return f"UNKNOWN STATE ({value})", False


def phase_for_packet(data):
    """GUI-level phase label for a packet, or None if it cannot be determined."""
    label = data.get("gui_phase")
    if isinstance(label, str) and label.strip():
        return label.strip().upper() if label.strip().upper() in PHASES else label.strip()
    state = data.get("flight_state")
    if isinstance(state, float) and state.is_integer():
        state = int(state)
    if isinstance(state, int) and not isinstance(state, bool):
        return LIVE_STATE_TO_PHASE.get(state)
    return None


# =====================================================================
# Built-in DUMMY flight simulation (placeholder values only -- NOT live hardware data)
# =====================================================================
SIM_INTERVAL_MS = 250        # real time between simulated packets
SIM_DT = 0.5                 # simulated seconds per packet (2x time compression)
SIM_PAD_S = 4.0              # LAUNCH_PAD duration
SIM_ASCENT_S = 20.0          # ASCENT duration
SIM_APOGEE_S = 2.0           # APOGEE window
SIM_DESCENT_S = 60.0         # PLACEHOLDER descent duration for GUI testing only
SIM_APOGEE_ALT = 1000.0      # target apogee 1000 m (+/-10 m => 990-1010 m)
SIM_APOGEE_BAND = (990.0, 1010.0)
SIM_TEAM_NUMBER = 9999       # placeholder team ID
SIM_GROUND_MSL = 265.0       # placeholder ground elevation (m MSL) for GNSS altitude/pressure
SIM_T_ASCENT_END = SIM_PAD_S + SIM_ASCENT_S
SIM_T_APOGEE_END = SIM_T_ASCENT_END + SIM_APOGEE_S
SIM_T_LAND = SIM_T_APOGEE_END + SIM_DESCENT_S


def sim_profile(t):
    """Dummy flight profile -> (phase, altitude AGL m, vertical accel m/s^2)."""
    if t < SIM_PAD_S:
        return "LAUNCH_PAD", 0.15 * (1.0 + math.sin(t * 1.5)), 0.0
    if t < SIM_T_ASCENT_END:
        u = (t - SIM_PAD_S) / SIM_ASCENT_S
        alt = SIM_APOGEE_ALT * (u * u * (3.0 - 2.0 * u))        # smoothstep up to 1000 m
        accel = 15.0 * (1.0 - 2.0 * u)                           # d2(alt)/dt2 of the curve
        return "ASCENT", alt, accel
    if t < SIM_T_APOGEE_END:
        u = (t - SIM_T_ASCENT_END) / SIM_APOGEE_S
        return "APOGEE", SIM_APOGEE_ALT + 2.0 * math.sin(math.pi * u), 0.0   # peak ~1002 m
    if t < SIM_T_LAND:
        u = (t - SIM_T_APOGEE_END) / SIM_DESCENT_S
        return "PRIMARY DESCENT", SIM_APOGEE_ALT * (1.0 - u), 0.0
    return "LANDED", 0.0, 0.0


def build_simulated_packet(t, packet_count):
    """
    Build one dummy telemetry dict using the exact telemetry_packet_t field names.
    All values are invented for GUI testing. No "checksum" key is produced because no
    byte representation is defined. "simulated": true and "gui_phase" are GUI extras.
    """
    phase, alt, accel = sim_profile(t)
    msl = SIM_GROUND_MSL + alt
    pressure = 101325.0 * max(0.01, (1.0 - 2.25577e-5 * msl) ** 5.25588)
    temperature = 24.0 - 0.0065 * alt + 0.3 * math.sin(t * 0.25)
    airborne = phase in ("ASCENT", "APOGEE", "PRIMARY DESCENT")
    spin = (22.0 + 6.0 * math.sin(t * 0.7)) if phase == "ASCENT" else (
        (4.0 + 2.0 * math.sin(t * 0.5)) if airborne else 0.2)
    secs = int(t)
    gnss_time = 12 * 10000 + (secs // 60) * 100 + (secs % 60)   # dummy clock starting 12:00:00
    return {
        "team_number": SIM_TEAM_NUMBER,
        "mission_time": secs,
        "packet_count": packet_count,
        "altitude": round(alt, 2),
        "pressure": round(pressure, 1),
        "temperature": round(temperature, 2),
        "gnss_time": gnss_time,
        "latitude": round(17.980000 + 0.00004 * math.sin(t / 10.0), 7),
        "longitude": round(79.530000 + 0.00005 * math.cos(t / 12.0), 7),
        "gnss_altitude": round(msl + 2.5 * math.sin(t * 0.35), 2),
        "gnss_sats": 8 + (1 if int(t / 7.0) % 3 else 0),
        "accel_x": round(0.15 * math.sin(t * 0.9), 3),
        "accel_y": round(0.15 * math.cos(t * 0.8), 3),
        "accel_z": round(9.81 + accel + 0.05 * math.sin(t * 1.3), 3),   # dummy scale, no unit asserted
        "gyro_spin_rate": round(spin, 2),
        "flight_state": SIM_PHASE_TO_STATE[phase],
        # ---- GUI extras (not part of telemetry_packet_t) ----
        "gui_phase": phase,
        "simulated": True,
        "rssi_dbm": round(-72.0 - 0.25 * (alt / 10.0) + 5.0 * math.sin(t * 0.45), 1),
        "snr_db": round(max(-10.0, 10.0 - alt / 180.0 + 2.0 * math.cos(t * 0.4)), 1),
    }


# =====================================================================
# Small UI helpers
# =====================================================================
def make_label(text, size=10, color=TEXT, bold=False):
    label = QLabel(str(text))
    label.setStyleSheet(
        f"color:{color};font-size:{size}px;font-weight:{'700' if bold else '400'};"
    )
    return label


def make_scroll(widget):
    """Wrap a page in a frameless scroll area so tall pages stay usable on small windows."""
    area = QScrollArea()
    area.setWidgetResizable(True)
    area.setFrameShape(QFrame.NoFrame)
    area.setWidget(widget)
    return area


class MetricCard(QFrame):
    """Compact metric tile with a large value and a small unit/description."""
    def __init__(self, title, unit="", accent=CYAN, compact=False):
        super().__init__()
        self.setObjectName("metricCard")
        self.setMinimumHeight(88 if compact else 108)
        self.default_unit = unit
        self.accent = accent
        layout = QVBoxLayout(self)
        layout.setContentsMargins(13, 11, 13, 10)
        layout.setSpacing(5)
        self.title_label = make_label(title.upper(), 8, MUTED, True)
        self.value_label = make_label("—", 24 if not compact else 20, accent, True)
        self.unit_label = make_label(unit, 8, MUTED)
        self.unit_label.setWordWrap(True)
        self.value_size = 24 if not compact else 20
        self.value_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.title_label)
        layout.addWidget(self.value_label)
        layout.addWidget(self.unit_label)
        layout.addStretch(1)

    def set_value(self, value, unit=None):
        if value is None or value == "":
            text = "—"
        elif isinstance(value, float):
            text = f"{value:.2f}"
        else:
            text = str(value)
        self.value_label.setText(text)
        if unit is not None:
            self.unit_label.setText(unit)

    def set_accent(self, color):
        self.value_label.setStyleSheet(f"color:{color};font-size:{self.value_size}px;font-weight:700;")

    def reset(self):
        self.value_label.setText("—")
        self.unit_label.setText(self.default_unit)
        self.set_accent(self.accent)


class SectionPanel(QFrame):
    def __init__(self, title, subtitle=None):
        super().__init__()
        self.setObjectName("panel")
        self.layout_box = QVBoxLayout(self)
        self.layout_box.setContentsMargins(15, 13, 15, 14)
        self.layout_box.setSpacing(10)
        heading = QHBoxLayout()
        title_label = make_label(title, 11, TEXT, True)
        heading.addWidget(title_label)
        heading.addStretch(1)
        self.subtitle_label = None
        if subtitle:
            self.subtitle_label = make_label(subtitle, 8, MUTED)
            heading.addWidget(self.subtitle_label)
        self.layout_box.addLayout(heading)


class GroundStation(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ORBIT | Rocket Ground Station")
        self.resize(1500, 940)
        self.setMinimumSize(1120, 720)

        # ---- connection / source state ----
        self.serial_port = None
        self.connected = False
        self.buffer = ""
        self.start_time = time.monotonic()          # app session clock (status strip only)

        # ---- simulation state (reset by reset_flight_state) ----
        self.simulation_active = False
        self.sim_state = "idle"                     # idle | running | complete | stopped
        self.simulation_elapsed = 0.0               # simulated seconds
        self.simulation_packet_count = 0
        self.sim_max_altitude = 0.0
        self._apogee_logged = False
        self._primary_deploy_logged = False
        self._landed_logged = False
        # ONE timer, created once. start() on an already-running QTimer just restarts it,
        # so duplicate timers / concurrent simulations are impossible.
        self.simulation_timer = QTimer(self)
        self.simulation_timer.setInterval(SIM_INTERVAL_MS)
        self.simulation_timer.timeout.connect(self.emit_simulated_packet)

        # ---- per-flight / per-stream telemetry state (reset by reset_flight_state) ----
        self.run_id = 0
        self.stream_t0 = None                       # monotonic time of first live packet
        self.last_packet_time = None
        self.packet_count = 0
        self.bad_packets = 0
        self.last_packet_count_field = None
        self.missed_packets = 0
        self.latest = {}
        self.last_phase_key = ""
        self._last_invalid_state = None
        self.last_altitude = None
        self.last_altitude_t = None
        self.history_t = deque(maxlen=6000)
        self.history_alt = deque(maxlen=6000)
        self.history_gnss_alt = deque(maxlen=6000)
        self.history_vel = deque(maxlen=6000)
        self.packet_rows = deque(maxlen=500)
        self.event_count = 0

        # ---- CSV ----
        self.csv_handle = None
        self.csv_writer = None


        # ---- Telecommand uplink ----
        self.pending_cmd = None                     # the one request currently being sent
        self.toggle_seq = {0x0A: 0, 0x0B: 0}        # independent 8-bit sequence counters
        self.cmd_timer = QTimer(self)
        self.cmd_timer.timeout.connect(self.cmd_tick)


        self.setStyleSheet(f"""
            QMainWindow, QWidget {{ background:{BG}; color:{TEXT}; }}
            QScrollArea {{ border:none; background:{BG}; }}
            QLabel {{ background:transparent; }}
            QFrame#sidebar {{ background:{SIDEBAR}; border-right:1px solid {BORDER}; }}
            QFrame#panel, QFrame#metricCard {{
                background:{PANEL}; border:1px solid {BORDER}; border-radius:11px;
            }}
            QFrame#warnBadge {{
                background:#2B2412; border:1px solid #7A6630; border-radius:11px;
            }}
            QPushButton {{
                background:{PANEL_ALT}; color:{TEXT}; border:1px solid {BORDER};
                border-radius:7px; padding:8px 12px; font-weight:600;
            }}
            QPushButton:hover {{ border-color:{CYAN}; background:#1B2B43; }}
            QPushButton:pressed {{ background:#203650; }}
            QPushButton:disabled {{ color:#53657B; background:#101927; }}
            QPushButton#navButton {{
                text-align:left; padding:12px 13px; border:1px solid transparent;
                background:transparent; color:{MUTED}; border-radius:8px;
            }}
            QPushButton#navButton:hover {{ background:{PANEL_ALT}; color:{TEXT}; }}
            QPushButton#navButton[active="true"] {{
                background:#132D3A; color:{CYAN}; border:1px solid #24515C;
            }}
            QPushButton#primaryButton {{
                background:#123B47; border:1px solid #286775; color:#DFFBFF;
            }}
            QPushButton#dangerButton {{
                background:#3B1D2A; border:1px solid #713447; color:#FFDDE3;
            }}
            QComboBox, QSpinBox, QLineEdit {{
                background:{PANEL_ALT}; color:{TEXT}; border:1px solid {BORDER};
                border-radius:6px; padding:7px 8px; min-height:18px;
            }}
            QComboBox QAbstractItemView {{ background:{PANEL}; color:{TEXT}; selection-background-color:{PANEL_ALT}; }}
            QPlainTextEdit, QTableWidget {{
                background:{PANEL}; color:{TEXT}; border:1px solid {BORDER};
                border-radius:7px; gridline-color:{BORDER};
                selection-background-color:#24415A; selection-color:{TEXT};
            }}
            QHeaderView::section {{
                background:{PANEL_ALT}; color:{MUTED}; padding:8px;
                border:0; border-bottom:1px solid {BORDER}; font-weight:700;
            }}
            QTableWidget {{ alternate-background-color:#142034; }}
            QTabWidget::pane {{ border:1px solid {BORDER}; border-radius:8px; }}
            QScrollBar:vertical {{ background:{BG}; width:10px; margin:0; }}
            QScrollBar::handle:vertical {{ background:{BORDER}; min-height:24px; border-radius:4px; }}
            QToolTip {{ background:{PANEL_ALT}; color:{TEXT}; border:1px solid {BORDER}; }}
        """)

        central = QWidget()
        outer = QHBoxLayout(central)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        # Left navigation rail
        sidebar = QFrame()
        sidebar.setObjectName("sidebar")
        sidebar.setFixedWidth(262)
        side = QVBoxLayout(sidebar)
        side.setContentsMargins(15, 20, 15, 16)
        side.setSpacing(7)

        brand = QHBoxLayout()
        logo = QLabel("◈")
        logo.setStyleSheet(f"color:{CYAN};font-size:29px;font-weight:700;")
        brand.addWidget(logo)
        brand_text = QVBoxLayout()
        brand_text.setSpacing(0)
        brand_text.addWidget(make_label("ORBIT", 17, TEXT, True))
        brand_text.addWidget(make_label("GROUND CONTROL", 8, MUTED, True))
        brand.addLayout(brand_text)
        brand.addStretch()
        side.addLayout(brand)
        side.addSpacing(22)
        side.addWidget(make_label("WORKSPACE", 8, MUTED, True))

        self.nav_buttons = []
        # Order must match the QStackedWidget pages and self.page_titles below.
        nav_items = [
            ("⌂", "Overview"),
            ("◉", "Telemetry"),
            ("⌁", "Radio & Packets"),
            (">_", "Flight Command Console"),
            ("≡", "Event Log"),
            ("⚙", "Connection Setup"),
        ]
        self.page_titles = [
            ("Mission overview", "Live rocket telemetry and receiver health"),
            ("Telemetry", "Sensor values received from the onboard flight computer"),
            ("Radio & packets", "Packet history and radio diagnostics"),
            ("Flight Command Console", "Local command selection — nothing is transmitted"),
            ("Event log", "Session events and CSV recording"),
            ("Connection setup", "USB serial settings, receiver protocol and field map"),
        ]
        for index, (glyph, name) in enumerate(nav_items):
            # "&" would be treated as a keyboard-mnemonic marker by Qt, so escape it.
            safe_name = name.replace("&", "&&")
            button = QPushButton(f"{glyph}   {safe_name}" if len(glyph) == 1 else f"{glyph}  {safe_name}")
            button.setObjectName("navButton")
            button.setProperty("active", index == 0)
            button.clicked.connect(lambda checked=False, i=index: self.navigate(i))
            side.addWidget(button)
            self.nav_buttons.append(button)

        side.addStretch(1)
        side.addWidget(make_label("MISSION LINK", 8, MUTED, True))
        self.sidebar_link = make_label("●  OFFLINE", 10, RED, True)
        side.addWidget(self.sidebar_link)
        side.addWidget(make_label("ESP32 · USB SERIAL", 8, MUTED))
        side.addSpacing(5)
        side.addWidget(make_label("ORBIT GS  /  v1.1", 8, "#60748D"))
        outer.addWidget(sidebar)

        # Main content area
        main_area = QWidget()
        main = QVBoxLayout(main_area)
        main.setContentsMargins(22, 18, 22, 14)
        main.setSpacing(13)

        topbar = QHBoxLayout()
        title_stack = QVBoxLayout()
        title_stack.setSpacing(2)
        self.page_title = make_label("Mission overview", 21, TEXT, True)
        self.page_subtitle = make_label(self.page_titles[0][1], 9, MUTED)
        title_stack.addWidget(self.page_title)
        title_stack.addWidget(self.page_subtitle)
        topbar.addLayout(title_stack)
        topbar.addStretch(1)
        topbar.addWidget(make_label("PORT", 8, MUTED, True))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(165)
        topbar.addWidget(self.port_combo)
        topbar.addWidget(make_label("BAUD", 8, MUTED, True))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["115200", "57600", "9600", "230400", "460800"])
        self.baud_combo.setCurrentText("115200")
        self.baud_combo.setFixedWidth(105)
        topbar.addWidget(self.baud_combo)
        self.scan_button = QPushButton("↻ Scan")
        self.scan_button.clicked.connect(self.scan_ports)
        topbar.addWidget(self.scan_button)
        self.connect_button = QPushButton("Connect")
        self.connect_button.setObjectName("primaryButton")
        self.connect_button.clicked.connect(self.toggle_connection)
        topbar.addWidget(self.connect_button)
        self.sim_button = QPushButton("▶ Simulate Flight")
        self.sim_button.setObjectName("primaryButton")
        self.sim_button.clicked.connect(self.toggle_simulation)
        topbar.addWidget(self.sim_button)
        main.addLayout(topbar)

        # Persistent connection status strip
        self.status_strip = QFrame()
        self.status_strip.setObjectName("panel")
        status_layout = QHBoxLayout(self.status_strip)
        status_layout.setContentsMargins(12, 8, 12, 8)
        self.status_dot = make_label("●", 12, RED, True)
        self.status_main = make_label("Receiver disconnected", 9, TEXT, True)
        self.status_detail = make_label("Connect the ground ESP32 using USB", 9, MUTED)
        status_layout.addWidget(self.status_dot)
        status_layout.addWidget(self.status_main)
        status_layout.addSpacing(10)
        status_layout.addWidget(self.status_detail)
        status_layout.addStretch(1)
        self.source_badge = make_label("DATA SOURCE: NONE", 9, MUTED, True)
        status_layout.addWidget(self.source_badge)
        status_layout.addSpacing(14)
        self.elapsed_label = make_label("SESSION 00:00:00", 9, MUTED, True)
        status_layout.addWidget(self.elapsed_label)
        main.addWidget(self.status_strip)

        self.pages = QStackedWidget()
        self.pages.addWidget(self.build_overview_page())
        self.pages.addWidget(make_scroll(self.build_telemetry_page()))
        self.pages.addWidget(self.build_radio_page())
        self.pages.addWidget(self.build_command_page())
        self.pages.addWidget(self.build_events_page())
        self.pages.addWidget(make_scroll(self.build_setup_page()))
        main.addWidget(self.pages, 1)

        footer = QHBoxLayout()
        self.footer_status = make_label("Ready · Waiting for serial connection", 8, MUTED)
        footer.addWidget(self.footer_status)
        footer.addStretch(1)
        footer.addWidget(make_label("ROCKET SYSTEMS  •  TELEMETRY CONSOLE", 8, "#60748D", True))
        main.addLayout(footer)
        outer.addWidget(main_area, 1)
        self.setCentralWidget(central)

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self.poll_serial)
        self.poll_timer.start(25)
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.update_status)
        self.ui_timer.start(250)

        self.scan_ports()
        self.refresh_command_display()
        self.add_event("Ground station initialized.")
        self.add_event("Protocol: newline-delimited JSON from the receiver ESP32 (canonical telemetry_packet_t field names).")
        self.add_event("Flight Command Console sends commands through the bridge (requires a live connection).")

    # ---------- Page construction ----------
    def make_plot(self, left_label):
        plot = pg.PlotWidget()
        plot.setBackground(PANEL)
        plot.showGrid(x=True, y=True, alpha=0.20)
        plot.setLabel("left", left_label, **{"color": MUTED})
        plot.setLabel("bottom", "Time (s)", **{"color": MUTED})
        for axis in ("left", "bottom"):
            plot.getAxis(axis).setTextPen(MUTED)
            plot.getAxis(axis).setPen(BORDER)
        plot.getViewBox().setLimits(xMin=0)     # time axis can never start below zero
        return plot

    def build_overview_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 2, 0, 0)
        layout.setSpacing(12)

        metric_grid = QGridLayout()
        metric_grid.setSpacing(10)
        self.phase_card = MetricCard("Mission phase", "GUI phase · awaiting telemetry", GREEN)
        self.state_card = MetricCard("Firmware state", "flight_state · fsw_state_t", GREEN)
        self.alt_card = MetricCard("Barometric altitude", "altitude · m AGL", CYAN)
        self.vel_card = MetricCard("Vertical velocity", "derived by GUI from altitude · m/s", BLUE)
        self.temp_card = MetricCard("Temperature", "temperature · degC", AMBER)
        self.time_card = MetricCard("Mission time", "mission_time · s since power-on", CYAN)
        for i, card in enumerate([self.phase_card, self.state_card, self.alt_card,
                                  self.vel_card, self.temp_card, self.time_card]):
            metric_grid.addWidget(card, 0, i)
        layout.addLayout(metric_grid)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)

        self.profile_panel = SectionPanel("ALTITUDE PROFILE", "BAROMETRIC (AGL) + GNSS (MSL)")
        self.alt_plot = self.gnss_plot = None
        self.alt_curve = self.gnss_curve = None
        if pg is not None:
            # Main graph: altitude (m AGL).  Secondary graph: gnss_altitude (m MSL, NOT AGL).
            self.alt_plot = self.make_plot("Altitude AGL (m)")
            self.alt_curve = self.alt_plot.plot(pen=pg.mkPen(CYAN, width=2))
            self.gnss_plot = self.make_plot("GNSS alt MSL (m)")
            self.gnss_curve = self.gnss_plot.plot(pen=pg.mkPen(AMBER, width=2))
            self.gnss_plot.setXLink(self.alt_plot)
            self.profile_panel.layout_box.addWidget(self.alt_plot, 3)
            self.profile_panel.layout_box.addWidget(self.gnss_plot, 2)
        else:
            missing = make_label("Live chart unavailable\n\nInstall pyqtgraph:\npy -m pip install pyqtgraph", 10, AMBER)
            missing.setAlignment(Qt.AlignCenter)
            self.profile_panel.layout_box.addWidget(missing, 1)
        splitter.addWidget(self.profile_panel)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(10)

        gps = SectionPanel("GPS FIX", "NEO-M6")
        gps_grid = QGridLayout()
        gps_grid.setHorizontalSpacing(16)
        gps_grid.setVerticalSpacing(10)
        self.lat_value = make_label("—", 13, TEXT, True)
        self.lon_value = make_label("—", 13, TEXT, True)
        self.gps_alt_value = make_label("— m MSL", 12, CYAN, True)
        self.gps_time_value = make_label("—", 12, TEXT, True)
        self.sats_value = make_label("—", 12, GREEN, True)
        gps_items = [
            ("LATITUDE", self.lat_value), ("LONGITUDE", self.lon_value),
            ("GNSS ALTITUDE (MSL)", self.gps_alt_value), ("GNSS TIME (HHMMSS)", self.gps_time_value),
            ("SATELLITES TRACKED", self.sats_value)
        ]
        for r, (name, value) in enumerate(gps_items):
            gps_grid.addWidget(make_label(name, 8, MUTED, True), r, 0)
            gps_grid.addWidget(value, r, 1)
        gps.layout_box.addLayout(gps_grid)
        gps.layout_box.addWidget(make_label("latitude / longitude units are not specified in the struct", 8, MUTED))
        right_layout.addWidget(gps)

        health = SectionPanel("RECEIVER HEALTH")
        health_grid = QGridLayout()
        self.valid_value = make_label("0", 15, GREEN, True)
        self.bad_value = make_label("0", 12, RED, True)
        self.missed_value = make_label("0", 12, AMBER, True)
        self.age_value = make_label("—", 12, TEXT, True)
        self.pktcount_value = make_label("—", 12, TEXT, True)
        for r, (name, value) in enumerate([
            ("VALID PACKETS", self.valid_value), ("MALFORMED LINES", self.bad_value),
            ("MISSED packet_count VALUES", self.missed_value), ("LAST packet_count", self.pktcount_value),
            ("LAST PACKET AGE", self.age_value)
        ]):
            health_grid.addWidget(make_label(name, 8, MUTED, True), r, 0)
            health_grid.addWidget(value, r, 1)
        health.layout_box.addLayout(health_grid)
        right_layout.addWidget(health)
        splitter.addWidget(right)
        splitter.setSizes([680, 390])
        layout.addWidget(splitter, 1)

        bottom = QGridLayout()
        bottom.setSpacing(10)
        self.mini_pressure = MetricCard("Pressure", "pressure · Pa", CYAN, True)
        self.mini_ax = MetricCard("Accel X", "accel_x · unit not specified", GREEN, True)
        self.mini_ay = MetricCard("Accel Y", "accel_y · unit not specified", GREEN, True)
        self.mini_az = MetricCard("Accel Z", "accel_z · unit not specified", GREEN, True)
        self.mini_gyro = MetricCard("Gyro spin rate", "gyro_spin_rate · deg/s", GREEN, True)
        self.mini_rssi = MetricCard("RSSI", "dBm · receiver-reported, if sent", AMBER, True)
        self.mini_snr = MetricCard("SNR", "dB · receiver-reported, if sent", AMBER, True)
        for i, card in enumerate([self.mini_pressure, self.mini_ax, self.mini_ay, self.mini_az,
                                  self.mini_gyro, self.mini_rssi, self.mini_snr]):
            bottom.addWidget(card, 0, i)
        layout.addLayout(bottom)
        return page

    def build_telemetry_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 2, 0, 0)
        layout.setSpacing(12)
        grid = QGridLayout()
        grid.setSpacing(10)
        self.telemetry_cards = {}
        # One card per telemetry_packet_t field (title, then "field_name · type · unit").
        cards = [(f[0], f[4], f"{f[0]} · {f[1]} · {f[2]}") for f in TELEMETRY_FIELDS]
        cards.append(("vertical_velocity", "Vertical velocity", "derived by GUI (not a struct field) · m/s"))
        cards.append(("rssi_dbm", "RSSI", "receiver-reported, not in struct · dBm"))
        cards.append(("snr_db", "SNR", "receiver-reported, not in struct · dB"))
        for i, (key, title, unit) in enumerate(cards):
            accent = CYAN if key in ("altitude", "pressure", "latitude", "longitude") else GREEN
            card = MetricCard(title, unit, accent)
            self.telemetry_cards[key] = card
            grid.addWidget(card, i // 6, i % 6)
        layout.addLayout(grid)

        note = SectionPanel("TELEMETRY NOTES")
        self.telemetry_note = make_label(
            "Card names match the flight-software struct telemetry_packet_t. A dash means the field has not been received. "
            "altitude is barometric altitude AGL; gnss_altitude is GPS altitude MSL (not AGL). Accelerometer units are not "
            "specified by the firmware, so none are assumed. Vertical velocity is derived by the GUI from altitude. "
            "Checksum is shown as received and is NEVER used to accept or reject a packet: the byte range 0..57 depends on "
            "struct padding/packing, which the GUI does not know.",
            9, MUTED
        )
        self.telemetry_note.setWordWrap(True)
        note.layout_box.addWidget(self.telemetry_note)
        layout.addWidget(note)

        legend = SectionPanel("FLIGHT STATE LEGEND (fsw_state_t)", "flight_state byte")
        table = QTableWidget(8, 3)
        table.setHorizontalHeaderLabels(["Value", "Enum name", "Meaning"])
        for row in range(8):
            if row in FSW_STATES:
                name, desc = FSW_STATES[row]
            else:
                name, desc = "ST_COUNT", "Invalid as a runtime flight state (count sentinel). Shown as INVALID if received."
            for col, text in enumerate([str(row), name, desc]):
                item = QTableWidgetItem(text)
                if row == 7:
                    item.setForeground(QColor(AMBER))
                table.setItem(row, col, item)
        self.style_static_table(table, stretch_col=2, row_height=26)
        legend.layout_box.addWidget(table)
        phases = make_label(
            "GUI phases: LAUNCH_PAD → ASCENT → APOGEE → PRIMARY DESCENT → LANDED.   Simulation mapping: LAUNCH_PAD = ST_LAUNCH_PAD (2), "
            "ASCENT = ST_ASCENT (3), APOGEE = phase/event only (stays ST_ASCENT), PRIMARY DESCENT = ST_DESCENT (5), LANDED = ST_IMPACT (6).",
            8, MUTED)
        phases.setWordWrap(True)
        legend.layout_box.addWidget(phases)
        layout.addWidget(legend)
        layout.addStretch(1)
        return page

    @staticmethod
    def style_static_table(table, stretch_col, row_height=26, fit_content=True):
        """Read-only table. fit_content=True sizes it to its rows (no inner scrollbars)."""
        table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        table.setSelectionBehavior(QAbstractItemView.SelectRows)
        table.setAlternatingRowColors(True)
        table.verticalHeader().setVisible(False)
        table.verticalHeader().setDefaultSectionSize(row_height)
        header = table.horizontalHeader()
        for c in range(table.columnCount()):
            header.setSectionResizeMode(c, QHeaderView.Stretch if c == stretch_col else QHeaderView.ResizeToContents)
        table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        if fit_content:
            table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            total = table.horizontalHeader().height() + row_height * table.rowCount() + 6
            table.setFixedHeight(total)

    def build_radio_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 2, 0, 0)
        layout.setSpacing(12)
        stats = QGridLayout()
        stats.setSpacing(10)
        self.radio_rssi = MetricCard("RSSI", "dBm · receiver reported", AMBER)
        self.radio_snr = MetricCard("SNR", "dB · receiver reported", AMBER)
        self.radio_packets = MetricCard("Valid packets", "JSON objects parsed", GREEN)
        self.radio_bad = MetricCard("Malformed lines", "non-JSON / invalid JSON", RED)
        for i, card in enumerate([self.radio_rssi, self.radio_snr, self.radio_packets, self.radio_bad]):
            stats.addWidget(card, 0, i)
        layout.addLayout(stats)

        packet_panel = SectionPanel("RECEIVED PACKET HISTORY", "LATEST 500 · CURRENT FLIGHT/STREAM")
        tools_row = QHBoxLayout()
        self.packet_filter = QLineEdit()
        self.packet_filter.setPlaceholderText("Filter by phase, packet_count, source (SIM/LIVE) or packet content…")
        self.packet_filter.textChanged.connect(self.apply_packet_filter)
        tools_row.addWidget(self.packet_filter, 1)
        self.export_button = QPushButton("Export packet CSV")
        self.export_button.clicked.connect(self.export_csv)
        tools_row.addWidget(self.export_button)
        packet_panel.layout_box.addLayout(tools_row)
        self.packet_table = QTableWidget(0, 7)
        self.packet_table.setHorizontalHeaderLabels(
            ["Received", "Source", "packet_count", "Phase", "altitude (m AGL)", "RSSI (dBm)", "JSON summary"])
        header = self.packet_table.horizontalHeader()
        for c in range(6):
            header.setSectionResizeMode(c, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(6, QHeaderView.Stretch)
        self.packet_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.packet_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.packet_table.setAlternatingRowColors(True)
        packet_panel.layout_box.addWidget(self.packet_table, 1)
        layout.addWidget(packet_panel, 1)
        hint = make_label(
            "RSSI/SNR are displayed only when the receiver firmware includes rssi_dbm / snr_db in its USB JSON output. "
            "Rows marked SIM are simulated, not hardware data.",
            8, MUTED
        )
        layout.addWidget(hint)
        return page

    # ----- Flight Command Console (LOCAL ONLY) -----
    def build_command_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 2, 0, 0)
        layout.setSpacing(12)

        # Top row: current selection + the "not transmitted" indicator
        top = QHBoxLayout()
        top.setSpacing(10)
        self.cmd_name_card = MetricCard("Selected command", "command_state.selected_command", AMBER)
        self.cmd_name_card.value_label.setStyleSheet(f"color:{AMBER};font-size:16px;font-weight:700;")
        self.cmd_name_card.value_size = 16
        self.cmd_dec_card = MetricCard("Opcode (decimal)", "command_state.selected_opcode", CYAN)
        self.cmd_hex_card = MetricCard("Opcode (hex)", "cmd_opcode_t value", CYAN)
        top.addWidget(self.cmd_name_card, 3)
        top.addWidget(self.cmd_dec_card, 2)
        top.addWidget(self.cmd_hex_card, 2)

        badge = QFrame()
        badge.setObjectName("warnBadge")
        badge_layout = QVBoxLayout(badge)
        badge_layout.setContentsMargins(14, 10, 14, 10)
        badge_layout.addWidget(make_label("⚠  LIVE UPLINK — ACK ≠ EXECUTION", 11, AMBER, True))
        info = make_label(
            "Commands are sent through the bridge to the rocket. An ACK means the rocket received and validated "
            "the packet; it does not confirm that the FSW accepted it or that anything moved. "
            "Needs a live bridge connection.", 9, "#D9C48A")
        info.setWordWrap(True)
        badge_layout.addWidget(info)
        top.addWidget(badge, 5)
        layout.addLayout(top)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)

        # Left: terminal-style console
        terminal = SectionPanel("COMMAND TERMINAL", "UPLINK VIA BRIDGE")
        self.cmd_history = QPlainTextEdit()
        self.cmd_history.setReadOnly(True)
        mono = QFont("Consolas", 10)
        mono.setStyleHint(QFont.Monospace)
        self.cmd_history.setFont(mono)
        self.cmd_history.setStyleSheet(f"QPlainTextEdit {{ color:{GREEN}; background:#0B1422; }}")
        self.cmd_history.setPlaceholderText("Command history appears here (accepted commands only, timestamped).")
        terminal.layout_box.addWidget(self.cmd_history, 1)

        input_row = QHBoxLayout()
        prompt = QLabel("cmd>")
        prompt.setFont(mono)
        prompt.setStyleSheet(f"color:{GREEN};font-weight:700;")
        input_row.addWidget(prompt)
        self.cmd_input = QLineEdit()
        self.cmd_input.setFont(mono)
        self.cmd_input.setPlaceholderText("Type a command, e.g. CMD_START_TELEMETRY or CMD_SWITCH_CHANNEL BACKUP, then press Enter")
        self.cmd_input.returnPressed.connect(self.submit_command_from_input)
        input_row.addWidget(self.cmd_input, 1)
        self.cmd_store_button = QPushButton("Send Command")
        self.cmd_store_button.setObjectName("primaryButton")
        self.cmd_store_button.clicked.connect(self.submit_command_from_input)
        input_row.addWidget(self.cmd_store_button)
        terminal.layout_box.addLayout(input_row)

        self.cmd_feedback = make_label("Ready. Initial selection: CMD_NONE / 0x00.", 9, MUTED)
        self.cmd_feedback.setWordWrap(True)
        terminal.layout_box.addWidget(self.cmd_feedback)

        picker_row = QHBoxLayout()
        self.cmd_combo = QComboBox()
        for name, entry in command_state.COMMANDS.items():
            if name == "CMD_NONE":
                continue                                    # not an operator command
            if name == "CMD_SWITCH_CHANNEL":
                for channel in CHANNEL_ARGS:
                    self.cmd_combo.addItem(f"{name} {channel}  ({command_state.opcode_hex(entry['opcode'])})",
                                           f"{name} {channel}")
                continue
            self.cmd_combo.addItem(f"{name}  ({command_state.opcode_hex(entry['opcode'])})", name)
        picker_row.addWidget(self.cmd_combo, 1)
        self.cmd_use_button = QPushButton("Send Selected")
        self.cmd_use_button.clicked.connect(self.submit_command_from_combo)
        picker_row.addWidget(self.cmd_use_button)
        self.cmd_clear_button = QPushButton("Clear History")
        self.cmd_clear_button.clicked.connect(self.clear_command_history)
        picker_row.addWidget(self.cmd_clear_button)
        terminal.layout_box.addLayout(picker_row)
        splitter.addWidget(terminal)

        # Right: legend
        legend = SectionPanel("COMMAND LEGEND (cmd_opcode_t)", "CLICK A ROW TO FILL THE TERMINAL")
        self.cmd_table = QTableWidget(len(command_state.COMMANDS), 3)
        self.cmd_table.setHorizontalHeaderLabels(["Opcode", "Command", "Description"])
        for row, (name, entry) in enumerate(command_state.COMMANDS.items()):
            for col, text in enumerate([command_state.opcode_hex(entry["opcode"]), name, entry["description"]]):
                self.cmd_table.setItem(row, col, QTableWidgetItem(text))
        self.style_static_table(self.cmd_table, stretch_col=2, row_height=34, fit_content=False)
        self.cmd_table.setWordWrap(True)
        self.cmd_table.cellClicked.connect(self.on_legend_clicked)
        legend.layout_box.addWidget(self.cmd_table, 1)
        splitter.addWidget(legend)
        splitter.setSizes([640, 640])
        layout.addWidget(splitter, 1)
        return page

    def build_events_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 2, 0, 0)
        panel = SectionPanel("EVENT LOG", "CONNECTIONS · FLIGHT PHASES · ERRORS")
        controls = QHBoxLayout()
        self.record_button = QPushButton("Start CSV recording")
        self.record_button.setObjectName("primaryButton")
        self.record_button.clicked.connect(self.toggle_recording)
        self.clear_events_button = QPushButton("Clear visible log")
        self.clear_events_button.clicked.connect(self.clear_event_log)
        controls.addWidget(self.record_button)
        controls.addWidget(self.clear_events_button)
        controls.addStretch(1)
        panel.layout_box.addLayout(controls)
        self.event_text = QPlainTextEdit()
        self.event_text.setReadOnly(True)
        self.event_text.setLineWrapMode(QPlainTextEdit.WidgetWidth)
        panel.layout_box.addWidget(self.event_text, 1)
        layout.addWidget(panel, 1)
        return page

    def build_setup_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 2, 0, 0)
        layout.setSpacing(12)

        serial_panel = SectionPanel("SERIAL CONNECTION", "USB · GROUND ESP32")
        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignLeft)
        form.setFormAlignment(Qt.AlignTop)
        self.setup_port_combo = QComboBox()
        self.setup_baud_combo = QComboBox()
        self.setup_baud_combo.addItems(["115200", "57600", "9600", "230400", "460800"])
        self.setup_baud_combo.setCurrentText("115200")
        form.addRow("Detected port", self.setup_port_combo)
        form.addRow("Baud rate", self.setup_baud_combo)
        serial_panel.layout_box.addLayout(form)
        buttons = QHBoxLayout()
        scan2 = QPushButton("Scan ports")
        scan2.clicked.connect(self.scan_ports)
        apply_settings = QPushButton("Use these settings")
        apply_settings.setObjectName("primaryButton")
        apply_settings.clicked.connect(self.apply_setup_settings)
        buttons.addWidget(scan2)
        buttons.addWidget(apply_settings)
        buttons.addStretch(1)
        serial_panel.layout_box.addLayout(buttons)
        layout.addWidget(serial_panel)

        protocol = SectionPanel("EXPECTED RECEIVER OUTPUT")
        protocol_text = make_label(
            "The desktop app reads newline-delimited JSON over USB serial: one JSON object per line. Keys use the "
            "telemetry_packet_t field names listed below. The binary struct layout is NOT defined by this GUI "
            "(serialization is implemented separately in C++), and the C struct is not itself JSON. Optional extras that are "
            "not struct fields: gui_phase, simulated, rssi_dbm, snr_db. Older keys (altitude_m, temperature_c, lat, lon, "
            "gps_altitude_m, satellites, packet_id, phase, ...) are still accepted and renamed internally. "
            "Simulation and live serial input are mutually exclusive.",
            9, MUTED
        )
        protocol_text.setWordWrap(True)
        protocol.layout_box.addWidget(protocol_text)
        layout.addWidget(protocol)

        fmap = SectionPanel("TELEMETRY FIELD MAP (telemetry_packet_t)", "C TYPE · UNIT · GUI LOCATION")
        table = QTableWidget(len(TELEMETRY_FIELDS), 4)
        table.setHorizontalHeaderLabels(["Field", "C type", "Unit / note", "Shown in"])
        for row, (field, ctype, note, where, _title) in enumerate(TELEMETRY_FIELDS):
            for col, text in enumerate([field, ctype, note, where]):
                table.setItem(row, col, QTableWidgetItem(text))
        self.style_static_table(table, stretch_col=2, row_height=26)
        fmap.layout_box.addWidget(table)
        caution = make_label(
            "Checksum caution: the struct comment says checksum is the XOR of bytes 0..57. Padding, alignment and packing "
            "change byte offsets and total size between compilers/layouts, so this range is valid only for one defined byte "
            "representation. The GUI documents this dependency, does not verify checksums, and never rejects packets because "
            "of one. Simulated packets have no checksum.",
            8, AMBER)
        caution.setWordWrap(True)
        fmap.layout_box.addWidget(caution)
        layout.addWidget(fmap)

        hardware = SectionPanel("CURRENT ROCKET CONFIGURATION")
        hardware_text = make_label(
            "Flight computer: ESP32  •  Barometer: BMP585  •  IMU: BNO086  •  Radio: SX1276 LoRa  •  GPS: NEO-M6\n"
            "Ground receiver: antenna + SX1276 + ESP32, connected to this computer by USB.\n\n"
            "This interface is a telemetry dashboard, not a flight-control system. It does not transmit anything to the "
            "rocket: the Flight Command Console sends commands through the bridge. It does not replace onboard safety logic.",
            9, TEXT
        )
        hardware_text.setWordWrap(True)
        hardware.layout_box.addWidget(hardware_text)
        layout.addWidget(hardware)
        layout.addStretch(1)
        return page

    # ---------- Navigation and connection ----------
    def navigate(self, index):
        self.pages.setCurrentIndex(index)
        for i, button in enumerate(self.nav_buttons):
            button.setProperty("active", i == index)
            button.style().unpolish(button)
            button.style().polish(button)
        self.page_title.setText(self.page_titles[index][0])
        self.page_subtitle.setText(self.page_titles[index][1])

    def notify(self, title, text):
        """Single place for informational pop-ups (also makes headless testing easy)."""
        QMessageBox.information(self, title, text)

    def scan_ports(self):
        old = self.port_combo.currentData()
        self.port_combo.clear()
        if list_ports is None:
            self.port_combo.addItem("Install pyserial first", "")
            self.footer_status.setText("Missing dependency: pyserial")
            self.sync_setup_ports()
            return
        ports = list(list_ports.comports())
        if ports:
            for port in ports:
                label = f"{port.device} — {port.description}"
                self.port_combo.addItem(label, port.device)
            if old:
                index = self.port_combo.findData(old)
                if index >= 0:
                    self.port_combo.setCurrentIndex(index)
        else:
            self.port_combo.addItem("No serial ports found", "")
        self.sync_setup_ports()
        self.footer_status.setText(f"Serial scan complete · {len(ports)} port(s) found")
        self.add_event(f"Serial port scan: {len(ports)} port(s) found.")

    def sync_setup_ports(self):
        if not hasattr(self, "setup_port_combo"):
            return
        current = self.port_combo.currentData()
        self.setup_port_combo.clear()
        for i in range(self.port_combo.count()):
            self.setup_port_combo.addItem(self.port_combo.itemText(i), self.port_combo.itemData(i))
        idx = self.setup_port_combo.findData(current)
        if idx >= 0:
            self.setup_port_combo.setCurrentIndex(idx)
        if hasattr(self, "setup_baud_combo"):
            self.setup_baud_combo.setCurrentText(self.baud_combo.currentText())

    def apply_setup_settings(self):
        if self.connected:
            self.notify("Disconnect first", "Disconnect the serial receiver before changing port settings.")
            return
        port = self.setup_port_combo.currentData()
        idx = self.port_combo.findData(port)
        if idx >= 0:
            self.port_combo.setCurrentIndex(idx)
        self.baud_combo.setCurrentText(self.setup_baud_combo.currentText())
        self.add_event(f"Connection settings selected: {port or 'no port'} at {self.baud_combo.currentText()} baud.")
        self.navigate(0)

    def toggle_connection(self):
        if self.connected:
            self.disconnect_serial()
            return
        # Simulation and live serial must never run together.
        if self.simulation_active:
            self.notify("Stop the simulation first",
                        "The built-in flight simulation is running. Stop it before connecting the real receiver, "
                        "so simulated and live data are never mixed.")
            return
        if serial is None:
            QMessageBox.critical(self, "Missing dependency", "Install the serial package:\npy -m pip install pyserial")
            return
        port = self.port_combo.currentData()
        if not port:
            QMessageBox.warning(self, "No serial port", "Connect the receiver ESP32 and scan ports again.")
            return
        try:
            new_port = serial.Serial(
                port=port,
                baudrate=int(self.baud_combo.currentText()),
                timeout=0,
                write_timeout=0
            )
        except Exception as exc:
            QMessageBox.critical(self, "Serial connection failed", str(exc))
            self.add_event(f"Serial connection failed: {exc}")
            return
        # A new live stream starts from a clean slate (no simulated/previous data).
        self.reset_flight_state("live")
        self.sim_state = "idle"
        self.sim_button.setText("▶ Simulate Flight")
        self.serial_port = new_port
        self.connected = True
        self.connect_button.setText("Disconnect")
        self.connect_button.setObjectName("dangerButton")
        self.connect_button.style().unpolish(self.connect_button)
        self.connect_button.style().polish(self.connect_button)
        self.set_link_status("connected", f"Serial open on {port}; waiting for first valid packet")
        self.footer_status.setText(f"Connected · {port} · {self.baud_combo.currentText()} baud")
        self.add_event(f"Serial connection opened on {port} at {self.baud_combo.currentText()} baud (receive-only).")

    def disconnect_serial(self):
        try:
            if self.serial_port is not None and self.serial_port.is_open:
                self.serial_port.close()
        except Exception as exc:
            self.add_event(f"Serial close warning: {exc}")
        self.serial_port = None
        was_connected = self.connected
        self.connected = False
        self.connect_button.setText("Connect")
        self.connect_button.setObjectName("primaryButton")
        self.connect_button.style().unpolish(self.connect_button)
        self.connect_button.style().polish(self.connect_button)
        self.set_link_status("offline", "Connect the ground ESP32 using USB")
        self.footer_status.setText("Disconnected")
        if was_connected:
            self.add_event("Serial connection closed.")

    def set_link_status(self, state, detail):
        if state == "live":
            color, title = GREEN, "Telemetry live"
        elif state == "connected":
            color, title = AMBER, "Serial connected"
        elif state == "stale":
            color, title = AMBER, "Telemetry stale"
        elif state == "sim":
            color, title = AMBER, "Simulated data"
        else:
            color, title = RED, "Receiver disconnected"
        self.status_dot.setStyleSheet(f"color:{color};font-size:12px;font-weight:700;")
        self.sidebar_link.setStyleSheet(f"color:{color};font-size:10px;font-weight:700;")
        self.status_main.setText(title)
        self.status_detail.setText(detail)
        self.sidebar_link.setText(f"●  {title.upper()}")

    # ---------- Per-flight reset ----------
    def reset_flight_state(self, source):
        """
        Wipe EVERYTHING belonging to a previous flight/stream: graph data and curves, time base,
        packet counters, derivative history, phase memory, milestone flags, dashboard values,
        packet history. Called when a simulation starts/replays and when a live port is opened.
        The session-wide event log and command console are intentionally NOT cleared.
        """
        self.run_id += 1
        self.buffer = ""
        self.latest = {}
        self.stream_t0 = None
        self.last_packet_time = None
        self.packet_count = 0
        self.bad_packets = 0
        self.last_packet_count_field = None
        self.missed_packets = 0
        self.last_phase_key = ""
        self._last_invalid_state = None
        self.last_altitude = None
        self.last_altitude_t = None
        self.history_t.clear()
        self.history_alt.clear()
        self.history_gnss_alt.clear()
        self.history_vel.clear()
        self.packet_rows.clear()
        # simulation bookkeeping
        self.simulation_elapsed = 0.0
        self.simulation_packet_count = 0
        self.sim_max_altitude = 0.0
        self._apogee_logged = False
        self._primary_deploy_logged = False
        self._landed_logged = False
        # graphs: empty curves, x axis restarts at zero
        label = "Simulated flight time (s)" if source == "SIM" else "Time since first packet (s)"
        if self.alt_plot is not None:
            self.alt_curve.setData([], [])
            self.gnss_curve.setData([], [])
            for plot in (self.alt_plot, self.gnss_plot):
                plot.setLabel("bottom", label, **{"color": MUTED})
                plot.enableAutoRange(axis="y", enable=True)
            self.alt_plot.setXRange(0, 10, padding=0)
            self.gnss_plot.setXRange(0, 10, padding=0)
        self.reset_dashboard()
        self.refresh_packet_table()

    def reset_dashboard(self):
        for card in [self.phase_card, self.state_card, self.alt_card, self.vel_card, self.temp_card,
                     self.time_card, self.mini_pressure, self.mini_ax, self.mini_ay, self.mini_az,
                     self.mini_gyro, self.mini_rssi, self.mini_snr, self.radio_rssi, self.radio_snr,
                     self.radio_packets, self.radio_bad]:
            card.reset()
        for card in self.telemetry_cards.values():
            card.reset()
        self.lat_value.setText("—")
        self.lon_value.setText("—")
        self.gps_alt_value.setText("— m MSL")
        self.gps_time_value.setText("—")
        self.sats_value.setText("—")
        self.valid_value.setText("0")
        self.bad_value.setText("0")
        self.missed_value.setText("0")
        self.age_value.setText("—")
        self.pktcount_value.setText("—")
        self.radio_packets.set_value(0)
        self.radio_bad.set_value(0)

    # ---------- Built-in dummy rocket flight simulation ----------
    def toggle_simulation(self):
        if self.simulation_active:
            self.stop_simulation(complete=False)
        else:
            self.start_simulation()      # also serves as "Replay Flight"

    def start_simulation(self):
        if self.connected:
            self.notify("Disconnect receiver first",
                        "Disconnect the real serial receiver before starting the built-in simulation, "
                        "so simulated and live data are never mixed.")
            return
        self.simulation_timer.stop()              # never leave a previous run's timer active
        self.reset_flight_state("SIM")
        self.simulation_active = True
        self.sim_state = "running"
        self.start_time = time.monotonic()
        self.sim_button.setText("■ Stop Simulation")
        self.footer_status.setText("SIMULATION running · generated dummy data, NOT hardware")
        self.add_event("SIMULATION started: dummy rocket flight, no hardware data (clean state, t = 0 s).")
        self.emit_simulated_packet()              # packet #1 at t = 0
        if self.simulation_active:
            self.simulation_timer.start()

    def stop_simulation(self, complete=False, reason=None):
        self.simulation_timer.stop()
        was_active = self.simulation_active
        self.simulation_active = False
        self.sim_state = "complete" if complete else "stopped"
        self.sim_button.setText("↻ Replay Flight" if complete else "▶ Simulate Flight")
        if complete:
            self.footer_status.setText("Simulated flight complete · dummy data only")
        else:
            self.footer_status.setText(reason or "Flight simulation stopped")
            if was_active:
                self.add_event(reason or "SIMULATION stopped by operator.")

    def emit_simulated_packet(self):
        """Timer slot: generate ONE dummy packet and feed it through the normal ingest path."""
        if not self.simulation_active:            # stale callback guard
            return
        if self.connected:                        # defence in depth: never mix sources
            self.stop_simulation(reason="SIMULATION aborted: a live receiver is connected.")
            return
        t = self.simulation_elapsed
        self.simulation_packet_count += 1
        packet = build_simulated_packet(t, self.simulation_packet_count)
        self.simulation_elapsed += SIM_DT
        self.sim_max_altitude = max(self.sim_max_altitude, packet["altitude"])
        self.ingest_line(json.dumps(packet), "SIM", t)

        phase = packet["gui_phase"]
        if phase == "APOGEE" and not self._apogee_logged:
            self._apogee_logged = True
            self.add_event("APOGEE phase reached (target 990–1010 m) [SIMULATED].")
        if phase == "PRIMARY DESCENT" and not self._primary_deploy_logged:
            self._primary_deploy_logged = True
            self.add_event(f"PRIMARY DESCENT deployed after apogee at ~{packet['altitude']:.0f} m [SIMULATED].")
        if phase == "LANDED" and not self._landed_logged:
            self._landed_logged = True
            lo, hi = SIM_APOGEE_BAND
            verdict = "within" if lo <= self.sim_max_altitude <= hi else "OUTSIDE"
            self.add_event(f"LANDED: simulated flight complete. Peak altitude {self.sim_max_altitude:.1f} m "
                           f"({verdict} the {lo:.0f}–{hi:.0f} m band) [SIMULATED].")
            self.stop_simulation(complete=True)

    # ---------- Serial parsing and display ----------
    def poll_serial(self):
        if not self.connected or self.serial_port is None or self.simulation_active:
            return
        try:
            waiting = self.serial_port.in_waiting
            if not waiting:
                return
            chunk = self.serial_port.read(min(waiting, 16384)).decode("utf-8", errors="replace")
            self.buffer += chunk
            if len(self.buffer) > 250000:
                self.buffer = self.buffer[-50000:]
                self.add_event("Serial input buffer exceeded safety limit; old partial data discarded.")
            while "\n" in self.buffer:
                line, self.buffer = self.buffer.split("\n", 1)
                line = line.strip().lstrip("\ufeff")
                if line:
                    self.handle_bridge_line(line)
        except Exception as exc:
            self.add_event(f"Serial read error: {exc}")
            self.disconnect_serial()
    def handle_bridge_line(self, line):
        """Classify one bridge line: RX|rssi|snr|payload, INFO|..., ERR|..., or other text."""
        if line.startswith("RX|"):
            parts = line.split("|", 3)
            try:
                if len(parts) != 4:
                    raise ValueError("incomplete RX line")
                extra = {"rssi_dbm": float(parts[1]), "snr_db": float(parts[2])}
            except ValueError:
                self.ingest_line(line, "LIVE")          # counted as a malformed packet
                return
            payload = parts[3].strip()
            if payload.startswith("ACK,"):              # rocket reply, not telemetry
                self.handle_rocket_ack(payload, parts[1])
                return
            self.ingest_line(payload, "LIVE", extra=extra)
        elif line.startswith(("INFO|", "ERR|")):
            self.add_event(f"Bridge: {line[:180]}")
            self.note_bridge_status(line)
        else:
            self.add_event(f"Bridge text: {line[:180]}")   # e.g. boot output; not counted as a bad packet
    
    def ingest_line(self, line, source, t_x=None, extra=None):
        """
        Single entry point for telemetry. `source` is "LIVE" (serial) or "SIM"; it is decided by
        the CALLER, never by packet content. Packets from the inactive source are dropped.
        t_x = graph x value in seconds (simulated time for SIM; time since first packet for LIVE).
        """
        if source == "LIVE" and self.simulation_active:
            return
        if source == "SIM" and not self.simulation_active:
            return
        try:
            data = parse_ascii_telemetry(line) if source == "LIVE" else parse_telemetry_line(line)
        except ValueError:
            self.bad_packets += 1
            self.bad_value.setText(str(self.bad_packets))
            self.radio_bad.set_value(self.bad_packets)
            if line and not line.startswith("{"):
                self.add_event(f"Receiver text: {line[:180]}")
            return

        raw_fields = data.pop("_raw_fields", None)   # the 17 text fields exactly as received
        if extra:
            data.update(extra)                        # RSSI / SNR reported by the bridge

        now = time.monotonic()
        if source == "LIVE":
            if self.stream_t0 is None:
                self.stream_t0 = now
            t_x = now - self.stream_t0
            self.last_packet_time = now
        elif t_x is None:
            t_x = 0.0
        self.latest = data
        self.packet_count += 1

        # Missed-packet estimate from the 16-bit packet_count field (handles wrap-around).
        pc = data.get("packet_count")
        if isinstance(pc, int) and not isinstance(pc, bool):
            if self.last_packet_count_field is not None:
                diff = (pc - self.last_packet_count_field) & 0xFFFF
                if 1 < diff < 0x8000:
                    self.missed_packets += diff - 1
                elif diff == 0 or diff >= 0x8000:
                    self.add_event(f"packet_count did not advance ({self.last_packet_count_field} → {pc}); "
                                   "possible duplicate or rocket restart.")
            self.last_packet_count_field = pc

        altitude = self.number(data, "altitude")
        gnss_alt = self.number(data, "gnss_altitude")
        temperature = self.number(data, "temperature")

        # Vertical velocity is DERIVED by the GUI (not a struct field).
        velocity = None
        if altitude is not None:
            if self.last_altitude is not None and self.last_altitude_t is not None:
                dt = t_x - self.last_altitude_t
                if dt > 0.02:
                    velocity = (altitude - self.last_altitude) / dt
            self.last_altitude = altitude
            self.last_altitude_t = t_x

        self.history_t.append(t_x)
        self.history_alt.append(altitude if altitude is not None else float("nan"))
        self.history_gnss_alt.append(gnss_alt if gnss_alt is not None else float("nan"))
        self.history_vel.append(velocity if velocity is not None else float("nan"))

        # Flight state / phase
        state_raw = data.get("flight_state")
        state_text, state_ok = describe_flight_state(state_raw)
        phase = phase_for_packet(data)
        tag = " [SIMULATED]" if source == "SIM" else ""
        self.phase_card.set_value(phase or "—",
                                  "GUI phase · SIMULATED" if source == "SIM" else "GUI phase · from flight_state")
        self.state_card.set_value(state_text, "flight_state · fsw_state_t")
        self.state_card.set_accent(GREEN if state_ok else RED)
        if not state_ok and self._last_invalid_state != state_text:
            self.add_event(f"Invalid flight_state received: {state_text}")
        self._last_invalid_state = None if state_ok else state_text

        self.alt_card.set_value(altitude, "altitude · m AGL" + (" · SIMULATED" if source == "SIM" else ""))
        self.vel_card.set_value(velocity, "derived by GUI from altitude · m/s")
        self.temp_card.set_value(temperature, "temperature · degC")
        mt = data.get("mission_time")
        self.time_card.set_value(mt if isinstance(mt, (int, float)) and not isinstance(mt, bool) else None,
                                 "mission_time · s since power-on")

        lat = self.number(data, "latitude")
        lon = self.number(data, "longitude")
        self.lat_value.setText(self.fmt(lat, 6))
        self.lon_value.setText(self.fmt(lon, 6))
        self.gps_alt_value.setText(self.fmt(gnss_alt) + " m MSL")
        self.gps_time_value.setText(self.fmt_gnss_time(data.get("gnss_time")))
        self.sats_value.setText(self.fmt(self.number(data, "gnss_sats"), 0))

        for key, card in self.telemetry_cards.items():
            card.set_value(self.card_value(key, data, velocity, state_text), None)
        self.mini_pressure.set_value(self.number(data, "pressure"))
        self.mini_ax.set_value(self.number(data, "accel_x"))
        self.mini_ay.set_value(self.number(data, "accel_y"))
        self.mini_az.set_value(self.number(data, "accel_z"))
        self.mini_gyro.set_value(self.number(data, "gyro_spin_rate"))
        self.mini_rssi.set_value(self.number(data, "rssi_dbm"))
        self.mini_snr.set_value(self.number(data, "snr_db"))

        self.valid_value.setText(str(self.packet_count))
        self.bad_value.setText(str(self.bad_packets))
        self.missed_value.setText(str(self.missed_packets))
        self.pktcount_value.setText(str(pc) if pc is not None else "—")
        self.radio_packets.set_value(self.packet_count)
        self.radio_rssi.set_value(self.number(data, "rssi_dbm"))
        self.radio_snr.set_value(self.number(data, "snr_db"))
        self.radio_bad.set_value(self.bad_packets)

        if self.alt_plot is not None and len(self.history_t) > 1:
            ts = list(self.history_t)
            self.alt_curve.setData(ts, list(self.history_alt))
            self.gnss_curve.setData(ts, list(self.history_gnss_alt))
            x_max = max(10.0, ts[-1])
            self.alt_plot.setXRange(0, x_max, padding=0)      # x axis always starts at zero
            self.gnss_plot.setXRange(0, x_max, padding=0)

        self.add_packet_row(data, altitude, source, phase)
        self.footer_status.setText(
            f"{'SIMULATED' if source == 'SIM' else 'Last valid'} packet · {datetime.now().strftime('%H:%M:%S')} · {len(data)} field(s)"
        )
        if self.csv_writer is not None and source == "LIVE" and raw_fields is not None:
            self.write_csv_row(raw_fields)

        phase_key = phase or state_text
        if phase_key and phase_key != self.last_phase_key:
            self.add_event(f"Flight phase changed: {self.last_phase_key or 'none'} → {phase_key}{tag}")
            self.last_phase_key = phase_key

    @staticmethod
    def number(data, *keys):
        for key in keys:
            value = data.get(key)
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                return float(value)
        return None

    @staticmethod
    def fmt(value, digits=2):
        return "—" if value is None else f"{value:.{digits}f}"

    @staticmethod
    def fmt_gnss_time(value):
        """gnss_time is HHMMSS packed into a uint32 -> 'HH:MM:SS (HHMMSS)'. Shows raw value if malformed."""
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return "—"
        v = int(value)
        hh, mm, ss = v // 10000, (v // 100) % 100, v % 100
        if v < 0 or hh > 23 or mm > 59 or ss > 59:
            return f"{v} (invalid HHMMSS)"
        return f"{hh:02d}:{mm:02d}:{ss:02d}  ({v:06d})"

    def card_value(self, key, data, velocity, state_text):
        """Value shown on a Telemetry-page card for field `key`."""
        if key == "team_number":
            value = data.get("team_number")
            return None if value is None else str(value)
        if key == "vertical_velocity":
            return velocity
        if key == "flight_state":
            return state_text if data.get("flight_state") is not None else None
        if key == "checksum":
            if data.get("simulated"):
                return "N/A (simulated)"
            cs = data.get("checksum")
            if isinstance(cs, int) and not isinstance(cs, bool):
                return f"0x{cs & 0xFF:02X} · unverified"
            return None if cs is None else f"{cs} · unverified"
        if key == "gnss_time":
            return self.fmt_gnss_time(data.get("gnss_time")) if data.get("gnss_time") is not None else None
        value = data.get(key)
        if value is None:
            return None
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return f"invalid: {value}"
        ctype = FIELD_INFO[key][1] if key in FIELD_INFO else "float"
        if ctype.startswith("uint") and float(value).is_integer():
            return int(value)
        return float(value)

    # ---------- Packet table ----------
    @staticmethod
    def row_matches(item, query):
        if not query:
            return True
        searchable = " ".join([item["source"], item["packet_count"], item["phase"], item["summary"]]).lower()
        return query in searchable

    def add_packet_row(self, data, altitude, source, phase):
        received = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        item = {
            "received": received,
            "source": source,
            "packet_count": str(data.get("packet_count", "—")),
            "phase": str(phase or "—"),
            "altitude": self.fmt(altitude),
            "rssi": self.fmt(self.number(data, "rssi_dbm")),
            "summary": json.dumps(data, ensure_ascii=False, separators=(",", ":")),
        }
        was_full = len(self.packet_rows) == self.packet_rows.maxlen
        self.packet_rows.append(item)
        query = self.packet_filter.text().strip().lower()
        if query:
            self.refresh_packet_table()
        else:
            if was_full:
                self.packet_table.removeRow(0)
            self.append_table_row(item)
            self.packet_table.scrollToBottom()

    def append_table_row(self, item):
        row = self.packet_table.rowCount()
        self.packet_table.insertRow(row)
        values = [item["received"], item["source"], item["packet_count"], item["phase"],
                  item["altitude"], item["rssi"], item["summary"]]
        for col, value in enumerate(values):
            cell = QTableWidgetItem(value)
            cell.setFlags(cell.flags() & ~Qt.ItemIsEditable)
            if col == 1 and value == "SIM":
                cell.setForeground(QColor(AMBER))
            self.packet_table.setItem(row, col, cell)

    def refresh_packet_table(self, filter_text=None):
        if not hasattr(self, "packet_table"):
            return
        if filter_text is None:
            filter_text = self.packet_filter.text() if hasattr(self, "packet_filter") else ""
        query = (filter_text or "").strip().lower()
        self.packet_table.setRowCount(0)
        for item in self.packet_rows:
            if self.row_matches(item, query):
                self.append_table_row(item)
        self.packet_table.scrollToBottom()

    def apply_packet_filter(self, text):
        self.refresh_packet_table(text)

    # ---------- Flight Command Console (LOCAL ONLY -- never transmits) ----------
    def refresh_command_display(self):
        # Read through the module so the GUI always shows the CURRENT stored values.
        self.cmd_name_card.set_value(command_state.selected_command)
        self.cmd_dec_card.set_value(str(command_state.selected_opcode))
        self.cmd_hex_card.set_value(command_state.opcode_hex())

    def cmd_note(self, text, color=GREEN, bold=False):
        self.cmd_feedback.setText(text)
        self.cmd_feedback.setStyleSheet(f"color:{color};font-size:9px;font-weight:{700 if bold else 400};")

    def cmd_log(self, text):
        stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.cmd_history.appendPlainText(f"[{stamp}] {text}")
        self.add_event(text)

    def cmd_reject(self, text):
        self.cmd_note(text, RED, True)
        self.add_event(text)

    def confirm_action(self, title, text):
        """Single place for yes/no pop-ups (also makes headless testing easy). Default answer: No."""
        answer = QMessageBox.question(self, title, text,
                                      QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        return answer == QMessageBox.Yes

    def apply_command(self, text):
        """Validate the typed/selected command, then send it to the rocket through the bridge."""
        parts = text.split() if isinstance(text, str) else []
        ok, result = command_state.set_command(parts[0] if parts else "")
        if ok and result == "CMD_NONE":
            ok, result = False, "CMD_NONE is not an operator command."
        arg = 0
        if ok:
            word = parts[1].upper() if len(parts) > 1 else None
            if result == "CMD_SWITCH_CHANNEL":
                if word not in CHANNEL_ARGS:
                    ok, result = False, "CMD_SWITCH_CHANNEL needs MAIN or BACKUP, e.g. CMD_SWITCH_CHANNEL BACKUP."
                else:
                    arg = CHANNEL_ARGS[word]
            elif word is not None:
                ok, result = False, f"{result} takes no argument (toggle sequence numbers are automatic)."
        if ok:
            ok = self.send_command(result, arg)
            if ok:
                self.cmd_input.clear()
        else:
            self.cmd_note(f"ERROR: {result}", RED, True)
            self.add_event(f"Command rejected (invalid input): {result}")
        self.refresh_command_display()
        return ok

    def send_command(self, name, arg):
        """Encode one request and start sending it. Returns True if the request was started."""
        opcode = command_state.COMMANDS[name]["opcode"]
        if self.simulation_active or not self.connected or self.serial_port is None:
            self.cmd_reject("Not sent: no live bridge connection (connect the bridge; stop the simulation).")
            return False
        if self.pending_cmd is not None:
            self.cmd_reject("Not sent: the previous command is still in progress.")
            return False
        if opcode in TOGGLE_OPCODES:
            state = self.latest.get("flight_state")
            if isinstance(state, int) and state > 2:        # 2 = ST_LAUNCH_PAD; the firmware also refuses
                self.cmd_reject(f"Not sent: telemetry shows the rocket has left LAUNCH_PAD (state {state}).")
                return False
        if name == "CMD_TRIGGER_SEP":
            if not self.confirm_action("Confirm separation",
                                       "Send CMD_TRIGGER_SEP?\n\nThis triggers the separation mechanism "
                                       "if the rocket accepts it."):
                self.cmd_log("CMD_TRIGGER_SEP cancelled by the operator (not sent).")
                return False
        if opcode in TOGGLE_OPCODES:
            arg = self.toggle_seq[opcode]                    # the argument IS this request's sequence number
            self.toggle_seq[opcode] = (arg + 1) & 0xFF
        packet = encode_command(opcode, arg)
        hex_text = " ".join(f"{b:02X}" for b in packet)
        self.pending_cmd = {
            "name": name, "opcode": opcode, "arg": arg, "hex": hex_text,
            "line": f"CMD {hex_text}\n".encode(),            # the same bytes are reused for every repeat
            "sent": 0, "ticks": 0, "bridge_tx": 0,
            "channel": ("BACKUP" if arg else "MAIN") if opcode == 0x08 else None,
        }
        self.cmd_log(f"{name} opcode {command_state.opcode_hex(opcode)} arg {arg}: {hex_text}")
        self.cmd_note(f"{name} sent to the bridge; waiting for the rocket's ACK. "
                      "An ACK is not proof the action was carried out.", AMBER)
        if self.cmd_transmit():
            self.cmd_timer.start(TC_TICK_MS)
        return True

    def cmd_transmit(self):
        p = self.pending_cmd
        try:
            self.serial_port.write(p["line"])
        except Exception as exc:
            self.cmd_log(f"{p['name']}: serial write FAILED ({exc}); not sent.")
            self.cmd_note(f"Serial write failed: {exc}", RED, True)
            self.cmd_finish()
            return False
        p["sent"] += 1
        p["ticks"] = 0
        self.cmd_log(f"WRITTEN to serial ({p['sent']}/{TC_REPEATS}): {p['hex']}")
        return True

    def cmd_tick(self):
        p = self.pending_cmd
        if p is None:
            self.cmd_timer.stop()
            return
        if not self.connected or self.serial_port is None:
            self.cmd_log(f"{p['name']}: connection lost before an ACK arrived.")
            self.cmd_finish()
            return
        p["ticks"] += 1
        if p["sent"] < TC_REPEATS:
            self.cmd_transmit()
        elif p["ticks"] >= TC_ACK_WAIT_TICKS:
            self.cmd_no_ack()

    def cmd_finish(self):
        self.pending_cmd = None
        self.cmd_timer.stop()

    def cmd_no_ack(self):
        p = self.pending_cmd
        self.cmd_finish()
        if p["opcode"] == 0x02:
            self.cmd_log("CMD_STOP_TELEMETRY: no ACK is expected (the rocket turns TX off first). "
                         "Confirm by watching telemetry stop.")
            self.cmd_note("STOP sent. No ACK is expected; confirm by telemetry stopping.", AMBER)
            return
        self.cmd_log(f"NO ACK for {p['name']} arg {p['arg']} after {p['sent']} transmissions. Delivery unknown: "
                     "the rocket only ACKs while its telemetry is ON, and the ACK itself may have been lost.")
        self.cmd_note(f"No ACK for {p['name']}. Delivery unknown.", RED, True)
        if p["channel"] is not None and self.confirm_action(
                "No ACK for channel switch",
                f"No ACK was received. The rocket may or may not have switched to {p['channel']}.\n\n"
                f"Retune the bridge to {p['channel']} anyway?"):
            self.bridge_retune(p["channel"])

    def handle_rocket_ack(self, payload, rssi_text):
        """payload: ACK,<team>,<opcode hex>,<argument>"""
        parts = payload.split(",")
        try:
            if len(parts) != 4 or parts[0] != "ACK":
                raise ValueError
            team, op, arg = int(parts[1]), int(parts[2], 16), int(parts[3])
        except ValueError:
            self.add_event(f"Malformed ACK ignored: {payload[:60]}")
            return
        p = self.pending_cmd
        if p is None or team != TC_TEAM_NUMBER or op != p["opcode"] or arg != p["arg"]:
            self.add_event(f"Rocket ACK with no matching pending command (repeat or late): {payload}")
            return
        self.cmd_finish()
        self.cmd_log(f"ACK received (RSSI {rssi_text} dBm): the rocket received and validated {p['name']} "
                     f"arg {p['arg']}. Not reported: FSW acceptance or execution.")
        self.cmd_note(f"{p['name']}: ACK received (rocket received and validated it). "
                      "Execution is NOT confirmed.", GREEN)
        if p["channel"] is not None:
            self.bridge_retune(p["channel"])               # the ACK left on the old channel; now follow

    def note_bridge_status(self, line):
        p = self.pending_cmd
        if p is None:
            return
        if line.startswith("INFO|sent"):
            p["bridge_tx"] += 1
            self.cmd_log(f"BRIDGE transmitted over radio ({p['bridge_tx']}/{p['sent']}): {line[5:]}")
        elif line.startswith("ERR|send failed"):
            self.cmd_log("BRIDGE reported: radio send FAILED")

    def bridge_retune(self, channel):
        try:
            self.serial_port.write(f"CH {channel}\n".encode())
            self.cmd_log(f"BRIDGE retune requested: CH {channel} (it should answer INFO|listening on ...)")
        except Exception as exc:
            self.cmd_log(f"BRIDGE retune write FAILED: {exc}")

    def submit_command_from_input(self):
        return self.apply_command(self.cmd_input.text())

    def submit_command_from_combo(self):
        return self.apply_command(self.cmd_combo.currentData())

    def on_legend_clicked(self, row, _col):
        item = self.cmd_table.item(row, 1)
        if item is not None:
            self.cmd_input.setText(item.text())
            self.cmd_input.setFocus()

    def clear_command_history(self):
        # Clears only the log. The stored selection is deliberately left alone.
        self.cmd_history.clear()
        self.cmd_feedback.setText(
            f"History cleared. Selected command unchanged: {command_state.selected_command} "
            f"({command_state.opcode_hex()}).")
        self.cmd_feedback.setStyleSheet(f"color:{MUTED};font-size:9px;font-weight:400;")
        self.refresh_command_display()

    # ---------- Logging and CSV ----------
    def add_event(self, message):
        if hasattr(self, "event_text"):
            stamp = datetime.now().strftime("%H:%M:%S")
            self.event_text.appendPlainText(f"[{stamp}] {message}")
            self.event_count += 1

    def clear_event_log(self):
        self.event_text.clear()
        self.add_event("Visible event log cleared.")

    def toggle_recording(self):
        """Telemetry CSV only. Operator commands are never written here."""
        if self.csv_writer is not None:
            try:
                self.csv_handle.flush()
                self.csv_handle.close()
            except Exception:
                pass
            self.csv_writer = None
            self.csv_handle = None
            self.record_button.setText("Start CSV recording")
            self.add_event("CSV recording stopped.")
            return

        path, _ = QFileDialog.getSaveFileName(
            self, "Record telemetry", f"rocket_telemetry_{datetime.now():%Y%m%d_%H%M%S}.csv",
            "CSV files (*.csv)"
        )
        if not path:
            return
        try:
            self.csv_handle = open(path, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_handle)
            self.csv_writer.writerow(ASCII_COLUMNS)
            self.record_button.setText("Stop CSV recording")
            self.add_event(f"CSV recording started: {path}")
        except Exception as exc:
            self.csv_handle = None
            self.csv_writer = None
            QMessageBox.critical(self, "Recording failed", str(exc))

    def write_csv_row(self, raw_fields):
        """One row per valid LIVE packet: the 17 fields exactly as the rocket sent them."""
        if self.csv_writer is None or self.csv_handle is None:
            return
        self.csv_writer.writerow(raw_fields)
        self.csv_handle.flush()

    def export_csv(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "Export packet history", f"rocket_packets_{datetime.now():%Y%m%d_%H%M%S}.csv",
            "CSV files (*.csv)"
        )
        if not path:
            return
        try:
            with open(path, "w", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle)
                writer.writerow(["received_at", "source", "packet_count", "mission_phase",
                                 "altitude_m_agl", "rssi_dbm", "raw_json"])
                query = self.packet_filter.text().strip().lower()
                count = 0
                for item in self.packet_rows:
                    if not self.row_matches(item, query):
                        continue
                    writer.writerow([
                        item["received"], "SIMULATED" if item["source"] == "SIM" else "LIVE",
                        item["packet_count"], item["phase"], item["altitude"], item["rssi"], item["summary"]
                    ])
                    count += 1
            QMessageBox.information(self, "Export complete", f"Exported {count} packet(s) to:\n{path}")
            self.add_event(f"Exported {count} packet(s) to CSV.")
        except Exception as exc:
            QMessageBox.critical(self, "Export failed", str(exc))
            self.add_event(f"CSV export failed: {exc}")

    # ---------- Status and shutdown ----------
    def refresh_source_badge(self):
        if self.simulation_active or self.sim_state == "complete":
            self.source_badge.setText("⚠ SIMULATED DATA — NOT LIVE HARDWARE")
            self.source_badge.setStyleSheet(f"color:{AMBER};font-size:9px;font-weight:700;")
            sub = "BAROMETRIC (AGL) + GNSS (MSL) · SIMULATED"
        elif self.connected:
            self.source_badge.setText("DATA SOURCE: LIVE SERIAL")
            self.source_badge.setStyleSheet(f"color:{GREEN};font-size:9px;font-weight:700;")
            sub = "BAROMETRIC (AGL) + GNSS (MSL)"
        else:
            self.source_badge.setText("DATA SOURCE: NONE")
            self.source_badge.setStyleSheet(f"color:{MUTED};font-size:9px;font-weight:700;")
            sub = "BAROMETRIC (AGL) + GNSS (MSL)"
        if self.profile_panel.subtitle_label is not None:
            self.profile_panel.subtitle_label.setText(sub)

    def update_status(self):
        elapsed = max(0, int(time.monotonic() - self.start_time))
        self.elapsed_label.setText(f"SESSION {elapsed//3600:02d}:{(elapsed%3600)//60:02d}:{elapsed%60:02d}")
        self.refresh_source_badge()
        if self.simulation_active:
            self.set_link_status("sim", f"SIMULATION · t = {max(0.0, self.simulation_elapsed - SIM_DT):.1f} s · "
                                        f"packet {self.simulation_packet_count} · not hardware data")
            self.age_value.setText("n/a (simulation)")
        elif self.sim_state == "complete":
            self.set_link_status("sim", "SIMULATION COMPLETE · press Replay Flight to run again")
        elif self.connected:
            if self.last_packet_time is None:
                self.set_link_status("connected", "Serial open; waiting for valid JSON telemetry")
            else:
                age = time.monotonic() - self.last_packet_time
                self.age_value.setText(f"{age:.1f} s")
                if age > 3.0:
                    self.set_link_status("stale", f"No valid packet for {age:.1f} seconds")
                else:
                    self.set_link_status("live", f"Last packet {age:.1f} s ago · {self.packet_count} valid")
        elif self.sim_state == "stopped":
            self.set_link_status("offline", "Simulation stopped · no live receiver connected")
        else:
            self.set_link_status("offline", "Connect the ground ESP32 using USB")

    def closeEvent(self, event):
        for timer in (self.simulation_timer, self.poll_timer, self.ui_timer, self.cmd_timer):
            timer.stop()
        self.simulation_active = False
        if self.csv_handle is not None:
            try:
                self.csv_handle.flush()
                self.csv_handle.close()
            except Exception:
                pass
            self.csv_handle = None
            self.csv_writer = None
        self.disconnect_serial()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("ORBIT Rocket Ground Station")
    app.setFont(QFont("Segoe UI", 9))
    window = GroundStation()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
