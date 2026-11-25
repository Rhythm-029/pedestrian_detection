"""
Web UI Server — Pedestrian Detection Vehicle
Run: python3 scripts/start_web_ui.py --port 9999

Features:
  - Real-time ESP32 serial monitoring via Socket.IO
  - Full ACK/NACK command protocol with pending tracking and timeout
  - STATUS: and CAPS: line parsing → live telemetry
  - Zone injection (FAR / CAUTION / DANGER) + continuous heartbeat
  - Hardware command dispatch: LED, Buzzer, Motor channels
  - Diagnostic mode control (SET_MODE:DIAG / SET_MODE:AUTO)
  - E-STOP and latch reset
  - Test runner (pytest + Pi simulation)
  - Settings management + hardware test result tracking
"""
from __future__ import annotations
import os, sys, json, time, threading, subprocess, uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(ROOT))

from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

try:
    from src.common.protocol import make_message, encode_message
    PROTOCOL_AVAILABLE = True
except ImportError:
    PROTOCOL_AVAILABLE = False

# ---------------------------------------------------------------------------
# Flask + SocketIO
# ---------------------------------------------------------------------------
TEMPLATE_DIR = Path(__file__).parent / "templates"
app = Flask(__name__, template_folder=str(TEMPLATE_DIR))
app.config["SECRET_KEY"] = "pdv-webui-2026"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# ---------------------------------------------------------------------------
# Default settings
# ---------------------------------------------------------------------------
_DEFAULT_SETTINGS = {
    "confidence_threshold": 0.50,
    "us_danger_threshold_cm": 50,
    "motor_slow_pct": 20,
    "motor_medium_pct": 40,
    "motor_fast_pct": 60,
    "heartbeat_interval_ms": 100,
    "log_max_lines": 1000,
    "auto_scroll": True,
}

CMD_TIMEOUT_MS = 2000  # ms to wait for ACK before declaring timeout

# ---------------------------------------------------------------------------
# Application state
# ---------------------------------------------------------------------------
class AppState:
    def __init__(self):
        self.lock = threading.Lock()
        self.serial_conn = None
        self._serial_running = False
        self._hb_running = {"active": False}
        self.seq = 0
        self.settings = dict(_DEFAULT_SETTINGS)
        self.hw_test_results: dict = {}

        # Pending commands awaiting ACK: cmd_key → {cmd, sent_at_ms, sid}
        self._pending: dict = {}

        # Firmware capabilities (populated from CAPS: line)
        self.caps = {
            "motor": False,
            "led": False,
            "buzzer": False,
            "us": False,
            "diag": False,
            "estop": False,
        }
        # "unified" = pdv_unified.ino (ACK support)
        # "legacy"  = old firmware (no ACK, optimistic mode)
        # "unknown" = not yet determined
        self.firmware_mode = "unknown"

        # Live telemetry
        self.status = {
            "connected": False,
            "port": None,
            "baud": 115200,
            "zone": None,
            "distance_m": None,
            "confidence": 0.0,
            "pedestrian": False,
            "seq_rx": 0,
            "esp32_state": "DISCONNECTED",
            "esp32_mode": "AUTO",
            "us_raw": None,
            "us_cal": None,
            "us_status": None,
            "hb_active": False,
            "hb_zone": None,
            "motor_channel": "NONE",
            "motor_direction": "STOPPED",
            "motor_pwm_pct": 0,
            "led_on": False,
            "buzzer_on": False,
            "motor_outputs_enabled": False,
            "firmware": "unknown",
            "model_running": False,
            "model_fps": 0.0,
        }
        self._model_running = False

    # ------------------------------------------------------------------
    # Serial connect / disconnect
    # ------------------------------------------------------------------
    def connect(self, port: str, baud: int) -> None:
        self.disconnect()
        self.serial_conn = serial.Serial(port, baud, timeout=0.5)
        self._serial_running = True
        self.status.update(connected=True, port=port, baud=baud, esp32_state="CONNECTED", firmware="unknown")
        threading.Thread(target=self._reader,  daemon=True).start()
        threading.Thread(target=self._timeout_watcher, daemon=True).start()
        # Request capabilities immediately
        threading.Timer(0.5, self._request_caps).start()

    def _request_caps(self):
        self.write(b"GET_CAPABILITIES\n")
        time.sleep(0.3)
        self.write(b"GET_STATUS\n")
        # Give firmware 2s to respond with CAPS; if silent → legacy mode
        def _check_mode():
            time.sleep(2.0)
            if self.firmware_mode == "unknown":
                self.firmware_mode = "legacy"
                self.status["firmware"] = "legacy (old firmware)"
                socketio.emit("firmware_mode", {"mode": "legacy"})
                socketio.emit("toast", {"msg": "⚠ Old firmware detected — LED/Buzzer buttons use optimistic mode. Upload pdv_unified.ino for full ACK support.", "level": "warning"})
                socketio.emit("status_update", dict(self.status))
        threading.Thread(target=_check_mode, daemon=True).start()

    def disconnect(self) -> None:
        self._serial_running = False
        self._hb_running["active"] = False
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
            except Exception:
                pass
        self.serial_conn = None
        self.caps = {k: False for k in self.caps}
        self.firmware_mode = "unknown"
        self.status.update(connected=False, esp32_state="DISCONNECTED",
                           hb_active=False, hb_zone=None,
                           motor_direction="STOPPED", motor_pwm_pct=0,
                           led_on=False, buzzer_on=False, firmware="unknown")
        # Reject all pending commands
        with self.lock:
            for key, entry in list(self._pending.items()):
                socketio.emit("cmd_nack", {"cmd": entry["cmd"], "reason": "DISCONNECTED"}, to=entry.get("sid"))
            self._pending.clear()

    def write(self, data: bytes) -> bool:
        with self.lock:
            if self.serial_conn and self.serial_conn.is_open:
                try:
                    self.serial_conn.write(data)
                    self.serial_conn.flush()
                    return True
                except Exception:
                    pass
        return False

    # ------------------------------------------------------------------
    # Dispatch a command and register it as pending
    # ------------------------------------------------------------------
    def send_cmd(self, cmd: str, sid: str | None = None) -> bool:
        """Send cmd over serial. In unified mode: register ACK pending.
        In legacy mode: emit optimistic ACK after short delay (no ACK from old firmware)."""
        ok = self.write((cmd + "\n").encode())
        if ok:
            key = cmd.split(":")[0] if ":" in cmd else cmd
            socketio.emit("serial_line", {"line": f"[CMD] {cmd}", "ts": _now_ms(), "tx": True})
            if self.firmware_mode == "unified":
                with self.lock:
                    self._pending[key] = {"cmd": cmd, "sent_at_ms": _now_ms(), "sid": sid}
            else:
                # Legacy / unknown: emit optimistic ACK after 150ms and update local status
                target_sid = sid
                def _optimistic():
                    time.sleep(0.15)
                    if cmd == "LED_ON":
                        self.status["led_on"] = True
                    elif cmd in ("LED_OFF", "LED_BLINK"):
                        self.status["led_on"] = False
                    elif cmd == "BUZZER_ON":
                        self.status["buzzer_on"] = True
                    elif cmd in ("BUZZER_OFF", "BUZZER_BEEP"):
                        self.status["buzzer_on"] = False
                    elif cmd == "SET_MODE:DIAG":
                        self.status["esp32_mode"] = "DIAG"
                    elif cmd == "SET_MODE:AUTO":
                        self.status["esp32_mode"] = "AUTO"
                    elif cmd == "RESET_LATCH":
                        self.status["esp32_state"] = "BOOT_STOPPED"
                    elif cmd == "EMERGENCY_STOP":
                        self.status["esp32_state"] = "DANGER_LATCHED"
                    elif cmd.startswith("MOTOR_"):
                        if cmd == "MOTOR_STOP":
                            self.status.update(motor_direction="STOPPED", motor_pwm_pct=0)
                    socketio.emit("status_update", dict(self.status))
                    socketio.emit("cmd_ack", {"cmd": cmd, "optimistic": True}, to=target_sid)
                threading.Thread(target=_optimistic, daemon=True).start()
        return ok

    def _resolve_pending(self, key: str, ack: bool, reason: str = "", sid: str | None = None):
        with self.lock:
            entry = self._pending.pop(key, None)
        if entry is None:
            return
        target_sid = entry.get("sid")
        if ack:
            socketio.emit("cmd_ack",  {"cmd": entry["cmd"]}, to=target_sid)
        else:
            socketio.emit("cmd_nack", {"cmd": entry["cmd"], "reason": reason}, to=target_sid)

    # ------------------------------------------------------------------
    # Background: command timeout watcher
    # ------------------------------------------------------------------
    def _timeout_watcher(self):
        while self._serial_running:
            now = _now_ms()
            with self.lock:
                timed_out = [(k, v) for k, v in self._pending.items()
                             if now - v["sent_at_ms"] > CMD_TIMEOUT_MS]
            for key, entry in timed_out:
                with self.lock:
                    self._pending.pop(key, None)
                socketio.emit("cmd_timeout", {"cmd": entry["cmd"]}, to=entry.get("sid"))
            time.sleep(0.1)

    # ------------------------------------------------------------------
    # Background serial reader
    # ------------------------------------------------------------------
    def _reader(self):
        while self._serial_running:
            try:
                raw = self.serial_conn.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                socketio.emit("serial_line", {"line": line, "ts": _now_ms()})
                changed = self._parse(line)
                if changed:
                    socketio.emit("status_update", dict(self.status))
            except Exception as exc:
                if self._serial_running:
                    socketio.emit("serial_line", {"line": f"[READ ERROR] {exc}", "ts": _now_ms()})
                    self.status.update(connected=False, esp32_state="DISCONNECTED")
                    socketio.emit("status_update", dict(self.status))
                break

    def _parse(self, line: str) -> bool:
        changed = False

        # ACK:CMD or ACK:CMD:DETAIL
        if line.startswith("ACK:"):
            rest = line[4:]
            key  = rest.split(":")[0]
            self._resolve_pending(key, ack=True)
            # Update local state from ACK
            if key == "LED_ON":
                self.status["led_on"] = True; changed = True
            elif key == "LED_OFF" or key == "LED_BLINK":
                self.status["led_on"] = False; changed = True
            elif key == "BUZZER_ON":
                self.status["buzzer_on"] = True; changed = True
            elif key == "BUZZER_OFF" or key == "BUZZER_BEEP":
                self.status["buzzer_on"] = False; changed = True
            elif key == "MOTOR_STOP":
                self.status.update(motor_direction="STOPPED", motor_pwm_pct=0); changed = True
            elif key == "EMERGENCY_STOP":
                self.status["esp32_state"] = "DANGER_LATCHED"; changed = True
            elif key == "SET_MODE":
                pass  # state updated from STATUS line
            return changed

        # NACK:CMD:REASON
        if line.startswith("NACK:"):
            parts = line[5:].split(":", 1)
            key    = parts[0]
            reason = parts[1] if len(parts) > 1 else "UNKNOWN"
            self._resolve_pending(key, ack=False, reason=reason)
            return False

        # STATUS:v=1,mode=AUTO,state=FAR,estop=0,led=0,buz=0,us_cal=42.1,pwm=0,moe=0
        if line.startswith("STATUS:"):
            try:
                for tok in line[7:].split(","):
                    if "=" not in tok:
                        continue
                    k, v = tok.split("=", 1)
                    if k == "state":
                        self.status["esp32_state"] = v; changed = True
                    elif k == "mode":
                        self.status["esp32_mode"] = v; changed = True
                    elif k == "led":
                        self.status["led_on"] = v == "1"; changed = True
                    elif k == "buz":
                        self.status["buzzer_on"] = v == "1"; changed = True
                    elif k == "us_cal":
                        self.status["us_cal"] = round(float(v), 1); changed = True
                    elif k == "pwm":
                        self.status["motor_pwm_pct"] = int(v); changed = True
                    elif k == "estop":
                        if v == "1":
                            self.status["esp32_state"] = "DANGER_LATCHED"; changed = True
                    elif k == "moe":
                        self.status["motor_outputs_enabled"] = v == "1"; changed = True
            except Exception:
                pass
            return changed

        # CAPS:v=1,motor=0,led=1,buzzer=1,us=1,diag=1,estop=1
        if line.startswith("CAPS:"):
            try:
                self.firmware_mode = "unified"
                self.status["firmware"] = "pdv_unified v1.0"
                for tok in line[5:].split(","):
                    if "=" not in tok:
                        continue
                    k, v = tok.split("=", 1)
                    if k in self.caps:
                        self.caps[k] = v == "1"
                socketio.emit("caps_update", dict(self.caps))
                socketio.emit("firmware_mode", {"mode": "unified"})
                changed = True
            except Exception:
                pass
            return changed

        # [US] raw=47.2cm cal=50.0cm OK
        if line.startswith("[US]"):
            try:
                tokens = line.split()
                raw = float(tokens[1].split("=")[1].replace("cm", ""))
                cal = float(tokens[2].split("=")[1].replace("cm", ""))
                st  = " ".join(tokens[3:]) if len(tokens) > 3 else "OK"
                self.status.update(us_raw=round(raw, 1), us_cal=round(cal, 1), us_status=st.strip())
                changed = True
            except Exception:
                pass
            return changed

        # [RX] seq=N zone=FAR ped=True conf=0.91
        if line.startswith("[RX]"):
            try:
                for tok in line.split()[1:]:
                    if "=" not in tok:
                        continue
                    k, v = tok.split("=", 1)
                    if k == "seq":    self.status["seq_rx"]    = int(v)
                    elif k == "zone": self.status["zone"]       = v
                    elif k == "ped":  self.status["pedestrian"] = v.lower() == "true"
                    elif k == "conf": self.status["confidence"] = round(float(v), 2)
                    elif k == "dist": self.status["distance_m"] = round(float(v), 2)
                z = self.status.get("zone")
                if self.status["esp32_state"] not in ("DANGER_LATCHED", "COMM_FAULT"):
                    if z == "FAR":     self.status["esp32_state"] = "FAR"
                    elif z == "CAUTION": self.status["esp32_state"] = "CAUTION"
                changed = True
            except Exception:
                pass
            return changed

        # Live ESP32 status format (e.g. Distance: 213.6 cm)
        if line.startswith("Distance:"):
            try:
                val = float(line.split(":")[1].replace("cm", "").strip())
                self.status.update(us_cal=round(val, 1), us_raw=round(val, 1),
                                   us_status="OK" if val > 50 else "DANGER")
                changed = True
            except Exception:
                pass
        elif line.startswith("LED:"):
            self.status["led_on"] = "ON" in line.upper()
            changed = True
        elif line.startswith("Buzzer:"):
            self.status["buzzer_on"] = "ON" in line.upper()
            changed = True
        elif "Current PWM:" in line or "Target PWM:" in line:
            try:
                pwm_val = int(line.split(":")[1].replace("%", "").strip())
                self.status["motor_pwm_pct"] = pwm_val
                changed = True
            except Exception:
                pass
        elif line.startswith("Motor state:"):
            st = line.split(":", 1)[1].strip()
            if "STOP" in st:
                self.status["motor_direction"] = "STOPPED"
                if "LATCH" in st:
                    self.status["esp32_state"] = "DANGER_LATCHED"
            changed = True

        # Keyword matching for legacy/additional lines
        if "BOOT:" in line or "BOOT_STOPPED" in line:
            self.status["esp32_state"] = "BOOT_STOPPED"; changed = True
        elif "resuming FAR" in line or "2s satisfied" in line:
            self.status["esp32_state"] = "FAR"; changed = True
        elif "DANGER" in line and "latched" in line.lower():
            self.status["esp32_state"] = "DANGER_LATCHED"; changed = True
        elif "COMM_FAULT" in line or "WATCHDOG" in line:
            self.status["esp32_state"] = "COMM_FAULT"; changed = True

        return changed

    # ------------------------------------------------------------------
    # Heartbeat loop (zone injection from web UI)
    # ------------------------------------------------------------------
    def start_heartbeat(self, zone: str, distance_m, confidence: float, pedestrian: bool):
        self._hb_running["active"] = False
        time.sleep(0.05)
        flag = {"active": True}
        self._hb_running = flag
        self.status.update(hb_active=True, hb_zone=zone)
        socketio.emit("status_update", dict(self.status))

        def _loop():
            while flag["active"]:
                with self.lock:
                    self.seq += 1
                    seq = self.seq
                if PROTOCOL_AVAILABLE:
                    msg = make_message(seq=seq, timestamp_ms=int(time.monotonic() * 1000),
                                       zone=zone, distance_m=distance_m,
                                       confidence=confidence, pedestrian=pedestrian, heartbeat=True)
                    pkt = encode_message(msg)
                else:
                    obj = {"v": 1, "seq": seq, "timestamp_ms": int(time.monotonic() * 1000),
                           "zone": zone, "distance_m": distance_m, "confidence": confidence,
                           "pedestrian": pedestrian, "heartbeat": True}
                    pkt = (json.dumps(obj, separators=(",", ":")) + "\n").encode()
                if self.write(pkt):
                    socketio.emit("serial_line", {"line": f"[TX] {pkt.decode().strip()}", "ts": _now_ms(), "tx": True})
                time.sleep(0.1)

        threading.Thread(target=_loop, daemon=True).start()

    def stop_heartbeat(self):
        self._hb_running["active"] = False
        self.status.update(hb_active=False, hb_zone=None)
        socketio.emit("status_update", dict(self.status))

    # ------------------------------------------------------------------
    # Real-Time Perception Pipeline (Camera + YOLO + Safety Zones)
    # ------------------------------------------------------------------
    def start_model(self):
        """Run perception pipeline (YOLO/Camera/Distance) in real-time background loop."""
        if self._model_running:
            return
        self._model_running = True
        self.status["model_running"] = True
        socketio.emit("status_update", dict(self.status))

        def _loop():
            try:
                from src.raspberry_pi.camera import open_camera
                from src.raspberry_pi.detector import PedestrianDetector
                from src.raspberry_pi.distance_estimator import DistanceEstimator
                from src.common.zones import classify_zone, ZoneConfig, ZoneDebouncer
                from src.common.protocol import make_message, encode_message
                
                cam = open_camera(mode="auto")
                det = PedestrianDetector(min_confidence=float(self.settings.get("confidence_threshold", 0.5)))
                dist_est = DistanceEstimator(k=310.5)
                z_cfg = ZoneConfig(danger_distance_m=1.0, caution_distance_m=2.0)
                debouncer = ZoneDebouncer(window_size=3)
            except Exception as e:
                socketio.emit("serial_line", {"line": f"[MODEL ERROR] Init failed: {e}", "ts": _now_ms()})
                self._model_running = False
                self.status["model_running"] = False
                socketio.emit("status_update", dict(self.status))
                return

            fps_count = 0
            fps_timer = time.monotonic()

            while self._model_running:
                loop_start = time.monotonic()
                try:
                    frame = cam.capture_array()
                    res = det.detect(frame)
                    d_m = None
                    if res.pedestrian and res.bbox_xyxy:
                        d_m = dist_est.estimate_from_bbox(res.bbox_xyxy)
                    inst_zone = classify_zone(d_m, res.confidence, res.pedestrian, z_cfg)
                    zone = debouncer.update(inst_zone)

                    fps_count += 1
                    now_mono = time.monotonic()
                    if now_mono - fps_timer >= 1.0:
                        self.status["model_fps"] = round(fps_count / (now_mono - fps_timer), 1)
                        fps_count = 0
                        fps_timer = now_mono

                    self.status.update(
                        pedestrian=res.pedestrian,
                        confidence=round(res.confidence, 2),
                        distance_m=round(d_m, 2) if d_m is not None else None,
                        zone=zone,
                    )

                    # Transmit detection packet to ESP32 if serial is open
                    with self.lock:
                        self.seq += 1
                        seq = self.seq
                    msg = make_message(seq=seq, timestamp_ms=int(now_mono * 1000),
                                       zone=zone, distance_m=d_m, confidence=res.confidence,
                                       pedestrian=res.pedestrian, heartbeat=True)
                    pkt = encode_message(msg)
                    if self.write(pkt):
                        socketio.emit("serial_line", {"line": f"[MODEL->ESP32] {pkt.decode().strip()}", "ts": _now_ms(), "tx": True})

                    socketio.emit("status_update", dict(self.status))
                    socketio.emit("model_detection", {
                        "pedestrian": res.pedestrian,
                        "confidence": round(res.confidence, 2),
                        "distance_m": round(d_m, 2) if d_m is not None else None,
                        "zone": zone,
                        "fps": self.status.get("model_fps", 10.0),
                    })
                except Exception as exc:
                    pass

                elapsed = time.monotonic() - loop_start
                sleep_time = max(0.01, 0.1 - elapsed)
                time.sleep(sleep_time)

        threading.Thread(target=_loop, daemon=True).start()

    def stop_model(self):
        """Stop background perception pipeline."""
        self._model_running = False
        self.status["model_running"] = False
        socketio.emit("status_update", dict(self.status))


state = AppState()


def _now_ms() -> int:
    return int(time.monotonic() * 1000)


# ---------------------------------------------------------------------------
# HTTP routes
# ---------------------------------------------------------------------------
@app.route("/")
def index():
    return render_template("index.html")


def _list_ports():
    """Scan and return real serial ports discovered by the system (no synthetic ports)."""
    if not SERIAL_AVAILABLE:
        return []
    ports = []
    try:
        for p in serial.tools.list_ports.comports():
            ports.append({
                "port": p.device,
                "desc": p.description or "",
                "hwid": p.hwid or ""
            })
    except Exception:
        pass
    # Sort USB / UART adapters to top so they are prioritized
    def _rank(item):
        text = (item["port"] + " " + item["desc"] + " " + item["hwid"]).lower()
        if any(k in text for k in ("usb", "cp21", "ch34", "ftdi", "uart")):
            return 0
        return 1
    ports.sort(key=_rank)
    return ports


@app.route("/api/ports")
def api_ports():
    return jsonify({"ports": _list_ports()})


@app.route("/api/status")
def api_status():
    return jsonify({**state.status, "caps": state.caps})


@app.route("/api/settings", methods=["GET", "POST"])
def api_settings():
    if request.method == "POST":
        data = request.get_json(silent=True) or {}
        for k, v in data.items():
            if k in state.settings:
                state.settings[k] = v
        socketio.emit("settings_data", state.settings)
        return jsonify({"ok": True, "settings": state.settings})
    return jsonify(state.settings)


@app.route("/api/hw_results")
def api_hw_results():
    return jsonify(state.hw_test_results)


@app.route("/api/raw_cmd", methods=["POST"])
def api_raw_cmd():
    """Send a raw command string over serial (for the Serial Monitor manual input)."""
    data = request.get_json(silent=True) or {}
    cmd  = str(data.get("cmd", "")).strip()
    if not cmd:
        return jsonify({"ok": False, "error": "empty command"})
    if not state.status.get("connected"):
        return jsonify({"ok": False, "error": "not connected"})
    ok = state.write((cmd + "\n").encode())
    if ok:
        socketio.emit("serial_line", {"line": f"[CMD] {cmd}", "ts": _now_ms(), "tx": True})
    return jsonify({"ok": ok})


# ---------------------------------------------------------------------------
# Socket.IO — connection lifecycle
# ---------------------------------------------------------------------------
@socketio.on("connect")
def on_connect():
    emit("status_update", dict(state.status))
    emit("caps_update",   dict(state.caps))
    emit("settings_data", state.settings)
    emit("hw_test_results_update", state.hw_test_results)


@socketio.on("list_ports")
def on_list_ports():
    emit("ports", {"ports": _list_ports()})


@socketio.on("connect_serial")
def on_connect_serial(data):
    port = data.get("port", "")
    baud = int(data.get("baud", 115200))
    if not port:
        emit("toast", {"msg": "Select a serial port first.", "level": "warning"}); return
    try:
        state.connect(port, baud)
        emit("toast", {"msg": f"Connected to {port} @ {baud}", "level": "success"})
        socketio.emit("status_update", dict(state.status))
    except Exception as exc:
        msg = str(exc)
        if "Resource busy" in msg or "Errno 16" in msg:
            msg = f"Port is busy! Please close Arduino IDE's Serial Monitor tab first."
        emit("toast", {"msg": msg, "level": "danger"})


@socketio.on("disconnect_serial")
def on_disconnect_serial():
    state.disconnect()
    socketio.emit("status_update", dict(state.status))
    socketio.emit("caps_update",   dict(state.caps))
    emit("toast", {"msg": "Disconnected from ESP32", "level": "secondary"})


# ---------------------------------------------------------------------------
# Socket.IO — hardware commands (ACK-based)
# ---------------------------------------------------------------------------
@socketio.on("led_cmd")
def on_led_cmd(data):
    s = data.get("state", "OFF").upper()
    if not state.status["connected"]:
        emit("toast", {"msg": "Not connected", "level": "warning"}); return
    # Transmit in multiple known formats to ensure execution on any firmware:
    if s == "ON":
        state.write(b"LED_ON\n")
        state.write(b"CMD:LED:ON\n")
        state.write(b"l\n")
        state.status["led_on"] = True
    elif s == "OFF":
        state.write(b"LED_OFF\n")
        state.write(b"CMD:LED:OFF\n")
        state.write(b"l\n")
        state.status["led_on"] = False
    elif s == "BLINK":
        state.write(b"LED_BLINK\n")
        state.write(b"CMD:LED:BLINK\n")
        state.write(b"w\n")
    socketio.emit("serial_line", {"line": f"[CMD] LED {s}", "ts": _now_ms(), "tx": True})
    socketio.emit("status_update", dict(state.status))
    emit("cmd_ack", {"cmd": f"LED_{s}"})
    emit("toast", {"msg": f"LED → {s}", "level": "success"})


@socketio.on("buzzer_cmd")
def on_buzzer_cmd(data):
    p = data.get("pattern", "OFF").upper()
    if not state.status["connected"]:
        emit("toast", {"msg": "Not connected", "level": "warning"}); return
    if p == "ON":
        state.write(b"BUZZER_ON\n")
        state.write(b"CMD:BUZZER:ON\n")
        state.write(b"b\n")
        state.status["buzzer_on"] = True
    elif p == "OFF":
        state.write(b"BUZZER_OFF\n")
        state.write(b"CMD:BUZZER:OFF\n")
        state.write(b"b\n")
        state.status["buzzer_on"] = False
    elif p in ("BEEP", "SHORT"):
        state.write(b"BUZZER_BEEP\n")
        state.write(b"CMD:BUZZER:SHORT\n")
        state.write(b"e\n")
    socketio.emit("serial_line", {"line": f"[CMD] BUZZER {p}", "ts": _now_ms(), "tx": True})
    socketio.emit("status_update", dict(state.status))
    emit("cmd_ack", {"cmd": f"BUZZER_{p}"})
    emit("toast", {"msg": f"Buzzer → {p}", "level": "info"})


@socketio.on("motor_cmd")
def on_motor_cmd(data):
    channel   = data.get("channel", "A").upper()
    direction = data.get("direction", "STOP").upper()
    pwm_pct   = int(data.get("pwm_pct", 0))
    pwm_raw   = int(pwm_pct * 255 / 100)

    if not state.status["connected"]:
        emit("toast", {"msg": "Not connected", "level": "warning"}); return
    if not state.caps.get("motor"):
        emit("toast", {"msg": "Motor outputs disabled in firmware (MOTOR_OUTPUTS_ENABLED=false)", "level": "warning"}); return
    if state.status.get("esp32_mode") != "DIAG":
        emit("toast", {"msg": "Enter DIAG mode first (Mode → Diagnostic)", "level": "warning"}); return
    if direction == "STOP":
        state.send_cmd("MOTOR_STOP", sid=request.sid)
        return
    ch_map = {"A": "MOTOR_A_TEST", "B": "MOTOR_B_TEST", "AB": "MOTOR_BOTH_TEST", "BOTH": "MOTOR_BOTH_TEST"}
    base = ch_map.get(channel, "MOTOR_A_TEST")
    state.send_cmd(f"{base}:{pwm_raw}", sid=request.sid)
    state.status.update(motor_channel=channel, motor_direction=direction, motor_pwm_pct=pwm_pct)
    socketio.emit("status_update", dict(state.status))


@socketio.on("emergency_stop")
def on_emergency_stop():
    if state.status["connected"]:
        state.send_cmd("EMERGENCY_STOP", sid=request.sid)
    state.stop_heartbeat()


@socketio.on("reset_latch")
def on_reset_latch():
    if not state.status["connected"]:
        emit("toast", {"msg": "Not connected", "level": "warning"}); return
    state.send_cmd("RESET_LATCH", sid=request.sid)


@socketio.on("set_mode")
def on_set_mode(data):
    mode = data.get("mode", "AUTO").upper()  # AUTO | DIAG
    cmd = f"SET_MODE:{mode}"
    if not state.status["connected"]:
        emit("toast", {"msg": "Not connected", "level": "warning"}); return
    state.send_cmd(cmd, sid=request.sid)


@socketio.on("get_status")
def on_get_status():
    if state.status["connected"]:
        state.write(b"GET_STATUS\n")


@socketio.on("get_capabilities")
def on_get_caps():
    if state.status["connected"]:
        state.write(b"GET_CAPABILITIES\n")


# ---------------------------------------------------------------------------
# Socket.IO — zone injection / heartbeat
# ---------------------------------------------------------------------------
@socketio.on("send_zone")
def on_send_zone(data):
    zone = data.get("zone", "FAR")
    dist = data.get("distance_m")
    conf = float(data.get("confidence", 0.95))
    ped  = data.get("pedestrian", zone != "FAR")
    if dist is not None:
        dist = float(dist)
    with state.lock:
        state.seq += 1
        seq = state.seq
    if PROTOCOL_AVAILABLE:
        msg = make_message(seq=seq, timestamp_ms=int(time.monotonic() * 1000),
                           zone=zone, distance_m=dist, confidence=conf,
                           pedestrian=ped, heartbeat=True)
        pkt = encode_message(msg)
    else:
        obj = {"v": 1, "seq": seq, "timestamp_ms": int(time.monotonic() * 1000),
               "zone": zone, "distance_m": dist, "confidence": conf,
               "pedestrian": ped, "heartbeat": True}
        pkt = (json.dumps(obj, separators=(",", ":")) + "\n").encode()
    if state.write(pkt):
        socketio.emit("serial_line", {"line": f"[TX] {pkt.decode().strip()}", "ts": _now_ms(), "tx": True})
    else:
        emit("toast", {"msg": "Not connected to ESP32", "level": "warning"})


@socketio.on("start_heartbeat")
def on_start_heartbeat(data):
    zone = data.get("zone", "FAR")
    dist = data.get("distance_m")
    conf = float(data.get("confidence", 0.95))
    ped  = data.get("pedestrian", zone != "FAR")
    if dist is not None:
        dist = float(dist)
    state.start_heartbeat(zone, dist, conf, ped)
    emit("toast", {"msg": f"Heartbeat → {zone} @ 10 Hz", "level": "info"})


@socketio.on("stop_heartbeat")
def on_stop_heartbeat():
    state.stop_heartbeat()
    emit("toast", {"msg": "Heartbeat stopped", "level": "secondary"})


@socketio.on("start_model")
def on_start_model():
    state.start_model()
    emit("toast", {"msg": "▶ Model Engine Started (Perception loop active)", "level": "success"})


@socketio.on("stop_model")
def on_stop_model():
    state.stop_model()
    emit("toast", {"msg": "⏹ Model Engine Stopped", "level": "secondary"})


@app.route("/api/model/start", methods=["POST"])
def api_model_start():
    state.start_model()
    return jsonify({"ok": True, "running": True})


@app.route("/api/model/stop", methods=["POST"])
def api_model_stop():
    state.stop_model()
    return jsonify({"ok": True, "running": False})


@app.route("/api/model/status")
def api_model_status():
    return jsonify({"running": state.status.get("model_running", False),
                    "fps": state.status.get("model_fps", 0.0),
                    "pedestrian": state.status.get("pedestrian", False),
                    "confidence": state.status.get("confidence", 0.0),
                    "distance_m": state.status.get("distance_m"),
                    "zone": state.status.get("zone")})


# ---------------------------------------------------------------------------
# Socket.IO — test runner
# ---------------------------------------------------------------------------
@socketio.on("run_scenario")
def on_run_scenario(data):
    scenario = data.get("scenario", "demo")
    port     = data.get("port")
    baud     = int(data.get("baud", 115200))
    script   = ROOT / "scripts" / "simulate_pi_messages.py"
    cmd      = [sys.executable, str(script), "--scenario", scenario]
    if port:
        cmd += ["--port", port, "--baud", str(baud)]

    def _run():
        socketio.emit("test_start", {"name": f"Scenario: {scenario}"})
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, cwd=str(ROOT))
            for line in proc.stdout:
                socketio.emit("test_output", {"line": line.rstrip()})
            proc.wait()
            socketio.emit("test_done", {"exit_code": proc.returncode})
        except Exception as exc:
            socketio.emit("test_output", {"line": f"ERROR: {exc}"})
            socketio.emit("test_done",   {"exit_code": 1})

    threading.Thread(target=_run, daemon=True).start()


@socketio.on("run_pytest")
def on_run_pytest():
    def _run():
        socketio.emit("test_start", {"name": "Python Test Suite (pytest)"})
        try:
            proc = subprocess.Popen(
                [sys.executable, "-m", "pytest", "tests/", "-v", "--tb=short", "--no-header"],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=str(ROOT))
            for line in proc.stdout:
                socketio.emit("test_output", {"line": line.rstrip()})
            proc.wait()
            socketio.emit("test_done", {"exit_code": proc.returncode})
        except Exception as exc:
            socketio.emit("test_output", {"line": f"ERROR: {exc}"})
            socketio.emit("test_done",   {"exit_code": 1})

    threading.Thread(target=_run, daemon=True).start()


# ---------------------------------------------------------------------------
# Socket.IO — settings
# ---------------------------------------------------------------------------
@socketio.on("get_settings")
def on_get_settings():
    emit("settings_data", state.settings)


@socketio.on("update_settings")
def on_update_settings(data):
    for k, v in data.items():
        if k in state.settings:
            try:
                orig = _DEFAULT_SETTINGS[k]
                state.settings[k] = type(orig)(v)
            except (ValueError, TypeError):
                pass
    socketio.emit("settings_data", state.settings)
    emit("toast", {"msg": "Settings saved", "level": "success"})


@socketio.on("reset_settings")
def on_reset_settings():
    state.settings = dict(_DEFAULT_SETTINGS)
    emit("settings_data", state.settings)
    emit("toast", {"msg": "Settings reset to defaults", "level": "secondary"})


# ---------------------------------------------------------------------------
# Socket.IO — hardware test results
# ---------------------------------------------------------------------------
@socketio.on("hw_test_result")
def on_hw_test_result(data):
    test_id = data.get("test_id")
    if not test_id:
        return
    state.hw_test_results[test_id] = {
        "status":    data.get("status", "unknown"),
        "timestamp": _now_ms(),
        "output":    data.get("output", ""),
        "category":  data.get("category", "hardware"),
    }
    socketio.emit("hw_test_results_update", state.hw_test_results)


@socketio.on("get_hw_results")
def on_get_hw_results():
    emit("hw_test_results_update", state.hw_test_results)


@socketio.on("clear_hw_results")
def on_clear_hw_results():
    state.hw_test_results.clear()
    socketio.emit("hw_test_results_update", state.hw_test_results)
    emit("toast", {"msg": "Hardware test results cleared", "level": "secondary"})


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print()
    print("=" * 55)
    print("  Pedestrian Detection Vehicle — Web UI")
    print("  Open: http://localhost:9999")
    print("=" * 55)
    print()
    socketio.run(app, host="0.0.0.0", port=9999, debug=False, allow_unsafe_werkzeug=True)
