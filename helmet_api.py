#!/usr/bin/env python3
"""
helmet_api.py — Lightweight Flask REST bridge for the Smart Helmet.
Reads shared sensor state written by smart_helmet.py to /tmp/helmet_api_state.json.
No GPIO needed — all sensors are handled by smart_helmet.py directly.

Provides:
  GET /api/health   → {"status": "ok"|"starting"|"error"}
  GET /api/sensors  → latest sensor snapshot (from shared file)
  POST /api/reset-fall → reset fall detection flag

The React app polls GET /api/sensors every 500 ms.
"""

import os
import time
import signal
import json
import sys
from datetime import datetime, timezone

from flask import Flask, jsonify
from flask_cors import CORS

# ═══════════════════════════════════════════════════════════════
#  CONFIG
# ═══════════════════════════════════════════════════════════════

STATE_FILE = "/tmp/helmet_api_state.json"
PORT      = int(os.getenv("HELMET_API_PORT", "5000"))
HOST      = "0.0.0.0"
STARTED  = time.time()
DATA_FRESHNESS_TIMEOUT = 10  # seconds before we call it stale

# ═══════════════════════════════════════════════════════════════
#  FLASK APP
# ═══════════════════════════════════════════════════════════════

app = Flask(__name__)
CORS(app, resources={r"/api/*": {"origins": "*"}})


def _read_state():
    """Read the latest sensor snapshot from the shared JSON file."""
    try:
        with open(STATE_FILE, "r") as f:
            return json.load(f), None
    except FileNotFoundError:
        return None, "No data yet — smart_helmet.py may not be running"
    except json.JSONDecodeError:
        return None, "Corrupted state file"


def _is_fresh(state):
    if state is None:
        return False
    try:
        mtime = os.path.getmtime(STATE_FILE)
        return (time.time() - mtime) < DATA_FRESHNESS_TIMEOUT
    except Exception:
        return False


@app.route("/", methods=["GET"])
def root():
    return jsonify({
        "service": "helmet-api",
        "status": "ok",
        "endpoints": ["/api/health", "/api/sensors", "/api/reset-fall"]
    }), 200


@app.route("/api/", methods=["GET"])
def api_root():
    return jsonify({
        "service": "helmet-api",
        "status": "ok",
        "endpoints": ["/api/health", "/api/sensors", "/api/reset-fall"]
    }), 200


@app.route("/api/health", methods=["GET"])
def health():
    state, err = _read_state()
    if err:
        return jsonify({
            "status": "error",
            "error": err,
            "since": datetime.fromtimestamp(STARTED, timezone.utc).isoformat()
        }), 503
    if not _is_fresh(state):
        return jsonify({
            "status": "starting",
            "since": datetime.fromtimestamp(STARTED, timezone.utc).isoformat()
        }), 200
    return jsonify({
        "status": "ok",
        "since": datetime.fromtimestamp(STARTED, timezone.utc).isoformat()
    }), 200


@app.route("/api/sensors", methods=["GET"])
def sensors():
    state, err = _read_state()
    if err:
        return jsonify({"error": err}), 503
    if not _is_fresh(state):
        return jsonify({
            "error": "Sensor data stale — is smart_helmet.py still running?",
            "since": datetime.fromtimestamp(STARTED, timezone.utc).isoformat()
        }), 503
    return jsonify(state), 200


@app.route("/api/reset-fall", methods=["POST"])
def reset_fall():
    """Reset the fallDetected flag in the shared state file."""
    state, err = _read_state()
    if err:
        return jsonify({"error": err}), 500
    if state:
        state["fallDetected"] = False
        try:
            with open(STATE_FILE, "w") as f:
                json.dump(state, f)
            return jsonify({"fallDetected": False}), 200
        except Exception:
            pass
    return jsonify({"error": "Could not reset"}), 500


# ═══════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════

def _signal_handler(sig, frame):
    print("\n[helmet_api] Shutting down...")
    sys.exit(0)

signal.signal(signal.SIGTERM, _signal_handler)
signal.signal(signal.SIGINT,  _signal_handler)

if __name__ == "__main__":
    print(f"[helmet_api] Reading from {STATE_FILE}")
    print(f"[helmet_api] Listening on http://{HOST}:{PORT}")
    app.run(host=HOST, port=PORT, threaded=True, use_reloader=False)
