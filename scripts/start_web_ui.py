#!/usr/bin/env python3
"""
Launch the Pedestrian Detection Vehicle Web UI.

Usage:
    python3 scripts/start_web_ui.py           # default port 5000
    python3 scripts/start_web_ui.py --port 8080

Then open: http://localhost:5000
"""

import sys
import argparse
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from src.web.server import app, socketio

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Pedestrian Detection Vehicle — Web UI")
    parser.add_argument("--host", default="0.0.0.0", help="Host to bind (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=5000, help="Port (default: 5000)")
    args = parser.parse_args()

    print()
    print("=" * 55)
    print("  Pedestrian Detection Vehicle — Web UI")
    print(f"  Open: http://localhost:{args.port}")
    print("=" * 55)
    print()

    socketio.run(app, host=args.host, port=args.port, debug=False, allow_unsafe_werkzeug=True)
