import json
from src.raspberry_pi.pi_controller import build_status_message


def test_status_message_is_newline_json():
    data = build_status_message(1, 0.7, 0.9, True)
    assert data.endswith(b"\n")
    obj = json.loads(data)
    assert obj["zone"] == "DANGER"
    assert obj["heartbeat"] is True
