import pytest
from src.common.protocol import encode_message, decode_message, make_message, ProtocolError


def valid():
    return make_message(1, 123, "FAR", 3.2, 0.95, False)


def test_round_trip():
    msg = valid()
    assert decode_message(encode_message(msg)) == msg


@pytest.mark.parametrize("zone", ["FAR", "CAUTION", "DANGER"])
def test_valid_zones(zone):
    msg = valid()
    msg["zone"] = zone
    assert decode_message(encode_message(msg))["zone"] == zone


def test_invalid_json_rejected():
    with pytest.raises(ProtocolError):
        decode_message("{broken")


def test_missing_field_rejected():
    msg = valid()
    del msg["heartbeat"]
    with pytest.raises(ProtocolError):
        encode_message(msg)


def test_invalid_zone_rejected():
    msg = valid()
    msg["zone"] = "GO_FAST"
    with pytest.raises(ProtocolError):
        encode_message(msg)


def test_nan_distance_rejected():
    msg = valid()
    msg["distance_m"] = float("nan")
    with pytest.raises(ProtocolError):
        encode_message(msg)


def test_bad_confidence_rejected():
    msg = valid()
    msg["confidence"] = 1.5
    with pytest.raises(ProtocolError):
        encode_message(msg)
