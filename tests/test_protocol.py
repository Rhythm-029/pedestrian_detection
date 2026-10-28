try:
    import pytest
except ImportError:
    pytest = None

from src.common.protocol import encode_message, decode_message, make_message, ProtocolError


def valid():
    return make_message(1, 123, "FAR", 3.2, 0.95, False)


def test_round_trip():
    msg = valid()
    assert decode_message(encode_message(msg)) == msg


def test_valid_zones():
    for zone in ["FAR", "CAUTION", "DANGER"]:
        msg = valid()
        msg["zone"] = zone
        assert decode_message(encode_message(msg))["zone"] == zone


def test_invalid_json_rejected():
    if pytest is not None:
        with pytest.raises(ProtocolError):
            decode_message("{broken")
    else:
        try:
            decode_message("{broken")
            assert False, "Should have raised ProtocolError"
        except ProtocolError:
            pass



def _assert_protocol_error(fn, *args, **kwargs):
    if pytest is not None:
        with pytest.raises(ProtocolError):
            fn(*args, **kwargs)
    else:
        try:
            fn(*args, **kwargs)
            assert False, "Should have raised ProtocolError"
        except ProtocolError:
            pass


def test_missing_field_rejected():
    msg = valid()
    del msg["heartbeat"]
    _assert_protocol_error(encode_message, msg)


def test_invalid_zone_rejected():
    msg = valid()
    msg["zone"] = "GO_FAST"
    _assert_protocol_error(encode_message, msg)


def test_nan_distance_rejected():
    msg = valid()
    msg["distance_m"] = float("nan")
    _assert_protocol_error(encode_message, msg)


def test_bad_confidence_rejected():
    msg = valid()
    msg["confidence"] = 1.5
    _assert_protocol_error(encode_message, msg)

