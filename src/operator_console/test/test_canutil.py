"""Unit tests for the CAN frame codec. Run by colcon test and by CI (no hardware)."""

import pytest

from operator_console.canutil import (
    CAN_FRAME_SIZE,
    build_can_frame,
    parse_can_frame,
)


def test_roundtrip_standard_id():
    raw = build_can_frame(0x123, b"\xde\xad\xbe\xef")
    assert len(raw) == CAN_FRAME_SIZE
    f = parse_can_frame(raw)
    assert f == {
        "id": "123", "ext": False, "rtr": False, "err": False,
        "dlc": 4, "data": "DEADBEEF",
    }


def test_roundtrip_extended_id():
    raw = build_can_frame(0x1ABCDE01, b"\x01", extended=True)
    f = parse_can_frame(raw)
    assert f["ext"] is True
    assert f["id"] == "1ABCDE01"
    assert f["dlc"] == 1
    assert f["data"] == "01"


def test_standard_id_is_masked_to_11_bits():
    # An id wider than 11 bits without the EFF flag must not leak into the display.
    raw = build_can_frame(0x7FF, b"")
    assert parse_can_frame(raw)["id"] == "7FF"


def test_payload_padding_not_reported_as_data():
    # The wire format always carries 8 data bytes; dlc bounds what is real.
    raw = build_can_frame(0x100, b"\xaa\xbb")
    f = parse_can_frame(raw)
    assert f["data"] == "AABB"          # not AABB000000000000


def test_oversize_payload_rejected():
    with pytest.raises(ValueError):
        build_can_frame(0x100, b"\x00" * 9)


def test_short_read_returns_none():
    assert parse_can_frame(b"\x00" * (CAN_FRAME_SIZE - 1)) is None
