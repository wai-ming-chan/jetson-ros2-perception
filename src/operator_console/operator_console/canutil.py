"""Pure SocketCAN frame helpers, split out so they are unit-testable without ROS or a
CAN interface. The struct layout mirrors the kernel's struct can_frame; getting the
flag masks or padding wrong produces frames that look plausible and are silently wrong,
which is exactly what unit tests are for."""

import struct

# struct can_frame: can_id (u32, host order), len (u8), 3 pad bytes, data[8].
CAN_FRAME_FMT = "<IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_EFF_MASK = 0x1FFFFFFF


def parse_can_frame(raw):
    """Decode one kernel can_frame into a dict, or None if `raw` is short.

    The id is rendered as hex text (3 digits standard, 8 digits extended) because every
    consumer here is a display or a log."""
    if len(raw) < CAN_FRAME_SIZE:
        return None
    can_id, length, data = struct.unpack(CAN_FRAME_FMT, raw[:CAN_FRAME_SIZE])
    extended = bool(can_id & CAN_EFF_FLAG)
    return {
        "id": "{:08X}".format(can_id & CAN_EFF_MASK) if extended
              else "{:03X}".format(can_id & CAN_SFF_MASK),
        "ext": extended,
        "rtr": bool(can_id & CAN_RTR_FLAG),
        "err": bool(can_id & CAN_ERR_FLAG),
        "dlc": length,
        "data": data[:length].hex().upper(),
    }


def build_can_frame(can_id, payload, extended=False):
    """Encode a kernel can_frame. Raises ValueError on a payload over 8 bytes."""
    if len(payload) > 8:
        raise ValueError("payload longer than 8 bytes")
    ident = can_id | (CAN_EFF_FLAG if extended else 0)
    return struct.pack(CAN_FRAME_FMT, ident, len(payload), payload.ljust(8, b"\0"))
