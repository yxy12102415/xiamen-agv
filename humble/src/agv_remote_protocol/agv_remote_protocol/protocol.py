import struct
import time


START_FLAG = 0x0A0A
MSG_DOMAIN_COMMAND = 0x7100
MSG_VEHICLE_STATE = 0x7000
ITEM_REMOTE_COMMAND = 0x7101
ITEM_REMOTE_DRIVING = 0x7102
ITEM_VEHICLE_STATE = 0x7101
HEADER_SIZE = 20


class ProtocolError(ValueError):
    pass


def endian_prefix(endian):
    normalized = str(endian).strip().lower()
    if normalized in ("little", "le", "<"):
        return "<"
    if normalized in ("big", "be", ">"):
        return ">"
    raise ProtocolError("endian must be little or big")


def header_struct(endian):
    return struct.Struct(endian_prefix(endian) + "HHHHIIHH")


def pack_tlv(item_type, value, endian="little", length_includes_header=False):
    value = bytes(value)
    item_length = len(value) + (4 if length_includes_header else 0)
    return struct.pack(endian_prefix(endian) + "HH", int(item_type), item_length) + value


def pack_packet(
    msg_type,
    msg_id,
    items,
    endian="little",
    length_includes_header=False,
    timestamp=None,
):
    encoded_items = [
        pack_tlv(item_type, value, endian, length_includes_header)
        for item_type, value in items
    ]
    body = b"".join(encoded_items)
    now = time.time() if timestamp is None else float(timestamp)
    seconds = int(now)
    milliseconds = int((now - seconds) * 1000.0)
    packet_length = HEADER_SIZE + len(body)
    header = header_struct(endian).pack(
        START_FLAG,
        int(msg_type),
        int(msg_id) & 0xFFFF,
        packet_length,
        seconds & 0xFFFFFFFF,
        milliseconds,
        len(encoded_items),
        len(body),
    )
    return header + body


def unpack_packet(data, endian="little", length_includes_header=False):
    data = bytes(data)
    if len(data) < HEADER_SIZE:
        raise ProtocolError("packet shorter than 20-byte header")
    fields = header_struct(endian).unpack_from(data)
    (
        start_flag,
        msg_type,
        msg_id,
        msg_length,
        sys_time_s,
        sys_time_ms,
        item_num,
        item_length,
    ) = fields
    if start_flag != START_FLAG:
        raise ProtocolError("invalid start flag 0x%04X" % start_flag)
    if msg_length != len(data):
        raise ProtocolError("msg_length=%d but datagram has %d bytes" % (msg_length, len(data)))
    if item_length != len(data) - HEADER_SIZE:
        raise ProtocolError("item_length does not match body length")

    items = []
    offset = HEADER_SIZE
    item_header = struct.Struct(endian_prefix(endian) + "HH")
    for _ in range(item_num):
        if offset + item_header.size > len(data):
            raise ProtocolError("truncated TLV header")
        item_type, encoded_length = item_header.unpack_from(data, offset)
        offset += item_header.size
        value_length = encoded_length - 4 if length_includes_header else encoded_length
        if value_length < 0 or offset + value_length > len(data):
            raise ProtocolError("invalid TLV length")
        items.append((item_type, data[offset : offset + value_length]))
        offset += value_length
    if offset != len(data):
        raise ProtocolError("unparsed bytes remain after TLV items")
    return {
        "start_flag": start_flag,
        "msg_type": msg_type,
        "msg_id": msg_id,
        "msg_length": msg_length,
        "sys_time_s": sys_time_s,
        "sys_time_ms": sys_time_ms,
        "item_num": item_num,
        "item_length": item_length,
        "items": items,
    }


def pack_remote_command(command, endian="little"):
    return struct.pack(
        endian_prefix(endian) + "IBBBBBBBB",
        int(command["igv_id"]),
        int(command["control_mode"]),
        int(command["gear_command"]),
        int(command["obstacle_bypass"]),
        int(command["estop"]),
        int(command["start_enable"]),
        int(command.get("spare1", 0)),
        int(command.get("spare2", 0)),
        int(command.get("spare3", 0)),
    )


def unpack_remote_command(data, endian="little"):
    fmt = struct.Struct(endian_prefix(endian) + "IBBBBBBBB")
    if len(data) != fmt.size:
        raise ProtocolError("remote command value must be 12 bytes")
    values = fmt.unpack(data)
    keys = (
        "igv_id",
        "control_mode",
        "gear_command",
        "obstacle_bypass",
        "estop",
        "start_enable",
        "spare1",
        "spare2",
        "spare3",
    )
    return dict(zip(keys, values))


def pack_remote_driving(driving, endian="little"):
    buttons = list(driving.get("buttons", [0] * 8))
    if len(buttons) != 8:
        raise ProtocolError("remote driving buttons must contain 8 values")
    return struct.pack(
        endian_prefix(endian) + "fff8B",
        float(driving["steering_scale"]),
        float(driving["speed_scale"]),
        float(driving["brake_scale"]),
        *[int(value) for value in buttons],
    )


def unpack_remote_driving(data, endian="little"):
    fmt = struct.Struct(endian_prefix(endian) + "fff8B")
    if len(data) != fmt.size:
        raise ProtocolError("remote driving value must be 20 bytes")
    values = fmt.unpack(data)
    return {
        "steering_scale": values[0],
        "speed_scale": values[1],
        "brake_scale": values[2],
        "buttons": list(values[3:]),
    }


def pack_vehicle_state(state, endian="little"):
    polygons = list(state.get("polygons", []))
    faults = list(state.get("faults", []))
    fixed = struct.pack(
        endian_prefix(endian) + "IBBBBBBBBffffffHH",
        int(state["igv_id"]),
        int(state["control_mode"]),
        int(state["gear_state"]),
        int(state.get("headlights_state", 0)),
        int(state.get("task_running_state", 0)),
        int(state.get("position_valid", 0)),
        int(state.get("heading_valid", 0)),
        int(state.get("obstacle_detect_state", 0)),
        int(state.get("start_ready", 0)),
        float(state.get("battery_level", 0.0)),
        float(state.get("speed_feedback", 0.0)),
        float(state.get("steering_feedback", 0.0)),
        float(state.get("pose_x", 0.0)),
        float(state.get("pose_y", 0.0)),
        float(state.get("pose_heading", 0.0)),
        len(polygons),
        len(faults),
    )
    polygon_data = b"".join(
        struct.pack(endian_prefix(endian) + "ff", float(x), float(y))
        for x, y in polygons
    )
    fault_data = b"".join(
        struct.pack(endian_prefix(endian) + "HH", int(code), int(level))
        for code, level in faults
    )
    return fixed + polygon_data + fault_data


def unpack_vehicle_state(data, endian="little"):
    fixed = struct.Struct(endian_prefix(endian) + "IBBBBBBBBffffffHH")
    if len(data) < fixed.size:
        raise ProtocolError("vehicle state value shorter than 40 bytes")
    values = fixed.unpack_from(data)
    polygon_num = values[-2]
    fault_num = values[-1]
    expected_length = fixed.size + polygon_num * 8 + fault_num * 4
    if len(data) != expected_length:
        raise ProtocolError(
            "vehicle state length=%d, expected=%d" % (len(data), expected_length)
        )
    offset = fixed.size
    polygons = []
    for _ in range(polygon_num):
        polygons.append(struct.unpack_from(endian_prefix(endian) + "ff", data, offset))
        offset += 8
    faults = []
    for _ in range(fault_num):
        faults.append(struct.unpack_from(endian_prefix(endian) + "HH", data, offset))
        offset += 4
    return {
        "igv_id": values[0],
        "control_mode": values[1],
        "gear_state": values[2],
        "headlights_state": values[3],
        "task_running_state": values[4],
        "position_valid": values[5],
        "heading_valid": values[6],
        "obstacle_detect_state": values[7],
        "start_ready": values[8],
        "battery_level": values[9],
        "speed_feedback": values[10],
        "steering_feedback": values[11],
        "pose_x": values[12],
        "pose_y": values[13],
        "pose_heading": values[14],
        "polygons": [list(point) for point in polygons],
        "faults": [list(fault) for fault in faults],
    }
