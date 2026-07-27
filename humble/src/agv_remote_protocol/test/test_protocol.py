import pytest

from agv_remote_protocol.protocol import ITEM_REMOTE_COMMAND
from agv_remote_protocol.protocol import ITEM_REMOTE_DRIVING
from agv_remote_protocol.protocol import ITEM_VEHICLE_STATE
from agv_remote_protocol.protocol import MSG_DOMAIN_COMMAND
from agv_remote_protocol.protocol import MSG_VEHICLE_STATE
from agv_remote_protocol.protocol import ProtocolError
from agv_remote_protocol.protocol import pack_packet
from agv_remote_protocol.protocol import pack_remote_command
from agv_remote_protocol.protocol import pack_remote_driving
from agv_remote_protocol.protocol import pack_vehicle_state
from agv_remote_protocol.protocol import unpack_packet
from agv_remote_protocol.protocol import unpack_remote_command
from agv_remote_protocol.protocol import unpack_remote_driving
from agv_remote_protocol.protocol import unpack_vehicle_state


COMMAND = {
    "igv_id": 7,
    "control_mode": 5,
    "gear_command": 1,
    "obstacle_bypass": 0,
    "estop": 0,
    "start_enable": 1,
}


@pytest.mark.parametrize("endian", ["little", "big"])
@pytest.mark.parametrize("length_includes_header", [False, True])
def test_domain_command_round_trip(endian, length_includes_header):
    driving = {
        "steering_scale": -25.5,
        "speed_scale": 60.0,
        "brake_scale": 0.0,
        "buttons": range(8),
    }
    packet = pack_packet(
        MSG_DOMAIN_COMMAND,
        123,
        [
            (ITEM_REMOTE_COMMAND, pack_remote_command(COMMAND, endian)),
            (ITEM_REMOTE_DRIVING, pack_remote_driving(driving, endian)),
        ],
        endian,
        length_includes_header,
        timestamp=1000.125,
    )

    decoded = unpack_packet(packet, endian, length_includes_header)
    assert len(packet) == 60
    assert decoded["msg_id"] == 123
    assert decoded["sys_time_ms"] == 125
    assert unpack_remote_command(decoded["items"][0][1], endian)["igv_id"] == 7
    assert unpack_remote_driving(decoded["items"][1][1], endian)[
        "steering_scale"
    ] == pytest.approx(-25.5)


def test_vehicle_state_variable_sections_round_trip():
    state = {
        "igv_id": 7,
        "control_mode": 5,
        "gear_state": 1,
        "battery_level": 88.5,
        "speed_feedback": 0.3,
        "steering_feedback": -7.2,
        "pose_x": 12.0,
        "pose_y": 34.0,
        "pose_heading": 90.0,
        "polygons": [(1.0, 2.0)],
        "faults": [(3, 2)],
    }
    packet = pack_packet(
        MSG_VEHICLE_STATE,
        9,
        [(ITEM_VEHICLE_STATE, pack_vehicle_state(state))],
    )
    value = unpack_packet(packet)["items"][0][1]
    decoded = unpack_vehicle_state(value)

    assert len(packet) == 76
    assert decoded["polygons"] == [[1.0, 2.0]]
    assert decoded["faults"] == [[3, 2]]


def test_rejects_header_length_mismatch():
    packet = pack_packet(
        MSG_DOMAIN_COMMAND,
        1,
        [(ITEM_REMOTE_COMMAND, pack_remote_command(COMMAND))],
    )
    with pytest.raises(ProtocolError, match="msg_length"):
        unpack_packet(packet[:-1])
