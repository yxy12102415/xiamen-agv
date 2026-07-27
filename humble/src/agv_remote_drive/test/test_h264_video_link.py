import socket
import time

from agv_remote_drive.h264_video_link import EncodedPacketProxy
from agv_remote_drive.h264_video_link import encoder_arguments


def free_udp_port():
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind(("127.0.0.1", 0))
    port = udp_socket.getsockname()[1]
    udp_socket.close()
    return port


def test_nvenc_arguments_are_low_latency():
    arguments = encoder_arguments("h264_nvenc", 1500, 5)
    assert "h264_nvenc" in arguments
    assert arguments[arguments.index("-zerolatency") + 1] == "1"
    assert arguments[arguments.index("-bf") + 1] == "0"
    assert arguments[arguments.index("-b:v") + 1] == "1500k"


def test_single_camera_proxy_applies_delay_without_corrupting_packet():
    input_port = free_udp_port()
    output_port = free_udp_port()
    while output_port == input_port:
        output_port = free_udp_port()

    proxy = EncodedPacketProxy(
        host="127.0.0.1",
        input_base_port=input_port,
        output_base_port=output_port,
        delay_ms=50.0,
        jitter_ms=0.0,
        loss_percent=0.0,
        seed=1,
        capacity_mbps=10.0,
        max_queue_delay_ms=500.0,
        camera_names=("front",),
    )
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", output_port))
    receiver.settimeout(0.5)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        proxy.start()
        payload = b"annex-b-test-packet"
        started_at = time.monotonic()
        sender.sendto(payload, ("127.0.0.1", input_port))
        received, _address = receiver.recvfrom(4096)
        elapsed_ms = (time.monotonic() - started_at) * 1000.0
        assert received == payload
        assert 45.0 <= elapsed_ms <= 250.0
        metrics = proxy.snapshot()["front"]
        assert metrics["network_drops"] == 0
        assert metrics["congestion_drops"] == 0
    finally:
        proxy.close()
        sender.close()
        receiver.close()
