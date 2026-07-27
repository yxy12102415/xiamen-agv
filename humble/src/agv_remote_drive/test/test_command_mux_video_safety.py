import json

from std_msgs.msg import String

from agv_remote_drive.command_mux import RemoteDriveCommandMux


def video_message(role, state):
    message = String()
    message.data = json.dumps(
        {
            "role": role,
            "cameras": {"front": {"state": state}},
        }
    )
    return message


def command_mux_without_ros_clock():
    mux = object.__new__(RemoteDriveCommandMux)
    mux.require_remote_video = True
    mux.required_video_cameras = ("front",)
    mux.remote_video_timeout = 1.5
    mux.video_times = {}
    mux.test_now = 10.0
    mux.now_seconds = lambda: mux.test_now
    return mux


def test_sender_status_does_not_satisfy_receiver_video_guard():
    mux = command_mux_without_ros_clock()
    mux.on_video_state(video_message("sender", "LIVE"))
    assert not mux.video_ready()


def test_receiver_live_allows_remote_until_timeout():
    mux = command_mux_without_ros_clock()
    mux.on_video_state(video_message("receiver", "LIVE"))
    assert mux.video_ready()
    mux.test_now += 1.6
    assert not mux.video_ready()


def test_receiver_stale_revokes_remote_video_guard():
    mux = command_mux_without_ros_clock()
    mux.on_video_state(video_message("receiver", "LIVE"))
    mux.on_video_state(video_message("receiver", "STALE"))
    assert not mux.video_ready()
