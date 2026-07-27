from setuptools import setup


package_name = "agv_remote_drive"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Safe remote-driving command input and AUTO/REMOTE arbitration for the AGV.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "remote_drive_command_mux = agv_remote_drive.command_mux:main",
            "remote_drive_keyboard = agv_remote_drive.keyboard_operator:main",
            "remote_drive_wasd = agv_remote_drive.wasd_operator:main",
            "remote_video_network_emulator = agv_remote_drive.video_network_emulator:main",
            "remote_h264_video_link = agv_remote_drive.h264_video_link:main",
            "remote_h264_udp_proxy = agv_remote_drive.udp_video_proxy:main",
        ],
    },
)
