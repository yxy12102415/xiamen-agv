from setuptools import setup

package_name = "agv_mpc_controller"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Centerline trajectory publisher and lightweight MPC tracker for the AGV simulation.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "centerline_trajectory_publisher = agv_mpc_controller.centerline_trajectory_publisher:main",
            "pid_trajectory_controller = agv_mpc_controller.pid_trajectory_controller:main",
            "sampling_mpc_controller = agv_mpc_controller.sampling_mpc_controller:main",
            "plot_mpc_error = agv_mpc_controller.plot_mpc_error:main",
        ],
    },
)
