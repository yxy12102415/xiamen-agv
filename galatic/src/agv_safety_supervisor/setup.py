from setuptools import setup

package_name = "agv_safety_supervisor"

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
    description="Safety gate for AGV control commands.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "safety_supervisor_node = agv_safety_supervisor.safety_supervisor_node:main",
        ],
    },
)
