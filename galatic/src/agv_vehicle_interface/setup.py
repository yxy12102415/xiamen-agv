from setuptools import setup

package_name = "agv_vehicle_interface"

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
    description="Reference vehicle interface for AGV chassis protocol prototyping.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "reference_vehicle_interface_node = agv_vehicle_interface.reference_vehicle_interface_node:main",
            "real_vehicle_interface_node = agv_vehicle_interface.real_vehicle_interface_node:main",
        ],
    },
)
