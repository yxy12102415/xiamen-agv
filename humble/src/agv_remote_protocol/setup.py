from setuptools import setup


package_name = "agv_remote_protocol"


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
    description="UDP/TLV remote-driving protocol prototype for IGV controllers.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "domain_udp_gateway = agv_remote_protocol.domain_gateway:main",
            "mock_br_controller = agv_remote_protocol.mock_br_controller:main",
        ],
    },
)
