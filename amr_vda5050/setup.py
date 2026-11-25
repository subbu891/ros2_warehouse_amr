from setuptools import find_packages, setup

package_name = "amr_vda5050"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "paho-mqtt>=2.0"],
    zip_safe=True,
    maintainer="Subrahmanyam",
    maintainer_email="subrahmanyam.kodela33@gmail.com",
    description="VDA 5050 v2 adapter (MQTT) and a minimal fleet manager.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "vda5050_adapter = amr_vda5050.vda5050_adapter_node:main",
        "fleet_manager = amr_vda5050.fleet_manager:main",
    ]},
)
