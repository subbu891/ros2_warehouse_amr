from glob import glob

from setuptools import find_packages, setup

package_name = "amr_mission"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Subrahmanyam",
    maintainer_email="subrahmanyam.kodela33@gmail.com",
    description="Mission executor (ExecuteMission action) with retries and recovery on top of Nav2.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["mission_executor = amr_mission.mission_node:main"]},
)
