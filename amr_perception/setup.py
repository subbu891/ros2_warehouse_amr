from setuptools import find_packages, setup

package_name = "amr_perception"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Subrahmanyam",
    maintainer_email="subrahmanyam.kodela33@gmail.com",
    description="Lidar-based dynamic obstacle detection, tracking and motion prediction.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["obstacle_tracker = amr_perception.obstacle_tracker_node:main"]},
)
