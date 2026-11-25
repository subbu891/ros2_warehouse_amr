from glob import glob

from setuptools import find_packages, setup

package_name = "amr_validation"

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
    description="KPI metrics, scenario test suite and simulation test helpers.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "scenario_runner = amr_validation.scenario_runner:main",
        "kpi_recorder = amr_validation.kpi_recorder_node:main",
        "forklift_mover = amr_validation.forklift_mover_node:main",
    ]},
)
