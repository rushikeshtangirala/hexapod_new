from setuptools import find_packages, setup

package_name = "hexapod_gait"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # Registers the package with the ament index. Without this entry
        # `ros2 pkg list` will not show the package and `ros2 run` cannot
        # find its executables, even though pip-style installation succeeded.
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Varun",
    maintainer_email="hexapod69420@gmail.com",
    description="Leg kinematics and tripod gait generation for the hexapod.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gait_node = hexapod_gait.gait_node:main",
        ],
    },
)
