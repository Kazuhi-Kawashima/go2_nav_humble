from setuptools import setup

package_name = "go2_autoware_bridge"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [
            "resource/" + package_name,
        ]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/go2_autoware_bridge.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="go2_nav_humble",
    maintainer_email="maintainer@example.com",
    description="Bridge topics from Go2 Gazebo to Autoware Universe conventions.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "go2_autoware_bridge_node = go2_autoware_bridge.bridge_node:main",
            "go2_cmd_vel_bridge_node = go2_autoware_bridge.cmd_vel_bridge:main",
        ],
    },
)
