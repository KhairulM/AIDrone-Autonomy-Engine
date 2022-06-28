from setuptools import setup
from glob import glob

package_name = "flight_director"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, glob("launch/*.launch.*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="khairulm",
    maintainer_email="khairul240999@gmail.com",
    description="Flight director package contains nodes for mission loading and overall mission execution progress",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "api_server_node = flight_director.APIServerNode:main",
            "director_node = flight_director.DirectorNode:main",
            "monitor_node =  flight_director.MonitorNode:main",
        ],
    },
)
