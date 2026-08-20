from glob import glob
from pathlib import Path
from setuptools import setup

package_name = "deyes_bringup"
package_root = Path(__file__).resolve().parent
config_glob = "../../config/stereo/*.yaml"
camera_config_glob = "../../config/camera/*.yaml"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob(config_glob)),
        ("share/" + package_name + "/config/camera", glob(camera_config_glob)),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TRAE Agent",
    maintainer_email="devnull@example.com",
    description="Deyes launch and parameter orchestration.",
    license="Proprietary",
    tests_require=["pytest"],
)
