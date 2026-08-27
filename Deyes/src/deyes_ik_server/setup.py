from glob import glob
import os

from setuptools import setup

PACKAGE_NAME = "deyes_ik_server"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=[PACKAGE_NAME],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        (os.path.join("share", PACKAGE_NAME, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy", "scipy", "ikpy"],
    zip_safe=True,
    maintainer="Robo Rak Mercy Team",
    maintainer_email="roborak@example.com",
    description="ExecuteCartesianStage IK action server for Mercury X1 (ikpy-backed).",
    license="TODO",
    entry_points={
        "console_scripts": [
            "ik_action_server = deyes_ik_server.ik_action_server:main",
            "ikpy_solver_smoketest = deyes_ik_server.ikpy_solver_smoketest:main",
        ],
    },
)