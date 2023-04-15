import os
from glob import glob
from setuptools import setup

package_name = "square_calib"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include all launch files.
        (os.path.join("share", package_name), glob("launch/*launch.[pxy][yma]*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Damien LaRocque",
    maintainer_email="phicoltan@gmail.com",
    description="Square Path Publisher",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["square_calibration = square_calib.square_calibration_node:main"],
    },
)
