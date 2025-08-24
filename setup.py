from setuptools import find_packages, setup
from glob import glob
import os

package_name = "sim_node"


def collect_resource_files(base_dir, package_name):
    """
    Recursively collects all files under base_dir and returns a list of tuples:
    (install_path, [list-of-files]) suitable for the data_files argument.

    The install_path will be:
    share/<package_name>/<base_dir>/<relative_path>
    """
    entries = []
    for root, dirs, files in os.walk(base_dir):
        if files:
            rel_path = os.path.relpath(root, base_dir)
            install_path = os.path.join(
                "share", package_name, base_dir, rel_path if rel_path != "." else ""
            )
            file_paths = [os.path.join(root, f) for f in files]
            entries.append((install_path, file_paths))
    return entries


data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    ("share/" + package_name + "/launch", glob("launch/*")),
]


data_files.extend(collect_resource_files("resource/models", package_name=package_name))

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    install_requires=["setuptools"],
    data_files=data_files,
    zip_safe=True,
    maintainer="keyush",
    maintainer_email="keyushattarde@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sim_node = sim_node.ros_bridge:main",
            "keyboard_controls = keyboard_controls.keyboard_controls:main",
        ],
    },
)
