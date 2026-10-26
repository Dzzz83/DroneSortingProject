#!/usr/bin/env python3

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=[
        "drone_sorting",
        "drone_sorting.mission",
        "drone_sorting.control",
        "drone_sorting.interfaces",
        "drone_sorting.perception",
        "drone_sorting.mapping",
    ],
    package_dir={"": "python"},
)

setup(**setup_args)
