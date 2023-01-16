#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author     : Hongzhuo Liang
# E-mail     : liang@informatik.uni-hamburg.de
# Description:
# File Name  : setup.py
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=["moveit_arm_commander"],
    package_dir={'': 'src'},
)

setup(**setup_args)
