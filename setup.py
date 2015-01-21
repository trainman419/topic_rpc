#!/usr/bin/env python
#
# this setup.py for catkin only; do not execute directly

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['topic_rpc', 'topic_rpc.msg'],
    package_dir={'': 'src'}
)

setup(**d)
