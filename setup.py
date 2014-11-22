#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['mico_rr_bridge']
d['package_dir'] = {'': 'scripts'}

setup(**d)
