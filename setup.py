"""
Setup.py. Makes sure that we have access to our gpio_control_utils lib, both within our node
and within other workspaces if that's something people want to use. Also make sure we can run the
no-root script, if necessary.
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['gpio_control'],
    package_dir={'': 'include'},
)

setup(**d)
