from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['gpio_control'],
    package_dir={'': 'include'}
)
#    scripts=['scripts/noroot-gpio.sh'],

setup(**d)
