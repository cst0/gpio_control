from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['gpio_control'],
    scripts=['scripts/noroot-gpio.sh'],
    package_dir={'': 'src'}
)

setup(**d)
