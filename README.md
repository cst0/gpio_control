# gpio\_control: a ROS package for reading/writing GPIO states on Raspberry Pi, Jetson, etc.
## Overview
_This package is currently unstable and in testing_

Devices such as the Raspberry Pi, NVidia Jetson, BeagleBone Black, etc have GPIO pins as an additional set of IO,
which can be toggled between high/low states or configured to read high/low inputs. This package allows for
ROS control of these pins, allowing for configuring each pin as desired and then reading/writing
them as a ROS node.

This package has been most thoroughly tested on ROS Melodic running on the NVidia Jetson Nano B01,
but has been designed with other platforms and ROS versions in mind. Please file an issue if
this package does not behave as expected.

The `gpio_control` node allows for control of GPIO through the standard Linux /dev/sys/gpio directories, allowing for support
of virtually every Linux device with GPIO pins. Additionally, devices such as the Raspberry Pi, which have
their own GPIO software API, are supported via this API in the same node.

NOTE: Writing GPIO control states via the filesystem may include writing to pins which are in use
by the operating system, which can have unexpected consequences (loss of SD card data, crashes, etc).
It is recommended that you use the device-specific flag, which will perform safety checks and prevent
you from doing anything too damaging. There are device specific flags for the Raspberry Pi's (--device pi),
Nvidia Jetson's (--device jetson), BeagleBone Black (--device beaglebone), and Onion Omega (--device onion).

WARNING: Installation of this node will allow for non-root users to control GPIO pins on your device. This may be
considered a security risk, and so deployment in a production environment is not recommended. Alternatively, you may run
ROS as root, but that has it's own set of security risks. Consider running just this node as root using `launch-prefix="sudo"`
in your roslaunch file if that's the route you'd like to go.

## Installation
Clone this package into your catkin workspace and then build it:

```
cd catkin_ws/src
git clone https://github.com/cst0/gpio_control
catkin_make
```

You will then need to run the non-root GPIO script, which will configure your GPIO pins to allow access from non-root users:

```
sudo src/gpio_control/scripts/non-root-gpio.sh
```

Optionally, skip this step if you intent to run ROS as root.

You may test your installation by toggling a pin on your device, e.g.:

```
roslaunch gpio_control pi_flash.launch set_device:=pi
```

If you do not use `set_device`, the system will do its best, which may not be correct. If your device is supported, it
is recommended you specify the device. If it is not, double check that the pin being used (12) will not damage
your hardware.

## Usage
Each node accepts the same command line parameters. In the following command, `gpio_control` is used
to create an input on pin 12 of the Raspberry Pi:

```
rosrun gpio_control gpio_control --device pi --input 12
```

Upon running this command, there should now be a topic titled '/gpio/input/12' which is publishing a `gpio_control/input` message.

Creating an output is similar:
```
rosrun gpio_control gpio_control --device pi --output 12
```

Upon running this command, there should now be a topic titled '/gpio/output/12' which will publish a `gpio_control/output` message
upon a state change. However, it is also possible to publish the current state every cycle using the `--every-cycle` command. 

