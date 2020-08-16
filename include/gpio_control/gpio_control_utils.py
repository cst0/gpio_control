#!/usr/bin/env python3

"""
Control GPIO pins via ROS. Made to be as generic as possible, allowing the same node to be
used in multiple configurations/devices.

@author cst <chris thierauf, christopher.thierauf@tufts.edu>
@version 0.0.1
@license Apache 2.0
@copyright Christopher Thierauf 2020.

The copyright holder uses this copyright to release the code in accordance with the license of
this repository, which is a Free and Open Source license: you may use, modify, and share this
code in any way you see fit as described by the terms of the license. You are not legally
obligated to provide attribution, but it would be greatly appreciated.
"""

# core stuff
import random
import sys
import rospy

# putting the pin number (e.g., 12) in the topic name is invalid in ROS, so convert it to
# 'twelve' or whatever
import num2word

# messages
from std_msgs.msg import Header
from gpio_control.msg import InputState, OutputState

# constants
# a list of devices we have support for
VALID_DEVICES = ['pi', 'jetson', 'beaglebone', 'onion', 'generic', 'simulated']
# rate at which to run the node (be default)
_DEFAULT_RATE_VAL = 10

# more Imports
# valid imports are going to depend on our hardware and what's installed. We'll try to import
# everything we might use, and if we fail, keep track of it for later.
_IMPORTED_PIGPIO = False
_IMPORTED_ADAFRUIT_BBIO = False
_IMPORTED_ONION_GPIO = False

try:
    import pigpio

    _IMPORTED_PIGPIO = True
except ImportError:
    pass

try:
    import Adafruit_BBIO.GPIO as BBGPIO

    _IMPORTED_ADAFRUIT_BBIO = True
except ImportError:
    pass

try:
    import onionGpio

    _IMPORTED_ONION_GPIO = True
except ImportError:
    pass


class _GenericOutputPin:
    """
    Class to provide consistent function calls to different pin outputs.

    Takes in three functions as arguments, which allows us to call 'configure', 'set_low',
    and 'set_high' on this class later, despite it being any actual implementation of gpio control
    Also provide the option of an 'additional_shutdown' to run on close() if necessary.

    We're using functional programming here and elsewhere. If you aren't familiar with that, give
    that topic a quick read: https://docs.python.org/3/howto/functional.html.
    If you're reading this because you'd like to contribute (thanks!) but are feeling daunted,
    please feel free to open an issue on the tracker so I can work with you on that.
    """

    def __init__(self, configured_pinbus_object, pin, set_high_=None, set_low_=None):
        self.type = 'output'
        if configured_pinbus_object is not None:
            self.configured_pinbus_object = configured_pinbus_object
        self.set_high_func = set_high_
        self.set_low_func = set_low_
        self.pin = pin

    def set_low(self):
        """ Pull the pin low (0v), but don't try to run None """
        return self.set_low_func() if self.set_low_func is not None else None

    def set_high(self):
        """ Pull the pin high (typically 3.3v or 5v), but don't try to run None """
        return self.set_high_func() if self.set_high_func is not None else None

    def close(self):
        """ Make sure we're cleaning up while we shut down here """
        self.set_low()


class _GenericInputPin:
    """
    Class to provide consistent function calls to different pin inputs.

    As before, takes in functions to be called later.
    """

    def __init__(self, configured_pinbus_object, pin, get_=None):
        self.type = 'input'
        if configured_pinbus_object is not None:
            self.configured_pinbus_object = configured_pinbus_object
        self.get_func = get_
        self.pin = pin

    def get(self):
        """ Get the current state of the GPIO pin. Returns true/false to indicate high/low """
        return self.get_func()


def _configure_input(pin, device: str, bus):
    """
    Configure the node as an input. Takes in a pin to access, and a string which is what was
    passed in at the command line.
    """

    # Run through our options, configure the generic input interfaces, and then return.
    if device in ('pi', 'jetson'):
        return_input = _GenericInputPin(bus, int(pin))
        return_input.get_func = lambda: return_input.configured_pinbus_object.read(int(pin))
        return return_input

    if device == 'beaglebone':
        BBGPIO.setup(pin, BBGPIO.IN)
        return _GenericInputPin(bus, pin, (lambda: BBGPIO.input(pin)))

    if device == 'onion':
        return_input = _GenericInputPin(bus, pin)
        return_input.configured_pinbus_object.setInputDirection()
        return_input.get_func = return_input.configured_pinbus_object.getValue
        return return_input

    if device == 'generic':
        exporter = open('/sys/class/gpio/export', 'w')
        exporter.write(str(pin))
        exporter.close()
        direction = open('/sys/class/gpio' + str(pin) + '/direction', 'w')
        direction.write('in')
        direction.close()

        def get_pin():
            pass

        return _GenericInputPin(bus, pin, get_pin)

    if device == 'simulated':
        return _GenericInputPin(None, pin, get_=(lambda: random.choice([True, False])))

    raise RuntimeError('Device was invalid: ' + device)


def _configure_output(pin: int, device: str, bus):
    """
    Configure the node as an output. Takes in a pin to access, and a string which is what was
    passed in at the command line.
    """
    if device in ('pi', 'jetson'):
        # as with configure_input, we can use pigpio for both
        return_output = _GenericOutputPin(bus, pin)
        return_output.set_low_func = (
            lambda: return_output.configured_pinbus_object.write(int(pin), pigpio.LOW)
        )
        return_output.set_high_func = (
            lambda: return_output.configured_pinbus_object.write(int(pin), pigpio.HIGH)
        )
        return return_output

    if device == 'beaglebone':
        return _GenericOutputPin(
            lambda: BBGPIO.setup(pin, BBGPIO.OUT),
            lambda: BBGPIO.output(pin, BBGPIO.LOW),
            lambda: BBGPIO.output(pin, BBGPIO.HIGH),
        )

    if device == 'onion':
        return_output = _GenericOutputPin(onionGpio.OnionGpio(pin), pin)
        return_output.configured_pinbus_object.setOutputDirection()
        return_output.set_low_func = lambda: return_output.configured_pinbus_object.setValue(0)
        return_output.set_high_func = lambda: return_output.configured_pinbus_object.setValue(1)
        return return_output

    if device == 'generic':
        exporter = open('/sys/class/gpio/export', 'w')
        exporter.write(str(pin))
        exporter.close()
        direction = open('/sys/class/gpio' + str(pin) + '/direction', 'w')
        direction.write('in')
        direction.close()

        def set_low():
            pass

        def set_high():
            pass

        return _GenericOutputPin(None, set_low, set_high)

    if device == 'simulated':
        return _GenericOutputPin(None, pin, set_high_=(lambda: print("[simulated] high!")),
                                 set_low_=(lambda: print("[simulated] low!")))

    raise RuntimeError('Device was invalid: ' + device)


def _to_valid_ros_topic_name(input_string):
    """
    Convert input to a valid ROS name (alphabetic). This is necessary because ROS best practice is
    to have topic names be only alphabetic (hyphens/slashes optional). Most GPIO pins will have
    numbers in them, which can be an issue.
    """

    # Don't bother dealing with things that are just numbers, convert them right away
    if isinstance(input_string) == int or input_string.isnumeric():
        return num2word.word(int(input_string)).lower()

    # If it's alphabetic, convert numbers to characters, append numbers, and separate the
    # two using underscores. Creates something like P8U4 -> p_eight_u_4
    output_string = ""
    just_did_str = False
    just_did_num = False
    for character in input_string:
        if character.isalpha():
            if just_did_num:
                output_string += '_'
                just_did_num = False
            output_string += character.lower()
            just_did_str = True
        elif character.isnumeric():
            if just_did_str:
                output_string += '_'
                just_did_str = False
            output_string += num2word.word(character).lower()
            just_did_num = True
        else:
            # Don't bother putting in special characters (even though I don't think they'll come up)
            pass

    return output_string


def configure_bus(device):
    """
    Configure a GPIO bus for the specific hardware we're dealing with. Return an object of the
    appropriate hardware type, if the specific implementation requires that.
    """
    # Both the Pi and Jetson have the same pinout and can use the same lib.
    if device in ('pi', 'jetson'):
        if not _IMPORTED_PIGPIO:
            rospy.logfatal("You want to control your device with pigpio, but it didn't import "
                           "properly. Node will exit.")
            sys.exit(2)

        return pigpio.pi()

    if device == 'beaglebone':
        if not _IMPORTED_ADAFRUIT_BBIO:
            rospy.logfatal("You want to control your device using the Adafruit BeagleBone Black "
                           "GPIO library, but it didn't import properly. Is it installed? Node "
                           "will exit.")
            sys.exit(2)

    if device == 'onion':
        if not _IMPORTED_ONION_GPIO:
            rospy.logfatal("You want to control your device using the Onion Omega "
                           "GPIO library, but it didn't import properly. Is it installed? Node "
                           "will exit.")
            sys.exit(2)

    if device == 'filesystem':
        rospy.logwarn('You are using the generic GPIO controller, which operates using the Linux '
                      'FHS to control GPIO states. It should not be considered as stable or safe '
                      'as non-generic options.')

    return None


def configure_cleanup(device):
    """
    If the device in question requires cleanup functions, give them a run.
    """
    if device == 'beaglebone':
        return BBGPIO.cleanup()

    if device == 'filesystem':
        unexporter = open('/sys/class/gpio/unexport', 'w')
        # unexporter.write(str(pin)) # todo: make this a 'foreach' in gpio dir
        unexporter.close()

    return None


class GpioControl:
    """
    Generic control of a GPIO device. Wraps the setup and then provides a spinner to run.
    """

    def __init__(self, device: str):
        self._device = device
        self._publishers = {}
        self._generic_pin_objects = {}
        self._subscribers = {}
        self._bus = configure_bus(device)
        self._cleanup = configure_cleanup(device)

        if device not in VALID_DEVICES:
            rospy.logerr("I don't know that device (" + device + "). Valid devices: " +
                         str(VALID_DEVICES) + "\nExiting.")
            sys.exit(1)

    def __del__(self):
        for pin_obj in self._generic_pin_objects.values():
            pin_obj.close()

    def add_input_pin(self, pin):
        """ Add a pin to perform input IO operations. """
        input_pin_obj = _configure_input(pin, self._device, self._bus)
        self._generic_pin_objects[pin] = input_pin_obj
        self._publishers[pin] = rospy.Publisher("gpio_inputs/" + _to_valid_ros_topic_name(pin),
                                                InputState,
                                                queue_size=1)

    def add_output_pin(self, pin):
        """ Add a pin to perform output IO operations. """
        output_pin_obj = _configure_output(pin, self._device, self._bus)

        def subscriber_callback(msg):
            """
            Subscriber callback.
            Called every time a message comes in telling us to change a pin state.
            """
            if msg.state:
                output_pin_obj.set_high_func()
            elif not msg.state:
                output_pin_obj.set_low_func()
            else:
                rospy.logerr("Not sure how to deal with " + str(msg))

        self._generic_pin_objects[pin] = output_pin_obj
        self._subscribers[pin] = rospy.Subscriber("gpio_outputs/" + _to_valid_ros_topic_name(pin),
                                                  OutputState,
                                                  subscriber_callback)

    def spin(self, rate_val: int = None):
        """ Wrapping the spinner function. """
        # Here's where we're doing the actual spinning: read the pin, set up a message, publish,
        # rate.sleep(), repeat.
        if rate_val is None:
            rate_val = _DEFAULT_RATE_VAL
        rate = rospy.Rate(rate_val)
        while not rospy.is_shutdown():
            for pin_obj in self._generic_pin_objects.values():
                if pin_obj.type == 'input':
                    val = pin_obj.get()

                    header = Header()
                    header.stamp = rospy.Time.now()
                    if val or val == 1:
                        val = True
                    elif val or val == 0:
                        val = False
                    else:
                        rospy.logerr("Not sure how to deal with " + str(val))

                    try:
                        self._publishers[str(pin_obj.pin)].publish(
                            InputState(header, val, str(pin_obj.pin))
                        )
                    except KeyError:
                        rospy.logfatal_once("KeyError, you tried getting " +
                                            str(pin_obj.pin) + " but only " +
                                            str(self._publishers.keys()) +
                                            " is acceptable")

            rate.sleep()

    def set_pin(self, pin, state: bool):
        """
        If using this code as an import, provide a simple function to set the pin.
        """
        if pin not in self._generic_pin_objects.keys():
            rospy.logerr("The pin you requested (" + str(pin) +
                         ") isn't in the list of ones we know about: " +
                         str(self._generic_pin_objects.keys()))

        if not self._generic_pin_objects[pin].type == 'output':
            raise EnvironmentError("This pin is not an output! Can't set the state.")

        if state:
            pin.set_high()
        else:
            pin.set_low()

    def get_pin(self, pin):
        """
        If using this code as an import, provide a simple function to get the state of the pin.
        """
        if pin not in self._generic_pin_objects.keys():
            rospy.logerr("The pin you requested (" + str(pin) +
                         ") isn't in the list of ones we know about: " +
                         str(self._generic_pin_objects.keys()))

        if not self._generic_pin_objects[pin].type == 'output':
            raise EnvironmentError("This pin is not an output! Can't set the state.")

        return pin.get()
