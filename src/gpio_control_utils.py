#!/usr/bin/env python3

"""
Control GPIO pins via ROS. Made to be as generic as possible, allowing the same node to be
used in multiple configurations/devices.

@author cst <chris thierauf, christopher.thierauf@tufts.edu>
@version 0.0.1
@license Apache 2.0
@copyright Christopher Thierauf 2020.
This copyright is used to release the code in accordance with the license of this repository.
"""

# core stuff
import sys
import rospy

# putting the pin number (e.g., 12) in the topic name is invalid in ROS, so convert it to
# 'twelve' or whatever
import num2word

# messages
from gpio_control.msg import InputState, OutputState
from std_msgs.msg import Header

# A list of devices we have support for
VALID_DEVICES = ['pi', 'jetson', 'beaglebone', 'onion', 'file-system', 'simulated']

# Valid imports are going to depend on our hardware and what's installed. We'll try to import
# everything we might use, and if we fail, keep it in mind.
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
    """

    def __init__(self, configure, set_high_=None, set_low_=None, additional_shutdown=None):
        if configure is not None:
            self.obj = configure()
        self.set_high_func = set_high_
        self.set_low_func = set_low_
        self.additional_shutdown = additional_shutdown

    def set_low(self):
        """ Pull the pin low (0v), but don't try to run None """
        return self.set_low_func() if self.set_low_func is not None else None

    def set_high(self):
        """ Pull the pin high (typically 3.3v or 5v), but don't try to run None """
        return self.set_high_func() if self.set_high_func is not None else None

    def close(self):
        """ Make sure we're cleaning up while we shut down here """
        self.set_low()
        if self.additional_shutdown is not None:
            self.additional_shutdown()


class _GenericInputPin:
    """
    Class to provide consistent function calls to different pin inputs.

    As before, takes in functions to be called later.
    """

    def __init__(self, configure, get_=None, additional_shutdown=None):
        if configure is not None:
            self.obj = configure()
        self.get_func = get_
        self.additional_shutdown = additional_shutdown

    def get(self):
        """ Get the current state of the GPIO pin. Returns true/false to indicate high/low """
        return self.get_func()

    def close(self):
        """ Do additional shutdown things, if necessary """
        if self.additional_shutdown is not None:
            self.additional_shutdown()


def _configure_input(pin, device: str):
    """
    Configure the node as an input. Takes in a pin to access, and a string which is what was
    passed in at the command line.
    """

    # Run through our options, configure the generic input interfaces, and then return.
    if device in ('pi', 'jetson'):
        # These use the same pinout and can use the same driver (we'll use pigpio), so our lives
        # just got easier
        if not _IMPORTED_PIGPIO:
            rospy.logfatal("You want to control your device with pigpio, but it didn't import "
                           "properly. Is it installed? Node will exit.")
            sys.exit(2)
        return_input = _GenericInputPin(pigpio.pi())
        return_input.get_func = return_input.obj.read(pin)
        return return_input
    if device == 'beaglebone':
        if not _IMPORTED_ADAFRUIT_BBIO:
            rospy.logfatal("You want to control your device using the Adafruit BeagleBone Black "
                           "GPIO library, but it didn't import properly. Is it installed? Node "
                           "will exit.")
            sys.exit(2)
        return _GenericInputPin(BBGPIO.setup(pin, BBGPIO.IN), BBGPIO.input(pin), BBGPIO.cleanup())
    if device == 'onion':
        if not _IMPORTED_ONION_GPIO:
            rospy.logfatal("You want to control your device using the Onion Omega "
                           "GPIO library, but it didn't import properly. Is it installed? Node "
                           "will exit.")
            sys.exit(2)
        return_input = _GenericInputPin(onionGpio.OnionGpio(pin))
        return_input.obj.setInputDirection()
        return_input.get_func = return_input.obj.getValue
        return return_input
    if device == 'file-system':
        def open_pin():
            exporter = open('/sys/class/gpio/export', 'w')
            exporter.write(str(pin))
            exporter.close()
            direction = open('/sys/class/gpio'+str(pin)+'/direction', 'w')
            direction.write('in')
            direction.close()

        def get_pin():
            pass

        def close_pin():
            unexporter = open('/sys/class/gpio/unexport', 'w')
            unexporter.write(str(pin))
            unexporter.close()

        return _GenericInputPin(open_pin,
                                get_pin,
                                additional_shutdown=close_pin)
    if device == 'simulated':
        return _GenericInputPin(None, get_=(lambda: True))

    raise RuntimeError('Device was invalid: ' + device)


def _configure_output(pin: int, device: str):
    """
    Configure the node as an output. Takes in a pin to access, and a string which is what was
    passed in at the command line.
    """
    if device in ('pi', 'jetson'):
        # as with configure_input, we can use pigpio for both
        if not _IMPORTED_PIGPIO:
            rospy.logfatal("You want to control your device with pigpio, but it didn't import "
                           "properly. Node will exit.")
            sys.exit(2)
        return_output = _GenericOutputPin(pigpio.pi())
        return_output.set_low_func = return_output.obj.write(pin, pigpio.LOW)
        return_output.set_high_func = return_output.obj.write(pin, pigpio.HIGH)
        return return_output
    if device == 'beaglebone':
        if not _IMPORTED_ADAFRUIT_BBIO:
            rospy.logfatal("You want to control your device using the Adafruit BeagleBone Black "
                           "GPIO library, but it didn't import properly. Is it installed? Node "
                           "will exit.")
            sys.exit(2)
        return _GenericOutputPin(
            BBGPIO.setup(pin, BBGPIO.OUT),
            BBGPIO.output(pin, BBGPIO.LOW),
            BBGPIO.output(pin, BBGPIO.HIGH),
            BBGPIO.cleanup())
    if device == 'onion':
        # as with configure_input, we can use pigpio for both
        if not _IMPORTED_PIGPIO:
            rospy.logfatal("You want to control your device with pigpio, but it didn't import "
                           "properly. Node will exit.")
            sys.exit(2)
        return_output = _GenericOutputPin(onionGpio.OnionGpio(pin))
        return_output.obj.setOutputDirection()
        return_output.set_low_func = return_output.obj.setValue(0)
        return_output.set_high_func = return_output.obj.setValue(1)
        return return_output
    if device == 'file-system':
        def open_pin():
            exporter = open('/sys/class/gpio/export', 'w')
            exporter.write(str(pin))
            exporter.close()
            direction = open('/sys/class/gpio'+str(pin)+'/direction', 'w')
            direction.write('in')
            direction.close()

        def set_low():

            pass

        def set_high():
            pass

        def close_pin():
            unexporter = open('/sys/class/gpio/unexport', 'w')
            unexporter.write(str(pin))
            unexporter.close()

        return _GenericOutputPin(open_pin(), set_low(), set_high(), additional_shutdown=close_pin())

    if device == 'simulated':
        return _GenericOutputPin(None, set_high_=(lambda: print("[simulated] high!")),
                                 set_low_=(lambda: print("[simulated] low!")))
    raise RuntimeError('Device was invalid: ' + device)


def _configure_publisher(_input_pin: _GenericInputPin, pin: int, publisher_rate: int = 10):
    """
    Set up a publisher, which we'll be using to display the current state of the input pin.

    First, we set up a class, which allows us to create a spinner which will check the state of
    the pin, publish it, and sleep at the amount specified by the rate.
    """

    def read_and_publish_spinner():
        # Set up a publisher, where the name is gpio_inputs plus the name of the pin
        publisher = rospy.Publisher("gpio_inputs/" + num2word.word(pin).lower(),
                                    InputState,
                                    queue_size=1)
        rate = rospy.Rate(publisher_rate)

        # Here's where we're doing the actual spinning: read the pin, set up a message, publish,
        # rate.sleep(), repeat.
        while not rospy.is_shutdown():
            val = _input_pin.get_func()
            header = Header()
            header.stamp.nsecs = rospy.Time.now()
            if val or val == 1:
                val = True
            elif val or val == 0:
                val = False
            else:
                rospy.logerr("Not sure how to deal with " + str(val))

            publisher.publish(InputState(header, val, pin))
            rate.sleep()

    # Return the function itself, not the return val of the function
    return read_and_publish_spinner


def _configure_subscriber(_output_pin: _GenericOutputPin, pin: int):
    """
    Set up a subscriber, which we'll be using to receive a new state to set the GPIO pin to.

    First, we'll set up a function to be the callback for the subscriber.
    """

    def subscriber_callback(msg):
        if msg.state:
            _output_pin.set_high_func()
        elif not msg.state:
            _output_pin.set_low_func()
        else:
            rospy.logerr("Not sure how to deal with " + str(msg))

    # Create a subscriber, where the name is gpio_outputs plus the name of the pin
    rospy.Subscriber("gpio_outputs/" + num2word.word(pin).lower(),
                     OutputState,
                     subscriber_callback)

    return rospy.spin()


class GpioControl:
    """
    Generic control of a GPIO device. Wraps the setup and then provides a spinner to run.
    """

    def __init__(self, is_input: bool, device: str, pin_num: int = None, pin_name: str = None):
        self.is_input = is_input
        self._pin = pin_num if pin_num is not None else pin_name
        self._device = device

        if device not in VALID_DEVICES:
            rospy.logerr("I don't know that device (" + device + "). Valid devices: " + str(
                VALID_DEVICES) + "\nExiting.")
            sys.exit(1)

        if is_input:  # if we're an input, configure as an input and set up a publisher
            self._input_pin_obj = _configure_input(self._pin, device)
            self._spinner = _configure_publisher(self._input_pin_obj, self._pin)
        else:  # if we're an output, configure as an output and set up a subscriber
            self._output_pin_obj = _configure_output(self._pin, device)
            self._spinner = _configure_subscriber(self._output_pin_obj, self._pin)

    def __del__(self):
        if not self.is_input:  # if not an input, meaning it's an output
            self._output_pin_obj.close()

    def spin(self):
        """ Wrapping the spinner function. """
        self._spinner()

    def set_pin(self, state: bool):
        """
        If using this code as an import, provide a simple function to set the pin.
        """
        if self.is_input:
            raise EnvironmentError('This pin is set up as an input, not an output!')
        if state:
            self._output_pin_obj.set_high()
        else:
            self._output_pin_obj.set_low()

    def get_pin(self):
        """
        If using this code as an import, provide a simple function to get the state of the pin.
        """
        if not self.is_input:
            raise EnvironmentError('This pin is set up as an output, not an input!')
        return self._input_pin_obj.get()
