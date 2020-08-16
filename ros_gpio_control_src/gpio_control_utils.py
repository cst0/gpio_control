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

    def __init__(self, configure, pin, set_high_=None, set_low_=None, additional_shutdown=None):
        self.type = 'output'
        if configure is not None:
            self.obj = configure()
        self.set_high_func = set_high_
        self.set_low_func = set_low_
        self.additional_shutdown = additional_shutdown
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
        if self.additional_shutdown is not None:
            self.additional_shutdown()


class _GenericInputPin:
    """
    Class to provide consistent function calls to different pin inputs.

    As before, takes in functions to be called later.
    """

    def __init__(self, configure, pin, get_=None, additional_shutdown=None):
        self.type = 'input'
        if configure is not None:
            self.obj = configure()
        self.get_func = get_
        self.additional_shutdown = additional_shutdown
        self.pin = pin

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
        return_input = _GenericInputPin(pigpio.pi(), pin)
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
        return_input = _GenericInputPin(onionGpio.OnionGpio(pin), pin)
        return_input.obj.setInputDirection()
        return_input.get_func = return_input.obj.getValue
        return return_input
    if device == 'generic':
        rospy.logwarn('You are using the generic GPIO controller, which operates using the Linux '
                      'FHS to control GPIO states. It should not be considered as stable or safe '
                      'as non-generic options.')

        def open_pin():
            exporter = open('/sys/class/gpio/export', 'w')
            exporter.write(str(pin))
            exporter.close()
            direction = open('/sys/class/gpio' + str(pin) + '/direction', 'w')
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
        return _GenericInputPin(None, pin, get_=(lambda: True))

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
        return_output = _GenericOutputPin(pigpio.pi(), pin)
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
        return_output = _GenericOutputPin(onionGpio.OnionGpio(pin), pin)
        return_output.obj.setOutputDirection()
        return_output.set_low_func = return_output.obj.setValue(0)
        return_output.set_high_func = return_output.obj.setValue(1)
        return return_output
    if device == 'generic':
        rospy.logwarn('You are using the generic GPIO controller, which operates using the Linux '
                      'FHS to control GPIO states. It should not be considered as stable or safe '
                      'as non-generic options.')

        def open_pin():
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

        def close_pin():
            unexporter = open('/sys/class/gpio/unexport', 'w')
            unexporter.write(str(pin))
            unexporter.close()

        return _GenericOutputPin(open_pin(), set_low(), set_high(), additional_shutdown=close_pin())

    if device == 'simulated':
        return _GenericOutputPin(None, pin, set_high_=(lambda: print("[simulated] high!")),
                                 set_low_=(lambda: print("[simulated] low!")))
    raise RuntimeError('Device was invalid: ' + device)


def _configure_publisher(_input_pin: _GenericInputPin, pin: int):
    """
    Set up a publisher, which we'll be using to display the current state of the input pin.

    First, we set up a class, which allows us to create a spinner which will check the state of
    the pin, publish it, and sleep at the amount specified by the rate.
    """

    publisher = rospy.Publisher("gpio_inputs/" + num2word.word(pin).lower(),
                                InputState,
                                queue_size=1)

    # Return the function itself, not the return val of the function
    return publisher


def _configure_subscriber(_output_pin: _GenericOutputPin, pin: int):
    """
    Set up a subscriber, which we'll be using to receive a new state to set the GPIO pin to.

    First, we'll set up a function to be the callback for the subscriber.
    """

    class subscriber_class:
        def subscriber_callback(self, msg):
            if msg.state:
                _output_pin.set_high_func()
            elif not msg.state:
                _output_pin.set_low_func()
            else:
                rospy.logerr("Not sure how to deal with " + str(msg))

    # Create a subscriber, where the name is gpio_outputs plus the name of the pin
    this_subscriber = subscriber_class()
    return rospy.Subscriber("gpio_outputs/" + num2word.word(pin).lower(),
                     OutputState,
                     this_subscriber.subscriber_callback)


class GpioControl:
    """
    Generic control of a GPIO device. Wraps the setup and then provides a spinner to run.
    """

    def __init__(self, device: str):
        self._device = device
        self._publishers = {}
        self._generic_pin_objects = {}
        self._subscribers = []

        if device not in VALID_DEVICES:
            rospy.logerr("I don't know that device (" + device + "). Valid devices: " +
                         str(VALID_DEVICES) + "\nExiting.")
            sys.exit(1)

    def __del__(self):
        for pin_obj in self._generic_pin_objects.values():
            pin_obj.close()

    def add_input_pin(self, pin):
        """ Add a pin to perform input IO operations. """
        input_pin_obj = _configure_input(pin, self._device)
        self._generic_pin_objects[pin] = input_pin_obj
        self._publishers[pin] = _configure_publisher(input_pin_obj, pin)

    def add_output_pin(self, pin):
        """ Add a pin to perform output IO operations. """
        output_pin_obj = _configure_output(pin, self._device)
        self._generic_pin_objects[pin] = output_pin_obj
        self._subscribers.append(_configure_subscriber(output_pin_obj, pin))

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

                    self._publishers[pin_obj.pin].publish(InputState(header, val, str(pin_obj.pin)))

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
