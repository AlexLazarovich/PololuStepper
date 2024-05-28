import RPi.GPIO as GPIO
from enum import Enum
import time

# Constants
PIN_UNCONNECTED = -1
IS_CONNECTED = lambda pin: pin != PIN_UNCONNECTED
STEP_PULSE = lambda steps, microsteps, rpm: 60.0 * 1000000 / steps / microsteps / rpm
MIN_YIELD_MICROS = 50
stepperMin = lambda a, b: a if a < b else b
stepperMax = lambda a, b: a if a > b else b


def delayMicros(delay_us, start_us=None):
    '''Delay for the given number of microseconds. If start_us is provided, the delay is relative to that time.'''
    if delay_us:
        if start_us is None:
            start_us = time.time_ns() // 1000  # Convert to microseconds
        if delay_us > MIN_YIELD_MICROS:
            time.sleep(delay_us / 1e6)  # Yield for the duration in seconds
        
        while (time.time_ns() // 1000) - start_us < delay_us:
            pass  # Busy wait

class State(Enum):
    '''State of the stepper motor.'''
    STOPPED = 0
    ACCELERATING = 1
    CRUISING = 2
    DECELERATING = 3

class Mode(Enum):
    '''Mode of operation for the stepper motor.'''
    CONSTANT_SPEED = 0
    LINEAR_SPEED = 1

class Profile:
    '''Speed profile for the stepper motor.'''
    def __init__(self, mode=Mode.CONSTANT_SPEED, accel=1000, decel=1000):
        self.mode = mode
        self.accel = accel
        self.decel = decel

class BasicStepperDriver:
    '''Basic stepper motor driver for controlling a stepper motor.'''
    __MAX_MICROSTEP = 128
    _step_high_min = 2.5
    _step_low_min = 2.5
    _wakeup_time = 0

    def __init__(self, steps, dir_pin, step_pin, enable_pin=PIN_UNCONNECTED):
        '''Initialize the stepper motor driver with the given parameters.'''
        self._motor_steps = steps
        self._dir_pin = dir_pin
        self._step_pin = step_pin
        self._enable_pin = enable_pin
        self._enable_active_state = GPIO.HIGH
        self._microsteps = 1
        self._rpm = 0
        self._profile = Profile()
        self._step_count = 0
        self._steps_remaining = 0
        self._steps_to_cruise = 0
        self._steps_to_brake = 0
        self._step_pulse = 0
        self._cruise_step_pulse = 0
        self._dir_state = 0

        self.__rest = 0
        self.__last_action_end = 0
        self.__next_action_interval = 0

        GPIO.setmode(GPIO.BCM)

    def begin(self, rpm=60, microsteps=1):
        '''Initialize the stepper motor with the given RPM and microsteps.'''
        # Set up the pins
        GPIO.setup(self._dir_pin, GPIO.OUT)
        GPIO.output(self._dir_pin, GPIO.HIGH)

        GPIO.setup(self._step_pin, GPIO.OUT)
        GPIO.output(self._step_pin, GPIO.LOW)

        # Set up the enable pin if connected
        if IS_CONNECTED(self._enable_pin):
            GPIO.setup(self._enable_pin, GPIO.OUT)
            self.disable()

        self._rpm = rpm
        self.setMicrostep(microsteps)
        self.enable()

    def setMicrostep(self, microsteps):
        '''Set the microstep resolution of the stepper motor.'''
        ms = 1
        while ms <= self._getMaxMicrostep():
            if microsteps == ms:
                self._microsteps = microsteps
                break
            ms <<= 1
        return self._microsteps

    def getMicrostep(self):
        '''Get the microstep resolution of the stepper motor.'''
        return self._microsteps

    def getSteps(self):
        '''Get the number of steps of the stepper motor.'''
        return self._motor_steps

    def setRPM(self, rpm):
        '''Set the RPM of the stepper motor.'''
        if self._rpm == 0:
            self.begin(rpm, self._microsteps) # Initialize the stepper motor if it hasn't been already
        self._rpm = rpm

    def getRPM(self):
        '''Get the RPM of the stepper motor.'''
        return self._rpm

    def getCurrentRPM(self):
        '''Get the current RPM of the stepper motor based on the last step interval.'''
        return 60.0 * 1000000 / self._step_pulse / self._microsteps / self._motor_steps

    def setSpeedProfile(self, mode, accel=1000, decel=1000):
        '''Set the speed profile of the stepper motor.'''
        if isinstance(mode, Profile):
            self._profile = mode
        else:
            self._profile.mode = mode
            self._profile.accel = accel
            self._profile.decel = decel

    def getSpeedProfile(self):
        '''Get the speed profile of the stepper motor.'''
        return self._profile

    def getAcceleration(self):
        '''Get the acceleration of the stepper motor.'''
        return self._profile.accel

    def getDeceleration(self):
        '''Get the deceleration of the stepper motor.'''
        return self._profile.decel

    def move(self, steps):
        '''Move the stepper motor the given number of steps.'''
        self.startMove(steps)
        while self.nextAction(): # Wait for the next action to complete
            pass

    def rotate(self, deg):
        '''Rotate the stepper motor the given number of degrees.'''
        self.move(self.calcStepsForRotation(deg))

    def setEnableActiveState(self, state):
        '''Set the active state of the enable pin.'''
        self._enable_active_state = state

    def enable(self):
        '''Enable the stepper motor.'''
        if IS_CONNECTED(self._enable_pin):
            GPIO.output(self._enable_pin, self._enable_active_state)
        delayMicros(2)

    def disable(self):
        '''Disable the stepper motor.'''
        if IS_CONNECTED(self._enable_pin):
            GPIO.output(self._enable_pin, GPIO.LOW if self._enable_active_state == GPIO.HIGH else GPIO.HIGH)

    def startMove(self, steps, time=0):
        '''Start moving the stepper motor the given number of steps. If time is provided, it will move at a constant speed.'''
        self._dir_state = GPIO.HIGH if steps >= 0 else GPIO.LOW # Set the direction
        self.__last_action_end = 0 # Reset the last action end time
        self._steps_remaining = abs(steps) # Set the number of steps remaining
        self._step_count = 0 # Reset the step count
        self.__rest = 0 # Reset the rest
        if self._profile.mode == Mode.LINEAR_SPEED:
            # Calculate the number of steps to accelerate and decelerate
            speed = self._rpm * self._motor_steps / 60
            if time > 0:
                t = time / 1e6
                d = self._steps_remaining / self._microsteps
                a2 = 1.0 / self._profile.accel + 1.0 / self._profile.decel
                sqrt_candidate = t * t - 2 * a2 * d
                if sqrt_candidate >= 0:
                    speed = stepperMin(speed, (t - sqrt_candidate ** 0.5) / a2)
            self._steps_to_cruise = self._microsteps * (speed * speed / (2 * self._profile.accel))
            self._steps_to_brake = self._steps_to_cruise * self._profile.accel / self._profile.decel
            if self._steps_remaining < self._steps_to_cruise + self._steps_to_brake:
                self._steps_to_cruise = self._steps_remaining * self._profile.decel / (self._profile.accel + self._profile.decel)
                self._steps_to_brake = self._steps_remaining - self._steps_to_cruise
            self._step_pulse = (1e6 * 0.676 * (2 / self._profile.accel / self._microsteps) ** 0.5)
            self._cruise_step_pulse = 1e6 / speed / self._microsteps
        else:
            # Calculate the step pulse
            self._steps_to_cruise = 0
            self._steps_to_brake = 0
            self._step_pulse = self._cruise_step_pulse = STEP_PULSE(self._motor_steps, self._microsteps, self._rpm)
            if time > self._steps_remaining * self._step_pulse:
                self._step_pulse = time / self._steps_remaining

    def startRotate(self, deg):
        '''Start rotating the stepper motor the given number of degrees.'''
        self.startMove(self.calcStepsForRotation(deg))

    def nextAction(self):
        '''Perform the next action of the stepper motor. Returns the time until the next action is needed.'''
        if self._steps_remaining > 0:
            delayMicros(self.__next_action_interval, self.__last_action_end)
            GPIO.output(self._dir_pin, self._dir_state)
            GPIO.output(self._step_pin, GPIO.HIGH)
            m = time.time_ns() // 1000
            pulse = self._step_pulse
            self._calcStepPulse()
            delayMicros(self._step_high_min)
            GPIO.output(self._step_pin, GPIO.LOW)
            self.__last_action_end = time.time_ns() // 1000
            m = self.__last_action_end - m
            self.__next_action_interval = pulse - m if pulse > m else 1
        else:
            self.__last_action_end = 0
            self.__next_action_interval = 0
        return self.__next_action_interval

    def startBrake(self):
        '''Start braking the stepper motor.'''
        current_state = self.getCurrentState()
        if current_state == State.CRUISING:
            self._steps_remaining = self._steps_to_brake
        elif current_state == State.ACCELERATING:
            self._steps_remaining = self._step_count * self._profile.accel / self._profile.decel

    def stop(self):
        '''Stop the stepper motor and return the number of steps remaining.'''
        retval = self._steps_remaining
        self._steps_remaining = 0
        return retval

    def getCurrentState(self):
        '''Get the current state of the stepper motor.'''
        if self._steps_remaining <= 0:
            return State.STOPPED
        if self._steps_remaining <= self._steps_to_brake:
            return State.DECELERATING
        if self._step_count <= self._steps_to_cruise:
            return State.ACCELERATING
        return State.CRUISING

    def getStepsCompleted(self):
        '''Get the number of steps completed by the stepper motor.'''
        return self._step_count

    def getStepsRemaining(self):
        '''Get the number of steps remaining for the stepper motor.'''
        return self._steps_remaining

    def getDirection(self):
        '''Get the direction of the stepper motor.'''
        return 1 if self._dir_state == GPIO.HIGH else -1

    def getTimeForMove(self, steps):
        '''Get the time needed for the stepper motor to move the given number of steps.'''
        if steps == 0:
            return 0
        if self._profile.mode == Mode.LINEAR_SPEED:
            self.startMove(steps)
            cruise_steps = self._steps_remaining - self._steps_to_cruise - self._steps_to_brake
            speed = self._rpm * self._motor_steps / 60
            t = (cruise_steps / (self._microsteps * speed)) + (2.0 * self._steps_to_cruise / self._profile.accel / self._microsteps) ** 0.5 + (2.0 * self._steps_to_brake / self._profile.decel / self._microsteps) ** 0.5
            t *= 1e6
        else:
            t = steps * STEP_PULSE(self._motor_steps, self._microsteps, self._rpm)
        return round(t)

    def calcStepsForRotation(self, deg):
        '''Calculate the number of steps needed for the stepper motor to rotate the given number of degrees.'''
        return deg * self._motor_steps * self._microsteps / 360

    def _getMaxMicrostep(self):
        '''Get the maximum microstep resolution of the stepper motor.'''
        return BasicStepperDriver.__MAX_MICROSTEP

    def _alterMove(self, steps):
        '''Alter the current move by the given number of steps.'''
        current_state = self.getCurrentState()
        if current_state in [State.ACCELERATING, State.CRUISING]:
            if steps >= 0:
                self._steps_remaining += steps
            else:
                self._steps_remaining = stepperMax(self._steps_to_brake, self._steps_remaining + steps)
        elif current_state == State.STOPPED:
            self.startMove(steps)

    def _calcStepPulse(self):
        '''Calculate the step pulse for the next step.'''
        if self._steps_remaining <= 0:
            return

        self._steps_remaining -= 1
        self._step_count += 1

        if self._profile.mode == Mode.LINEAR_SPEED:
            current_state = self.getCurrentState()
            if current_state == State.ACCELERATING:
                if self._step_count < self._steps_to_cruise:
                    self._step_pulse = self._step_pulse - (2 * self._step_pulse + self.__rest) / (4 * self._step_count + 1)
                    self.__rest = (2 * self._step_pulse + self.__rest) % (4 * self._step_count + 1)
                else:
                    self._step_pulse = self._cruise_step_pulse
                    self.__rest = 0
            elif current_state == State.DECELERATING:
                self._step_pulse = self._step_pulse - (2 * self._step_pulse + self.__rest) / (-4 * self._steps_remaining + 1)
                self.__rest = (2 * self._step_pulse + self.__rest) % (-4 * self._steps_remaining + 1)
