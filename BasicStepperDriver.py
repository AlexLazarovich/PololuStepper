import RPi.GPIO as GPIO
from enum import Enum
import time

PIN_UNCONNECTED  = -1
IS_CONNECTED = lambda pin: pin != PIN_UNCONNECTED
STEP_PULSE = lambda steps, microsteps, rpm: 60 * 1000000 / steps / microsteps / rpm
MIN_YIELD_MICROS = 50
stepperMin = lambda a, b: a if a < b else b
stepperMax = lambda a, b: a if a > b else b
delayMicros = lambda micros: time.sleep(micros / 1e+6)

class State(Enum):
    STOPPED = 0
    ACCELERATING = 1
    CRUISING = 2
    DECELERATING = 3

class Mode(Enum):
    CONSTANT_SPEED = 0
    LINEAR_SPEED = 1
    
class Profile:
    def __init__(self, mode = Mode.CONSTANT_SPEED, accel = 1000, decel = 1000):
        self.mode = mode
        self.accel = accel
        self.decel = decel

class BasicStepperDriver:
    __MAX_MICROSTEP = 128
    _step_high_min = 1
    _step_low_min = 1
    _wakeup_time = 0

    def __init__(self, steps, dir_pin, step_pin, enable_pin=PIN_UNCONNECTED):
        # Proteced
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

        # Private
        self.__rest = 0
        self.__last_action_end = 0
        self.__next_action_interval = 0

        GPIO.setmode(GPIO.BCM)
        
    def begin(self, rpm=60, microsteps=1):
        GPIO.setup(self._dir_pin, GPIO.OUT)
        GPIO.output(self._dir_pin, GPIO.HIGH)

        GPIO.setup(self._step_pin, GPIO.OUT)
        GPIO.output(self._step_pin, GPIO.LOW)

        if IS_CONNECTED(self._enable_pin):
            GPIO.setup(self._enable_pin, GPIO.OUT)
            self.disable()

        self._rpm = rpm
        self.setMicrostep(microsteps)

        self.enable()

    def setMicrostep(self, microsteps):
        ms = 1
        while ms <= self._getMaxMicrostep():
            if microsteps == ms:
                self._microsteps = microsteps
                break
            ms <<= 1  # Equivalent to ms *= 2 or left shift in Python
        return self._microsteps

    def getMicrostep(self):
        return self._microsteps

    def getSteps(self):
        return self._motor_steps

    def setRPM(self, rpm):
        if self._rpm == 0:     # begin() has not been called (old 1.0 code)
            self.begin(rpm, self._microsteps)
        self._rpm = rpm

    def getRPM(self):
        return self._rpm

    def getCurrentRPM(self):
        return 60.0*1000000 / self._step_pulse / self._microsteps / self._motor_steps

    def setSpeedProfile(self, mode, accel=1000, decel=1000): # TODO profile as parameter
        if isinstance(mode, Profile):
            self._profile = mode
        else:
            self._profile.mode = mode
            self._profile.accel = accel
            self._profile.decel = decel

    def getSpeedProfile(self)->Profile:
        return self._profile
    
    def getAcceleration(self):
        return self._profile.accel

    def getDeceleration(self):
        return self._profile.decel

    def move(self, steps):
        self.startMove(steps)
        while self.nextAction():
            pass

    def rotate(self, deg):
        self.move(self.calcStepsForRotation(deg))

    def setEnableActiveState(self, state: State):
        self._enable_active_state = state

    def enable(self):
        if IS_CONNECTED(self._enable_pin):
            GPIO.output(self._enable_pin, self._enable_active_state)
        delayMicros(2)

    def disable(self):
        if IS_CONNECTED(self._enable_pin):
            GPIO.output(self._enable_pin, GPIO.LOW if self._enable_active_state == GPIO.HIGH else GPIO.HIGH)

    def startMove(self, steps, time=0):
        self._dir_state = GPIO.HIGH if steps >= 0 else GPIO.LOW
        self.__last_action_end = 0
        self._steps_remaining = abs(steps)
        self._step_count = 0
        self.__rest = 0
        if self._profile.mode == Mode.LINEAR_SPEED:
            # // speed is in [steps/s]
            speed = self._rpm * self._motor_steps / 60
            if time > 0:
                # // Calculate a new speed to finish in the time requested
                t = time / 1e+6                #// convert to seconds
                d = self._steps_remaining / self._microsteps;   #// convert to full steps
                a2 = 1.0 / self._profile.accel + 1.0 / self._profile.decel
                sqrt_candidate = t*t - 2 * a2 * d;  #// in √b^2-4ac
                if sqrt_candidate >= 0:
                    speed = stepperMin(speed, (t - sqrt_candidate**0.5) / a2)
            # // how many microsteps from 0 to target speed
            self._steps_to_cruise = self._microsteps * (speed * speed / (2 * self._profile.accel))
            # // how many microsteps are needed from cruise speed to a full stop
            self._steps_to_brake = self._steps_to_cruise * self._profile.accel / self._profile.decel
            if self._steps_remaining < self._steps_to_cruise + self._steps_to_brake:
                # // cannot reach max speed, will need to brake early
                self._steps_to_cruise = self._steps_remaining * self._profile.decel / (self._profile.accel + self._profile.decel);
                self._steps_to_brake = self._steps_remaining - self._steps_to_cruise
            # // Initial pulse (c0) including error correction factor 0.676 [us]
            self._step_pulse = (1e+6)*0.676*(2/self._profile.accel/self._microsteps)**0.5
            # // Save cruise timing since we will no longer have the calculated target speed later
            self._cruise_step_pulse = 1e+6 / speed / self._microsteps
        
        else:
            self._steps_to_cruise = 0
            self._steps_to_brake = 0
            self._step_pulse = self._cruise_step_pulse = STEP_PULSE(self._motor_steps, self._microsteps, self._rpm)
            if time > self._steps_remaining * self._step_pulse:
                self._step_pulse = time / self._steps_remaining
        
    def startRotate(self, deg):
        self.startMove(self.calcStepsForRotation(deg))
    
    def nextAction(self):
        if self._steps_remaining > 0:
            # Is it needed?
            # delayMicros(next_action_interval, last_action_end)
            GPIO.output(self._dir_pin, self._dir_state)
            GPIO.output(self._step_pin, GPIO.HIGH)
            m = time.time_ns // 1000
            pulse = self._step_pulse
            self._calcStepPulse()
            # We should pull HIGH for at least 1-2us (step_high_min)
            delayMicros(self._step_high_min)
            GPIO.output(self._step_pin, GPIO.LOW)
            # account for calcStepPulse() execution time; sets ceiling for max rpm on slower MCUs
            self.__last_action_end = time.time_ns // 1000
            m = self.__last_action_end - m
            self.__next_action_interval = pulse - m if pulse > m else 1
        else:
            self.__last_action_end = 0
            self.__next_action_interval = 0
        return self.__next_action_interval

    def startBrake(self):
        current_state = self.getCurrentState();
        if current_state == State.CRUISING:
            self._steps_remaining = self._steps_to_brake
        elif current_state == State.ACCELERATING:
            self._steps_remaining = self._step_count * self._profile.accel / self._profile.decel
        else:
            pass

    def stop(self):
        retval = self._steps_remaining
        self._steps_remaining = 0
        return retval
    
    def getCurrentState(self)->State:
        if self._steps_remaining <= 0:
            state = State.STOPPED
        else:
            if self._steps_remaining <= self._steps_to_brake:
                state = State.DECELERATING
            elif self._step_count <= self._steps_to_cruise:
                state = State.ACCELERATING
            else:
                state = State.CRUISING
        return state

    def getStepsCompleted(self):
        return self._step_count

    def getStepsRemaining(self):
        return self._steps_remaining

    def getDirection(self):
        return 1 if self._dir_state == GPIO.HIGH else -1

    def getTimeForMove(self, steps):
        if steps == 0:
            return 0
        if self._profile.mode == Mode.LINEAR_SPEED:
            self.startMove(steps)
            cruise_steps = self._steps_remaining - self._steps_to_cruise - self._steps_to_brake;
            speed = self._rpm * self._motor_steps / 60
            t = (cruise_steps / (self._microsteps * speed)) + (2.0 * self._steps_to_cruise / self._profile.accel / self._microsteps)**0.5 + (2.0 * self._steps_to_brake / self._profile.decel / self._microsteps)**0.5
            t *= 1e+6
        else:
            t = steps * STEP_PULSE(self._motor_steps, self._microsteps, self._rpm)
        return round(t)

    def calcStepsForRotation(self, deg):
        return deg * self._motor_steps * self._microsteps / 360

    def _getMaxMicrostep(self):
        return BasicStepperDriver.__MAX_MICROSTEP
    
    def _alterMove(self, steps):
        current_state = self.getCurrentState()
        if current_state == State.ACCELERATING or current_state == State.CRUSING:
            if steps >= 0:
                self._steps_remaining += steps
            else:
                self._steps_remaining = stepperMax(self._steps_to_brake, self._steps_remaining + steps)
        elif current_state == State.STOPPED:
            self.startMove(steps)
        elif current_state == State.DECELERATING:
            pass
        
    def _calcStepPulse(self):
        if self._steps_remaining <= 0: #{  // this should not happen, but avoids strange calculations
            return
        
        self._steps_remaining-=1
        self._step_count+=1

        if self._profile.mode == Mode.LINEAR_SPEED:
            current_state = self.getCurrentState()
            if current_state == State.ACCELERATING:
                if self._step_count < self._steps_to_cruise:
                    self._step_pulse = self._step_pulse - (2*self._step_pulse+self.__rest)/(4*self._step_count+1)
                    self.__rest = (2*self._step_pulse+self.__rest) % (4*self._step_count+1)
                else:
                    self._step_pulse = self._cruise_step_pulse
                    self.__rest = 0
            elif current_state == State.DECELERATING:
                self._step_pulse = self._step_pulse - (2*self._step_pulse+self.__rest)/(-4*self._steps_remaining+1)
                self.__rest = (2*self._step_pulse+self.__rest) % (-4*self._steps_remaining+1)
            else:
                pass

    # @staticmethod
    # def delayMicros(delay_us, start_us=0):
    #     if delay_us:
    #         if not start_us:
    #             start_us = time.micros()
    #         if delay_us > MIN_YIELD_MICROS:
    #             # Here you can add your own yield function for the Raspberry Pi
    #             pass
    #         # Wait until the specified microsecond delay has passed
    #         while time.micros() - start_us < delay_us:
    #             pass  