from .BasicStepperDriver import BasicStepperDriver, IS_CONNECTED, PIN_UNCONNECTED
import RPi.GPIO as GPIO

class DRV8834(BasicStepperDriver):
    # Microstep table
    __MAX_MICROSTEP = 32
    _step_high_min = 2
    _step_low_min = 2
    _wakeup_time = 1000

    def __init__(self, steps, dir_pin, step_pin, enable_pin=PIN_UNCONNECTED, m0_pin=PIN_UNCONNECTED, m1_pin=PIN_UNCONNECTED):
        ''' Constructor. Calls BasicStepperDriver constructor'''
        super(DRV8834, self).__init__(steps, dir_pin, step_pin, enable_pin)
        self._m0_pin = m0_pin
        self._m1_pin = m1_pin

    def setMicrostep(self, microsteps):
        ''' Set microstep resolution'''
        super().setMicrostep(microsteps)

        if not IS_CONNECTED(self._m0_pin) or not IS_CONNECTED(self._m1_pin):
            return self._microsteps
       
        GPIO.setup(self._m1_pin, GPIO.OUT)
        GPIO.output(self._m1_pin, GPIO.LOW if self._microsteps < 8 else GPIO.HIGH)

        if self._microsteps in (1,8): # Full, 1/8 step
            GPIO.setup(self._m0_pin, GPIO.OUT)
            GPIO.output(self._m0_pin, GPIO.LOW)
        elif self._microsteps in (2,16): # Half, 1/16 step
            GPIO.setup(self._m0_pin, GPIO.OUT)
            GPIO.output(self._m0_pin, GPIO.HIGH)
        elif self._microsteps in (4,32): # 1/4, 1/32 step
            GPIO.setup(self._m0_pin, GPIO.IN)
        
        return self._microsteps
    
    def getMaxMicrostep(self):
        ''' Return maximum microstep value'''
        return self.__MAX_MICROSTEP