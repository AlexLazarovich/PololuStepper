# /*
#  * DRV8834 - LV Stepper Motor Driver Driver (A4988-compatible - mostly)
#  * Indexer mode only.
#  *
#  * Copyright (C)2015 Laurentiu Badea
#  *
#  * This file may be redistributed under the terms of the MIT license.
#  * A copy of this license has been included with this distribution in the file LICENSE.
#  */
#ifndef DRV8834_H
#define DRV8834_H
#include <Arduino.h>
#include "BasicStepperDriver.h"
from BasicStepperDriver import *

class DRV8834(BasicStepperDriver):
    __MAX_MICROSTEP = 32
    _step_high_min = 2
    _step_low_min = 2
    _wakeup_time = 1000

    def __init__(self, steps, dir_pin, step_pin, enable_pin=None, m0_pin=PIN_UNCONNECTED, m1_pin=PIN_UNCONNECTED):
        super(DRV8834, self).__init__(steps, dir_pin, step_pin, enable_pin)
        self._m0_pin = m0_pin
        self._m1_pin = m1_pin

    def setMicrostep(self, microsteps):
        super().setMicrostep(microsteps)

        if not IS_CONNECTED(self._m0_pin) or not IS_CONNECTED(self._m1_pin):
            return self._microsteps
       
        GPIO.setup(self._m1_pin, GPIO.OUT)
        GPIO.output(self._m1_pin, GPIO.LOW if self._microsteps < 8 else GPIO.HIGH)

        if self._microsteps in (1,8):
            GPIO.setup(self._m0_pin, GPIO.OUT)
            GPIO.output(self._m0_pin, GPIO.LOW)
        elif self._microsteps in (2,16):
            GPIO.setup(self._m0_pin, GPIO.OUT)
            GPIO.output(self._m0_pin, GPIO.HIGH)
        elif self._microsteps in (4,32):
            GPIO.setup(self._m0_pin, GPIO.IN)
        
        return self._microsteps
    
    def getMaxMicrostep(self):
        return self.__MAX_MICROSTEP
# protected:
#     short m0_pin = PIN_UNCONNECTED;
#     short m1_pin = PIN_UNCONNECTED;
#     // tWH(STEP) pulse duration, STEP high, min value (1.9us)
#     static const int step_high_min = 2;
#     // tWL(STEP) pulse duration, STEP low, min value (1.9us)
#     static const int step_low_min = 2;
#     // tWAKE wakeup time, nSLEEP inactive to STEP (1000us)
#     static const int wakeup_time = 1000;
#     // also 200ns between ENBL/DIR/Mx changes and STEP HIGH

#     // Get max microsteps supported by the device
#     short getMaxMicrostep() override;

# private:
#     // microstep range (1, 16, 32 etc)
#     static const short MAX_MICROSTEP = 32;

# public:
#     /*
#      * Basic connection: only DIR, STEP are connected.
#      * Microstepping controls should be hardwired.
#      */
#     DRV8834(short steps, short dir_pin, short step_pin);
#     DRV8834(short steps, short dir_pin, short step_pin, short enable_pin);
#     /*
#      * Fully wired. All the necessary control pins for DRV8834 are connected.
#      */
#     DRV8834(short steps, short dir_pin, short step_pin, short m0_pin, short m1_pin);
#     DRV8834(short steps, short dir_pin, short step_pin, short enable_pin, short m0_pin, short m1_pin);
#     short setMicrostep(short microsteps) override;
# };
# #endif // DRV8834_H