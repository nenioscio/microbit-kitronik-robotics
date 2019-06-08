/*
The MIT License (MIT)

Copyright (c) 2016 British Broadcasting Corporation.
This software is provided by Lancaster University by arrangement with the BBC.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

#ifndef KITRONIKROBOTICS_H
#define KITRONIKROBOTICS_H

#include "MicroBit.h"
#include "MicroBitFiber.h"

// MicroBit::flags values
#define KITRONIKROBOTICS_INITIALIZED                    0x01
#define KITRONIKROBOTICS_FORWARD                        0x02
#define KITRONIKROBOTICS_REVERSE                        0x04


/**
  * Class definition for a MicroBit device.
  *
  * Represents the device as a whole, and includes member variables that represent various device drivers
  * used to control aspects of the micro:bit.
  */
class  KitronikRobotics
{
    private:

    static const uint8_t        prescale_reg      = 0xFE;
    static const uint8_t        mode_1_reg        = 0x00;
    static const uint8_t        srv_reg_base      = 0x08;
    static const uint8_t        mot_reg_base      = 0x28;
    static const uint8_t        reg_offset        = 4;
    static const uint8_t        servo_multiplier  = 226;
    static const uint8_t        servo_zero_offset = 0x66;

    MicroBitI2C                 &i2c; 
    const uint8_t               chip_address; // will be defaulted to 0x6C
    uint8_t                     status;

    uint16_t                    stepper_steps[2];

    inline uint16_t angle_to_steps(uint16_t angle, uint16_t steps) {
        return ((angle - 1) * (steps - 1)) / (360-1) + 1;
    };

    inline uint16_t clamp_u16(uint16_t x, uint16_t upper, uint16_t lower) {
        return min(upper, max(x, lower));
    }

    public:

    /**
      * Constructor.
      *
      * Create a representation of a Kitronik Robotics board device
      *
      * @param i2c Refernce to uBit.i2c class
      * @param chip_address I2C address of the Kitronik Robotics board device
      *        defaults to 0x6C
      * @param stepper_1_steps Number os steps required to make a 360 turn on the stepper 1
      *        defaults to 200
      * @param stepper_2_steps Number os steps required to make a 360 turn on the stepper 2
      *        defaults to 200
      *
      *
      * @code
      * krobotics = new KitronikRobotics(uBit.i2c, 0x6c, 200, 200);
      * @endcode
      */
    KitronikRobotics(MicroBitI2C &_i2c, uint8_t _chip_address = 0x6C, uint16_t stepper_1_steps = 200, uint16_t stepper_2_steps = 200);

    /**
      * Post constructor initialisation method.
      *
      * @return MICROBIT_OK on success. Returns ID of the chip if no communication could be established
      *
      * @code
      * krobotics = new KitronikRobotics(uBit.i2c);
      * krobotics->init();
      * @endcode
      *
      * @note This method must be called before user code utilises any functionality
      *       contained by kRobotics device.
      */
    int init();

    /**
     * Sets the requested servo to the reguested angle.
     * If the PCA has not yet been initialised calls the initialisation routine.
     * @param servo Which servo to set
     * @param degrees the angle to set the servo to
     *
     * @code
     * krobotics->servo_write(1, 90);
     * @endcode
     */
    //% servo.min=1 servo.max=8
    //% degrees.min=0 degrees.max=180
    void servo_write(uint8_t servo, uint16_t degrees);

    /**
      * Turn Motor on and set to given rotational speed in given direction
      *
      * @param motor The id of the motor to start speeding up
      * @param direction The direction of the motor rotation (eighter of KROBOTIKROBOTICS_FORWARD or
      *        KROBOTIKROBOTICS_REVERSE)
      * @param speed The motor speed for the given direction in Percent
      *
      * @code
      * krobotics->motor_on(1, KROBOTIKROBOTICS_FORWARD, 10);
      * @endcode
      */
    //% motor.min=1 speed.max=4
    //% speed.min=0 speed.max=100
    void motor_on(uint8_t motor, uint8_t direction, uint16_t speed);

    /**
     * Turns off the specified motor.
     * @param motor which motor to turn off
     */
    //% motor.min=1 speed.max=4
    void motor_off(uint8_t motor);


    /**
     * Turns off all motors and servos.
     */
    void all_off();


    /**
     * Sets the requested stepper motor to a chosen angle relative to the start position.
     * if the PCA has not yet been initialised calls the initialisation routine.
     * @param stepper which stepper motor to turn on
     * @param direction   which direction to go
     * @param angle how far to turn the motor relative to start in degrees
     */
    //% stepper.min=1 stepper.max=2
    //% angle.min=1 angle.max=360
    void stepper_motor_turn_angle(uint8_t stepper, uint8_t direction, uint16_t angle);

    /**
     * Sets the requested stepper motor to turn a set number of steps.
     * if the PCA has not yet been initialised calls the initialisation routine.
     * @param stepper which stepper motor to turn on
     * @param direction   which direction to go
     * @param steps how many steps to turn the motor
     */
    //% stepper.min=1 stepper.max=2
    void stepper_motor_turn_steps(uint8_t stepper, uint8_t direction, uint16_t steps);
};


#endif
