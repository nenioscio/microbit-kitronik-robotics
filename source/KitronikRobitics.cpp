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

#include "KitronikRobotics.h"

template < typename T >
T clamp(T value, T min, T max) {
        return std::max(std::min(value, max), min);
}

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
KitronikRobotics::KitronikRobotics(MicroBitI2C &_i2c, uint8_t _chip_address, uint16_t stepper_1_steps, uint16_t stepper_2_steps) : i2c(_i2c), chip_address(_chip_address << 1), status(0)
{
  stepper_steps[0] = stepper_1_steps;
  stepper_steps[1] = stepper_2_steps;
}

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
int KitronikRobotics::init()
{
    int result;
    if (status & KITRONIKROBOTICS_INITIALIZED)
        return MICROBIT_OK;

    result = i2c.writeRegister(chip_address, prescale_reg, 0x85); // 50Hz
    if (result != MICROBIT_OK)
        return result;

    for (uint8_t block_reg=0xFA; block_reg < 0xFE; block_reg++) {
        result = i2c.writeRegister(chip_address, block_reg, 0x00);
        if (result != MICROBIT_OK)
            return result;
    }

    result = i2c.writeRegister(chip_address, mode_1_reg, 0x01);
    if (result != MICROBIT_OK)
        return result;

    status |= KITRONIKROBOTICS_INITIALIZED;
    return MICROBIT_OK;
}

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
void KitronikRobotics::servo_write(uint8_t servo, uint16_t degrees) {
    uint8_t value_buf[2];
    uint16_t output_value;
    uint8_t servo_reg;

    if (!(status & KITRONIKROBOTICS_INITIALIZED))
        init();

    servo   = clamp(servo, (uint8_t) 1, (uint8_t) 8);
    degrees = clamp(degrees, (uint16_t) 0, (uint16_t) 180);

    servo_reg    = srv_reg_base + ((servo - 1) * reg_offset);
    output_value = (degrees * 100 * servo_multiplier) / 10000 + servo_zero_offset;
    value_buf[0] = output_value & 0xFF;
    value_buf[1] = (output_value >> 8);

    for (uint8_t i = 0; i < 2; i++) {
        i2c.writeRegister(chip_address, servo_reg + i, value_buf[i]);
    }
}

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
void KitronikRobotics::motor_on(uint8_t motor, uint8_t direction, uint16_t speed){
    uint8_t value_buf[4];
    uint8_t forward_offset[4] = {0, 1, 4, 5};
    uint8_t reverse_offset[4] = {4, 5, 0, 1};
    uint8_t *offset_buf;
    uint8_t motor_reg;
    uint16_t output_value;

    if (!(status & KITRONIKROBOTICS_INITIALIZED))
        init();

    motor = clamp(motor, (uint8_t) 1, (uint8_t) 4);
    speed = clamp(speed, (uint16_t) 0, (uint16_t) 100);

    motor_reg = mot_reg_base + (2 * (motor - 1) * reg_offset);
    output_value = speed * 40;
    value_buf[0] = output_value & 0xFF;
    value_buf[1] = (output_value >> 8);
    value_buf[2] = 0;
    value_buf[3] = 0;
    if (direction & KITRONIKROBOTICS_FORWARD) {
        offset_buf = forward_offset;
    } else {
        offset_buf = reverse_offset;
    }
    for (uint8_t i = 0; i < 4; i++) {
        i2c.writeRegister(chip_address, motor_reg + offset_buf[i], value_buf[i]);
    }
}

/**
 * Turns off the specified motor.
 * @param motor which motor to turn off
 */
//% motor.min=1 speed.max=4
void KitronikRobotics::motor_off(uint8_t motor){
    motor_on(motor, KITRONIKROBOTICS_FORWARD, 0);
}

/**
 * Turns off all motors and servos.
 */
void KitronikRobotics::all_off(){
    for (uint8_t motor_i = 1; motor_i < 5; motor_i++) {
        motor_off(motor_i);
    }

    for (uint8_t servo_i = 1; servo_i < 5; servo_i++) {
        servo_write(servo_i, 0);
    }
}

/**
 * Sets the requested stepper motor to a chosen angle relative to the start position.
 * if the PCA has not yet been initialised calls the initialisation routine.
 * @param stepper which stepper motor to turn on
 * @param direction   which direction to go
 * @param angle how far to turn the motor relative to start in degrees
 */
//% stepper.min=1 stepper.max=2
//% angle.min=1 angle.max=360
void KitronikRobotics::stepper_motor_turn_angle(uint8_t stepper, uint8_t direction, uint16_t angle) {
    uint8_t stepper_i;
    stepper_i  = clamp(stepper, (uint8_t) 1, (uint8_t) 2) - 1;
    uint16_t steps;

    steps = angle_to_steps(angle, stepper_steps[stepper_i]);
    stepper_motor_turn_steps(stepper, direction, steps);
}

/**
 * Sets the requested stepper motor to turn a set number of steps.
 * if the PCA has not yet been initialised calls the initialisation routine.
 * @param stepper which stepper motor to turn on
 * @param direction   which direction to go
 * @param steps how many steps to turn the motor
 */
    //% stepper.min=1 stepper.max=2
void KitronikRobotics::stepper_motor_turn_steps(uint8_t stepper, uint8_t direction, uint16_t steps) {
    uint8_t motors[2][2] = {{1,2},{3,4}};
    uint8_t motor;
    uint8_t stepper_i;
    uint8_t current_direction;
    uint16_t step_stage = 0;

    stepper_i  = clamp(stepper, (uint8_t) 1, (uint8_t) 2) - 1;

    if (!(status & KITRONIKROBOTICS_INITIALIZED))
        init();

    for (uint16_t step_counter = 0; step_counter < steps; step_counter ++) {
        /* step_stage % 2 speedup by using binary & 0x01 */
        if ((step_stage & 0x01) == 0) {
            motor = motors[stepper_i][0];
        } else {
            motor = motors[stepper_i][1];
        }
        if (step_stage == 0 || step_stage == 3) {
            current_direction = KITRONIKROBOTICS_FORWARD;
        } else {
            current_direction = KITRONIKROBOTICS_REVERSE;
        }
        motor_on(motor, current_direction, 100);
        fiber_sleep(20);

        if (direction == KITRONIKROBOTICS_FORWARD) {
            step_stage++;
        } else {
            step_stage--;
        }
        step_stage %= 4;
    }
}
