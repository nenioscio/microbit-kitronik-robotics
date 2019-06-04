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

/**
  * Constructor.
  *
  * Create a representation of a Kitronik Robotics board  device, which includes
  * member variables that represent various device drivers used to control
  * aspects of the micro:bit.
  */
KitronikRobotics::KitronikRobotics(MicroBitI2C &_i2c, uint8_t _chip_address) : i2c(_i2c), chip_address(_chip_address << 1), status(0), step_init(0), step_stage(0), stepper_1_steps(200), stepper_2_steps(200)
{
}

/**
  * Post constructor initialisation method.
  *
  * This call will initialised the scheduler, memory allocator and Bluetooth stack.
  *
  * This is required as the Bluetooth stack can't be brought up in a
  * static context i.e. in a constructor.
  *
  * @code
  * kitronik_board.init();
  * @endcode
  *
  * @note This method must be called before user code utilises any functionality
  *       contained by uBit.
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

void KitronikRobotics::motor_on(uint8_t motor, uint8_t direction, uint16_t speed){
    uint8_t value_buf[4];
    uint8_t forward_offset[4] = {0, 1, 4, 5};
    uint8_t reverse_offset[4] = {4, 5, 0, 1};
    uint8_t *offset_buf;
    uint8_t motor_reg;
    uint16_t output_value;
    if (!(status & KITRONIKROBOTICS_INITIALIZED))
        init();

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
        i2c.writeRegister(chip_address, motor_reg + offset_buf[i], value_buf[1]);
    }
}
