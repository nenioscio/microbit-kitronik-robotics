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
KitronikRobotics::KitronikRobotics(MicroBitI2C &_i2c, uint8_t _chip_address) : i2c(_i2c), chip_address(_chip_address), status(0), step_init(0), step_stage(0), stepper_1_steps(200), stepper_2_steps(200)
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
void KitronikRobotics::init()
{
    char buf[2];
    if (status & KITRONIKROBOTICS_INITIALIZED)
        return;

    buf[0] = prescale_reg;
    buf[1] = 0x85; //50Hz
    i2c.write(chip_address, buf, 2);

    for (uint8_t block_reg=0xFA; block_reg < 0xFE, block_reg++) {
        buf[0] = block_reg;
        buf[1] = 0x00;
        i2c.write(chip_address, buf, 2);
    }

    buf[0] = mode_1_reg;
    buf[1] = 0x01;
    i2c.write(chip_address, buf, 2);

    status |= KITRONIKROBOTICS_INITIALIZED;
}

void KitronikRobotics::motor_on(uint8_t motor, uint8_t direction, uint16_t speed){
    char buf[2];
    char value_buf[4];
    uint8_t offset_buf[4];
    uint8_t motor_reg;
    uint16_t output_value;
    if !(status & KITRONIKROBOTICS_INITIALIZED)
        init();

    moror_reg = mot_reg_base + (2 * (motor - 1) * reg_offset);
    output_value = speed * 40;
    memcpy(value_buf, &output_value, sizeof(uint16_t));
    value_buf[2] = value_buf[3] = 0x00;
    if (direction | KITRONIKROBOTICS_FORWARD) {
        offset_buf = {0, 1, 4, 5};
    } else {
        offset_buf = {4, 5, 0, 1};
    }
    for (uint8_t i = 0; i < 4; i++) {
        buf[0] = motor_reg + register_buf[i];
        buf[1] = value_buf[i];
        i2c.write(chip_address, buf, 2);
    }
}
