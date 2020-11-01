/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#include "Pin.h"
#include "Pwm.h"
#include "SoftPWM.h"

#include <math.h>
#include <string>

class Gcode;
class StreamOutput;

namespace mbed
{
    class PwmOut;
}

#define DIGITALJOGGERER_AXIS_PINCOUNT 3
#define DIGITALJOGGERER_FEED_PINCOUNT 3

class DigitalJogger : public Module
{
public:
    DigitalJogger();

    void on_module_loaded();
    void on_main_loop(void *argument);
    void on_config_reload(void *argument);
    void on_get_public_data(void *argument);
    void on_set_public_data(void *argument);

private:
    uint32_t pinpoll_tick(uint32_t dummy);
    uint32_t encoder_tick(uint32_t dummy);

    void on_encoder_a_rise();
    void on_encoder_a_fall();

    int16_t encoder_getValue(void);

    void send_gcode(const char *format, ...);

    mbed::InterruptIn *encoder_a_interrupt_pin;
    Pin *encoder_b_input_pin;
    
    Pin *axis_pins[DIGITALJOGGERER_AXIS_PINCOUNT];
    Pin *feed_pins[DIGITALJOGGERER_FEED_PINCOUNT];

    struct
    {
        int8_t axis_state : 4;
        bool axis_state_changed : 1;

        int8_t feed_state : 4;
        bool feed_state_changed : 1;
    };

    bool axis_switch_state[DIGITALJOGGERER_AXIS_PINCOUNT];
    bool feed_switch_state[DIGITALJOGGERER_FEED_PINCOUNT];

    bool    encoder_updated;
    
    int16_t encoder_position_last;
    int16_t encoder_position;
    // int16_t encoder_last;
    // int16_t encoder_delta;
    // int8_t encoder_steps;
    // bool encoder_changed;
    // bool encoder_accelerationEnabled;
    // uint16_t encoder_acceleration;
};
