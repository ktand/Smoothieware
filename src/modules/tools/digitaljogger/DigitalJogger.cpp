/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/SerialMessage.h"
#include <math.h>
#include "DigitalJogger.h"
#include "libs/Pin.h"
#include "modules/robot/Conveyor.h"
#include "PublicDataRequest.h"
#include "SlowTicker.h"
#include "Config.h"
#include "Gcode.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "StreamOutput.h"
#include "StreamOutputPool.h"

#include "MRI_Hooks.h"

#include <algorithm>

// digitaljogger.enable true

// digitaljogger.toggle_axis_input_pin
// digitaljogger.toggle_distance_input_pin

// digitaljogger.encoder_a_input_pin
// digitaljogger.encoder_b_input_pin

// digitaljogger.axis_x_output_pin
// digitaljogger.axis_y_output_pin
// digitaljogger.axis_z_output_pin

// digitaljogger.distance_min_output_pin
// digitaljogger.distance_mid_output_pin
// digitaljogger.distance_max_output_pin

// digitaljogger.feedrade_percentage

// digitaljogger.distance_min
// digitaljogger.distance_mid
// digitaljogger.distance_max

#define digitaljogger_checksum CHECKSUM("digitaljogger")

#define enable_checksum CHECKSUM("enable")

#define encoder_a_input_pin_checksum CHECKSUM("encoder_a_input_pin")
#define encoder_b_input_pin_checksum CHECKSUM("encoder_b_input_pin")

#define axis_x_pin_checksum CHECKSUM("axis_x_pin")
#define axis_y_pin_checksum CHECKSUM("axis_y_pin")
#define axis_z_pin_checksum CHECKSUM("axis_z_pin")

#define feed_min_pin_checksum CHECKSUM("feed_min_pin")
#define feed_mid_pin_checksum CHECKSUM("feed_mid_pin")
#define feed_max_pin_checksum CHECKSUM("feed_max_pin")

#define feedrade_percentage_checksum CHECKSUM("feedrade_percentage")

#define axis_switch_state_x 0
#define axis_switch_state_y 1
#define axis_switch_state_z 2

#define feed_switch_state_min 0
#define feed_switch_state_mid 1
#define feed_switch_state_max 2

#define ENC_ACCEL_TOP 3072 // max. acceleration: *12 (val >> 8)
#define ENC_ACCEL_INC 25
#define ENC_ACCEL_DEC 2

DigitalJogger::DigitalJogger() {}

void DigitalJogger::on_module_loaded()
{
    uint16_t const axis_pins_checksum[DIGITALJOGGERER_AXIS_PINCOUNT] = {
        axis_x_pin_checksum, // X
        axis_y_pin_checksum, // Y
        axis_z_pin_checksum  // Z
    };

    uint16_t const feed_pins_checksum[DIGITALJOGGERER_FEED_PINCOUNT] = {
        feed_min_pin_checksum, // Min
        feed_mid_pin_checksum, // Mid
        feed_max_pin_checksum  // Max
    };

    // if the module is disabled -> do nothing
    if (!THEKERNEL->config->value(digitaljogger_checksum, enable_checksum)->by_default(false)->as_bool())
    {
        // as this module is not needed free up the resource
        delete this;
        return;
    }

    this->axis_state = -1;
    this->feed_state = -1;

    // Settings
    for (int8_t state = 0; state < DIGITALJOGGERER_AXIS_PINCOUNT; state++)
    {
        this->axis_pins[state] = new Pin();
        this->axis_pins[state]->from_string(THEKERNEL->config->value(digitaljogger_checksum, axis_pins_checksum[state])->by_default("nc")->as_string())->as_open_drain()->pull_up()->set_inverting(true);
        this->axis_switch_state[state] = true;

        if (!this->axis_pins[state]->connected())
        {
            delete this->axis_pins[state];
            this->axis_pins[state] = nullptr;
        }
    }

    for (int8_t state = 0; state < DIGITALJOGGERER_FEED_PINCOUNT; state++)
    {
        this->feed_pins[state] = new Pin();
        this->feed_pins[state]->from_string(THEKERNEL->config->value(digitaljogger_checksum, feed_pins_checksum[state])->by_default("nc")->as_string())->as_open_drain()->pull_up()->set_inverting(true);
        this->feed_switch_state[state] = true;

        if (!this->feed_pins[state]->connected())
        {
            delete this->feed_pins[state];
            this->feed_pins[state] = nullptr;
        }
    }

    Pin dummy_pin;

    dummy_pin.from_string(THEKERNEL->config->value(digitaljogger_checksum, encoder_a_input_pin_checksum)->by_default("nc")->as_string())->pull_up();
    this->encoder_a_interrupt_pin = dummy_pin.interrupt_pin();

    this->encoder_b_input_pin = new Pin();
    this->encoder_b_input_pin->from_string(THEKERNEL->config->value(digitaljogger_checksum, encoder_b_input_pin_checksum)->by_default("nc")->as_string())->pull_up()->as_input();

    if (!this->encoder_b_input_pin->connected())
    {
        delete this->encoder_b_input_pin;
        this->encoder_b_input_pin = nullptr;
    }

    if (this->encoder_a_interrupt_pin != nullptr && this->encoder_b_input_pin != nullptr)
    {
        this->encoder_a_interrupt_pin->fall(this, &DigitalJogger::on_encoder_a_fall);
        // this->encoder_a_interrupt_pin->rise(this, &DigitalJogger::on_encoder_a_fall);

        NVIC_SetPriority(EINT3_IRQn, 16); // set to low priority
    }

    this->on_config_reload(this);

    this->register_for_event(ON_MAIN_LOOP);

    THEKERNEL->slow_ticker->attach(20, this, &DigitalJogger::pinpoll_tick);
}

void DigitalJogger::on_encoder_a_fall()
{
    this->encoder_position += this->encoder_b_input_pin->get() ? 1 : -1;
    this->encoder_updated = true;

    // this->encoder_position += this->encoder_a_interrupt_pin->read()
    //                               ? encoder_b_input_pin->get() ? 1 : -1
    //                               : encoder_b_input_pin->get() ? -1 : 1;
}

void DigitalJogger::on_config_reload(void *argument)
{
}

void DigitalJogger::on_get_public_data(void *argument)
{
}

void DigitalJogger::on_set_public_data(void *argument)
{
}

void DigitalJogger::on_main_loop(void *argument)
{
    if (this->axis_state_changed)
    {
        THEKERNEL->streams->printf(">>> axis_state = %d\n", this->axis_state);

        this->axis_state_changed = false;
    }
    if (this->feed_state_changed)
    {
        THEKERNEL->streams->printf(">>> feed_state = %d\n", this->feed_state);

        this->feed_state_changed = false;
    }

    if (this->encoder_updated)
    {
        int16_t delta = this->encoder_position - this->encoder_position_last;

        char axis;

        switch (this->axis_state)
        {
        case axis_switch_state_x:
            axis = 'X';
            break;
        case axis_switch_state_y:
            axis = 'Y';
            break;
        case axis_switch_state_z:
            axis = 'Z';
            break;
        default:
            axis = 0;
        }

        if (axis != 0)
        {
            send_gcode("$J %c%f", axis, delta * 10.0f);
        }

        this->encoder_updated = false;
        this->encoder_position_last += delta;

        THEKERNEL->streams->printf(">>> encoder =%d, %d\n", this->encoder_position, delta);
    }
}

uint32_t DigitalJogger::pinpoll_tick(uint32_t dummy)
{
    for (int8_t state = 0; state < DIGITALJOGGERER_AXIS_PINCOUNT; state++)
    {
        if (this->axis_pins[state] != nullptr)
        {
            // Turn of the led to be able to read the switch status
            this->axis_pins[state]->set(false);

            if (this->axis_switch_state[state] != this->axis_pins[state]->get())
            {
                this->axis_switch_state[state] ^= 1;

                if (this->axis_switch_state[state])
                {
                    this->axis_state = this->axis_state != state ? state : -1;
                    this->axis_state_changed = true;
                }
            }
        }
    }

    // Turn on the led for the active state
    if (this->axis_state != -1)
    {
        this->axis_pins[this->axis_state]->set(true);
    }

    return 0;
}

void DigitalJogger::send_gcode(const char *format, ...)
{
    char line[32]; // max length for an gcode line

    // format string
    va_list args;
    va_start(args, format);
    vsnprintf(line, sizeof(line), format, args);
    va_end(args);

    struct SerialMessage message;
    message.message = line;
    message.stream = &(StreamOutput::NullStream);
    THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);

    THEKERNEL->streams->printf(line);
}
