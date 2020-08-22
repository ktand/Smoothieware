/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "FeedOverride.h"

#include <math.h>
#include "Kernel.h"
#include "Robot.h"
#include "Conveyor.h"
#include "Gcode.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "SlowTicker.h"

#include "JoystickPublicAccess.h"
#include "PublicData.h"
#include "utils.h"

#include "StreamOutputPool.h" //just for debugging

#define feedoverride_checksum CHECKSUM("feedoverride")
#define enable_checksum CHECKSUM("enable")
#define data_source_checksum CHECKSUM("data_source")
#define min_checksum CHECKSUM("min")
#define max_checksum CHECKSUM("max")

FeedOverride::FeedOverride()
{
    override_factor = 1.0F;
}

void FeedOverride::on_module_loaded()
{
    if (!THEKERNEL->config->value(feedoverride_checksum, enable_checksum)->by_default(false)->as_bool())
    {
        // as not needed free up resource
        delete this;
        return;
    }

    this->on_config_reload(this);

    THEKERNEL->slow_ticker->attach(10, this, &FeedOverride::update_tick);
}

void FeedOverride::on_config_reload(void *argument)
{
    this->data_source = get_checksum(THEKERNEL->config->value(feedoverride_checksum, data_source_checksum)->by_default("")->as_string());

    this->min = THEKERNEL->config->value(feedoverride_checksum, min_checksum)->by_default(this->min)->as_number();
    this->max = THEKERNEL->config->value(feedoverride_checksum, max_checksum)->by_default(this->max)->as_number();
}

float FeedOverride::get_override_factor(float pos)
{
    // scale the joystick position from [-1, 1] to [min, max]
    return (max - min) * (pos * 0.5 + 0.5) + min;
}

uint32_t FeedOverride::update_tick(uint32_t dummy)
{
    // update joystick reading
    update_Joystick();

    // update the override functionality
    update_Override();

    return 0;
}

// perform a public data request from the joystick, also determine from the result if the joystick is active
void FeedOverride::update_Joystick(void)
{
    struct PAD_joystick s;
    if (PublicData::get_value(joystick_checksum, this->data_source, &s))
    {
        this->override_factor = get_override_factor(s.position);
    }
    else
    {
        this->override_factor = 1.0F;
    }

    // map the joystick position to a speed
    this->override_factor = get_override_factor(s.position);

    // THEKERNEL->streams->printf(">>> %f, %d, %f, %f, %f, %f\r\n", s.position, s.raw, override_factor, min, max, THEROBOT->get_seconds_per_minute());
}

void FeedOverride::update_Override(void)
{
    THEROBOT->set_feed_override_factor(this->override_factor);
}