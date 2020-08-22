/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "Module.h"

#include <stdint.h>

class FeedOverride : public Module {
    public:
        FeedOverride();
        virtual ~FeedOverride() {};
        void on_module_loaded();
        void on_config_reload(void* argument);
        uint32_t update_tick(uint32_t dummy);

    private:
        float get_override_factor(float pos);

        uint16_t data_source;
        float override_factor {1.0};
        float min{ 0.2 };
        float max{ 1.2 };

    void update_Joystick(void);
    void update_Override(void);        
};
