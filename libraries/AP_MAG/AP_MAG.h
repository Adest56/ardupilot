/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <cmath>

class AP_MAG {
public:
    AP_MAG();

    /* Do not allow copies */
    AP_MAG(const AP_MAG &other) = delete;
    AP_MAG &operator=(const AP_MAG&) = delete;

    // init - perform required initialisation
    void init(const AP_SerialManager &serial_manager);

    bool update(void);

    int32_t liTemp[3];
	float lfValueX, lfValueY, lfValueZ, lfValueF;
    uint32_t last_frame_ms;

    uint8_t errorlog;
    uint16_t errorlog2;
    uint8_t errorlog3;
    


private:

    AP_HAL::UARTDriver *_port;                  // UART used to send data to FrSky receiver
    uint8_t _step;
    uint8_t lucDC[10];
};
