/*

   Inspired by work done here https://github.com/PX4/Firmware/tree/master/src/drivers/frsky_telemetry from Stefan Rado <px4@sradonia.net>

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

/* 
   FRSKY Telemetry library
*/
#define AP_SERIALMANAGER_MAG_BAUD 38400
#define AP_SERIALMANAGER_MAG_BUFSIZE_TX 64
#define AP_SERIALMANAGER_MAG_BUFSIZE_RX 64

#include "AP_MAG.h"

extern const AP_HAL::HAL& hal;



//constructor
AP_MAG::AP_MAG(void) 
{
    _port = NULL;
    _step = 0;
    for (int16_t i = 0; i< 10; i++)
    {
        lucDC[i] = 0;
    }
    lfValueX= 0;
    lfValueY= 0;
    lfValueZ= 0;
    lfValueF= 0;
    for (int16_t i = 0; i< 3; i++)
    {
        liTemp[i] = 0;
    }
    
}

/*
 * init - perform required initialisation
 */
void AP_MAG::init(const AP_SerialManager &serial_manager)
{
    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_MAG, 0))) {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
         _port->begin(AP_SERIALMANAGER_MAG_BAUD, AP_SERIALMANAGER_MAG_BUFSIZE_RX, AP_SERIALMANAGER_MAG_BUFSIZE_TX);
    } 
}

bool AP_MAG::update()
{
    
    if (_port == NULL) 
        return false;

    

    int16_t numc = _port->available();
    uint8_t data;
    //uint8_t checksum;

    for (int16_t i = 0; i< numc; i++)
    {
        data = _port->read();
        switch (_step)
        {
        case 0:
            if(data == 0xFF)
                _step = 1;
            break;
        case 1:
            if(data == 0x7E)
                _step = 2;
            else
                _step = 0;
            break;
        case 2:
            if(data == 0x01)
                _step = 3;
            else
                _step = 0;
            break;
        case 3:
            if(data == 0x09)
                _step = 4;
            else
                _step = 0;
            break;
        case 4:
            _step = 0;
            if (i < numc - 10)
            {
                for (int16_t j = 0; j< 10; j++)
                {
                    i += 1;
                    data = _port->read();
                    // no checksum
                    lucDC[j] = data;
                }
                ////////////////////////////
                liTemp[0] = lucDC[0] + (lucDC[1] << 8) + (lucDC[2] << 16);
                liTemp[1] = lucDC[3] + (lucDC[4] << 8) + (lucDC[5] << 16);
                liTemp[2] = lucDC[6] + (lucDC[7] << 8) + (lucDC[8] << 16);
                if(liTemp[0] & 0x800000)
                {
                    liTemp[0] |= 0xff000000;
                }
                if(liTemp[1] & 0x800000)
                {
                    liTemp[1] |= 0xff000000;
                }
                if(liTemp[2] & 0x800000)
                {
                    liTemp[2] |= 0xff000000;
                }
                lfValueX = liTemp[0] * 0.011920929;
                lfValueY = liTemp[1] * 0.011920929;
                lfValueZ = liTemp[2] * 0.011920929;
                lfValueF = sqrt((lfValueX*lfValueX) + (lfValueY*lfValueY) + (lfValueZ*lfValueZ));
                ////////////////////////////
                last_frame_ms = AP_HAL::millis();
                return true;
            }
            break;
        default:
            _step = 0;
            break;
        }
    }
    return false;
}


