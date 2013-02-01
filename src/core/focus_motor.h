/*
   * (C) 2013, Aurel Wildfellner
   *
   * This file is part of Beholder.
   *
   * Beholder is free software: you can redistribute it and/or modify
   * it under the terms of the GNU General Public License as published by
   * the Free Software Foundation, either version 3 of the License, or
   * (at your option) any later version.
   *
   * Beholder is distributed in the hope that it will be useful,
   * but WITHOUT ANY WARRANTY; without even the implied warranty of
   * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   * GNU General Public License for more details.
   *
   * You should have received a copy of the GNU General Public License
   * along with Beholder. If not, see <http://www.gnu.org/licenses/>. */

#include <string>

#include "transfer1d1d.h"

#ifndef __FOCUS_MOTOR_H__
#define __FOCUS_MOTOR_H__

class FocusMotor {

    public:

        FocusMotor(Transfer1d1d &t, std::string motorDevicePath) :
            transfer(t),
            connected(false),
            devicePath(motorDevicePath),
            pushedMarkPoint(false)
        {
            physical_upper_bound = 200;
        }

        void connect();
        void setDistance(float m);
        void sendRawBytePos(unsigned char byte);
        bool getButtonStateChange();

        bool pushedMarkPoint;

    private:

        Transfer1d1d &transfer;

        std::string devicePath;
        int fd;
        bool connected;

        unsigned char physical_upper_bound;

};

#endif

