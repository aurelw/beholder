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


#ifndef __BYTE_MOTOR_H__
#define __BYTE_MOTOR_H__

#include "motor.h"

class ByteMotor : public Motor {

    public:

        ByteMotor(const RigConfig &rigConfig, std::string id);

        ~ByteMotor();

        virtual bool connect();
        virtual void disconnect();
        virtual void setPosition(float p);

    private:

        void sendRawBytePos(unsigned char byte); 

        std::string devicePath;
        int fd;
        bool connected;
        float lLimit, hLimit;

};

#endif

