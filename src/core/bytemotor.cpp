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

#include "bytemotor.h"

#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "mathutils.h"


ByteMotor::ByteMotor(const RigConfig &rigConfig, std::string id) : 
    connected(false)
{
    devicePath = rigConfig.fMotorDevice;
    lLimit = rigConfig.fMotorLimitL;
    hLimit = rigConfig.fMotorLimitH;

    setBounds(lLimit, 0.0f, 1.0f);
    setBounds(hLimit, 0.0f, 1.0f);
}


ByteMotor::~ByteMotor() {
    disconnect();
}


bool ByteMotor::connect() {
    fd = open(devicePath.c_str(), O_RDWR|O_NOCTTY|O_NDELAY);
    fcntl(fd, F_SETFL, FNDELAY);
    if (fd == -1) {
        std::cerr << "[ERROR] Can't connect to servo driver." << std::endl;
        return false;
    }

    int retval;
    struct termios my_termios;

    retval = tcgetattr( fd, &my_termios );
    if (retval == -1) {
        std::cerr << "[ERROR] Can't connect to servo driver." << std::endl;
        return false;
    } 

    // baudrate
    retval = cfsetospeed(&my_termios, B38400);
    if (retval == -1) {
        std::cerr << "[ERROR] Can't connect to servo driver (baudrate)." 
            << std::endl;
        return false;
    }

    cfmakeraw(&my_termios);

    my_termios.c_iflag &= ~(IXON | IXOFF | IXANY); 
    my_termios.c_iflag |= IGNBRK; 
    my_termios.c_cflag &= ~(PARENB|CSTOPB|CSIZE|CREAD|CRTSCTS);
    my_termios.c_cflag |= CS8;
    my_termios.c_cflag |= HUPCL;
    my_termios.c_lflag &= ~(ICANON|ECHO|ECHOE|ISIG);
    my_termios.c_cc[VMIN]=1; 
    my_termios.c_cc[VTIME] = 5;

    my_termios.c_cflag |= CRTSCTS;
    retval = tcsetattr( fd, TCSANOW, &my_termios );

    my_termios.c_cflag &= ~(PARENB|CSTOPB|CSIZE|CREAD|CRTSCTS);

    retval = tcsetattr( fd, TCSANOW, &my_termios );
    if (retval == -1) {
        std::cerr << "[ERROR] Can't connect to servo driver (termios)." 
            << std::endl;
        return false;
    }
    
    connected = true;
    return true;
}


void ByteMotor::disconnect() {
    close(fd);
    connected = false;
}


bool ByteMotor::checkLimits(const float p) {
    return (p > lLimit && p < hLimit);
}


void ByteMotor::setPosition(float p) {
    if (!checkLimits(p)) {
        return;
    }

    position = p;
    int new_bytePos =  position*256;
    setBounds(new_bytePos, 0, 255);
    bytePos = new_bytePos;
    sendRawBytePos(bytePos);
}


void ByteMotor::sendRawBytePos(unsigned char byte) {
    if (connected) {
        /* write position to controller */
        if (write(fd, &byte, 1) == -1) {
            std::cerr << "[ERROR] Can't write to servo driver" << std::endl;
        }
    } 
}


void ByteMotor::stepUp() {
    // prevent overflow
    if (bytePos < 254) {
        unsigned char new_byte_pos = bytePos + 1;
        float new_position = new_byte_pos / 255.0f;
        setPosition(new_position);
    }
}


void ByteMotor::stepDown() {
    // prevent overflow
    if (bytePos > 1) {
        unsigned char new_byte_pos = bytePos - 1;
        float new_position = new_byte_pos / 255.0f;
        setPosition(new_position);
    }
}
