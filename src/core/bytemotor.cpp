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
#include "console_utils.h"


ByteMotor::ByteMotor(const RigConfig &rigConfig, std::string id) : 
    connected(false)
{
    devicePath = rigConfig.fMotorDevice;

    /* convert limits to byte positions */
    float ll = rigConfig.fMotorLimitL;
    float hl = rigConfig.fMotorLimitH;
    setBounds(ll, 0.0f, 1.0f);
    setBounds(hl, 0.0f, 1.0f);
    int ll_i = ll * 256;
    int hl_i = hl * 256;
    setBounds(ll_i, 0, 255);
    setBounds(hl_i, 0, 255);
    lLimit = ll_i;
    hLimit = hl_i;
}


ByteMotor::~ByteMotor() {
    disconnect();
}


bool ByteMotor::connect() {
    fd = open(devicePath.c_str(), O_RDWR|O_NOCTTY|O_NDELAY);
    fcntl(fd, F_SETFL, FNDELAY);
    if (fd == -1) {
        printError("[ByteMotor]", "Can't connect to servo driver.\n");
        return false;
    }

    int retval;
    struct termios my_termios;

    retval = tcgetattr( fd, &my_termios );
    if (retval == -1) {
        printError("[ByteMotor]", "Can't connect to servo driver.\n");
        return false;
    } 

    // baudrate
    retval = cfsetospeed(&my_termios, B38400);
    if (retval == -1) {
        printError("[ByteMotor]", "Can't connect to servo driver (baudrate).\n");
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
        printError("[ByteMotor]", "Can't connect to servo driver (termios).\n");
        return false;
    }
    
    printBrightInfo("[ByteMotor]", " connected.\n");
    connected = true;
    return true;
}


void ByteMotor::disconnect() {
    printBrightInfo("[ByteMotor]", " disconnected.\n");
    close(fd);
    connected = false;
}


bool ByteMotor::checkLimits(unsigned char p) {
    return (p >= lLimit && p <= hLimit);
}


void ByteMotor::setPosition(float p) {
    DEBUG_PRINT(p);
    if (p < 0 || p > 1.0) {
        return;
    }
    int new_bytePos =  p*256;
    setBounds(new_bytePos, 0, 255);
    setBytePosition(new_bytePos);
}


void ByteMotor::setBytePosition(unsigned char p) {
    if (!checkLimits(p)) {
        return;
    }

    bytePos = p;
    position = bytePos / 255.0f;
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


bool ByteMotor::isUpperLimit() {
    /* prevent overflow */
    if (bytePos == 255) {
        return true;
    }
    return (bytePos + 1 > hLimit);
}


bool ByteMotor::isLowerLimit() {
    /* prevent overflow */
    if (bytePos == 0) {
        return true;
    }
    return (bytePos - 1 < lLimit);
}


void ByteMotor::stepUp() {
    // prevent overflow
    if (bytePos < 254) {
        setBytePosition(bytePos + 1);
    }
}


void ByteMotor::stepDown() {
    // prevent overflow
    if (bytePos > 1) {
        setBytePosition(bytePos - 1);
    }
}

