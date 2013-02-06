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

#include "focus_motor.h"

#include <iostream>

#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

void FocusMotor::setDistance(float m) {
    float pos = transfer.transfer(m);
    unsigned char byte_pos =  256 * pos;

    /* check physical bounds! */
    if (byte_pos > physical_upper_bound) {
        byte_pos = physical_upper_bound;
    }

    //std::cout << "byte_pos: " << (int)byte_pos << "pos: " << pos << " dist: " << m << std::endl;
    sendRawBytePos(byte_pos);
}


void FocusMotor::sendRawBytePos(unsigned char byte) {
    //FIXME !!!!! firmware bug, just go +50 for home
    // is scaling!
    //byte += 50;
    //std::cout << "rawbyte: " << (int)byte << std::endl;
    if (connected) {
        /* write position to controller */
        if (write(fd, &byte, 1) == -1) {
            std::cout << "[ERROR] Can't write to servo driver" << std::endl;
        }
    } 
}


bool FocusMotor::getButtonStateChange() {
    char buttonc = 'N';
    if (connected) {
        int nbytes = read(fd, &buttonc, sizeof(char));
        if (nbytes > 0) {
            switch (buttonc) {
                case '0':
                    pushedMarkPoint = false;
                    return true;
                    break;
                case '1':
                    pushedMarkPoint = true;
                    return true;
                    break;
                case '2':
                    pushedMarkPoint = false;
                    return false;
                    break;
                default:
                    pushedMarkPoint = false;
                    return false;
                    break;
            }
        }
    }
    return false;
}


void FocusMotor::connect() {
    fd = open(devicePath.c_str(), O_RDWR|O_NOCTTY|O_NDELAY);
    fcntl(fd, F_SETFL, FNDELAY);
    if (fd == -1) {
        std::cout << "[ERROR] Can't connect to servo driver." << std::endl;
        return;
    }

    int retval;
    struct termios my_termios;

    retval = tcgetattr( fd, &my_termios );
    if (retval == -1) {
        std::cout << "[ERROR] Can't connect to servo driver." << std::endl;
        return;
    } 

    // baudrate
    retval = cfsetospeed(&my_termios, B38400);
    if (retval == -1) {
        std::cout << "[ERROR] Can't connect to servo driver (baudrate)." << std::endl;
        return;
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
        std::cout << "[ERROR] Can't connect to servo driver (baudrate)." << std::endl;
        return;
    }
    
    connected = true;

}


