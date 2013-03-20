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

#include "testmotor.h"

#include "console_utils.h"

TestMotor::TestMotor(const RigConfig &rigConfig, std::string id) {
    lLimit = rigConfig.fMotorLimitL;
    hLimit = rigConfig.fMotorLimitH;
}


TestMotor::~TestMotor() {
    disconnect();
}


bool TestMotor::connect() {
    printBrightInfo("[TestMotor]", " connected\n");
    connected = true;
    return true;
}


void TestMotor::disconnect() {
    printBrightInfo("[TestMotor]", " disconnected\n");
    connected = false;
}


void TestMotor::setPosition(float p) {

    /* nothing to change */
    if (position == p) {
        return;
    }

    if (p < lLimit || p > hLimit) {
        printWarning("[TestMotor]", " out of limits.\n");
        return;
    }

    printSimpleInfo("[TestMotor]", " set position to ");
    std::cout << p << std::endl;
    position = p;
}


void TestMotor::stepUp() {
    setPosition(position+0.01);
}


void TestMotor::stepDown() {
    setPosition(position-0.01);
}


bool TestMotor::isUpperLimit() {
    return (position+0.01 > hLimit);
}


bool TestMotor::isLowerLimit() {
    return (position-0.01 < lLimit);
}

