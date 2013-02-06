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

#include "focusmotor_twopolys.h"

#include "motor_factory.h"
#include "transfer1d1d_constantpoly.h"


FocusMotorTwoPolys::FocusMotorTwoPolys(const RigConfig &rigConfig) {
    motor.reset( createMotor(rigConfig, "") );
    motor->connect();
    lastDistance = 0.0;
    createTransferFunctions(rigConfig);
}


void FocusMotorTwoPolys::createTransferFunctions(const RigConfig &rigConfig) {
    hTransfer.reset (new Transfer1d1dConstantPoly(
            rigConfig.fMotorHa, rigConfig.fMotorHb, rigConfig.fMotorHc, 
            rigConfig.fMotorHd, rigConfig.fMotorHe, rigConfig.fMotorHf, 
            rigConfig.fMotorHg));
    lTransfer.reset (new Transfer1d1dConstantPoly(
            rigConfig.fMotorLa, rigConfig.fMotorLb, rigConfig.fMotorLc, 
            rigConfig.fMotorLd, rigConfig.fMotorLe, rigConfig.fMotorLf, 
            rigConfig.fMotorLg));
}


void FocusMotorTwoPolys::setDistance(float m) {
    float position;
    if (m > lastDistance) {
        position = hTransfer->transfer(m);
    } else {
        position = lTransfer->transfer(m);
    }
    lastDistance = m;

    motor->setPosition(position);
}


