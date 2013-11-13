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

#include "console_utils.h"
#include "motor_factory.h"
#include "transfer1d1d_constantpoly.h"
#include "transfer1d1d_exppow.h"


FocusMotorTwoPolys::FocusMotorTwoPolys(const RigConfig &rigConfig) {
    motor = createMotor(rigConfig, "");
    motor->connect();
    lastDistance = 0.0;
    createTransferFunctions(rigConfig);
}


void FocusMotorTwoPolys::createTransferFunctions(const RigConfig &rigConfig) {
    if (rigConfig.fMotorTransferType == "poly") {
        hTransfer.reset (new Transfer1d1dConstantPoly(
                rigConfig.fMotorHa, rigConfig.fMotorHb, rigConfig.fMotorHc, 
                rigConfig.fMotorHd, rigConfig.fMotorHe, rigConfig.fMotorHf, 
                rigConfig.fMotorHg));
        lTransfer.reset (new Transfer1d1dConstantPoly(
                rigConfig.fMotorLa, rigConfig.fMotorLb, rigConfig.fMotorLc, 
                rigConfig.fMotorLd, rigConfig.fMotorLe, rigConfig.fMotorLf, 
                rigConfig.fMotorLg));
    } else if (rigConfig.fMotorTransferType == "exppow") {
        hTransfer.reset (new Transfer1d1dExpPow(
                    rigConfig.fMotorHa, rigConfig.fMotorHb, 
                    rigConfig.fMotorHc, rigConfig.fMotorHd));
        lTransfer.reset (new Transfer1d1dExpPow(
                    rigConfig.fMotorLa, rigConfig.fMotorLb, 
                    rigConfig.fMotorLc, rigConfig.fMotorLd));
    }
}


void FocusMotorTwoPolys::setDistance(float m) {
    float position;

    if (doUpTransfer) {
        position = hTransfer->transfer(m);
    } else {
        position = lTransfer->transfer(m);
    }

    if (position < currentLowest) {
        currentLowest = position;
    }

    if (position > currentHighest) {
        currentHighest = position;
    }

    /* down movement, change to lTransfer */
    if (doUpTransfer && (currentHighest - position > directionChangeTresh)) {
        doUpTransfer = false;
        currentLowest = position;
        currentHighest = position;
    /* up movement, change to hTransfer */
    } else if (!doUpTransfer && 
            (position - currentLowest > directionChangeTresh))
    {
        doUpTransfer = true;
        currentLowest = position;
        currentHighest = position;
    }

    motor->setPosition(position);
}

