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

#ifndef __FOCUS_MOTOR_TWO_POLYS_H__
#define __FOCUS_MOTOR_TWO_POLYS_H__

#include "focusmotor.h"
#include "motor.h"
#include "rig_config.h"
#include "transfer1d1d.h"

class FocusMotorTwoPolys : public FocusMotor {

    public:

        FocusMotorTwoPolys(const RigConfig &rigConfig);
        ~FocusMotorTwoPolys();

        virtual void setDistance(float m);

    protected:

        virtual void createTransferFunctions(const RigConfig &rigConfig);

        Motor* motor;
        Transfer1d1d* hTransfer;
        Transfer1d1d* lTransfer;

        float lastDistance;
};

#endif

