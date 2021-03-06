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

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <iostream>
#include <string>

#include <boost/smart_ptr.hpp>

#include "rig_config.h"


class Motor {

    public:

        typedef boost::shared_ptr<Motor> Ptr;

        virtual bool connect() = 0;
        virtual void disconnect() = 0;
        virtual void setPosition(float p) = 0;
        virtual void stepUp() = 0;
        virtual void stepDown() = 0;
        virtual bool isUpperLimit() = 0;
        virtual bool isLowerLimit() = 0;
        virtual float getPosition();

    protected:

        float position;

};

#endif

