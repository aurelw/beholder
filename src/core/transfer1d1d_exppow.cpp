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

#include "transfer1d1d_exppow.h"

#include <math.h>


Transfer1d1dExpPow::Transfer1d1dExpPow(
        float a, float b, float c, float d) :
    a_(a), b_(b), c_(c), d_(d)
{
}


float Transfer1d1dExpPow::transfer(float x) {
            if (x<=0.00) //lower bound
                return 0.;
            return a_ * exp(-b_ * pow(x, c_)) + d_;
}

