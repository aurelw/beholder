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

#include "transfer1d1d_constantpoly.h"


Transfer1d1dConstantPoly::Transfer1d1dConstantPoly(
        float a, float b, float c, float d,
        float e, float f, float g) :
    a_(a), b_(b), c_(c), d_(d), e_(e), f_(f), g_(g)
{
}


float Transfer1d1dConstantPoly::transfer(float x) {
            if (x<=0.00) //lower bound
                return 0.;
            //if (x>= 10.0) //upper bound
            //    return 0.785;
            //return  a_*(x*x*x) + b_*(x*x) + c_*(x) + d_ ;
            return  (a_) * (x*x*x*x*x*x) + 
                    (b_) * (x*x*x*x*x) + 
                    (c_) * (x*x*x*x) +
                    (d_) * (x*x*x) + 
                    (e_) * (x*x) +
                    (f_) * (x) + 
                    (g_);
}

