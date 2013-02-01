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

#include "transfer1d1d.h"

#ifndef __TRANSFER_1D1D_CONSTANTPOLY_H___
#define __TRANSFER_1D1D_CONSTANTPOLY_H___


class Transfer1d1dConstantPoly : public Transfer1d1d {

    public:

        Transfer1d1dConstantPoly(float a, float b, float c, float d) :
            a_(a),
            b_(b),
            c_(c),
            d_(d)
        {
        }

        Transfer1d1dConstantPoly() 
        {
            a_ = 0.02;
            b_ = 0.01;
            c_ = 0.1;
            d_ = 0.05;
        }


        virtual float transfer(float x) {

            if (x<=0.01) //lower bound
                return 0.;
            if (x>= 10.0) //upper bound
                return 0.785;
            //return  a_*(x*x*x) + b_*(x*x) + c_*(x) + d_ ;
            return  (-1.6894e-05) *(x*x*x*x*x*x) + 
                    (6.5192e-04) * (x*x*x*x*x) + 
                    (-9.9874e-03) * (x*x*x*x) +
                    (7.7845e-02) * (x*x*x) + 
                    (-3.2874e-01) * (x*x) +
                    (7.4318e-01) * (x) + 
                    (-4.2223e-02);

        }


    protected:

        float bounds_n(float x) {
            std::cout << x << std::endl;
            if (x < 0) {
                return 0;
            }
            if (x > 1.0) {
                return 1.0;
            }
            return x;
        }

        float a_, b_, c_, d_;

};


#endif

