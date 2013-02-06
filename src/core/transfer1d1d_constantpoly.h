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

        Transfer1d1dConstantPoly(float a, float b, float c, float d,
                float e, float f, float g);

        virtual float transfer(float x);

    protected:

        float a_, b_, c_, d_, e_, f_, g_;

};


#endif

