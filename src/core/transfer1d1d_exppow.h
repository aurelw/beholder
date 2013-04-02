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

#ifndef __TRANSFER_1D1D_EXP_POW_H__
#define __TRANSFER_1D1D_EXP_POW_H__


class Transfer1d1dExpPow : public Transfer1d1d {

    public:

        Transfer1d1dExpPow(float a, float b, float c, float d);

        virtual float transfer(float x);

    protected:

        float a_, b_, c_, d_;

};


#endif

