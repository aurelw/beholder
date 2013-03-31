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

#if 0
#include <stdafx.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <ratinterpolation.h>
#include <ap.h>
#include <ialglib.h>
using namespace ap;
#endif


int main(int argc, char **argv)
{
#if 0
    //
    // In this example we demonstrate penalized spline fitting of noisy data
    //
    // We have:
    // * x - abscissas
    // * y - vector of experimental data, straight line with small noise
    //
    real_1d_array x = "[0.00,0.10,0.20,0.30,0.40,0.50,0.60,0.70,0.80,0.90]";
    real_1d_array y = "[0.10,0.00,0.30,0.40,0.30,0.40,0.62,0.68,0.75,0.95]";
    ae_int_t info;
    double v;
    spline1dinterpolant s;
    spline1dfitreport rep;
    double rho;

    //
    // Fit with VERY small amount of smoothing (rho = -5.0)
    // and large number of basis functions (M=50).
    //
    // With such small regularization penalized spline almost fully reproduces function values
    //
    rho = -5.0;
    spline1dfitpenalized(x, y, 50, rho, info, s, rep);
    printf("%d\n", int(info)); // EXPECTED: 1
    v = spline1dcalc(s, 0.0);
    printf("%.1f\n", double(v)); // EXPECTED: 0.10

    //
    // Fit with VERY large amount of smoothing (rho = 10.0)
    // and large number of basis functions (M=50).
    //
    // With such regularization our spline should become close to the straight line fit.
    // We will compare its value in x=1.0 with results obtained from such fit.
    //
    rho = +10.0;
    spline1dfitpenalized(x, y, 50, rho, info, s, rep);
    printf("%d\n", int(info)); // EXPECTED: 1
    v = spline1dcalc(s, 1.0);
    printf("%.2f\n", double(v)); // EXPECTED: 0.969

    //
    // In real life applications you may need some moderate degree of fitting,
    // so we try to fit once more with rho=3.0.
    //
    rho = +3.0;
    spline1dfitpenalized(x, y, 50, rho, info, s, rep);
    printf("%d\n", int(info)); // EXPECTED: 1
#endif
    return 0;
}

