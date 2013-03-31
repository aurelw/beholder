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

#ifndef __BSPLINE_INTERPOLATION_H__
#define __BSPLINE_INTERPOLATION_H__

#include <vector>
#include <utility>
#include <algorithm>

#include <gsl/gsl_bspline.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_statistics.h>


class BSplineInterpolation {

    public:

        typedef std::vector<std::pair<float, float>> Samples;

    public:

        BSplineInterpolation(int ncoeffs, int k=4);

        void fitFromData(const Samples &samples);
        Samples evaluateRange(const float start, const float stop, 
                const float stepsize);
        float evaluate(float pos);

    private:

        int nCoeffs;
        const int splineK = 4; //cubic bspline

        /* spline data */
        gsl_bspline_workspace *bSplineWorkspace = NULL;
        gsl_vector *bSpline = NULL;
        /* bspline parameterization */
        gsl_vector *cParameters = NULL;
        gsl_matrix *covMatrix = NULL;
        float splineMinX, splineMaxX;


};

#endif

