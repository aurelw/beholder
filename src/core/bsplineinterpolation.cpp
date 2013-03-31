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

#include "bsplineinterpolation.h"


BSplineInterpolation::BSplineInterpolation(int ncoeffs, int k) :
    nCoeffs(ncoeffs), splineK(k)
{

    /* init bSpline */
    bSplineWorkspace = gsl_bspline_alloc(splineK, nCoeffs + 2 - splineK);
    bSpline = gsl_vector_alloc(nCoeffs);

    /* parameters and covariance */
    cParameters = gsl_vector_alloc(nCoeffs);
    covMatrix = gsl_matrix_alloc(nCoeffs, nCoeffs);
}


void BSplineInterpolation::fitFromData(const Samples &samples) {

    /* preprocess samples and extract some info */
    Samples ssamples = samples;
    std::sort(ssamples.begin(), ssamples.end());
    const int numSamples = ssamples.size();
    const float minSampleX = ssamples[0].first;
    const float maxSampleX = ssamples.back().first;

    /* prepare fitting data */
    gsl_vector *x = gsl_vector_alloc(ssamples.size());
    gsl_vector *y = gsl_vector_alloc(ssamples.size());

    for (int i=0; i<ssamples.size(); i++) {
        gsl_vector_set(x, i, ssamples[i].first);
        gsl_vector_set(y, i, ssamples[i].second);
    }

    /* uniform knots distributed in sample range */
    gsl_bspline_knots_uniform(minSampleX, maxSampleX, bSplineWorkspace);

    /* construct a fit matrix */
    gsl_matrix *fitMatrix = gsl_matrix_alloc(numSamples, nCoeffs);
    for (int i=0; i<numSamples; i++) {
        /* compute B_j(xi) for all j */
        double xi = gsl_vector_get(x, i);
        gsl_bspline_eval(xi, bSpline, bSplineWorkspace);

        /* fill in row i */
        for (int j=0; j<nCoeffs; j++) {
            double Bj = gsl_vector_get(bSpline, j);
            gsl_matrix_set(fitMatrix, i, j, Bj);
        }
    }

    /* fit spline to data */
    gsl_multifit_linear_workspace *mws = 
        gsl_multifit_linear_alloc(numSamples, nCoeffs);
    double chisq;
    size_t rank;
    double tol = 0.1;
    gsl_multifit_linear(fitMatrix, y, cParameters, covMatrix, &chisq, mws);
    //gsl_multifit_linear_svd(fitMatrix, y, tol,
    //        &rank, cParameters, covMatrix, &chisq, mws);

    splineMinX = minSampleX;
    splineMaxX = maxSampleX;

    /* clean up */
    gsl_vector_free(x);
    gsl_vector_free(y);
    gsl_matrix_free(fitMatrix);
    gsl_multifit_linear_free(mws);
    
}


float BSplineInterpolation::evaluate(float pos) {
    double y, yerr;
    float dy = 0;
    const float dstep = 0.01;

    /* if x is out of spline definition, etrapolate */
    if (pos > splineMaxX || pos < splineMinX) {

        float steps, endpoint, dy;
        /* get gradient at right end */
        if (pos > splineMaxX) {
            endpoint = evaluate(splineMaxX);
            dy = endpoint - evaluate(splineMaxX - dstep);
            steps = (pos - splineMaxX) / dstep;
            y = endpoint + (dy*steps);
        } else if (pos < splineMinX) {
            endpoint = evaluate(splineMinX);
            dy = evaluate(splineMinX + dstep) - endpoint;
            steps = (splineMinX - pos) / dstep;
            y = endpoint - (dy*steps);
        }

    } else {
        gsl_bspline_eval(pos, bSpline, bSplineWorkspace);
        gsl_multifit_linear_est(bSpline, cParameters, covMatrix, &y, &yerr);
    }

    return (float)y;
}


BSplineInterpolation::Samples BSplineInterpolation::evaluateRange(
        const float start, const float stop, const float stepsize)
{
    Samples samples;
    for (float x=start; x<stop; x+=stepsize) {
        float y = evaluate(x);
        samples.push_back(std::make_pair(x, y));
    }

    return samples;
}

