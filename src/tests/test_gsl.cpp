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

#include <utility>
#include <iostream>
#include <sstream>
#include <string>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <gsl/gsl_bspline.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_statistics.h>

/* number of data points to fit */
#define N        200

/* number of fit coefficients */
#define NCOEFFS  12

/* nbreak = ncoeffs + 2 - k = ncoeffs - 2 since k = 4 */
#define NBREAK   (NCOEFFS - 2)

int main (void) {
    const size_t n = N;
    const size_t ncoeffs = NCOEFFS;
    const size_t nbreak = NBREAK;
    size_t i, j;
    gsl_bspline_workspace *bw;
    gsl_vector *B;
    double dy;
    gsl_rng *r;
    gsl_vector *c, *w;
    gsl_vector *x, *y;
    gsl_matrix *X, *cov;
    gsl_multifit_linear_workspace *mw;
    double chisq, Rsq, dof, tss;

    gsl_rng_env_setup();
    r = gsl_rng_alloc(gsl_rng_default);

    /* allocate a cubic bspline workspace (k = 4) */
    bw = gsl_bspline_alloc(4, nbreak);
    B = gsl_vector_alloc(ncoeffs);

    x = gsl_vector_alloc(n);
    y = gsl_vector_alloc(n);
    X = gsl_matrix_alloc(n, ncoeffs);
    c = gsl_vector_alloc(ncoeffs);
    w = gsl_vector_alloc(n);
    cov = gsl_matrix_alloc(ncoeffs, ncoeffs);
    mw = gsl_multifit_linear_alloc(n, ncoeffs);

    std::stringstream ox_ss, oy_ss;
    ox_ss << "ox = [";
    oy_ss << "oy = [";

    printf("#m=0,S=0\n");
    /* this is the data to be fitted */
    for (i = 0; i < n; ++i) {
        double sigma;
        double xi = (15.0 / (N - 1)) * i;
        double yi = cos(xi) * exp(-0.1 * xi);

        sigma = 0.1 * yi;
        dy = gsl_ran_gaussian(r, sigma);
        yi += dy;

        gsl_vector_set(x, i, xi);
        gsl_vector_set(y, i, yi);
        gsl_vector_set(w, i, 1.0 / (sigma * sigma));
        
        ox_ss << xi << ",";
        oy_ss << yi << ",";
    }
    ox_ss << "]" << std::endl;
    oy_ss << "]" << std::endl;

    /* use uniform breakpoints on [0, 15] */
    gsl_bspline_knots_uniform(0.0, 15.0, bw);

    /* construct the fit matrix X */
    for (i = 0; i < n; ++i)
        {
            double xi = gsl_vector_get(x, i);

            /* compute B_j(xi) for all j */
        gsl_bspline_eval(xi, B, bw);

        /* fill in row i of X */
        for (j = 0; j < ncoeffs; ++j)
        {
            double Bj = gsl_vector_get(B, j);
            gsl_matrix_set(X, i, j, Bj);
        }
    }

    /* do the fit */
    gsl_multifit_wlinear(X, w, y, c, cov, &chisq, mw);

    dof = n - ncoeffs;
    tss = gsl_stats_wtss(w->data, 1, y->data, 1, y->size);
    Rsq = 1.0 - chisq / tss;

    fprintf(stderr, "chisq/dof = %e, Rsq = %f\n", 
                     chisq / dof, Rsq);

    /* output the smoothed curve */
    std::stringstream x_ss, y_ss;
    x_ss << "x = [";
    y_ss << "y = [";
    {
      double xi, yi, yerr;

      printf("#m=1,S=0\n");
      for (xi = 0.0; xi < 15.0; xi += 0.1)
          {
          gsl_bspline_eval(xi, B, bw);
          gsl_multifit_linear_est(B, c, cov, &yi, &yerr);
          x_ss << xi << ",";
          y_ss << yi << ",";
      }
    }
    x_ss << "]" << std::endl;
    y_ss << "]" << std::endl;

    gsl_rng_free(r);
    gsl_bspline_free(bw);
    gsl_vector_free(B);
    gsl_vector_free(x);
    gsl_vector_free(y);
    gsl_matrix_free(X);
    gsl_vector_free(c);
    gsl_vector_free(w);
    gsl_matrix_free(cov);
    gsl_multifit_linear_free(mw);

    /* print results */
    std::cout << ox_ss.str();
    std::cout << oy_ss.str();
    std::cout << x_ss.str();
    std::cout << y_ss.str();

    return 0;
} /* main() */
