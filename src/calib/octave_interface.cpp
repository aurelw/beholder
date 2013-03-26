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

#include "octave_interface.h"

#include <octave/oct.h>
#include <octave/octave.h>
#include <octave/parse.h>


OctaveInterface::OctaveInterface() {
    /* init octave interpretor */
   string_vector argv (2);
   argv(0) = "embedded";
   argv(1) = "-q";
   octave_main (2, argv.c_str_vec(), 1);
}


std::vector<float> OctaveInterface::fitPolynome(
        const std::vector<std::pair<float, float>> &samples,
        int degree)
{
    std::vector<float> poly;

    /* create two octave vectors */
    int size = samples.size();
    Matrix m_x = Matrix(1, size);
    Matrix m_y = Matrix(1, size);
    for (int i=0; i<samples.size(); i++) {
        m_x(1, i) = samples[i].first;
        m_y(1, i) = samples[i].second;
    }

    /* build value list for interpretor */
    octave_value_list parameters;
    parameters.append(octave_value(m_x));
    parameters.append(octave_value(m_y));
    parameters.append(octave_value(degree));

    /* run octave interpretor */
    octave_value_list out_vl;
    out_vl = feval("polyfit", parameters);

    Matrix m_out = out_vl(0).matrix_value();
    for (int i=0; i<m_out.numel(); i++) {
        poly.push_back(m_out(1, i));
    }

    return poly;
}

