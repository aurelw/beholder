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

#include "octave_interface.h"

int main() {

    /* get octave interface singleton */
    OctaveInterface &octif = OctaveInterface::getInstance();

    /* prepare samples */
    std::vector<std::pair<float, float>> samples;
    for (int i=1; i<11; i++) {
        float x = i;
        float y = i*i;
        samples.push_back( std::make_pair(x, y) );
    }

    /* compute polynome */
    std::vector<float> poly = octif.fitPolynome(samples, 4);

    /* print polynome */
    std::cout << "Poly: ";
    for (float &coef : poly ) {
        std::cout << coef << " ";
    }
    std::cout << std::endl;

}

