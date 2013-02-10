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

#include "rangeimagewriter.h"

RangeImageWriter::RangeImageWriter(::RangeImage::Ptr ri) :
    rangeImage(ri)
{
}


void RangeImageWriter::save(const std::string& fileName) {
    Magick::Image image(Magick::Geometry(rangeImage->width, rangeImage->height), 
            Magick::Color("orange"));

    float maxRange = getMaxRange();
    std::cout << "maximum range in the image: " << maxRange << std::endl;

    for(int y=0; y<rangeImage->height; y++) {
        for(int x=0; x<rangeImage->width; x++) {
            image.pixelColor(x, y, 
                    Magick::ColorGray(rangeImage->getPoint(x,y).range/maxRange));
            //FIXME better transfer function
            float nRange = rangeImage->getPoint(x,y).range/maxRange;
            image.pixelColor(x, y, Magick::ColorRGB(nRange, 0.0f, 1.0f - nRange));
        }
    }
    
    image.magick("png");
    image.write(fileName);
}


float RangeImageWriter::getMaxRange() {
    float limit = 15.0f; //must be wrong/undefined
    float maxRange = 0.001f;
    for(int y=0; y<rangeImage->height; y++) {
        for(int x=0; x<rangeImage->width; x++) {
            float cr = rangeImage->getPoint(x,y).range;
            if (cr < limit && cr > maxRange) {
                maxRange = cr;
            }
        }
    }
    return maxRange;
}
