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

#ifndef __RANGE_IMAGE_WRITER_H__
#define __RANGE_IMAGE_WRITER_H__

#include "pcl/range_image/range_image.h"
#include "pcl/range_image/range_image_planar.h"

#include <Magick++.h>

using namespace pcl;

class RangeImageWriter {

    public:

        typedef boost::shared_ptr<pcl::RangeImage> RangeImagePtr;

        RangeImageWriter(pcl::RangeImage::Ptr ri);

        void save(const std::string& fileName);

    private:

        RangeImagePtr rangeImage;

        float getMaxRange();

};

#endif
