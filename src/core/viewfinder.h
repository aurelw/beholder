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

#ifndef __VIEW_FINDER_H__
#define __VIEW_FINDER_H__

#include <stdio.h>
#include <iostream>

#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>

#include "cameraparameters.h"
#include "rangefinder.h"


template <class PointType>
class ViewFinder {

    public:
        
        typedef typename pcl::PointCloud<PointType> Cloud;
        typedef typename boost::shared_ptr<Cloud> CloudPtr;
        typedef typename boost::shared_ptr<const Cloud> CloudConstPtr;
        typedef typename RangeFinder<PointType>::Ptr RangeFinderPtr;

        void setCameraParameters(CameraParameters::Ptr cam) {
            camera = cam;
        }

        void setRangeFinder(RangeFinderPtr rf) {
            rangeFinder = rf;
        }
        
        virtual void compute() = 0;
        virtual pcl::PointXYZ getMiddlePoint() = 0;
        virtual bool getMiddleRange(float &distance) = 0;

    public:

        RangeFinderPtr rangeFinder;
        CameraParameters::Ptr camera;

};

#endif
