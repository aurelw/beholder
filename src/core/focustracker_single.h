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

#include "focustracker.h"

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
            
#include "cameraparameters.h"
#include "viewfinder.h"
#include "ffocus_messure.h"


#ifndef __FOCUS_TRACKER_SINGLE_H__
#define __FOCUS_TRACKER_SINGLE_H__

/* Tracks a single picked point directly when visible.
 * Maintains constant focus when the point isn't visible.
 */
class FocusTrackerSingle : public FocusTracker {

    public:
        FocusTrackerSingle () {
        }

        virtual void init();
        virtual void reset();

        virtual float getDistance();
        virtual pcl::PointXYZ pick();
        virtual bool isVisible();

    protected:
        ViewFinder viewFinder;
        FFocusMessure messure;
        float currentFPlane;
        bool trackedPointVisible;

        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr;

        pcl::PointXYZ trackedPoint;

};

#endif

