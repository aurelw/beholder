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

#ifndef __CALIB_VISUALIUER_H__
#define __CALIB_VISUALIUER_H__

#include <boost/thread.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

#include "basic_visualizer.h"


class CalibVisualizer : public BasicVisualizer {

    public:

        typedef typename pcl::PointCloud<pcl::PointXYZRGBA> RGBCloud;
        typedef typename pcl::PointCloud<pcl::PointXYZ> PlainCloud;
        typedef typename pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ColorHandler;

    public:

        /* set properties */
        void setMarkerCenter(pcl::PointXYZ center, bool found=true);
        void setCorrespondence(PlainCloud::ConstPtr cloud0, 
                               PlainCloud::ConstPtr cloud1);

        /* set draw options */
        void setDrawMarker(bool doDraw);
        void setDrawCorrespondence(bool doDraw, bool arrow=true);

        /* just another plain cloud */
        void setPlainCloud(PlainCloud::ConstPtr cloud);
        void setDrawPlainCloud(bool doDraw);

        /* to visualizer global registration */
        void setRegistration(PlainCloud::ConstPtr cloud0,
                PlainCloud::ConstPtr cloud1);
        void setDrawRegistration(bool doDraw);

    protected:

        void updateAllProperties() override;
        void initVisualizer() override;

        /* plane marker */
        pcl::PointXYZ markerCenter;
        bool flagUpdateMarker = false;
        void updateMarker();
        bool drawMarker = true;
        bool foundMarker = false;
        bool markerAdded = false;

        /* correspondence */
        PlainCloud::ConstPtr cpCloudOne;
        PlainCloud::ConstPtr cpCloudTwo;
        bool flagUpdateCorrespondence = false;
        void updateCorrespondence();
        bool drawCorrespondence = false;
        bool correspondenceAdded = false;
        int numCorrespondences;
        bool drawCorrespondenceArrows = true;

        /* plain cloud */
        PlainCloud::ConstPtr plainCloud;
        bool drawPlainCloud = false;
        bool flagUpdatePlainCloud = false;
        bool plainCloudAdded = false;
        void updatePlainCloud();

        /* registration */
        PlainCloud::ConstPtr regCloud0;
        PlainCloud::ConstPtr regCloud1;
        bool drawRegistration = false;
        bool flagUpdateRegistration = false;
        bool registrationCloudAdded = false;
        void updateRegistration();
        ColorHandler::Ptr targetColorHandler;
        ColorHandler::Ptr regColorHandler;

};

#endif

