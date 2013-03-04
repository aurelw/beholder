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


class CalibVisualizer {

    public:

        typedef typename pcl::PointCloud<pcl::PointXYZRGBA> RGBCloud;
        typedef typename pcl::PointCloud<pcl::PointXYZ> PlainCloud;

    public:

        CalibVisualizer();
        ~CalibVisualizer();

        /* dispatch */
        //FIXME visualizer is not local thread data
        //so rendering fails
        void start();
        void stop();

        void spinOnce();

        /* set properties */
        void setMainCloud(RGBCloud::Ptr cloud);
        void setMarkerCenter(pcl::PointXYZ center, bool found=true);
        void setCorrespondence(PlainCloud::ConstPtr cloud0, 
                               PlainCloud::ConstPtr cloud1);

        void setDrawMarker(bool doDraw);
        void setDrawCorrespondence(bool doDraw);

    protected:

        /* initial setup for a visualizer */
        void initVisualizer();

        void updateAllProperties();

        /* threading */
        boost::thread *thread;
        bool threadRunning = false;
        bool stopThread = false;
        void runVisualizer();
        boost::shared_mutex mutex;

        /* pcl visualizer */
        pcl::visualization::PCLVisualizer::Ptr visualizer;
        void registerCallbacks();

        /* main cloud */
        RGBCloud::Ptr mainCloud;
        bool flagUpdateMainCloud = false;
        void updateMainCloud();
        bool drawMainCloud = true;
        bool mainCloudAdded = false;

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

};

#endif
