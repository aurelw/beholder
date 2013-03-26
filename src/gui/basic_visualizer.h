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

#ifndef __BASIC_VISUALIUER_H__
#define __BASIC_VISUALIUER_H__

#include <boost/thread.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>


class BasicVisualizer {

    public:

        typedef typename pcl::PointCloud<pcl::PointXYZRGBA> RGBCloud;
        typedef typename pcl::PointCloud<pcl::PointXYZ> PlainCloud;

    public:

        BasicVisualizer();
        ~BasicVisualizer();

        /* set properties */
        void setMainCloud(RGBCloud::ConstPtr cloud);

        /* dispatch */
        void start(bool waitInit=true);
        void stop();
        void spinOnce();

        /* general coordinate systems */
        void addCoordinateSystem(const Eigen::Affine3f &pose, 
                const std::string &id="default", float scale=1.0,
                bool drawMarker=false, 
                float mRed=1.0, float mGreen=0.0, float mBlue=0.0);
        void removeCoordinateSystem(const std::string &id="default");

    protected:

        /* initial setup for a visualizer */
        virtual void initVisualizer();

        /* overwrite this to add update methods */
        virtual void updateAllProperties();

        /* threading */
        boost::thread *thread;
        bool threadRunning = false;
        bool stopThread = false;
        bool isInitialized = false;
        void runVisualizer();
        boost::shared_mutex mutex;

        /* pcl visualizer */
        pcl::visualization::PCLVisualizer::Ptr visualizer;
        void registerCallbacks();

        /* main cloud */
        RGBCloud::ConstPtr mainCloud;
        bool flagUpdateMainCloud = false;
        void updateMainCloud();
        bool drawMainCloud = true;
        bool mainCloudAdded = false;

};

#endif
