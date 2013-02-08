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

#ifndef __OPENNI_INTERFACE_H__
#define __OPENNI_INTERFACE_H__

#include <stdio.h>
#include <iostream>
#include <vector>

#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/openni_camera/openni_driver.h>

#include "update_signal.h"
#include "cloudprovider.h"


class OpenNiInterface : 
    public UpdateSignal, 
    public CloudProvider<pcl::PointXYZRGB> 
{

    public:

        typedef pcl::PointXYZRGB PointType;
        typedef pcl::PointCloud<PointType> Cloud;
        typedef Cloud::ConstPtr CloudConstPtr;

        OpenNiInterface() :
            isStreaming(false),
            isConnected(false)
        {
        }

        bool init();
        void waitForFirstFrame();

        /* CloudProvider implementation */
        CloudConstPtr getLastCloud();
        Eigen::Affine3f getCloudPose();

    private:

        void setupGrabber();
        void cloud_callback(const CloudConstPtr& cld);

        pcl::OpenNIGrabber *grabber;
        CloudConstPtr cloud;

        bool isStreaming;
        bool isConnected;
};

#endif
