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
    public UpdateSignal
{

    public:

        typedef typename pcl::PointCloud<pcl::PointXYZRGBA> RGBCloud;
        typedef typename pcl::PointCloud<pcl::PointXYZ> PlainCloud;
        typedef typename RGBCloud::Ptr RGBCloudPtr;
        typedef typename RGBCloud::ConstPtr RGBCloudConstPtr;

        typedef typename  boost::shared_ptr<OpenNiInterface> Ptr;

        OpenNiInterface(std::string device) :
            isStreaming(false),
            isConnected(false),
            deviceId(device)
        {
        }

        ~OpenNiInterface();

        bool init();
        void waitForFirstFrame();


    protected:

        void setupGrabber();
        virtual void cloud_callback(const RGBCloudConstPtr& cld);
        virtual void plain_cloud_callback(const PlainCloud::ConstPtr& cld);

        pcl::OpenNIGrabber *grabber;
        RGBCloudConstPtr cloud;

        bool isStreaming;
        bool isConnected;
        std::string deviceId;
};




class OpenNiInterfaceRGB : 
    public OpenNiInterface, 
    public CloudProvider<pcl::PointXYZRGBA>
{

public:

    typedef typename  boost::shared_ptr<OpenNiInterfaceRGB> Ptr;

public:

    OpenNiInterfaceRGB(std::string device) :
        OpenNiInterface(device)
    {
    }

    /* CloudProvider implementation */
    RGBCloudConstPtr getLastCloud();
    RGBCloudPtr getCloudCopy();
    Eigen::Affine3f getCloudPose();

};




class OpenNiInterfacePlain : 
    public OpenNiInterface, 
    public CloudProvider<pcl::PointXYZ>
{

public :

    typedef typename  boost::shared_ptr<OpenNiInterfacePlain> Ptr;

public:

    OpenNiInterfacePlain(std::string device) :
        OpenNiInterface(device)
    {
    }

    PlainCloud::ConstPtr plainCloud;
    virtual void cloud_callback(const RGBCloudConstPtr& cld) override;

    /* CloudProvider implementation */
    PlainCloud::ConstPtr getLastCloud();
    PlainCloud::Ptr getCloudCopy();
    Eigen::Affine3f getCloudPose();
        
};


#endif
