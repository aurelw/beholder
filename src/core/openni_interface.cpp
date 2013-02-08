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

#include "openni_interface.h"

#include <boost/foreach.hpp>


void OpenNiInterface::cloud_callback(const CloudConstPtr &cld) {
    //FIXME mutex on cloud
    cloud = cld;
    if (cloud != NULL) {
        isStreaming = true;
        update();
    }
}


OpenNiInterface::CloudConstPtr OpenNiInterface::getLastCloud() {
    //FIXME mutex on cloud
    return cloud;
}


Eigen::Affine3f OpenNiInterface::getCloudPose() {
    Eigen::Affine3f mid;
    mid.setIdentity();
    return mid;
}


void OpenNiInterface::setupGrabber() {

    // modes can be specified
    pcl::OpenNIGrabber::Mode depth_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
    pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;

    try {
        grabber = new pcl::OpenNIGrabber();
    } catch (pcl::IOException &exc) {
        return;
    }
    
    // setup callback
    boost::function <void (const CloudConstPtr&)> cloud_cb = 
        boost::bind (&OpenNiInterface::cloud_callback, this, _1);
    grabber->registerCallback(cloud_cb);

    // start 
    grabber->start();
    isConnected = true;
}


bool OpenNiInterface::init() {
    setupGrabber();
    return isConnected;
}


void OpenNiInterface::waitForFirstFrame() {
    while (!isStreaming) {
        usleep(10);
    }
}

