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

#include "mathutils.h"


OpenNiInterface::~OpenNiInterface() {
    grabber->stop();
}


void OpenNiInterface::cloud_callback(const CloudConstPtr &cld) {
    // assignment to shared pointers is threadsafe
    cloud = cld;
    if (cloud != NULL) {
        isStreaming = true;
        update();
    }
}


void OpenNiInterface::plain_cloud_callback(const PlainCloud::ConstPtr &cld) {
    // assignment to shared pointers is threadsafe
    cloud = cloudXYZtoRGBA(cld);
    if (cloud != NULL) {
        isStreaming = true;
        update();
    }
}


OpenNiInterface::CloudConstPtr OpenNiInterface::getLastCloud() {
    return cloud;
}


OpenNiInterface::CloudPtr OpenNiInterface::getCloudCopy() {
    CloudPtr ncloud(new Cloud (*getLastCloud()));
    return ncloud;
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
        //FIXME deviceId
        grabber = new pcl::OpenNIGrabber();
    } catch (pcl::IOException &exc) {
        return;
    }
    
    /* callbacks for XYZRGBA or XYZ */
    bool captureRGB = true;
    if (captureRGB) {
        boost::function <void (const CloudConstPtr&)> cloud_cb = 
            boost::bind (&OpenNiInterface::cloud_callback, this, _1);
        grabber->registerCallback(cloud_cb);
    } else {
        boost::function <void (const PlainCloud::ConstPtr&)> plain_cloud_cb = 
            boost::bind (&OpenNiInterface::plain_cloud_callback, this, _1);
        grabber->registerCallback(plain_cloud_cb);
    }

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

