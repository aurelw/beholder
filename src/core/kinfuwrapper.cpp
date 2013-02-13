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

#include "kinfuwrapper.h"

using namespace std;
using namespace pcl;

KinfuWrapper::KinfuWrapper(float volumeSize, 
        bool setPosition, float xpos, 
        float ypos, float zpos) :
            lastFrameDevice(480, 640)
{
    /* set the size of the volume */
    //Eigen::Vector3f volume_size = Eigen::Vector3f(3.0f, volumeSize, volumeSize); //meters
    Eigen::Vector3f volume_size = Eigen::Vector3f::Constant(3.0f);
    kinfu.volume().setSize (volume_size);

    std::cout << "KinfuWrapper Position: " << setPosition << std::endl;

    //FIXME a good starting pose
    /* set the initial pose, in the middle of the volume */
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity ();   // * AngleAxisf(-30.f/180*3.1415926, Vector3f::UnitX());
    Eigen::Vector3f t;
    if (setPosition) {
        std::cout << "set Position" << std::endl;
        t << xpos, ypos, zpos;
    } else {
        t = volume_size * 0.5f - Eigen::Vector3f (0, 0, volume_size (2) / 2 * 1.2f);
    }
    //Eigen::Vector3f t(0,0,0);
    Eigen::Affine3f pose = Eigen::Translation3f (t) * Eigen::AngleAxisf (R);
    kinfu.setInitalCameraPose (pose);

    /* icp parameters */
    kinfu.volume().setTsdfTruncDist (0.030f/*meters*/);
    kinfu.setIcpCorespFilteringParams (0.1f/*meters*/, 
            sin (20.f * 3.14159254f / 180.f));
    kinfu.setCameraMovementThreshold(0.001f);
}
    

void KinfuWrapper::setCaptureSource(pcl::gpu::CaptureOpenNI& source) {
    capture = &source;
}


void KinfuWrapper::init() {
    /* set the focal length of depth */
    float f = capture->depth_focal_length_VGA;
    kinfu.setDepthIntrinsics(f,f);
}


void KinfuWrapper::spinOnce() {
    if (!capture->grab (depthGrab, rgbGrab)) {
        std::cout << "Can't catpure frame." << std::endl;
        return;
    }

    //std::cout << "Upload raw cloud to device: " << depthGrab.rows << "," << depthGrab.cols << std::endl;

    /* upload to device memory */
    depthDevice.upload (depthGrab.data, depthGrab.step, depthGrab.rows, depthGrab.cols);

    /* process frame */
    bool hasImage = kinfu(depthDevice);

    //std::cout << "has image: " << hasImage << std::endl;
}


KinfuWrapper::PointCloudConstPtr KinfuWrapper::getLastCloud() {
    //FIXME cache that cloud!
    return getCloudCopy();
}


KinfuWrapper::PointCloudPtr KinfuWrapper::getCloudCopy() {
    PointCloudPtr cloud_ptr = PointCloudPtr (new PointCloud);
    
    //also cache extraction
    kinfu.getLastFrameCloud (lastFrameDevice);

    int c;
    lastFrameDevice.download(cloud_ptr->points, c);
    cloud_ptr->width = lastFrameDevice.cols();
    cloud_ptr->height = lastFrameDevice.rows();
    cloud_ptr->is_dense = false;

    cloudPose = getLastPose();
    return cloud_ptr;
}


Eigen::Affine3f KinfuWrapper::getCloudPose() {
    return cloudPose;
}


Eigen::Affine3f KinfuWrapper::getLastPose() {
    return kinfu.getCameraPose();
}


void KinfuWrapper::reset() {
    //kinfu.reset();
}

