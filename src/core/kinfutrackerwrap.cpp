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

#include "kinfutrackerwrap.h"

using namespace std;
using namespace pcl;


void KinfuTrackerWrap::setCaptureSource(pcl::gpu::CaptureOpenNI& source) {
    capture = &source;
}


void KinfuTrackerWrap::init() {
    /* set the focal length of depth */
    float f = capture->depth_focal_length_VGA;
    kinfu.setDepthIntrinsics(f,f);
}


void KinfuTrackerWrap::spinOnce() {
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


KinfuTrackerWrap::PointCloudPtr KinfuTrackerWrap::getLastFrameCloud() {
    PointCloudPtr cloud_ptr = PointCloudPtr (new PointCloud);
    
    kinfu.getLastFrameCloud (lastFrameDevice);

    int c;
    lastFrameDevice.download(cloud_ptr->points, c);
    cloud_ptr->width = lastFrameDevice.cols();
    cloud_ptr->height = lastFrameDevice.rows();
    cloud_ptr->is_dense = false;

    return cloud_ptr;
}


Eigen::Affine3f KinfuTrackerWrap::getLastPose() {
    return kinfu.getCameraPose();
}


void KinfuTrackerWrap::reset() {
    //kinfu.reset();
}

