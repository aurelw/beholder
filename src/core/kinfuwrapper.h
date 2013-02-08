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

#ifndef __KINFU_WRAPPER_H__
#define __KINFU_WRAPPER_H__

#include <stdio.h>
#include <iostream>

#include <pcl/common/common_headers.h>
#include <pcl/gpu/kinfu/kinfu.h>
#include <pcl/gpu/kinfu/raycaster.h>
#include <pcl/gpu/kinfu/marching_cubes.h>
#include <pcl/gpu/containers/initialization.h>

#include "openni_capture.h"
#include "cloudprovider.h"
#include "update_signal.h"


class KinfuWrapper : public UpdateSignal, public CloudProvider<pcl::PointXYZ> {

    public:

        typedef pcl::PointXYZ PointT;
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef boost::shared_ptr<PointCloud> PointCloudPtr;
        typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;
        typedef boost::shared_ptr<KinfuWrapper> Ptr;

        KinfuWrapper(float volumeSize=3.0f, 
                bool setPosition=false, float xpos=0.0f, 
                float ypos=0.0f, float zpos=0.0f);

        void setCaptureSource(pcl::gpu::CaptureOpenNI& source);

        void init();
        void spinOnce();
        void reset();

        Eigen::Affine3f getLastPose();

        /* CloudProvider implementation */
        PointCloudConstPtr getLastCloud();
        Eigen::Affine3f getCloudPose();

    private:

        pcl::gpu::KinfuTracker kinfu;

        pcl::gpu::KinfuTracker::DepthMap depthDevice;
        pcl::gpu::PtrStepSz<const unsigned short> depthGrab;
        pcl::gpu::PtrStepSz<const pcl::gpu::KinfuTracker::PixelRGB> rgbGrab;
        pcl::gpu::CaptureOpenNI* capture;

        pcl::gpu::DeviceArray2D<PointT> lastFrameDevice;

        Eigen::Affine3f cloudPose;

};

#endif
