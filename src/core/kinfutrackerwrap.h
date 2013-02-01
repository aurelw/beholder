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

#include <stdio.h>
#include <iostream>


#include <pcl/common/common_headers.h>

#include <pcl/gpu/kinfu/kinfu.h>
#include <pcl/gpu/kinfu/raycaster.h>
#include <pcl/gpu/kinfu/marching_cubes.h>
#include <pcl/gpu/containers/initialization.h>

#include "openni_capture.h"

#ifndef __KINFU_TRACKER_WRAP_H__
#define __KINFU_TRACKER_WRAP_H__


class KinfuTrackerWrap {

    public:

        typedef pcl::PointXYZ PointT;
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef boost::shared_ptr<PointCloud> PointCloudPtr;
        typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

        KinfuTrackerWrap(float volumeSize=3.0f, bool setPosition=false, float xpos=0.0f, float ypos=0.0f, float zpos=0.0f) :
            lastFrameDevice(480, 640)
        {
            /* set the size of the volume */
            //Eigen::Vector3f volume_size = Eigen::Vector3f(3.0f, volumeSize, volumeSize); //meters
            Eigen::Vector3f volume_size = Eigen::Vector3f::Constant(3.0f);
            kinfu.volume().setSize (volume_size);

            std::cout << "KinfuTrackerWrap Position: " << setPosition << std::endl;

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
            kinfu.setIcpCorespFilteringParams (0.1f/*meters*/, sin (20.f * 3.14159254f / 180.f));
            kinfu.setCameraMovementThreshold(0.001f);
        }

        void setCaptureSource(pcl::gpu::CaptureOpenNI& source);

        void init();
        void spinOnce();
        void reset();

        PointCloudPtr getLastFrameCloud();
        Eigen::Affine3f getLastPose();

    private:

        pcl::gpu::KinfuTracker kinfu;

        pcl::gpu::KinfuTracker::DepthMap depthDevice;
        pcl::gpu::PtrStepSz<const unsigned short> depthGrab;
        pcl::gpu::PtrStepSz<const pcl::gpu::KinfuTracker::PixelRGB> rgbGrab;
        pcl::gpu::CaptureOpenNI* capture;

        pcl::gpu::DeviceArray2D<PointT> lastFrameDevice;


};

#endif
