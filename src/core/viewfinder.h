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
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>

#include "cameraparameters.h"

#ifndef __VIEW_FINDER_H__
#define __VIEW_FINDER_H__

class ViewFinder {

    public:
        
        ViewFinder() :
            rangeImage_ptr(new pcl::RangeImagePlanar)
        {
            cloudTransform.setIdentity();
        }

        typedef pcl::PointXYZ PointT;
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef boost::shared_ptr<PointCloud> PointCloudPtr;
        typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;
        typedef boost::shared_ptr<pcl::RangeImagePlanar> RangeImagePtr;


        void setInputCloud(PointCloudConstPtr cloud);
        void setCamera(CameraParameters* cam);
        
        /* set an additional transformation on the cloud,
         * which will be added to the calibrated pose of 
         * the camera. */
        void setTransform(Eigen::Affine3f pose);

        void compute();

        pcl::PointXYZ getMiddlePoint();

        RangeImagePtr getRangeImage();

    private:
        PointCloudConstPtr inCloud;
        CameraParameters *camera;
        Eigen::Affine3f cloudTransform;
        RangeImagePtr rangeImage_ptr;

};

#endif
