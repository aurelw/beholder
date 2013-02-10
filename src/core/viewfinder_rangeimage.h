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

#ifndef __VIEW_FINDER_RANGEIMAGE_H__
#define __VIEW_FINDER_RANGEIMAGE_H__

#include <stdio.h>
#include <iostream>

#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>

#include "viewfinder.h"
#include "cameraparameters.h"


template <class PointType>
class ViewFinderRangeImage : public ViewFinder<PointType> {

    public:
        
        typedef typename pcl::PointCloud<PointType> Cloud;
        typedef typename boost::shared_ptr<Cloud> CloudPtr;
        typedef typename boost::shared_ptr<const Cloud> CloudConstPtr;

        typedef boost::shared_ptr<pcl::RangeImagePlanar> RangeImagePtr;

        ViewFinderRangeImage(float rangeImageScale=0.10) :
            rangeImage_ptr(new pcl::RangeImagePlanar),
            imageScale(rangeImageScale)
        {
        }

        /**** ViewFinder Implementation ****/
        void compute() {

            float noise_level = 0.0;
            float min_range = 0.0f;
            float scale = imageScale;

            CloudConstPtr inCloud = this->rangeFinder->getLastCloud();

            //TODO add cloud transform
            //Eigen::Affine3f pose = camera->getPose();
            Eigen::Affine3f pose = 
                Eigen::Affine3f (Eigen::Translation3f (
                            inCloud->sensor_origin_[0],
                            inCloud->sensor_origin_[1],
                            inCloud->sensor_origin_[2]))
                * this->rangeFinder->getCloudPose();


            rangeImage_ptr->createFromPointCloudWithFixedSize(*inCloud,
                    this->camera->getResX()*scale, this->camera->getResY()*scale,
                    this->camera->getcX()*scale, this->camera->getcY()*scale,
                    this->camera->getfX()*scale, this->camera->getfY()*scale,
                    pose, pcl::RangeImage::CAMERA_FRAME, noise_level, min_range);

            //rangeImage_ptr->setTransformationToRangeImageSystem(pose);
        }

        pcl::PointXYZ getMiddlePoint() {
            //FIXME apply cloud transform <- ???
            pcl::PointWithRange pr = 
                rangeImage_ptr->at(rangeImage_ptr->width/2, 
                                   rangeImage_ptr->height/2);

            pcl::PointXYZ p;
            p.x = pr.x;
            p.y = pr.y;
            p.z = pr.z;
            return p;
        }

        /**** RangeImage specific *****/
        RangeImagePtr getRangeImage() {
            return rangeImage_ptr;
        }

    private:
        RangeImagePtr rangeImage_ptr;
        float imageScale;

};

#endif
