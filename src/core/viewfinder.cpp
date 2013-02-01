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

#include "viewfinder.h"

using namespace std;
using namespace pcl;


void ViewFinder::setInputCloud(PointCloudConstPtr cloud) {
   inCloud = cloud; 
}


void ViewFinder::setCamera(CameraParameters* cam) {
    camera = cam;
}


void ViewFinder::setTransform(Eigen::Affine3f pose) {
    cloudTransform = pose;
}


void ViewFinder::compute() {
    float noise_level = 0.0;
    float min_range = 0.0f;

    float scale = 0.10;

    //TODO add cloud transform
    //Eigen::Affine3f pose = camera->getPose();
    Eigen::Affine3f pose = Eigen::Affine3f (Eigen::Translation3f (inCloud->sensor_origin_[0],
                                                               inCloud->sensor_origin_[1],
                                                               inCloud->sensor_origin_[2])) *
                       cloudTransform * camera->getStaticExtrinsic();


    rangeImage_ptr->createFromPointCloudWithFixedSize(*inCloud,
            camera->getResX()*scale, camera->getResY()*scale,
            camera->getcX()*scale, camera->getcY()*scale,
            camera->getfX()*scale, camera->getfY()*scale,
            pose, pcl::RangeImage::CAMERA_FRAME, noise_level, min_range);

    //rangeImage_ptr->setTransformationToRangeImageSystem(pose);
}


PointXYZ ViewFinder::getMiddlePoint() {
    //FIXME apply cloud transform
    PointWithRange pr = rangeImage_ptr->at(rangeImage_ptr->width/2, rangeImage_ptr->height/2);

    PointXYZ p;
    p.x = pr.x;
    p.y = pr.y;
    p.z = pr.z;
    return p;
}


ViewFinder::RangeImagePtr ViewFinder::getRangeImage() {
    return rangeImage_ptr;
}

