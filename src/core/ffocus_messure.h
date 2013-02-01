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

#include <pcl/common/common_headers.h>

#include <cv.h>

#include "cameraparameters.h"


#ifndef __FFOCUS_MESSURE_H__
#define __FFOCUS_MESSURE_H__

class FFocusMessure {

    public:

        FFocusMessure()
        {
            exRotationMat = cv::Mat_<double>(3,3);
        }

        void setPose(Eigen::Affine3f& pose);
        void setCamera(CameraParameters *par);

        float getFocalPlaneDistance(Eigen::Vector3f p);
        Eigen::Vector3f getFocalPoint(Eigen::Vector3f p);
        bool isVisible(Eigen::Vector3f p);

        float distancePP(Eigen::Vector3f p0, Eigen::Vector3f p1);

    private:

        void extractTranslationRotation();

        Eigen::Affine3f camPose;
        CameraParameters *camPara;
        cv::Vec3f exTranslation, exRotation;
        cv::Mat exRotationMat;

};

#endif
