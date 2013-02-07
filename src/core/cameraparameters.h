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

#ifndef __CAMERA_PARAMETERS_H__
#define __CAMERA_PARAMETERS_H__

#include <stdio.h>
#include <iostream>

#include <pcl/common/common_headers.h>
#include <cv.h>

#include "rig_config.h"


class CameraParameters {

    public:

        CameraParameters(const RigConfig &rigConfig);

        void loadIntrinsicFromYAML(const std::string& filename);
        void loadExtrinsicFromXML(const std::string& filename);

        float getResX();
        float getResY();
        float getfX();
        float getfY();
        float getcX();
        float getcY();
        cv::Mat getCameraMatrix();
        cv::Mat getDistortionCoefficients();

        /* The calibrated pose relative to the pointcloud sensor. */
        Eigen::Affine3f getStaticExtrinsic();

        void mapCoord(const float x, const float y, float& mx, float& my);

        void print();

    private:

        cv::Mat cameraMatrix, distCoeffs;
        float res_x, res_y;

        void updateDistMap();
};

#endif
