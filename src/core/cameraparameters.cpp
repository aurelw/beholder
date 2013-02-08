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

#include "cameraparameters.h"

using namespace std;
using namespace cv;
using namespace pcl;


CameraParameters::CameraParameters(const RigConfig &rigConfig) {
    /* load from rig config */
    res_x = rigConfig.cameraImageWidth;
    res_y = rigConfig.cameraImageHeight;
    cameraMatrix = rigConfig.cameraMatrix;
    distCoeffs = rigConfig.cameraDistortionCoefficients;

    /* update map for distortion mapping */
    updateDistMap();
}


void CameraParameters::updateDistMap() {
}


float CameraParameters::getResX() {
    return res_x;
}

float CameraParameters::getResY() {
    return res_y;
}

float CameraParameters::getfX() {
    return cameraMatrix.at<double>(0, 0);
}

float CameraParameters::getfY() {
    return cameraMatrix.at<double>(1, 1);
}

float CameraParameters::getcX() {
    return cameraMatrix.at<double>(0, 2);
}

float CameraParameters::getcY() {
    return cameraMatrix.at<double>(1, 2);
}


void CameraParameters::mapCoord(const float x, const float y, float& mx, float& my) {
    
}


void CameraParameters::print() {
    cout << "res-x:" << getResX() << endl;
    cout << "res-y:" << getResY() << endl;
    cout << "focal x:" << getfX() << endl;
    cout << "focal y:" << getfY() << endl;
}


cv::Mat CameraParameters::getCameraMatrix() {
    return cameraMatrix;
}


cv::Mat CameraParameters::getDistortionCoefficients() {
    return distCoeffs;
}

