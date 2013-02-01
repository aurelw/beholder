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


void CameraParameters::updateDistMap() {
}



void CameraParameters::loadIntrinsicFromYAML(const string& filename) {
    FileStorage fs(filename, FileStorage::READ);
    //FIXME load resolution
    fs["image_width"] >> res_x;
    fs["image_height"] >> res_y;
    //res_x = 4272;
    //res_y = 2848;
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();

    updateDistMap();
}
    

void CameraParameters::loadExtrinsicFromXML(const string& filename) {
    FileStorage fs(filename, FileStorage::READ);
    fs["exRotation"] >> exRotationVec;
    fs["exTranslation"] >> exTranslation;

    // load the rotation matrix from the vector not directly
    //fs["exRotationMat"] >> exRotationMat;
    Rodrigues(exRotationVec, exRotationMat);

    fs.release();
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


Eigen::Affine3f CameraParameters::getStaticExtrinsic() {
    Eigen::Affine3f pose, camRot;

    //TODO check if M_PI is legit * M_PI (propably not) should be in r
    camRot = Eigen::AngleAxisf(exRotationVec.at<double>(0,0), Eigen::Vector3f::UnitX())
      * Eigen::AngleAxisf(exRotationVec.at<double>(1,0),  Eigen::Vector3f::UnitY())
      * Eigen::AngleAxisf(exRotationVec.at<double>(2,0), Eigen::Vector3f::UnitZ());

    pose = Eigen::Affine3f (Eigen::Translation3f (
                exTranslation.at<double>(0, 0),
                exTranslation.at<double>(1, 0),
                exTranslation.at<double>(2, 0))) *
            Eigen::Affine3f(camRot);
    
    return pose;
}


void CameraParameters::mapCoord(const float x, const float y, float& mx, float& my) {
    
}


void CameraParameters::print() {
    cout << "res-x:" << getResX() << endl;
    cout << "res-y:" << getResY() << endl;
    cout << "focal x:" << getfX() << endl;
    cout << "focal y:" << getfY() << endl;
    cout << "extrinsic, translation:" << exTranslation.at<double>(0,0) << "," 
        << exTranslation.at<double>(1,0) << "," 
        << exTranslation.at<double>(2,0) << endl;
}


cv::Mat CameraParameters::getCameraMatrix() {
    return cameraMatrix;
}


cv::Mat CameraParameters::getDistortionCoefficients() {
    return distCoeffs;
}

