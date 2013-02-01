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

#include <math.h>

#include "ffocus_messure.h"

using namespace pcl;
using namespace std;

void FFocusMessure::setPose(Eigen::Affine3f& pose) {
    camPose = pose;
    extractTranslationRotation();
}


void FFocusMessure::setCamera(CameraParameters *par) {
    camPara = par;
}


float FFocusMessure::distancePP(Eigen::Vector3f p0, Eigen::Vector3f p1) {
    return (p0 - p1).norm();
}


Eigen::Vector3f FFocusMessure::getFocalPoint(Eigen::Vector3f p) {
    Eigen::Vector3f camP = camPose * Eigen::Vector3f(0,0,0);
    Eigen::Vector3f viewP = camPose * Eigen::Vector3f(0,0,1);
    Eigen::Vector3f viewVec = viewP - camP;
    Eigen::ParametrizedLine<float, 3> viewLine(camP, viewVec);
    return viewLine.projection(p);
}


float FFocusMessure::getFocalPlaneDistance(Eigen::Vector3f p) {
    Eigen::Vector3f camP = camPose * Eigen::Vector3f(0,0,0);
    Eigen::Vector3f focalP = getFocalPoint(p);
    return (focalP-camP).norm();
}


bool FFocusMessure::isVisible(Eigen::Vector3f p) {

    //FIXME FIX THIS MESS! (type, opencv, outrage, fuck!)
    /* manual projection because opencv does shit! http://www.youtube.com/watch?v=OyxNgnQ9m30 */

    cv::Mat rotatMat = (cv::Mat_<double>(3,3) << exRotationMat.at<float>(0,0),exRotationMat.at<float>(0,1),exRotationMat.at<float>(0,2),
                                                 exRotationMat.at<float>(1,0),exRotationMat.at<float>(1,1),exRotationMat.at<float>(1,2),
                                                 exRotationMat.at<float>(2,0),exRotationMat.at<float>(2,1),exRotationMat.at<float>(2,2));

    cv::Mat calib=(cv::Mat_<double>(3,3)<< camPara->getfX(), 0.0, camPara->getcX(),
                                       0.0, camPara->getfY(), camPara->getcY(),
                                       0.0,         0.0,         1.0); 

    cv::Mat ptMat = (cv::Mat_<double>(4,1) << p(0), p(1), p(2), 1.0);
   
    cv::Mat trans = (cv::Mat_<double>(3,4)<<1,0,0,-1.0*exTranslation(0),
                                      0,1,0,-1.0*exTranslation(1),
                                      0,0,1,-1.0*exTranslation(2));

    cv::Mat projMat = calib * (rotatMat * trans);

    cv::Mat pelMat = projMat * ptMat; 

    double projX = pelMat.at<double>(0,0) / pelMat.at<double>(2,0);  
    double projY = pelMat.at<double>(1,0) / pelMat.at<double>(2,0); 

    if (projX < 0 || projY < 0 || projX > camPara->getResX() || projY > camPara->getResY()) {
        return false;
    }

/*
    //FIXME strange values.. like as in -> http://opencv-users.1802565.n2.nabble.com/Problem-using-projectPoints-td5514197.html
    // maybe type mismatch,...
    cv::Point3f cv_p;
    cv_p.x = p(0);
    cv_p.y = p(1);
    cv_p.z = p(2);

    std::vector<cv::Point3f> ptVec;
    ptVec.push_back(cv_p); 
    std::vector<cv::Point2f> pelVec; 

    projectPoints(cv::Mat(ptVec), exRotation, exTranslation, camPara->getCameraMatrix(), camPara->getDistortionCoefficients(), pelVec);

    std::cout << pelVec[0].x << "," << pelVec[0].y << std::endl; 
*/
    return true;
}


void FFocusMessure::extractTranslationRotation() {
    float x, y, z, r, t, ya;
    pcl::getTranslationAndEulerAngles(camPose, x, y, z, r, t, ya);

    exTranslation(0) = x;
    exTranslation(1) = y;
    exTranslation(2) = z;

    exRotation(0) = -r;
    exRotation(1) = -t;
    exRotation(2) = -ya;

    cv::Rodrigues(exRotation, exRotationMat);
}


