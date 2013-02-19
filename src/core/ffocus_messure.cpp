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
#include "mathutils.h"

using namespace pcl;
using namespace std;

void FFocusMessure::setPose(const Eigen::Affine3f &pose) {
    camPose = pose;
    updateMatrices();
}


void FFocusMessure::setCameraParameters(CameraParameters::Ptr par) {
    //TODO register to update of camera parameters
    // to recompute camera matrix and other parameters.
    camPara = par;
}


float FFocusMessure::distancePP(
        const Eigen::Vector3f &p0, const Eigen::Vector3f &p1) 
{
    return (p0 - p1).norm();
}


Eigen::Vector3f FFocusMessure::getFocalPoint(const Eigen::Vector3f &p) {
    Eigen::Vector3f camP = camPose * Eigen::Vector3f(0,0,0);
    Eigen::Vector3f viewP = camPose * Eigen::Vector3f(0,0,1);
    Eigen::Vector3f viewVec = viewP - camP;
    Eigen::ParametrizedLine<float, 3> viewLine(camP, viewVec);
    return viewLine.projection(p);
}


float FFocusMessure::getFocalPlaneDistance(const Eigen::Vector3f &p) {
    Eigen::Vector3f camP = camPose * Eigen::Vector3f(0,0,0);
    Eigen::Vector3f focalP = getFocalPoint(p);
    return (focalP-camP).norm();
}


Eigen::Vector2f FFocusMessure::projectPoint(const Eigen::Vector3f &p)
{
    cv::Mat ptMat = (cv::Mat_<float>(4,1) << p(0), p(1), p(2), 1.0);
    cv::Mat pelMat = projCameraMatrix * ptMat; 

    /* project the point to image space */
    float px = pelMat.at<float>(0,0) / pelMat.at<float>(2,0);  
    float py = pelMat.at<float>(1,0) / pelMat.at<float>(2,0); 

    /* normalize to [0.0, 1.0] bounds */
    float xProj = px / camPara->getResX();
    float yProj = py / camPara->getResY();
    return Eigen::Vector2f(xProj, yProj);
}


Eigen::Vector3f FFocusMessure::projectPoint(const Eigen::Vector2f &p) {
    /* convert 2d coordinate (0.0-1.0) to proper image coordinate */
    Eigen::Vector3f aP(p[0]*camPara->getResX(), 
                       p[1]*camPara->getResY(), 
                       1.0);

    /* project with inverse camera calibration matrix 
     * to 3d vector in camera space */
    cv::Mat ptMat = (cv::Mat_<float>(3,1) << aP[0], aP[1], aP[2]);
    cv::Mat vec3d = inverseCalibMatrix * ptMat; 
    Eigen::Vector3f pvec(vec3d.at<float>(0,0),
                         vec3d.at<float>(1,0),
                         vec3d.at<float>(2,0));

    /* normalize, then transform to world space */
    pvec.normalize();
    return camPose * pvec;
}


bool FFocusMessure::isVisible(const Eigen::Vector3f &p) {
    Eigen::Vector2f pPoint = projectPoint(p);
    return (pPoint[0] >= 0 && pPoint[0] <= 1.0 && 
            pPoint[1] >= 0 && pPoint[1] <= 1.0);
}


void FFocusMessure::updateMatrices() {
    float x, y, z, r, t, ya;
    pcl::getTranslationAndEulerAngles(camPose, x, y, z, r, t, ya);

    exTranslation(0) = x;
    exTranslation(1) = y;
    exTranslation(2) = z;

    exRotation(0) = -r;
    exRotation(1) = -t;
    exRotation(2) = -ya;

    cv::Rodrigues(exRotation, exRotationMat);

    /* convert pose and camera parameters
     * to a 4x3 camera porjection matrix */
    cv::Mat rotatMat = (cv::Mat_<float>(3,3) << exRotationMat.at<float>(0,0),exRotationMat.at<float>(0,1),exRotationMat.at<float>(0,2),
                                                 exRotationMat.at<float>(1,0),exRotationMat.at<float>(1,1),exRotationMat.at<float>(1,2),
                                                 exRotationMat.at<float>(2,0),exRotationMat.at<float>(2,1),exRotationMat.at<float>(2,2));
    cv::Mat calibM=(cv::Mat_<float>(3,3)<< camPara->getfX(), 0.0, camPara->getcX(),
                                       0.0, camPara->getfY(), camPara->getcY(),
                                       0.0,         0.0,         1.0); 
    cv::Mat trans = (cv::Mat_<float>(3,4)<<1,0,0,-1.0*exTranslation(0),
                                      0,1,0,-1.0*exTranslation(1),
                                      0,0,1,-1.0*exTranslation(2));
    projCameraMatrix = calibM * (rotatMat * trans);

    /* inverse camera matrix.
     * may be used to reproject points into the scene */
    //FIXME not sure if to use SVD decomposition
    cv::invert(projCameraMatrix, inverseProjCameraMatrix, cv::DECOMP_SVD);
    cv::invert(calibM, inverseCalibMatrix);
}


Eigen::Vector3f FFocusMessure::getClosestOnSegment(
        const Eigen::Vector3f &p0, const Eigen::Vector3f &p1,
        const Eigen::Vector3f &tViewingVector)
{
    Eigen::Vector3f q0 = camPose * Eigen::Vector3f(0,0,0);
    Eigen::Vector3f q1 = tViewingVector;
    Eigen::Vector3f segmentPoint;
    Eigen::Vector3f viewingPoint;

    /* compute the two closest point on both lines */
    intersectLines(p0, p1, q0, q1, segmentPoint, viewingPoint);

    /* check if the point lies inside the segment,
     * else return the endpoints */
    float dp0p1 = distancePP(p0, p1);
    float dp0 = distancePP(p0, segmentPoint);
    float dp1 = distancePP(p1, segmentPoint);
    if (dp0 > dp0p1) {
        segmentPoint = p1;
    } else if (dp1 > dp0p1) {
        segmentPoint = p0;
    }

    return segmentPoint;
}

