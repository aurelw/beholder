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
            exRotationMat = cv::Mat_<float>(3,3);
        }

        /* objects needed for proper calculation */
        void setCameraParameters(CameraParameters::Ptr par);
        void setPose(const Eigen::Affine3f& pose);

        /* the distance of any point to the camera */
        float getFocalPlaneDistance(const Eigen::Vector3f &p);

        /* projects a point onto the viewing axis 
         * which sort of represents a virtual focal
         * point for the actual point. */
        Eigen::Vector3f getFocalPoint(const Eigen::Vector3f &p);

        /* checks if any point projects into the camera frame */
        bool isVisible(const Eigen::Vector3f &p);

        /* project a point with the camera */
        Eigen::Vector2f projectPoint(const Eigen::Vector3f &p);

        /* project to image point 3d point in world space */
        Eigen::Vector3f projectPoint(const Eigen::Vector2f &p);

        /* euclidean distance between two points */
        //FIXME move to mathutils
        float distancePP(const Eigen::Vector3f &p0, const Eigen::Vector3f &p1);

        /* Get the closest point on the segment
         *  by the viewing vector transformed to world space */ 
        Eigen::Vector3f getClosestOnSegment(
            const Eigen::Vector3f &p0, const Eigen::Vector3f &p1,
            const Eigen::Vector3f &tViewingVector);

    private:

        void updateMatrices();

        /* input models */
        Eigen::Affine3f camPose;
        CameraParameters::Ptr camPara;

        cv::Vec3f exTranslation, exRotation;
        cv::Mat exRotationMat;
        cv::Mat projCameraMatrix;
        cv::Mat inverseProjCameraMatrix;
        cv::Mat inverseCalibMatrix;

};

#endif
