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

#ifndef __MATH_UTILS_H__
#define __MATH_UTILS_H__

#include <pcl/common/common_headers.h>
#include <opencv2/opencv.hpp>

template <class T>
void setBounds(T &v, const T &low, const T &high) {
    if (v < low) {
        v = low;
    } else if (v > high) {
        v = high;
    }
}


Eigen::Affine3f transRotVecToAffine3f(
        const cv::Mat &translationVec, 
        const cv::Mat &rotationVec);

void affine3fToTransRotVec(
        const Eigen::Affine3f &aff,
        cv::Mat &tvec, cv::Mat &rvec);


template<class T>
bool isPointNaN(T point) {
    return (!(pcl_isfinite(point.x) &&
              pcl_isfinite(point.y) &&
              pcl_isfinite(point.z)));
}


inline pcl::PointXYZ vecToPoint(const Eigen::Vector3f &vec) {
    pcl::PointXYZ point;
    point.x = vec[0];
    point.y = vec[1];
    point.z = vec[2];
    return point;
}


inline Eigen::Vector3f pointToVec(pcl::PointXYZ point) {
    return Eigen::Vector3f(point.x, point.y, point.z);
}

inline pcl::PointXYZ pointRGBAtoXYZ(const pcl::PointXYZRGBA &p) {
    pcl::PointXYZ np;
    np.x = p.x;
    np.y = p.y;
    np.z = p.z;
    return np;
}


inline pcl::PointXYZRGBA pointXYZtoRGBA(const pcl::PointXYZ &p) {
    pcl::PointXYZRGBA np;
    np.x = p.x;
    np.y = p.y;
    np.z = p.z;
    return np;
}


pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZtoRGBA(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr inCloud);


inline pcl::PointXYZ pointCVtoPCL(const cv::Point3f &p) {
    pcl::PointXYZ np;
    np.x = p.x;
    np.y = p.y;
    np.z = p.z;
    return np;
}


inline cv::Point3f pointPCLtoCV(const pcl::PointXYZ &p) {
    cv::Point3f np;
    np.x = p.x;
    np.y = p.y;
    np.z = p.z;
    return np;
}


/* computes the two closest points on two lines in 3d space
 * p0 - p1 for the first line
 * q0 - p1 for the second line
 * pointOnP and pointOnP output the two closest points
 * results may not be in the bounds of p0 to p1 or q0 to q1 respectively */
void intersectLines(const Eigen::Vector3f &p0, const Eigen::Vector3f &p1,
                    const Eigen::Vector3f &q0, const Eigen::Vector3f &q1,
                    Eigen::Vector3f &pointOnP, Eigen::Vector3f &pointOnQ);

/* linearly interpolate between twon affine transformations.
 * lerp with quaternions, blender may be in [0,1] range. */
Eigen::Affine3f interpolateAffine(const Eigen::Affine3f &pose0, 
        const Eigen::Affine3f &pose1, float blend);


void printAffine3f(const Eigen::Affine3f m);

#endif

