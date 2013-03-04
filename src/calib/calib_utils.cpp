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

#include "calib_utils.h"


void getPatternTransform(cv::Size patternSize, float squareSize,
        const std::vector<cv::Point2f>& imgPoints,
        const cv::Mat &cameraMatrix, const cv::Mat &distCoeefs,
        cv::Mat &tvec, cv::Mat &rvec)
{
    
    /* prepare objekt points */
    std::vector<cv::Point3f> objectPoints;
    for( int i = 0; i < patternSize.height; i++ ) {
        for( int j = 0; j < patternSize.width; j++ ) {
            objectPoints.push_back(cv::Point3f(
                    float(j*squareSize) - 
                        float((patternSize.width/2)*squareSize),
                    float(i*squareSize) - 
                        float((patternSize.height/2)*squareSize),
                    0));
        }
    }

    /* find transformation */
    cv::solvePnP(objectPoints, imgPoints,
            cameraMatrix, distCoeefs,
            rvec, tvec);

    /* convert to transform in camera spave */
    //FIXME check if this gives the right transformation
    tvec = tvec;
    rvec = rvec;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudFromPoints(
        const std::vector<cv::Point3f> &points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);

    for (auto point : points) {
        cloud->push_back(pointCVtoPCL(point));
    }

    return cloud;
}

