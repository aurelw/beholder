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

#include <iostream>
#include <cmath>

#include <opencv2/opencv.hpp>

#include "console_utils.h"
#include "mathutils.h"

/* returns a full transformation of a pattern,
 * centered at the middle point */
void getPatternTransform(cv::Size patternSize, float squareSize,
        const std::vector<cv::Point2f>& imgPoints,
        const cv::Mat &cameraMatrix, const cv::Mat &distCoeefs,
        cv::Mat &tvec, cv::Mat &rvec);


pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudFromPoints(
        const std::vector<cv::Point3f> &points);

std::string samplesToOctaveString(
        const std::vector<std::pair<float, float>> &samples,
        const std::string &prefix="");

