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

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>

int main() {
/*
    int numPoints = 20;
    cv::Mat in0(numPoints, 3, CV_32F);
    cv::Mat in1(numPoints, 3, CV_32F);
    cv::Mat affinTransform(3, 4, CV_32F);
    cv::Mat inliers;

    for (int i=0; i<20; i++) {
        in0.at<float>(i,0) = i;
        in0.at<float>(i,1) = 0;
        in0.at<float>(i,2) = 0;

        in1.at<float>(i,0) = i;
        in1.at<float>(i,1) = 1;
        in1.at<float>(i,2) = 1;
    }

    cv::estimateAffine3D(in0, in1, affinTransform, inliers);
*/
    std::vector<cv::Point3f> first, second;
    std::vector<uchar> inliers;
    cv::Mat aff(3,4,CV_64F);

    for (int i = 0; i <6; i++)
    {
        first.push_back(cv::Point3f(i,i%3,1));
        second.push_back(cv::Point3f(i,i%3,1));
    }

    int ret = cv::estimateAffine3D(first, second, aff, inliers);
    std::cout << aff << std::endl;

}
