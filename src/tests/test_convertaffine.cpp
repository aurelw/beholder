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

#include "mathutils.h"
#include "console_utils.h"


int main() {
    //cv::Mat rvec = (cv::Mat_<float>(3,1) << 0.04, 0.12, 0.456);
    cv::Mat rvec = (cv::Mat_<float>(3,1) << 0.0048058364, 0.0073052156, 0.019809863);
    cv::Mat tvec = (cv::Mat_<float>(3,1) << 2.0, 1.456, 0.212);

    Eigen::Affine3f aff;
    aff = transRotVecToAffine3f(tvec, rvec);
    cv::Mat nRvec(3,1, CV_32F);
    cv::Mat nTvec(3,1, CV_32F);
    affine3fToTransRotVec(aff, nTvec, nRvec);

    std::cout << "original rotation: " << rvec << std::endl;
    std::cout << "converted rotation: " << nRvec << std::endl;
    std::cout << "original translation: " << tvec << std::endl;
    std::cout << "converted translation: " << nTvec << std::endl;

    std::cout << "Eigen Affine:" << std::endl;
    std::cout << affineToString(aff) << std::endl;

    cv::Mat rotMat(3,3, CV_32F);
    cv::Rodrigues(rvec, rotMat);

    std::cout << "CV Rotation Matrix:" << std::endl;
    std::cout << rotMat << std::endl;

    cv::Mat rVecFromMat(3,1, CV_32F);
    cv::Rodrigues(rotMat, rVecFromMat);
    std::cout << "Rotation Vec from mat: " << rVecFromMat << std::endl;

    std::cout << "----------------------------------" << std::endl;

    //FIXME this is a BUG, rot mat should be equal both times
    /* matrix convert test */
    /*
    cv::Mat calRotMat = (cv::Mat_<double>(3,3) << 1.0235926, 0.02855067, -0.001949106,
                                            -0.012628916, 1.0562134, 0.0050560236,
                                            0.012989579, -0.004787772, 1.0266194);
                                            */
    /*
    cv::Mat calRotMat = (cv::Mat_<double>(3,3) << 1.0235926, -0.012628916, 0.012989579,
                                                  0.02855067, 1.0562134, -0.004787772,
                                                 -0.001949106, 0.0050560236, 1.0266194);
                                                 */
    cv::Mat calRotMat_0 = (cv::Mat_<double>(3,3) << 1.0, 0.0, 1.0,
                                                  0.0, 1.0, 0.0,
                                                  0.0, 0.0, 1.0);

    cv::Mat calRotVec_0(3,1, CV_64F);
    cv::Rodrigues(calRotMat_0, calRotVec_0);

    cv::Mat calRotMat_1(3,1, CV_64F);
    cv::Rodrigues(calRotVec_0, calRotMat_1);

    cv::Mat calRotVec_1(3,1, CV_64F);
    cv::Rodrigues(calRotMat_1, calRotVec_1);

    std::cout << "Calibration Rot Mat inital:" << std::endl;
    std::cout << calRotMat_0 << std::endl;
    std::cout << "Calibration Rot Vector from initial Mat:" << std::endl;
    std::cout << calRotVec_0 << std::endl;
    std::cout << "Calibration Rot Mat From Vector:" << std::endl;
    std::cout << calRotMat_1 << std::endl;
    std::cout << "Calibration Rot Vector:" << std::endl;
    std::cout << calRotVec_1 << std::endl;


}

