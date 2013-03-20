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

#include "contrast_messure.h"


ContrastMessure::ContrastMessure(ContrastMessure::Method m) {
    method = m;
}


float ContrastMessure::messure(const cv::Mat& img) {

    float contrast = 0;

    /* convert to gray level image */
    cv::Mat gray_img;
    cv::cvtColor(img, gray_img, CV_RGB2GRAY);
    gray_img.convertTo(gray_img, CV_32F);
    cv::normalize(gray_img, gray_img, 0, 1.0, CV_MINMAX);


    /* messure the standard deviation */
    if (method == STD) {
        cv::Scalar mean;
        cv::Scalar sigma;
        cv::meanStdDev(gray_img, mean, sigma);
        contrast = sigma[0];

    /* modified laplacian */
    } else if (method == MLAPLACIAN) {
        cv::Mat kernel_x = (cv::Mat_<float>(3,1) << -1, 2, -1);
        cv::Mat kernel_y;
        cv::transpose(kernel_x, kernel_y);

        cv::Mat lX, lY;
        cv::filter2D(gray_img, lX, -1, kernel_x);
        cv::filter2D(gray_img, lY, -1, kernel_y);

        cv::Mat lapImg;
        lX = cv::abs(lX);
        lY = cv::abs(lY);
        cv::add(lX, lY, lapImg);

        cv::Scalar mean;
        cv::Scalar sigma;
        cv::meanStdDev(gray_img, mean, sigma);

        contrast = 1 - mean[0];
    }

    return contrast;
}


