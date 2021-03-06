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

#ifndef __CONTRAST_MESSURE_H__
#define __CONTRAST_MESSURE_H__

#include <opencv2/opencv.hpp>


class ContrastMessure {

    public:

        typedef enum Method { 
            STD,
            MLAPLACIAN,
        } Method;

    public:

        ContrastMessure(Method m=MLAPLACIAN);

        cv::Mat preprocess(const cv::Mat& img);
        float messure(const cv::Mat& img);

    private:

        Method method;
};


#endif

