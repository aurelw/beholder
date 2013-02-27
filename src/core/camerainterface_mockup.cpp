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

#include "camerainterface_mockup.h"

#include "console_utils.h"

using namespace boost::filesystem;


void CameraInterfaceMockup::loadFilesFromDirectory(const std::string &dir) {
    path p(dir);
    if (is_directory(p)) {
        std::copy(directory_iterator(p), directory_iterator(), 
                back_inserter(imageFiles));
        std::sort(imageFiles.begin(), imageFiles.end());

        itImageFiles = imageFiles.begin();
    }
}


cv::Mat CameraInterfaceMockup::captureImage() {
    cv::Mat img;

    if (imageFiles.size() > 0) {
        /* wrap iterator around */
        if (itImageFiles == imageFiles.end()) {
            itImageFiles = imageFiles.begin();
        }

        img = cv::imread((*itImageFiles).string(), CV_LOAD_IMAGE_COLOR);
        itImageFiles++;
    } else {
        printWarning("[CameraInterfaceMockup]", " No files found.\n");
        img = cv::Mat::eye(640,480, CV_32F);
    }

    return img;
}


void CameraInterfaceMockup::captureImageToFile(const std::string &dir) {
    cv::Mat img = captureImage();
    cv::imwrite(dir, img);
}


bool CameraInterfaceMockup::checkConnection() {
    return (imageFiles.size() > 0);
}

