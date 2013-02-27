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

#include "camerainterface_gphoto.h"

#include <boost/filesystem.hpp>

CameraInterfaceGPhoto::CameraInterfaceGPhoto(
        const std::string &tmpFile) :
    tempFile(tmpFile)
{
}


void CameraInterfaceGPhoto::captureImageToFile(const std::string &fname) {
    system(buildCmd(fname).c_str());
}


cv::Mat CameraInterfaceGPhoto::captureImage() {
    captureImageToFile(tempFile);
    cv::Mat img = cv::imread(tempFile, CV_LOAD_IMAGE_COLOR);
    boost::filesystem::remove(tempFile);
    return img;
}


bool CameraInterfaceGPhoto::checkConnection() {
    int ret = system(buildCheckCamCmd().c_str());
    return (ret == 0);
}


std::string CameraInterfaceGPhoto::buildCmd(const std::string &capfile) {
    std::stringstream cmd;
    cmd << GPHOTO_BIN << " ";
    cmd << "--port usb ";
    cmd << "--set-config /main/settings/capture=1 ";
    cmd << "--capture-image-and-download ";
    cmd << "--filename " << capfile << " ";
    return cmd.str();
}


std::string CameraInterfaceGPhoto::buildCheckCamCmd() {
    std::stringstream cmd;
    cmd << GPHOTO_BIN << " ";
    cmd << "--port usb ";
    cmd << "--set-config /main/settings/capture=1 ";
    return cmd.str();
}

