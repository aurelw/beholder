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

#ifndef __CAMERA_INTERFACE_MOCKUP_H__
#define __CAMERA_INTERFACE_MOCKUP_H__

#include <boost/filesystem.hpp>

#include "opencv2/opencv.hpp"

#include "camerainterface.h"


class CameraInterfaceMockup : public CameraInterface {

    public:

        void loadFilesFromDirectory(const std::string &dir);

        /* CameraInterface implementation */
        virtual void captureImageToFile(const std::string &fname) override;
        virtual cv::Mat captureImage() override;

    protected:

        std::vector<boost::filesystem::path> imageFiles;
        std::vector<boost::filesystem::path>::const_iterator itImageFiles;

};

#endif

