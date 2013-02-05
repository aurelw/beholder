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

#ifndef __RIG_CONFIG_H__
#define __RIG_CONFIG_H__

#include <iostream>

#include <pcl/common/common_headers.h>

#include "opencv2/opencv.hpp"


class RigConfig {

    public:

        RigConfig();

        void loadFromFile(std::string fname);
        void saveToFile(std::string fname);

        /* capabilities */
        bool hasRangeFinder;
        bool hasMotor;
        bool hasButtonInterface;
        bool hasTrackingCamera;
        bool hasStereo;
        bool hasIMU;
        bool hasVideoCapture;
        bool hasPhotoCapture;

        /* rangefinder properties */
        cv::Mat rangefinderExTranslation;
        cv::Mat rangefinderExRotationVec;
        //device id

        /* camera properties */
        int cameraImageWidth;
        int cameraImageHeight;
        cv::Mat cameraMatrix;
        cv::Mat cameraDistortionCoefficients;

        /* focus motor properties */
        std::string fMotorType;
        std::string fMotorDevice;
        // cooeffitiens of the transfer functions
        float fMotorHa, fMotorHb, fMotorHc, fMotorHd, fMotorHe, fMotorHf, fMotorHg;
        float fMotorLa, fMotorLb, fMotorLc, fMotorLd, fMotorLe, fMotorLf, fMotorLg;
        // additional axis limits (0.0 - 1.0)
        float fMotorLimitH, fMotorLimitL;
        
        /* video capture properties */
        // v4l device id

        /* photo capture properties */
        // usb device id
        
        /* meta data */
        std::string rigName;
        std::string rigConfigDate;
        int configVersion;

    private:

        cv::FileStorage fs;

        void saveCapabilities();
        void loadCapabilities();
        void saveMetaData();
        void loadMetaData();
        void saveRangeFinder();
        void loadRangeFinder();
        void saveCamera();
        void loadCamera();
        void saveFocusMotor();
        void loadFocusMotor();

};

#endif
