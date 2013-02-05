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

#include "rig_config.h"

using namespace cv;


RigConfig::RigConfig() {
    
    /* default capabilities */
    hasRangeFinder = true;
    hasMotor = true;
    hasButtonInterface = true;
    hasTrackingCamera = false;
    hasStereo = false;
    hasIMU = false;
    hasVideoCapture = false;
    hasPhotoCapture = false;

    /* default meta data */
    rigName = "DefaultRig";
    rigConfigDate = "today";
    configVersion = 1;

    /* default rangefinder data */
    rangefinderExTranslation = Mat::zeros(3, 1, CV_32F);
    rangefinderExRotationVec = Mat::zeros(3, 1, CV_32F);

    /* default camera data */
    cameraImageWidth = 1920;
    cameraImageWidth = 1080;
    cameraMatrix = Mat::zeros(3, 3, CV_32F);
    cameraDistortionCoefficients = Mat::zeros(5, 1, CV_32F);

    /* defualt focus motor data */
    fMotorType = "ByteServo";
    fMotorDevice = "/dev/ttyUSB0";
    fMotorLimitH = 0.0f;
    fMotorLimitL = 0.0f;
}


void RigConfig::saveToFile(std::string fname) {
    fs.open(fname, FileStorage::WRITE);
    saveCapabilities();
    saveMetaData();
    saveRangeFinder();
    saveCamera();
    saveFocusMotor();
    fs.release();
}


void RigConfig::loadFromFile(std::string fname) {
    fs.open(fname, FileStorage::READ);
    loadCapabilities();
    loadMetaData();
    loadRangeFinder();
    loadCamera();
    loadFocusMotor();
    fs.release();
}


void RigConfig::saveCapabilities() {
    fs << "capabilities" << "{";

    fs << "hasRangeFinder" << hasRangeFinder;
    fs << "hasMotor" << hasMotor;
    fs << "hasButtonInterface" << hasButtonInterface;
    fs << "hasTrackingCamera" << hasTrackingCamera;
    fs << "hasStereo" << hasStereo;
    fs << "hasIMU" << hasIMU;
    fs << "hasVideoCapture" << hasVideoCapture;
    fs << "hasPhotoCapture" << hasPhotoCapture;

    fs << "}";
}


void RigConfig::loadCapabilities() {
    FileNode node = fs["capabilities"];
    node["hasRangeFinder"] >> hasRangeFinder;
    node["hasMotor"] >> hasMotor;
    node["hasButtonInterface"] >> hasButtonInterface;
    node["hasTrackingCamera"] >> hasTrackingCamera;
    node["hasStereo"] >> hasStereo;
    node["hasIMU"] >> hasIMU;
    node["hasVideoCapture"] >> hasVideoCapture;
    node["hasPhotoCapture"] >> hasPhotoCapture;
}


void RigConfig::saveMetaData() {
    fs << "metaData" << "{";

    fs << "name" << rigName;
    fs << "configDate" << rigConfigDate;
    fs << "configVersion" << configVersion;

    fs << "}";
}


void RigConfig::loadMetaData() {
    FileNode node = fs["metaData"];
    node["name"] >> rigName;
    node["configDate"] >> rigConfigDate;
    node["configVersion"] >> configVersion;
}


void RigConfig::saveRangeFinder() {
    fs << "rangeFinder" << "{";

    fs << "exTanslation" << rangefinderExTranslation;
    fs << "exRotationVec" << rangefinderExRotationVec;

    fs << "}";
}


void RigConfig::loadRangeFinder() {
    FileNode node = fs["rangeFinder"];
    node["exTanslation"] >> rangefinderExTranslation;
    node["exRotationVec"] >> rangefinderExRotationVec;
}


void RigConfig::saveCamera() {
    fs << "camera" << "{";

    fs << "imageWidth" << cameraImageWidth;
    fs << "imageHeight" << cameraImageHeight;
    fs << "cameraMatrix" << cameraMatrix;
    fs << "distortionCoefficients" << cameraDistortionCoefficients;

    fs << "}";
}


void RigConfig::loadCamera() {
    FileNode node = fs["camera"];
    node["imageWidth"] >> cameraImageWidth;
    node["imageHeight"] >> cameraImageHeight;
    node["cameraMatrix"] >> cameraMatrix;
    node["distortionCoefficients"] >> cameraDistortionCoefficients;
}


void RigConfig::saveFocusMotor() {
    fs << "focusMotor" << "{";

    fs << "type" << fMotorType;
    fs << "device" << fMotorDevice;

    fs << "ha" << fMotorHa;
    fs << "hb" << fMotorHb;
    fs << "hc" << fMotorHc;
    fs << "hd" << fMotorHd;
    fs << "he" << fMotorHe;
    fs << "hf" << fMotorHf;
    fs << "hg" << fMotorHg;

    fs << "la" << fMotorLa;
    fs << "lb" << fMotorLb;
    fs << "lc" << fMotorLc;
    fs << "ld" << fMotorLd;
    fs << "le" << fMotorLe;
    fs << "lf" << fMotorLf;
    fs << "lg" << fMotorLg;

    fs << "limitH" << fMotorLimitH;
    fs << "limitL" << fMotorLimitL;

    fs << "}";
}


void RigConfig::loadFocusMotor() {
    FileNode node = fs["focusMotor"];

    node["type"] >> fMotorType;
    node["device"] >> fMotorDevice;

    node["ha"] >> fMotorHa;
    node["hb"] >> fMotorHb;
    node["hc"] >> fMotorHc;
    node["hd"] >> fMotorHd;
    node["he"] >> fMotorHe;
    node["hf"] >> fMotorHf;
    node["hg"] >> fMotorHg;

    node["la"] >> fMotorLa;
    node["lb"] >> fMotorLb;
    node["lc"] >> fMotorLc;
    node["ld"] >> fMotorLd;
    node["le"] >> fMotorLe;
    node["lf"] >> fMotorLf;
    node["lg"] >> fMotorLg;

    node["limitH"] >> fMotorLimitH;
    node["limitL"] >> fMotorLimitL;
}
