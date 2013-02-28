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

#include "console_utils.h"

using namespace cv;


RigConfig::RigConfig() {
    
    /* default capabilities */
    hasRangeFinder = true;
    hasMotor = true;
    hasButtonInterface = true;
    hasTrackingCamera = false;
    hasStereo = false;
    hasIMU = false;
    hasStreaming = false;
    hasPhotoCapture = false;

    /* default meta data */
    rigName = "DefaultRig";
    rigConfigDate = "today";
    configVersion = 1;

    /* default rangefinder data */
    rangefinderExTranslation = Mat::zeros(3, 1, CV_32F);
    rangefinderExRotationVec = Mat::zeros(3, 1, CV_32F);
    rangefinderDeviceID = "0";

    /* default camera data */
    cameraImageWidth = 1920;
    cameraImageHeight = 1080;
    cameraMatrix = Mat::zeros(3, 3, CV_32F);
    cameraDistortionCoefficients = Mat::zeros(5, 1, CV_32F);

    /* defualt focus motor data */
    fMotorType = "ByteServo";
    fMotorDevice = "/dev/ttyUSB0";
    fMotorLimitH = 0.0f;
    fMotorLimitL = 0.0f;

    /* default tracking camera data */
    trackingCameraExTranslation = Mat::zeros(3, 1, CV_32F);
    trackingCameraExRotationVec = Mat::zeros(3, 1, CV_32F);
    trackingCameraDeviceID = "1";


    /* default photo capture poperties */
    photoCaptureType = "gphoto";
    photoCapturePort = "usb";
    photoCaptureMockupDir = "/tmp/camerainterface_mockup_files/";

    /* default main camera streaming */
    streamingType = "v4l";
    streamingDevice = "0";
    streamingDoCrop = false;
    streamingCropX = 0.0;
    streamingCropY = 0.0;
    streamingCropXX = 1.0;
    streamingCropYY = 1.0;
    streamingWidth = 640;
    streamingHeight = 480;
}

void RigConfig::saveToFile(std::string fname) {
    fs.open(fname, FileStorage::WRITE);
    saveCapabilities();
    saveMetaData();
    saveRangeFinder();
    saveCamera();
    saveFocusMotor();
    saveStreaming();
    savePhotoCapture();
    fs.release();
    printSimpleInfo("[RigConfig] ", "saved to file " + fname + "\n");
}


void RigConfig::loadFromFile(std::string fname) {
    fs.open(fname, FileStorage::READ);
    loadCapabilities();
    loadMetaData();
    loadRangeFinder();
    loadCamera();
    loadFocusMotor();
    loadPhotoCapture();
    loadStreaming();
    fs.release();
    printSimpleInfo("[RigConfig] ", "loaded from file " + fname + "\n");
}


void RigConfig::saveCapabilities() {
    fs << "capabilities" << "{";

    fs << "hasRangeFinder" << hasRangeFinder;
    fs << "hasMotor" << hasMotor;
    fs << "hasButtonInterface" << hasButtonInterface;
    fs << "hasTrackingCamera" << hasTrackingCamera;
    fs << "hasStereo" << hasStereo;
    fs << "hasIMU" << hasIMU;
    fs << "hasStreaming" << hasStreaming;
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
    node["hasStreaming"] >> hasStreaming;
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
    fs << "deviceID" << rangefinderDeviceID;

    fs << "}";
}


void RigConfig::loadRangeFinder() {
    FileNode node = fs["rangeFinder"];
    node["exTanslation"] >> rangefinderExTranslation;
    node["exRotationVec"] >> rangefinderExRotationVec;
    node["deviceID"] >> rangefinderDeviceID;

    /* convert matrix to float */
    rangefinderExTranslation.convertTo(rangefinderExTranslation, CV_32F);
    rangefinderExRotationVec.convertTo(rangefinderExRotationVec, CV_32F);
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

    /* convert to float */
    cameraMatrix.convertTo(cameraMatrix, CV_32F);
    cameraDistortionCoefficients.convertTo(
            cameraDistortionCoefficients, CV_32F);
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


void RigConfig::saveStreaming() {
    fs << "streaming" << "{";

    fs << "type" << streamingType;
    fs << "device" << streamingDevice;
    fs << "doCrop" << streamingDoCrop;
    fs << "cropX" << streamingCropX;
    fs << "cropY" << streamingCropY;
    fs << "cropXX" << streamingCropXX;
    fs << "cropYY" << streamingCropYY;
    fs << "width" << streamingWidth;
    fs << "height" << streamingHeight;

    fs << "}";
}


void RigConfig::loadStreaming() {
    FileNode node = fs["streaming"];
    node["type"] >> streamingType;
    node["device"] >> streamingDevice;
    node["doCrop"] >> streamingDoCrop;
    node["cropX"] >> streamingCropX;
    node["cropY"] >> streamingCropY;
    node["cropXX"] >> streamingCropXX;
    node["cropYY"] >> streamingCropYY;
    node["width"] >> streamingWidth;
    node["height"] >> streamingHeight;
}


void RigConfig::savePhotoCapture() {
    fs << "photoCapture" << "{";

    fs << "type" << photoCaptureType;
    fs << "port" << photoCapturePort;
    fs << "mockupDir" << photoCaptureMockupDir;

    fs << "}";
}


void RigConfig::loadPhotoCapture() {
    FileNode node = fs["photoCapture"];
    node["type"] >> photoCaptureType;
    node["port"] >> photoCapturePort;
    node["mockupDir"] >> photoCaptureMockupDir;
}


void RigConfig::saveTrackingCamera() {
    fs << "trackingCamera" << "{";

    fs << "exTanslation" << trackingCameraExTranslation;
    fs << "exRotationVec" << trackingCameraExRotationVec;
    fs << "deviceID" << trackingCameraDeviceID;

    fs << "}";
}


void RigConfig::loadTrackingCamera() {
    FileNode node = fs["trackingCamera"];
    node["exTanslation"] >> trackingCameraExTranslation;
    node["exRotationVec"] >> trackingCameraExRotationVec;
    node["deviceID"] >> trackingCameraDeviceID;

    /* conver to float matrices */
    trackingCameraExTranslation.convertTo(
            trackingCameraExTranslation, CV_32F);
    trackingCameraExRotationVec.convertTo(
            trackingCameraExRotationVec, CV_32F);
}


