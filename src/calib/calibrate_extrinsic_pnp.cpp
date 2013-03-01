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

#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>

#include "calibstorage_contract.h"
#include "rig_config.h"
#include "basicappoptions.h"
#include "console_utils.h"
#include "mathutils.h"

#define KEY_ESC 27
#define KEY_ENTER 13
#define KEY_c 99
#define KEY_y 121
#define KEY_n 110

void print_usage() {
    std::cout << "--calibstorage <path> --rigconfig <file> [--method <\"det\"/\"ransac\">]" << std::endl;
}


int main(int argc, char **argv) {

    /*** command line arguments ***/
    BasicAppOptions appopt(argc, argv);

    if (!appopt.gotCalibStorageDir) {
        pcl::console::print_error(
            "No calibration storage directory provided. --calibstorage <path>\n");
        exit(1);
    }

    if (!appopt.gotRigConfigFile) {
        pcl::console::print_error(
                "No rigconfig provided. --rigconfig <path>\n");
        exit(1);
    }

    /* calibration method */
    std::string calibMethod = "det"; // det, ransac
    pcl::console::parse(argc, argv, "--method", calibMethod);

    /* help */
    if (pcl::console::find_switch(argc, argv, "-h") ||
        pcl::console::find_switch(argc, argv, "--help")) 
    {
        print_usage();
        exit(0);
    }

    /*****************************/


    /* the calibration storage */
    CalibStorageContract calibStorage(appopt.calibStorageDir);

    /* the rig config */
    RigConfig rigConfig;
    rigConfig.loadFromFile(appopt.rigConfigFile);

    /* get the point pairs */
    std::pair<cv::Mat, cv::Mat> pointPairs;
    pointPairs = calibStorage.getExtrinsicPointsMatrices();
    cv::Mat objectPoints = pointPairs.first;
    cv::Mat imagePoints = pointPairs.second;

    /* print info about calibration data */
    std::stringstream ss;
    ss << imagePoints.rows;
    printSimpleInfo("[Point Pairs] ", "Calibrating with #" + 
            ss.str() + " pairs.\n");

    /* calibration results */
    // translation vector
    cv::Mat exTranslation;
    // axis angle rotation
    cv::Mat exRotationVec;

    /* do calibration */
    if (calibMethod == "ransac") {
        //TODO find better ransac parameters
        cv::solvePnPRansac(objectPoints, imagePoints,
            rigConfig.cameraMatrix, 
            rigConfig.cameraDistortionCoefficients,
            exRotationVec, exTranslation);
    } else {
        cv::solvePnP(objectPoints, imagePoints,
            rigConfig.cameraMatrix, 
            rigConfig.cameraDistortionCoefficients,
            exRotationVec, exTranslation);
    }

    /* display calibration result */
    printBrightInfo("[Extrinsic] ", "calibrated!\n");
    std::cout << "Translation: " << exTranslation << std::endl;
    std::cout << "Rotation: " << exRotationVec << std::endl;

    /* save extrinsic to rigconfig */
    rigConfig.rangefinderExTranslation = exTranslation;
    rigConfig.rangefinderExRotationVec = exRotationVec;
    rigConfig.saveToFile(appopt.rigConfigFile);

}

