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
#include "basicappoptions.h"
#include "calib_visualizer.h"
#include "console_utils.h"
#include "mathutils.h"
#include "plane_marker.h"

#define KEY_ESC 27
#define KEY_ENTER 13
#define KEY_c 99
#define KEY_y 121
#define KEY_n 110

void print_usage() {
    pcl::console::print_error("[Error] -pw <patternWidth> -ph <patternHeight> -ps <boardSigma> [--onlyprint]\n");
}


int main(int argc, char **argv) {

    /*** command line arguments ***/
    BasicAppOptions appopt(argc, argv);

    if (!appopt.gotCalibStorageDir) {
        pcl::console::print_error("No calibration storage" 
                "directory provided. --calibstorage <path>\n");
        exit(1);
    }

    if (!appopt.gotRigConfigFile) {
        pcl::console::print_error(
                "No rigonfig provided. --rigconfig <file>\n");
        exit(1);
    }

    /* 2d marker properties */
    int patternWidth, patternHeight;
    float squareSize = 0.032;
    if (pcl::console::parse(argc, argv, "-pw", patternWidth) == -1) {
        print_usage();
        exit(1);
    }
    if (pcl::console::parse(argc, argv, "-ph", patternHeight) == -1) {
        print_usage();
        exit(1);
    }
    if (pcl::console::parse(argc, argv, "-ps", squareSize) == -1) {
        print_usage();
        exit(1);
    }

    bool storeResults = true;
    storeResults = !pcl::console::find_switch(argc, argv, "--onlyprint");

    /* help */
    if (pcl::console::find_switch(argc, argv, "-h") ||
        pcl::console::find_switch(argc, argv, "--help")) 
    {
        print_usage();
        exit(0);
    }

    /*****************************/


    /* the calibdation storage */
    CalibStorageContract calibStorage(appopt.calibStorageDir);

    /* image windows */
    cv::namedWindow("image", CV_WINDOW_NORMAL|CV_GUI_EXPANDED);

    /* the captured image files */
    std::vector<std::string> imageFiles;
    imageFiles = calibStorage.getMainIntrinsicFiles();

    /* pattern info */
    cv::Size boardSize(patternWidth, patternHeight);

    /* calibration results */
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Size imageSize;

    /* image points */
    std::vector<std::vector<cv::Point2f>> imagePoints;

    /* determine the size */
    cv::Mat img0 = cv::imread(imageFiles[0], CV_LOAD_IMAGE_COLOR);
    imageSize = img0.size();

    /* get all image points */
    int frameCounter = 0;
    for (std::string file : imageFiles) {
        /* print info */
        frameCounter ++;
        std::stringstream ss;
        ss << "Processing frame " << frameCounter 
            << "/" << imageFiles.size() << std::endl;
        printSimpleInfo("[Image] ", ss.str()); 

        /* load image */
        cv::Mat img = cv::imread(file, CV_LOAD_IMAGE_COLOR);
        //cv::imshow("image", img);
        cv::waitKey(1);

        /* check if all images have the same size */
        if (img.cols != imageSize.width || 
                img.rows != imageSize.height) 
        {
            printWarning("[Image] ", 
                    file + " has different resolution.\n");
        }

        /* find image points of chessboard */
        std::vector<cv::Point2f> pointbuff;
        bool found = cv::findChessboardCorners(img, boardSize, pointbuff, 
            CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | 
            CV_CALIB_CB_NORMALIZE_IMAGE);

        /* store valid pattern image points */
        if (found) {
            printBrightInfo("[Chessboard] ", "found.\n");
            imagePoints.push_back(pointbuff);
        } else {
            printWarning("[Chessboard] ", "not found.\n");
        }
        
        /* display results */
        cv::drawChessboardCorners(img, boardSize, pointbuff, found);
        cv::imshow("image", img);
        cv::waitKey(1);
    }


    /*** do calibration ***/

    /* prepare objectPoints */
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<cv::Point3f> pointbuff;

    for( int i = 0; i < boardSize.height; i++ ) {
        for( int j = 0; j < boardSize.width; j++ ) {
            pointbuff.push_back(cv::Point3f(float(j*squareSize),
            float(i*squareSize), 0));
        }
    }

    objectPoints.resize(imagePoints.size(), pointbuff);

    /* do actual calibration */
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = calibrateCamera( objectPoints, imagePoints, imageSize,
            cameraMatrix, distCoeffs, rvecs, tvecs,
            CV_CALIB_FIX_K4|CV_CALIB_FIX_K5 );


    /* display results */
    std::cout << std::endl;
    printBrightInfo("====== [Calibration Done] ======\n");
    std::stringstream ss;
    ss << imageSize.width << "/" << imageSize.height << std::endl;
    printSimpleInfo("[Resolution] ", ss.str());
    ss.str("");
    printSimpleInfo("[Camera Matrix]\n");
    std::cout << cameraMatrix << std::endl;
    printSimpleInfo("[Distortion Coefficients]\n");
    std::cout << distCoeffs << std::endl;
    ss << rms << std::endl;
    printSimpleInfo("[RMS] ", ss.str());
    printBrightInfo("================================\n");
    std::cout << std::endl;

    /* store the results in the rigconfig */
    if (storeResults) {
        RigConfig rigConfig;
        rigConfig.loadFromFile(appopt.rigConfigFile);

        rigConfig.cameraMatrix = cameraMatrix;
        rigConfig.cameraDistortionCoefficients = distCoeffs;
        rigConfig.cameraImageWidth = imageSize.width;
        rigConfig.cameraImageHeight = imageSize.height;

        rigConfig.saveToFile(appopt.rigConfigFile);
    }

}

