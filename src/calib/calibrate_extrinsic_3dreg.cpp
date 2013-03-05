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
#include <opencv2/calib3d/calib3d.hpp>

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
    pointPairs = calibStorage.getExtrinsicPoints3dMatrices();
    cv::Mat rfMat = pointPairs.first;
    cv::Mat camMat = pointPairs.second;
    rfMat.convertTo(rfMat, CV_32F);
    camMat.convertTo(camMat, CV_32F);

    /* prepare point pairs for estimateAffine3D */
    std::vector<cv::Point3f> rangeFinderPoints;
    std::vector<cv::Point3f> cameraPoints;

    for (int i=0; i<rfMat.rows; i++) {
        cv::Point3f rfPoint;
        rfPoint.x = rfMat.at<float>(i, 0);
        rfPoint.y = rfMat.at<float>(i, 1);
        rfPoint.z = rfMat.at<float>(i, 2);
        rangeFinderPoints.push_back(rfPoint);

        cv::Point3f camPoint;
        camPoint.x = camMat.at<float>(i, 0);
        camPoint.y = camMat.at<float>(i, 1);
        camPoint.z = camMat.at<float>(i, 2);
        cameraPoints.push_back(camPoint);
    }

    /* calibration results */
    cv::Mat affineTransform(3,4,CV_32F);
    // translation vector
    cv::Mat exTranslation;
    // axis angle rotation
    cv::Mat exRotationVec;

    /* align the two clouds */
    //std::vector<int> inliers(1);
    //inliers.resize(cameraPoints.rows);
    std::vector<uchar> inliers;
    cv::estimateAffine3D(rangeFinderPoints, cameraPoints,
            affineTransform, inliers);
    affineTransform.convertTo(affineTransform, CV_32F);

    /* decompose matrix */
    exTranslation = affineTransform.col(3);
    cv::Mat rotationMat(3,3, CV_32F);

    /* extract rotation matrix */
    rotationMat.at<float>(0,0) = affineTransform.at<float>(0,0);
    rotationMat.at<float>(1,0) = affineTransform.at<float>(1,0);
    rotationMat.at<float>(2,0) = affineTransform.at<float>(2,0);
    rotationMat.at<float>(0,1) = affineTransform.at<float>(0,1);
    rotationMat.at<float>(1,1) = affineTransform.at<float>(1,1);
    rotationMat.at<float>(2,1) = affineTransform.at<float>(2,1);
    rotationMat.at<float>(0,2) = affineTransform.at<float>(0,2);
    rotationMat.at<float>(1,2) = affineTransform.at<float>(1,2);
    rotationMat.at<float>(2,2) = affineTransform.at<float>(2,2);
    cv::Rodrigues(rotationMat, exRotationVec);

    /* print info about calibration data */
    std::stringstream ss;
    ss << rangeFinderPoints.size();
    //ss << rangeFinderPoints.rows;
    printSimpleInfo("[Point Pairs] ", "Calibrating with #" + 
            ss.str() + " pairs.\n");
    /* display calibration result */
    ss.str("");
    ss << "Inliers #" << inliers.size() << std::endl;
    printSimpleInfo("[RANSAC]", ss.str());
    printBrightInfo("[Extrinsic] ", "calibrated!\n");
    std::cout << "Translation: " << exTranslation << std::endl;
    std::cout << "Rotation: " << exRotationVec << std::endl;

    /* save extrinsic to rigconfig */
    rigConfig.rangefinderExTranslation = exTranslation;
    rigConfig.rangefinderExRotationVec = exRotationVec;
    rigConfig.saveToFile(appopt.rigConfigFile);

}

