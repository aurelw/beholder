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

#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

#include <opencv2/opencv.hpp>

/* core */
#include "rig_config.h"
#include "cameraparameters.h"
#include "rangefinder.h"
#include "posetrackerkinfu.h"
#include "viewfinder_rangeimage.h"
#include "rangeimagewriter.h"
#include "basicappoptions.h"

/* calib */
#include "calib_utils.h"
#include "calibstorage_contract.h"

/* gui */
#include "calib_visualizer.h"


void print_usage() {
    std::cout << "--calibstorage <path> --rigconfig <file> [-e] [-l] [-k] [--icp]" << std::endl;
}


int main(int argc, char **argv) {

    /*** command line arguments ***/
    /* help */
    if (pcl::console::find_switch(argc, argv, "-h") ||
        pcl::console::find_switch(argc, argv, "--help")) 
    {
        print_usage();
        exit(0);
    }

    BasicAppOptions appopt(argc, argv);

    if (!appopt.gotCalibStorageDir) {
        pcl::console::print_error(
                "No calibstorage provided. --calibstorage <path>\n");
        exit(1);
    }

    if (!appopt.gotRigConfigFile) {
        pcl::console::print_error(
                "No rigconfig provided. --rigconfig <file>\n");
        exit(1);
    }

    bool doExtrinsic = false;
    doExtrinsic = pcl::console::find_switch(argc, argv, "-e");

    bool doArrows = true;
    doArrows = !pcl::console::find_switch(argc, argv, "-l");

    bool drawKinectFrame = false;
    drawKinectFrame = pcl::console::find_switch(argc, argv, "-k");

    bool doICP = false;
    doICP = pcl::console::find_switch(argc, argv, "--icp");

    /*****************************/

    /* calibration storage */
    CalibStorageContract calibStorage(appopt.calibStorageDir);

    /* the basic rig config */
    RigConfig rc;
    rc.loadFromFile(appopt.rigConfigFile);

    /* visualizer */
    CalibVisualizer visualizer; 
    visualizer.start();


    /* get the correspondence point clouds */
    std::vector<CalibStorageContract::PointPair3d3d> ppairs;
    ppairs = calibStorage.getExtrinsicPoints3d();

    std::vector<cv::Point3f> points0, points1;
    for (auto pair : ppairs) {
        points0.push_back(pair.first);
        points1.push_back(pair.second);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudKinect, cloudCam;
    cloudKinect = pointCloudFromPoints(points0);
    cloudCam = pointCloudFromPoints(points1);

    /* handle extrinsic of the camera */
    Eigen::Affine3f ext = transRotVecToAffine3f(
            rc.rangefinderExTranslation,
            rc.rangefinderExRotationVec);

    if (drawKinectFrame) {
        visualizer.addCoordinateSystem(ext, "ext", 1.0, true);
    }

    printBrightInfo("=== [Extrinsic] ===", "\n");
    printSimpleInfo("[Affine]", "\n");
    std::cout << affineToString(ext) << std::endl;
    printSimpleInfo("[tvec]", "\n");
    std::cout << rc.rangefinderExTranslation << std::endl;
    printSimpleInfo("[rvec]", "\n");
    std::cout << rc.rangefinderExRotationVec << std::endl;
    printBrightInfo("===================", "\n");

    /* transform the cloud if specified */
    if (doExtrinsic) {
        // transform the kinect cloud into main camera space
        pcl::transformPointCloud(*cloudKinect, *cloudKinect, ext);
    }

    /* do additiwonal icp */
    if (doICP) {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputCloud(cloudKinect);
        icp.setInputTarget(cloudCam);
        icp.setMaximumIterations(5000);
        pcl::PointCloud<pcl::PointXYZ> alignCloud;
        icp.align(alignCloud);
        Eigen::Matrix4f mat;
        mat = icp.getFinalTransformation();
        Eigen::Affine3f icpTrans(mat);
        pcl::transformPointCloud(*cloudKinect, *cloudKinect, icpTrans);
    }

    /* print some information */
    std::stringstream ss;
    ss << "#" << ppairs.size() << " corresponding pairs." << std::endl; 
    printSimpleInfo("[Correspondence] ", ss.str());

    /* visualize */
    visualizer.setCorrespondence(cloudKinect, cloudCam);
    visualizer.setDrawCorrespondence(true, doArrows);

    /* just loop */
    for (;;) {
        sleep(1);
    }
}

