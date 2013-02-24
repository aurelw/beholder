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

#include "opencv2/opencv.hpp"

#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>

#include "calibstorage_contract.h"
#include "basicappoptions.h"
#include "calib_visualizer.h"
#include "console_utils.h"
#include "plane_marker.h"

#define KEY_ESC 27
#define KEY_ENTER 13
#define KEY_c 99
#define KEY_y 121
#define KEY_n 110

typedef typename pcl::PointXYZRGBA PointT;
typedef typename pcl::PointCloud<PointT> Cloud;


void print_usage() {
    pcl::console::print_error("[Error] -bw <boardWidth> -bh <boardHeight> -pw <patternWidth> -ph <patternHeight> [-bs <boardSigma>] [-nc]\n");
}


int main(int argc, char **argv) {

    /*** command line arguments ***/
    BasicAppOptions appopt(argc, argv);

    if (!appopt.gotCalibStorageDir) {
        pcl::console::print_error(
            "No calibration storage directory provided. --calibstorage <path>\n");
        return 1;
    }

    /* 3d marker properties */
    float boardWidth, boardHeight;
    if (pcl::console::parse(argc, argv, "-bw", boardWidth) == -1) {
        print_usage();
        return 1;
    }
    if (pcl::console::parse(argc, argv, "-bh", boardHeight) == -1) {
        print_usage();
        return 1;
    }

    /* 2d marker properties */
    int patternWidth, patternHeight;
    if (pcl::console::parse(argc, argv, "-pw", patternWidth) == -1) {
        print_usage();
        return 1;
    }
    if (pcl::console::parse(argc, argv, "-ph", patternHeight) == -1) {
        print_usage();
        return 1;
    }

    /* tolerance for the 3d marker */
    float boardSigma = 0.05;
    pcl::console::parse(argc, argv, "-bs", boardSigma);

    /* confirm */
    bool doConfirm = true;
    doConfirm = !pcl::console::find_switch(argc, argv, "-nc");
    /*****************************/


    /* the calibdation storage */
    CalibStorageContract calibStorage(appopt.calibStorageDir);

    /* setup visualizer */
    CalibVisualizer visualizer;
    visualizer.start();

    /* image windows */
    //cv::namedWindow("camera", CV_WINDOW_AUTOSIZE);

    /* the captured file pairs */
    std::vector<CalibStorageContract::FilePair> pairPaths;
    pairPaths = calibStorage.getExtrinsicFiles();

    for (auto fpair : pairPaths) {
        /* load image from file and display */
        cv::Mat img = cv::imread(fpair.first, CV_LOAD_IMAGE_COLOR);
        cv::imshow("camera", img);
        // spin once
        cv::waitKey(1);

        /* load cloud from file and display */
        Cloud::Ptr cloud( new Cloud);
        pcl::io::loadPCDFile<PointT>(fpair.second, *cloud);
        visualizer.setMainCloud(cloud);


        /*** extract points from pattern ***/
        cv::Size patternSize(patternWidth, patternHeight);
        std::vector<cv::Point2f> patternCorners;
        bool found2d = cv::findChessboardCorners
            (img, patternSize, patternCorners,
             CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | 
             CV_CALIB_CB_NORMALIZE_IMAGE); 

        /* display results on image */
        if (found2d) {
            printSimpleInfo("[Chessboard] ", "found.\n");

            /* draw the 2d pattern */
            cv::drawChessboardCorners(img, patternSize, patternCorners, true);
            cv::imshow("camera", img);
            cv::waitKey(1);
        }


        /*** extract 3d marker ***/
        PointT boardPoint;
        PlaneMarker<PointT> planeMarker(boardWidth, boardHeight, boardSigma);
        bool found3d = planeMarker.computeMarkerCenter(cloud, boardPoint);

        /* display results in cloud viewer */
        if (found3d) {
            printSimpleInfo("[BoardMarker] ", "found.\n");

            /* draw the 3d marker */
            //TODO
        }


        /* a marker pair was found */
        if (found2d && found3d) {
            printBrightInfo("[Marker Pair] ", "found.\n");

            /* set the center point of the 3d marker */
            cv::Point3f point3d;
            point3d.x = planeMarker.centerPoint.x;
            point3d.y = planeMarker.centerPoint.y;
            point3d.z = planeMarker.centerPoint.z;

            /* get the middle point from 2d pattern */
            cv::Point2f point2d = patternCorners[patternCorners.size()/2];


            /* query the user if the sample should be stored */
            bool doAddPointPair = true;
            if (doConfirm) {
                printBrightInfo("[Add Marker Pair] ", "store? [y]/[n].");
                while (!doAddPointPair) {
                    int key = cv::waitKey(1);
                    if (key == KEY_y) {
                        doAddPointPair = true;
                        break;
                    } else if (key == KEY_n) {
                        doAddPointPair = false;
                        break;
                    } else if (key > 0) {
                        printBrightInfo("[Add Marker Pair] ", 
                                "store? [y]/[n].");
                    }
                }
            }

            /* store point pair */
            if (doAddPointPair) {
                calibStorage.addExtrinsicPointPair(point3d, point2d);
            }

        } else { // no marker pair
            printWarning("[Marker Pair] ", "not found.\n");
            cv::waitKey(1);
            if (doConfirm) {
                while (true) {
                    printWarning("[Continue] ", "Hit any key.\n");
                    int key = cv::waitKey(1);
                    if (key > 0) {
                        break;
                    }
                }
            }
        }

    } // for all pairs

    /* finaly save to calibration storage */
    calibStorage.saveExtrinsicPointPairs();
}

