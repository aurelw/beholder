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

/* core */
#include "basicappoptions.h"
#include "calib_visualizer.h"
#include "console_utils.h"
#include "mathutils.h"

/* calib */
#include "calibstorage_contract.h"
#include "plane_marker.h"
#include "calib_utils.h"

#define KEY_ESC 27
#define KEY_ENTER 13
#define KEY_c 99
#define KEY_y 121
#define KEY_n 110

typedef typename pcl::PointXYZRGBA PointT;
typedef typename pcl::PointCloud<PointT> Cloud;


void print_usage() {
    pcl::console::print_error("[Error] -bw <boardWidth> -bh <boardHeight> -pw <patternWidth> -ph <patternHeight> -ps <patternSquareSize> [-bs <boardSigma>] [-nc] [-nosave]\n");
}


int main(int argc, char **argv) {

    /*** command line arguments ***/
    BasicAppOptions appopt(argc, argv);

    if (!appopt.gotCalibStorageDir) {
        pcl::console::print_error(
            "No calibration storage provided. --calibstorage <path>\n");
        exit(1);
    }

    if (!appopt.gotRigConfigFile) {
        pcl::console::print_error(
            "No rig config provided. --rigconfig <file>\n");
        exit(1);
    }

    /* 3d marker properties */
    float boardWidth, boardHeight;
    if (pcl::console::parse(argc, argv, "-bw", boardWidth) == -1) {
        print_usage();
        exit(1);
    }
    if (pcl::console::parse(argc, argv, "-bh", boardHeight) == -1) {
        print_usage();
        exit(1);
    }

    /* 2d marker properties */
    int patternWidth, patternHeight;
    float squareSize;
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

    /* tolerance for the 3d marker */
    float boardSigma = 0.05;
    pcl::console::parse(argc, argv, "-bs", boardSigma);

    /* confirm */
    bool doConfirm = true;
    doConfirm = !pcl::console::find_switch(argc, argv, "-nc");

    bool storeResults = true;
    storeResults = !pcl::console::find_switch(argc, argv, "-nosave");

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

    /* the rig config */
    RigConfig rigConfig;
    rigConfig.loadFromFile(appopt.rigConfigFile);

    /* setup visualizer */
    CalibVisualizer visualizer;
    visualizer.start();

    /* image windows */
    cv::namedWindow("camera", CV_WINDOW_NORMAL|CV_GUI_EXPANDED);

    /* the captured file pairs */
    std::vector<CalibStorageContract::FilePair> pairPaths;
    pairPaths = calibStorage.getExtrinsicFiles();

    int pairCounter = 0;
    for (auto fpair : pairPaths) {
        /* print loop information */
        std::stringstream ss;
        ss << "[Find Pairs] ==== PROCESSING PAIR " 
            << ++pairCounter << "/" << pairPaths.size()
            << " ====" << std::endl;
        printSimpleInfo(ss.str());

        /* load image from file */
        cv::Mat img = cv::imread(fpair.first, CV_LOAD_IMAGE_COLOR);

        /* load cloud from file */
        Cloud::Ptr cloud( new Cloud);
        pcl::io::loadPCDFile<PointT>(fpair.second, *cloud);

        /* display image */
        cv::imshow("camera", img);
        // spin once
        cv::waitKey(1);

        /* dispaly cloud */
        visualizer.setMainCloud(cloud);
        visualizer.setDrawMarker(false);


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
            /* draw center point */
            cv::Point2f patternCenter = 
                patternCorners[patternCorners.size()/2];
                cv::circle(img, patternCenter, 15.0, 
                        cv::Scalar(0, 255, 0), -1);
        } else {
            printWarning("[Chessboard] ", "not found.\n");
        }
        /* draw the 2d pattern */
        cv::drawChessboardCorners(img, patternSize, 
                patternCorners, found2d);
        cv::imshow("camera", img);
        cv::waitKey(1);


        /*** extract 3d marker ***/
        PointT bPoint;
        PlaneMarker<PointT> planeMarker(boardWidth, boardHeight, boardSigma);
        bool found3d = planeMarker.computeMarkerCenter(cloud, bPoint);
        pcl::PointXYZ boardPoint = pointRGBAtoXYZ(bPoint);

        /* display results in cloud viewer */
        if (found3d) {
            printSimpleInfo("[BoardMarker] ", "found.\n");
            /* draw the 3d marker */
            visualizer.setMarkerCenter(boardPoint, true);
            visualizer.setDrawMarker(true);
        } else {
            printWarning("[BoardMarker] ", "not found.\n");
            visualizer.setMarkerCenter(boardPoint, false);
        }


        /* a marker pair was found */
        if (found2d && found3d) {
            printBrightInfo("[Marker Pair] ", "found.\n");

            /* set the center point of the 3d marker */
            cv::Point3f point3d;
            point3d.x = boardPoint.x;
            point3d.y = boardPoint.y;
            point3d.z = boardPoint.z;

            /* get the middle point from 2d pattern */
            cv::Point2f point2d = patternCorners[patternCorners.size()/2];

            /* get the transformation of the pattern
             * and it's middle point in 3d */
            cv::Mat patternTvec, patternRvec;
            getPatternTransform(patternSize, squareSize,
                    patternCorners, 
                    rigConfig.cameraMatrix, 
                    rigConfig.cameraDistortionCoefficients,
                    patternTvec, patternRvec);

            //FIXME transform point properly
            // center point of the marker in object space
            cv::Point3f objectPoint(0.0, 0.0, 0.0);
            // point in camera space
            cv::Point3f patternPoint3d;

            cv::Point3f t;
            t.x = patternTvec.at<double>(0,0);
            t.y = patternTvec.at<double>(1,0);
            t.z = patternTvec.at<double>(2,0);

            cv::Mat rmat;
            cv::Rodrigues(patternRvec, rmat);
            cv::Matx33f rotation_matrix = rmat;

            //pre rotation
            // -> FIXME check again
            // at the moment this looks correct.
            // also float/double convertion is correct
            patternPoint3d = (rotation_matrix * objectPoint) + t;

            std::stringstream ss;
            ss << "Chessboard at tvec:" << patternTvec << " rvec: ";
            ss << patternRvec << std::endl;
            printSimpleInfo("[Chessboard 3D] ", ss.str());


            /* query the user if the sample should be stored */
            bool doAddPointPair = true;
            if (doConfirm) {
                printBrightInfo("[Add Marker Pair] ", "store? [y]/[n]\n");
                for (;;) {
                    int key = cv::waitKey(1);
                    if (key == KEY_y) {
                        doAddPointPair = true;
                        break;
                    } else if (key == KEY_n) {
                        doAddPointPair = false;
                        break;
                    } else if (key > 0) {
                        printBrightInfo("[Add Marker Pair] ", 
                                "store? [y]/[n]\n");
                    }
                }
            }

            /* store point pair */
            if (doAddPointPair) {
                calibStorage.addExtrinsicPointPair(point3d, point2d);
                calibStorage.addExtrinsicPointPair3d(
                        point3d, patternPoint3d);
                //FIXME don't store every time, only on exit
                if (storeResults) {
                    calibStorage.saveExtrinsicPointPairs();
                    calibStorage.saveExtrinsicPointPairs3d();
                }
            }

        } else { // no marker pair
            printWarning("[Marker Pair] ", "not found.\n");
            cv::waitKey(1);

            /* confirm before next iteration */
            if (doConfirm) {
                printWarning("[Continue] ", "Hit any key.\n");
                while (true) {
                    int key = cv::waitKey(1);
                    if (key > 0) {
                        break;
                    }
                }
            }
        }

    } // for all pairs

    /* finaly save to calibration storage */
    if (storeResults) {
        calibStorage.saveExtrinsicPointPairs();
        calibStorage.saveExtrinsicPointPairs3d();
    }
}

