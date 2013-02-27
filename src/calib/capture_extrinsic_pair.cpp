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

#include "calibstorage_contract.h"
#include "basicappoptions.h"
#include "calib_visualizer.h"
#include "console_utils.h"
#include "camerainterface_factory.h"
#include "cloudprovider.h"
#include "openni_interface.h"

#define KEY_ESC 27
#define KEY_ENTER 13
#define KEY_c 99


int main(int argc, char **argv) {

    /* command line arguments */
    BasicAppOptions appopt(argc, argv);

    if (!appopt.gotCalibStorageDir) {
        pcl::console::print_error(
            "No calibration storage directory provided. --calibstorage <path>\n");
        exit(1);
    }

    if (!appopt.gotRigConfigFile) {
        pcl::console::print_error(
            "No rig config provided. --rigconfig <file>\n");
        exit(1);
    }

    /* print usage info */
    printSimpleInfo("[Capture Pairs] \n", 
           "Focus windows 'capture' and hit [c] to capture a pair.\n[Esc] to quit.\n");

    /* the calibdation storage */
    CalibStorageContract calibStorage(appopt.calibStorageDir);

    /* rig config */
    RigConfig rigConfig;
    rigConfig.loadFromFile(appopt.rigConfigFile);

    /* camera capturing interfaces */
    CameraInterface::Ptr camIf = createCameraInterface(rigConfig);
    if (!camIf->checkConnection()) {
        pcl::console::print_error("Camera not connected!\n");
        exit(1);
    }

    /* cloud interface */
    OpenNiInterface::Ptr oniIf(
            new OpenNiInterface(rigConfig.rangefinderDeviceID));
    bool cloudConnected = oniIf->init();
    if (!cloudConnected) {
        pcl::console::print_error("Can't connect to cloud interface!\n");
        exit(1);
    }
    oniIf->waitForFirstFrame();
    CloudProvider<pcl::PointXYZRGBA>::Ptr cloudIf;
    cloudIf = oniIf;

    /* setup visualizer */
    CalibVisualizer visualizer;

    /* captcher windows */
    cv::namedWindow("capture", CV_WINDOW_NORMAL|CV_GUI_EXPANDED);

    /* the captured pair */
    cv::Mat img;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;

    for (;;) {

        /* update and display cloud */
        cloud = cloudIf->getCloudCopy();
        visualizer.setMainCloud(cloud);

        visualizer.spinOnce();

        /* handle input */
        int key = cv::waitKey(2);
        if (key == KEY_ESC) {
            break;
        } else if (key == KEY_c) {
            /* capture photo and display */
            img = camIf->captureImage();
            cv::Mat displayImg;
            displayImg = img.clone();
            cv::imshow("capture", displayImg);

            /* query again for storage */
            printSimpleInfo("[Capture]", 
                    " to store cloud/image hit [c] again, any other key to abort.\n");
            /* wait for another key */
            do { 
                key = cv::waitKey(1);
                visualizer.spinOnce();
            } while (key < 0);

            if (key == KEY_c) {
                calibStorage.addExtrinsicPairRGB(img, cloud);
            } else {
                printSimpleInfo("[Capture] ", "aborted.\n");
            }
        }

    }

}

