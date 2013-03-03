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

#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>

#include "rig_config.h"
#include "cameraparameters.h"
#include "basicappoptions.h"

#define KEY_ESC 27


void print_usage() {
    std::cout << "--rigconfig <file> --image <file> [-o <outputfile>] [-d] [-nc]" << std::endl;
}


int main(int argc, char **argv) {

    /*** command line arguments ***/
    BasicAppOptions appopt(argc, argv);

    if (!appopt.gotRigConfigFile) {
        pcl::console::print_error(
                "No rigconfig provided. --rigconfig <path>\n");
        exit(1);
    }

    /* the input image */
    std::string imageFile; 
    if (pcl::console::parse(argc, argv, "--image", imageFile) == -1) {
        print_usage();
        exit(1);
    }

    /* output file to write the range image to */
    std::string outputFile; 
    bool writeOutput = false;
    writeOutput = pcl::console::parse(argc, argv, "-o", outputFile);

    /* wait for user input to take the range image */
    bool doDistort = false;
    doDistort = pcl::console::find_switch(argc, argv, "-d");

    /* wait for user input to take the range image */
    bool doShow = true;
    doShow = !pcl::console::find_switch(argc, argv, "-nc");

    /* help */
    if (pcl::console::find_switch(argc, argv, "-h") ||
        pcl::console::find_switch(argc, argv, "--help")) 
    {
        print_usage();
        exit(0);
    }

    /*****************************/


    /* the basic rig config */
    RigConfig rc;
    rc.loadFromFile(appopt.rigConfigFile);

    /* camera parameter data */
    CameraParameters::Ptr cameraParameters(new CameraParameters(rc));
    cameraParameters->print();

    /* load the image and scale properly */
    cv::Mat img = cv::imread(imageFile, CV_LOAD_IMAGE_COLOR);
    cv::Size imgSize(rc.cameraImageWidth,
                     rc.cameraImageHeight);
    cv::resize(img, img, imgSize);


    /* do distortion if defined so */
    cv::Mat distCoeffs = rc.cameraDistortionCoefficients.clone();
    if (doDistort) {
        distCoeffs = distCoeffs * -1;
    }

    /* (un)distort */
    cv::Mat outImg;
    cv::undistort(img, outImg, rc.cameraMatrix, distCoeffs);

    /* show results */
    if (doShow) {
        cv::namedWindow("IN", CV_WINDOW_NORMAL|CV_GUI_EXPANDED);
        cv::namedWindow("OUT", CV_WINDOW_NORMAL|CV_GUI_EXPANDED);

        for (;;) {
            cv::imshow("OUT", outImg);
            cv::imshow("IN", img);
            int key = cv::waitKey(2);
            if (key == KEY_ESC) {
                break;
            }
        }
    }

    /* store undistorted image */
    if (writeOutput) {
        cv::imwrite(outputFile, outImg);
    }

}

