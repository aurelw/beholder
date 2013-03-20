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

#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>

/* core */
#include "basicappoptions.h"
#include "rig_config.h"
#include "cameraparameters.h"
#include "rangefinder.h"
#include "viewfinder_rangeimage.h"
#include "videostream_factory.h"
#include "motor_factory.h"

/* calib */
#include "contrast_autofocus.h"

#define KEY_ESC 27
#define KEY_f 102
#define KEY_u 117
#define KEY_d 100
#define KEY_p 112


void print_usage() {
    std::cout << "--rigconfig <file>" << std::endl;
}


int main(int argc, char **argv) {

    /*** command line arguments ***/
    BasicAppOptions appopt(argc, argv);

    /* help */
    if (pcl::console::find_switch(argc, argv, "-h") ||
        pcl::console::find_switch(argc, argv, "--help")) 
    {
        print_usage();
        exit(0);
    }

    if (!appopt.gotRigConfigFile) {
        pcl::console::print_error(
                "No rigconfig provided. --rigconfig <path>\n");
        exit(1);
    }
    /*****************************/


    /* the basic rig config */
    RigConfig rc;
    rc.loadFromFile(appopt.rigConfigFile);

    /* create video stream */
    VideoStream::Ptr vStream = createMainCameraVideoStream(rc);
    vStream->start();
    vStream->waitForFirstFrame();
    sleep(1);

    /* focus driving motor */
#if 1
    Motor::Ptr fMotor = createMotor(rc, "default");
    fMotor->connect();
#else
    Motor::Ptr fMotor(NULL);
#endif

    ContrastAutofocus af(vStream, fMotor);

    cv::namedWindow("camera", CV_WINDOW_NORMAL|CV_GUI_EXPANDED);

    for (;;) {
        cv::Mat img = vStream->getFrame();
        img.convertTo(img, CV_32F);
        cv::Mat nimg;
        cv::normalize(img, nimg, 0, 1.0, CV_MINMAX);

        cv::imshow("camera", nimg);

        int key = cv::waitKey(2);
        if (key == KEY_ESC) {
            break;
        }

        if (key == KEY_f) {
            std::cout << "Focusing.... " << std::endl;
            float pos = af.focus(false);
            std::cout << "focused at position " << pos << std::endl;
            fMotor->setPosition(pos);
        }

        if (key == KEY_u) {
            fMotor->stepUp();
        }

        if (key == KEY_d) {
            fMotor->stepDown();
        }

        if (key == KEY_p) {
            std::cout << "Enter Position:# ";
            std::string input;
            std::cin >> input;
            std::stringstream ss;
            ss << input;
            float position;
            if (!(ss >> position).fail()) {
                fMotor->setPosition(position);
            }
        }

        float contrast = af.currentContrastLevel();
        float position = fMotor->getPosition();
        std::cout << "Contrast Level: " << contrast 
                  << " Position: " << position << std::endl;
    }

}

