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

#include <boost/thread.hpp>

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
#include "openni_interface.h"

/* calib */
#include "contrast_autofocus.h"
#include "calibstorage_contract.h"

/* gui */
#include "calib_visualizer.h"

#define KEY_ESC 27
#define KEY_f 102
#define KEY_u 117
#define KEY_d 100
#define KEY_p 112
#define KEY_r 114
#define KEY_m 109


class CaptureFocusSamples {

    public:

        CaptureFocusSamples(const BasicAppOptions &appOpt, bool doVis3d=false);
        ~CaptureFocusSamples();

        bool init();
        void run();

    private:

        void querySamples(float upPos, bool doUp, 
                          float downPos, bool doDown,
                          float distance, bool validDistance);

        RigConfig rigConfig;
        CalibStorageContract calibStorage;

        VideoStream::Ptr vStream;
        ContrastAutofocus::Ptr cAutofocus;
        Motor::Ptr fMotor;

        CloudProvider<pcl::PointXYZRGBA>::Ptr cloudProvider;
        CameraParameters::Ptr camParas;
        RangeFinder<pcl::PointXYZRGBA>::Ptr rangeFinder;
        ViewFinderRangeImage<pcl::PointXYZRGBA>::Ptr viewFinder;

        /* cloud visualization */
        bool doVisualize3d;
        CalibVisualizer visualizer;
        boost::thread *thread;
        bool stopThread = false;
        void visualizerThread();
};


CaptureFocusSamples::CaptureFocusSamples(
        const BasicAppOptions &appopt, bool doVis3d) :
    calibStorage(appopt.calibStorageDir)
{
    rigConfig.loadFromFile(appopt.rigConfigFile);
    doVisualize3d = doVis3d;
}


CaptureFocusSamples::~CaptureFocusSamples() {
    stopThread = true;
    thread->join();
    delete(thread);
}


bool CaptureFocusSamples::init() {

    /* init video stream */
    vStream = createMainCameraVideoStream(rigConfig);
    vStream->start();
    vStream->waitForFirstFrame();
    sleep(1);

    /* focus drive motor */
    fMotor = createMotor(rigConfig, "default");
    fMotor->connect();

    /* for automatic focusing */
    cAutofocus.reset( new ContrastAutofocus(vStream, fMotor) );

    /* gui */
    cv::namedWindow("camera", CV_WINDOW_NORMAL|CV_GUI_EXPANDED);

    /* cloud interface */
    OpenNiInterface::Ptr oniIf(
            new OpenNiInterface(rigConfig.rangefinderDeviceID));
    if (!oniIf->init()) {
        pcl::console::print_error("Can't connect to cloud interface!\n");
        return false;
    }
    oniIf->waitForFirstFrame();
    cloudProvider = oniIf;

    /* setup range and viewfinder */
    rangeFinder.reset( new RangeFinder<pcl::PointXYZRGBA>(rigConfig) );
    rangeFinder->setCloudSource(cloudProvider);

    /* load camera parameters */
    camParas.reset( new CameraParameters(rigConfig) );

    /* finaly create a viewfinder */
    viewFinder.reset( new ViewFinderRangeImage<pcl::PointXYZRGBA>() );
    viewFinder->setCameraParameters(camParas);
    viewFinder->setRangeFinder(rangeFinder);

    /* cloud visualizer */
    if (doVisualize3d) {
        thread = new boost::thread(
            boost::bind( &CaptureFocusSamples::visualizerThread, this ));
    }

    return true;
}


void CaptureFocusSamples::visualizerThread() {
    visualizer.start();
    Eigen::Affine3f camPose = rangeFinder->getStaticExtrinsic().inverse();
    visualizer.addCoordinateSystem(camPose, "camPose", 1.0, true);

    while (!stopThread) {
        visualizer.setMainCloud(rangeFinder->getLastCloud());
        sleep(0.01);
    }
}


void CaptureFocusSamples::querySamples(float upPos, bool doUp,
        float downPos, bool doDown, 
        float distance, bool validDistance) 
{
    if (!validDistance) {
        distance = -1;
    }

    /* query for a valid distance if not provided */
    if (!validDistance) {
        do {
            std::cout << "Distance: ";
        } while ( (std::cin >> distance).fail());
    }

    while (true) {

        /* display sample info */
        printSimpleInfo("[SAMPLE]  ");
        if (doUp) {
            std::cout << "Up-Pos: " << upPos << "  "; 
        }
        if (doDown) {
            std::cout << "Down-Pos: " << downPos << "  ";
        }
        std::cout << "Distance: " << distance << std::endl;

        /* query for command input */
        std::cout << "Store [y]/[n], Change distance [d]: ";
        std::string input;
        std::cin >> input;

        /* store samples */
        if (input == "y") {
            if (doUp) {
                calibStorage.addFocusSample(distance, upPos, false);
            }
            if (doDown) {
                calibStorage.addFocusSample(distance, downPos, true);
            }
            calibStorage.saveFocusSamples();
            break;

        /* discard samples */
        } else if (input == "n") {
            break;

        /* query for new distance */
        } else if (input == "d") {
            float newDistance;
            std::cout << "   Distance: ";
            if (! (std::cin >> newDistance).fail()) {
                distance = newDistance;
            }
        }

    } // while

}


void CaptureFocusSamples::run() {

    viewFinder->compute();

    for (;;) {
        
        /* get image from stream and display */
        cv::Mat img = vStream->getFrame().clone();
        cv::Mat gray_img = cAutofocus->cMessure.preprocess(img);
        cv::imshow("camera", gray_img);

        /* compute information for this frame */
        float contrast = cAutofocus->cMessure.messure(gray_img);
        float position = fMotor->getPosition();
        //FIXME faster method
        //viewFinder->compute();
        float distance; 
        viewFinder->getMiddleRange(distance);

        /* display frame information */
        std::cout << "Contrast Level: " << contrast 
                  << "  Position: " << position
                  << "  Distance: " << distance << std::endl;


        int key = cv::waitKey(2);
        if (key == KEY_ESC) {
            break;
        }

        if (key == KEY_f) {
            std::cout << "Focusing.... ";
            std::cout.flush();

            /* get updatedted distance */
            viewFinder->compute();
            bool validDistance = viewFinder->getMiddleRange(distance);

            float posUp, posDown;

            if (cAutofocus->focusBothDirections(posUp, posDown)) {
                std::cout << "done." << std::endl;
                querySamples(posUp, true, posDown, true, 
                        distance, validDistance);
            } else {
                std::cout << "failed." << std::endl;
            }
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

        /* manual sample */
        if (key == KEY_m) {
            /* get current distance */
            bool validDistance = viewFinder->getMiddleRange(distance);

            /* query for up or down */
            printSimpleInfo("[MANUAL-SAMPLE]\n");
            bool doUp = false;
            bool doDown = false;
            while (true) {
                std::cout << "Up or Down [u]/[d]: ";
                std::string input;
                std::cin >> input;
                if (input == "u") {
                    doUp = true;
                    break;
                } else if (input == "d") {
                    doDown = true;
                    break;
                }
            }

            /* query sample store */
            querySamples(position, doUp, position, doDown,
                    distance, validDistance);
        }
        
        /* update range information */
        if (key == KEY_r) {
            viewFinder->compute();
        }

    } // for - ever

    calibStorage.saveFocusSamples();
}




/* =============================== */

void print_usage() {
    std::cout << "--rigconfig <file> --calibstorage <dir> --vis3d" << std::endl;
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
                "No rigconfig provided. --rigconfig <file>\n");
        exit(1);
    }

    if (!appopt.gotCalibStorageDir) {
        pcl::console::print_error(
                "No calibstorage provided. --calibstorage <path>\n");
        exit(1);
    }

    bool doVis3d = pcl::console::find_switch(argc, argv, "--vis3d");
    /*****************************/

    CaptureFocusSamples captureFS(appopt, doVis3d);

    if (captureFS.init()) {
        captureFS.run();
    }
}

