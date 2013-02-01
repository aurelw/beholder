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

#include "ffocus_app.h"

void FFocusApp::pickFocusPoint() {
    pcl::PointXYZ fp = focusTracker->pick();
    visualizer.setFocusPoint(fp);
    /* in the case of multitracking */
    visualizer.addSecondaryFocusPoint(fp);
}


void FFocusApp::updateViewfinder() {
    viewFinder.setInputCloud(point_cloud_ptr);
    viewFinder.setTransform(kinfu.getLastPose());
    viewFinder.compute();
    rangeImage_ptr = viewFinder.getRangeImage();
    visualizer.setViewFinderRangeImage(rangeImage_ptr);

    pcl::PointXYZ testp = viewFinder.getMiddlePoint();
    std::cout << testp.x << "," << testp.y << "," << testp.z << std::endl;

}


void FFocusApp::doFocusPlane() {
    float distance = focusTracker->getDistance(); // focal plane distance in meter.
    motor.setDistance(distance);
    visualizer.setFocusPlaneDistance(distance);
    visualizer.setFocusPointVisibility(focusTracker->isVisible());
    visualizer.setFocusPoint(focusTracker->getTrackedPoint());
}


void FFocusApp::spinOnce() {

    kinfu.spinOnce();

    /* check hardware button interface */
    if (motor.getButtonStateChange()) {
        if (motor.pushedMarkPoint) {
            pickFocusPoint();
        }
    }

    /* update the focus point */
    if (visualizer.pickFocusPointFlag) {
        pickFocusPoint();
    }

    /* track the focusplane, visualize, set motor,... */
    doFocusPlane();

    /* update camera in the visualizer */
    Eigen::Affine3f lastPose = kinfu.getLastPose();
    visualizer.setKinectPose(lastPose);

    /* update the environment cloud */
    if (visualizer.capStreamFlag || visualizer.capStreamAndCastFlag || visualizer.capCloudFlag) {
        point_cloud_ptr = kinfu.getLastFrameCloud();
        visualizer.setEnvironmentCloud(point_cloud_ptr);
    }

    /* viewfinder cloud */
    if (visualizer.capStreamAndCastFlag || visualizer.capCloudFlag)  {
        updateViewfinder();
    }

    /* reset kinfu tracking */
    if (visualizer.resetFlag) {
        kinfu.reset();
    }

    /* reset of the the current focus tracker */
    if (visualizer.multiTrackingResetFlag) {
        visualizer.FFocusVisualizer::removeSecondaryFocusPoints();
        focusTracker->reset();
    }

    /* reset all flags */
    visualizer.resetFlag = false;
    visualizer.pickFocusPointFlag = false;
    visualizer.capCloudFlag = false;
    visualizer.multiTrackingResetFlag = false;

    /* process visualization */
    visualizer.spinOnce();
}



#include <pcl/console/parse.h>

using namespace std;


int main(int argc, char** argv) {

    /* the tracking type used */
    string trackerType = "nearest";
    pcl::console::parse (argc, argv, "-t", trackerType);

    string motorDevice = "/dev/ttyUSB0";
    pcl::console::parse (argc, argv, "-d", motorDevice);

    bool driveMotor = !pcl::console::find_switch(argc, argv, "--nodrive");

    /* just send the focus motor to a position and exit */
    int motor_pos;
    pcl::console::parse (argc, argv, "--position", motor_pos);
    if (pcl::console::find_switch(argc, argv, "--position")) {
        unsigned char rawpos = (unsigned char) motor_pos;
        std::cout << "Setting Motor Position: " << (int)rawpos << endl;
        /* send bypte to motor */
        Transfer1d1dConstantPoly transferF;
        FocusMotor motor(transferF, motorDevice);
        motor.connect();
        motor.sendRawBytePos(rawpos);
        return 0;
    }


    /* position */
    float xpos = 0.0f; 
    float ypos = 0.0f;
    float zpos = 0.0f;
    bool setPosition=false;
    if (pcl::console::find_switch(argc, argv, "-x") || 
            pcl::console::find_switch(argc, argv, "-y") ||
            pcl::console::find_switch(argc, argv, "-z") ) 
    {
        pcl::console::parse (argc, argv, "-x", xpos);
        pcl::console::parse (argc, argv, "-y", ypos);
        pcl::console::parse (argc, argv, "-z", zpos);
        setPosition = true;
    }

    std::cout << "Position: " << setPosition << std::endl;

    /* volume size */
    float volumeSize = 3.0f;
    pcl::console::parse (argc, argv, "-s", volumeSize);

    
    /* run the app */
    FFocusApp app(trackerType, driveMotor, motorDevice, volumeSize, setPosition, xpos, ypos, zpos);

    while (true) {
        app.spinOnce();
    }

}
