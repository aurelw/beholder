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

#include "console_utils.h"


FFocusApp::FFocusApp(RigConfig::Ptr rigConf, std::string trackerType, 
        bool driveFocus, 
        std::string motorDevicePath,
        float volumeSize, 
        bool setPosition, 
        float xpos, float ypos, float zpos) :
            //transferF(),
            //motor(transferF, motorDevicePath),
            doDriveFocus(driveFocus)
{

    rfCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    /* set the RigConfig */
    rigConfig = rigConf;

    /* setup CameraParameters */
    camParameters.reset( new CameraParameters(*rigConfig) );

    /* start pose tracking */
    poseTracker.reset( new PoseTrackerKinfu(*rigConfig) );
    poseTracker->start();
    cloudProvider = poseTracker->getKinfu();

    /* setup RangeFinder */
    doSeparateRangeFinder = rigConfig->hasTrackingCamera;
    rangeFinder.reset( new RangeFinder<pcl::PointXYZ>(*rigConfig) );
    if (doSeparateRangeFinder) {
        printSimpleInfo("[FTrack]", " using separate kinect for tracking.\n");
        openNiInterface.reset( 
                new OpenNiInterfacePlain(rigConfig->rangefinderDeviceID) );
        openNiInterface->init();
        openNiInterface->waitForFirstFrame();
        rangeFinder->setCloudSource(openNiInterface);
    } else {
        printSimpleInfo("[FTrack]", 
                " same sensor for tracking and rangefinder.\n");
        rangeFinder->setCloudSource(poseTracker->getKinfu());
    }
    printSimpleInfo("[FTrack]", " rangefinder initialized.\n");

    /* setup ViewFinder */
    viewFinder.reset( new ViewFinderRangeImage<pcl::PointXYZ>(0.1) );
    viewFinder->setCameraParameters(camParameters);
    viewFinder->setRangeFinder(rangeFinder);

    /* visualizer setup */
    visualizer.register_callbacks();
    Eigen::Affine3f identity;
    identity.setIdentity();
    visualizer.setRangeFinderExtrinsic(rangeFinder->getStaticExtrinsic());

    /* setup focus tracking */
    /*
    if (trackerType=="multi") {
        focusTracker = new FocusTrackerMulti();
    } else if (trackerType=="nearest") {
        focusTracker = new FocusTrackerNearest();
    } else if (trackerType=="interpolate") {
        focusTracker = new FocusTrackerInterpolate();
    } else { // default
        focusTracker = new FocusTrackerNearest();
    }

    focusTracker->setCameraParameters(camParameters);
    focusTracker->setPoseTracker(poseTracker);
    focusTracker->setRangeFinder(rangeFinder);
    focusTracker->init();
    */

    /* init the motor interface */
    fmotor.reset(new FocusMotorTwoPolys(*rigConfig));

    /* setup a focus controller */
    FocusControllerSinglePoint::Ptr fctrl(
            new FocusControllerInterpolate(camParameters, poseTracker));
            //new FocusControllerMultiNearest(camParameters, poseTracker));
    fctrl->setPriority(5);
    fctrl->setIdentifier("SinglePoint");
    fctrl->start();

    /* setup central focus control */
    fControl.setFocusMotor(fmotor);
    fControl.addFocusController(fctrl);
    fControl.start();
}


void FFocusApp::pickFocusPoint() {
    DEBUG_PRINT("pickFocusPoint");
    /* get selected point */
    viewFinder->compute();
    pcl::PointXYZ fp = viewFinder->getMiddlePoint();

    /* viewfinding is done in global coordinate space
     * so transform to camera space */
    if (doSeparateRangeFinder) {
        Eigen::Vector3f fp_v = pointToVec(fp);
        fp_v = poseTracker->getPose() * rangeFinder->getStaticExtrinsic() * fp_v;
        fp = vecToPoint(fp_v);
    }

    //FIXME
    RangeImageWriter writer(
        viewFinder->getRangeImage());
    writer.save("pick_focus_ri.png");

    /* create a POI */
    poi.reset(new PointOfInterest("poi1"));
    poi->setPoint(fp);
    poiCollection.addPOI(poi);
    fControl.addPOI(poi); // also do this with event handling later

    /* handle to visualizer directly.
     * do this with signals and slots later */
    visualizer.setFocusPoint(fp);
    /* in the case of multitracking */
    visualizer.addSecondaryFocusPoint(fp);
}


void FFocusApp::updateViewfinder() {
    DEBUG_PRINT("updateViewfinder");
    viewFinder->compute();
    rangeImage_ptr = viewFinder->getRangeImage();
    visualizer.setViewFinderRangeImage(rangeImage_ptr);

    pcl::PointXYZ testp = viewFinder->getMiddlePoint();
    std::cout << testp.x << "," << testp.y << "," << testp.z << std::endl;
}


void FFocusApp::doFocusPlane() {
    float distance = fControl.focusDistance(); // focal plane distance in meter.
    visualizer.setFocusPlaneDistance(distance);
    visualizer.setFocusPointVisibility(fControl.activlyControlled());
    visualizer.setFocusPoint(fControl.getFocusedPoint());
}


void FFocusApp::spinOnce() {

    /* check hardware button interface */
    /*
    if (motor.getButtonStateChange()) {
        if (motor.pushedMarkPoint) {
            pickFocusPoint();
        }
    }
*/
    /* update the focus point */
    if (visualizer.pickFocusPointFlag) {
        pickFocusPoint();
    }

    /* track the focusplane, visualize, set motor,... */
    doFocusPlane();

    /* update camera in the visualizer */
    Eigen::Affine3f lastPose = poseTracker->getPose();
    visualizer.setCameraPose(lastPose);

    /* update the environment cloud */
    if (visualizer.capStreamFlag || visualizer.capStreamAndCastFlag || visualizer.capCloudFlag) {
        point_cloud_ptr = cloudProvider->getCloudCopy();
        visualizer.setEnvironmentCloud(point_cloud_ptr);

        /* display a separate cloud for the range finder */
        if (doSeparateRangeFinder) {
            pcl::PointCloud<pcl::PointXYZ>::ConstPtr tmpCld = 
                rangeFinder->getLastCloud();
            rfCloud.reset( new 
                    pcl::PointCloud<pcl::PointXYZ>(*tmpCld));

            /* transform to camera space */
            Eigen::Affine3f rfCloudPose =
                poseTracker->getPose() * rangeFinder->getStaticExtrinsic();
            pcl::transformPointCloud(*rfCloud, *rfCloud,
                    rfCloudPose);

            visualizer.setRangeFinderCloud(rfCloud);
        }
    }

    /* viewfinder cloud */
    if (visualizer.capStreamAndCastFlag || visualizer.capCloudFlag)  {
        updateViewfinder();
    }

    /* reset kinfu tracking */
    if (visualizer.resetFlag) {
        poseTracker->reset();
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

    /* the rig config */
    string rigConfigFile = "rig_config.xml";
    pcl::console::parse (argc, argv, "--rigconfig", rigConfigFile);

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
        //Transfer1d1dConstantPoly transferF;
        //FocusMotor motor(transferF, motorDevice);
        //motor.connect();
        //motor.sendRawBytePos(rawpos);
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

    /* load the rig config */
    RigConfig::Ptr rigConfig( new RigConfig());
    rigConfig->loadFromFile(rigConfigFile);

    /* run the app */
    FFocusApp app(rigConfig, trackerType, driveMotor, motorDevice, volumeSize, setPosition, xpos, ypos, zpos);

    while (true) {
        app.spinOnce();
    }

}
