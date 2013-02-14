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

#include <pcl/io/pcd_io.h>

#include "rig_config.h"
#include "cameraparameters.h"
#include "focusmotor_twopolys.h"
#include "rangefinder.h"
#include "posetrackerkinfu.h"
#include "viewfinder_rangeimage.h"
#include "rangeimagewriter.h"
#include "centralfocuscontrol.h"
#include "focuscontroller_singlepoint.h"
#include "pointofinterest.h"

int main() {

    /* the basic rig config */
    RigConfig rc;
    rc.loadFromFile("rigconfig_core_pullup.xml");

    /* camera parameter data */
    CameraParameters::Ptr cameraParameters(new CameraParameters(rc));

    /* pose tracking for the camera*/
    PoseTrackerKinfu::Ptr poseTracker(new PoseTrackerKinfu(rc));
    poseTracker->start();

    /* create a focus controller */
    FocusControllerSinglePoint::Ptr fCtrlSingle(new FocusControllerSinglePoint(
                cameraParameters, poseTracker));
    fCtrlSingle->setIdentifier("SingeTracker0");
    fCtrlSingle->setPriority(5);
    fCtrlSingle->start();


    /* simulate a main loop */
    for (int i=0; i<50; i++) {
        poseTracker->getPose();
        fCtrlSingle->getFocusDistance();
        fCtrlSingle->getFocusedPoint();
        fCtrlSingle->canFocus();
        sleep(0.55);
    }
    std::cout << "can focus: " << fCtrlSingle->canFocus() << std::endl;

    pcl::PointXYZ point;
    point.x = 1.5;
    point.y = 1.5;
    point.z = 1.5;
    PointOfInterest::Ptr poi(new PointOfInterest("first"));
    poi->setPoint(point, poi);

    fCtrlSingle->addPOI(poi);
    sleep(1);
    std::cout << "can focus: " << fCtrlSingle->canFocus() << std::endl;
    fCtrlSingle->addPOI(poi);
    sleep(1);
    std::cout << "can focus: " << fCtrlSingle->canFocus() << std::endl;

    PointOfInterest::Ptr poi2(new PointOfInterest("second"));
    poi2->setPoint(point, poi2);
    fCtrlSingle->addPOI(poi2);
    sleep(1);
    std::cout << "can focus: " << fCtrlSingle->canFocus() << std::endl;

    fCtrlSingle->stop();
}

