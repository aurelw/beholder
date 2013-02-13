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

int main() {

    /* the basic rig config */
    RigConfig rc;
    rc.loadFromFile("rigconfig_core_pullup.xml");

    /* camera parameter data */
    CameraParameters::Ptr cameraParameters(new CameraParameters(rc));
    cameraParameters->print();

    /* pose tracking for the camera*/
    PoseTrackerKinfu::Ptr poseTracker(new PoseTrackerKinfu(rc));
    poseTracker->start();

    /* range finder with kinfu cloud provider */
    CloudProvider<pcl::PointXYZ>::Ptr rangeimageProvider = poseTracker->getKinfu();
    RangeFinder<pcl::PointXYZ>::Ptr rangeFinder(new RangeFinder<pcl::PointXYZ>(rc));
    rangeFinder->setCloudSource(rangeimageProvider);

    /* viewfinder to extract pois from the scene */
    ViewFinderRangeImage<pcl::PointXYZ> viewFinder(0.5);
    viewFinder.setRangeFinder(rangeFinder);
    viewFinder.setCameraParameters(cameraParameters);

    /* the focus motor */
    FocusMotor::Ptr focusMotor(new FocusMotorTwoPolys(rc));

    /* create a focus controller */
    FocusControllerSinglePoint::Ptr fCtrlSingle(new FocusControllerSinglePoint(
                cameraParameters, poseTracker));
    fCtrlSingle->setIdentifier("SingeTracker0");
    fCtrlSingle->setPriority(5);
    fCtrlSingle->start();

    /* focus control */
    CentralFocusControl fControl;
    fControl.setFocusMotor(focusMotor);
    fControl.addFocusController(fCtrlSingle);
    fControl.start();

    /* simulate a main loop */
    for (int i=0; i<50; i++) {
        poseTracker->getPose();
        fControl.focusDistance();
        fControl.getFocusedPoint();
        fControl.activlyControlled();
        sleep(0.55);
    }

    /* save cloud from kinfu */
    pcl::PCDWriter writer;
    CloudProvider<pcl::PointXYZ>::CloudPtr outCloud(
            new CloudProvider<pcl::PointXYZ>::Cloud(
                *rangeimageProvider->getLastCloud()));
    writer.writeASCII("kinfu_cloud_pullup.pcd", *outCloud );

    viewFinder.compute();
    pcl::RangeImagePlanar::Ptr rangeImage = viewFinder.getRangeImage();
    RangeImageWriter riWriter(rangeImage);
    riWriter.save("rangeimage_pullup.png");
    viewFinder.getMiddlePoint();
    
}

