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

int main() {

    RigConfig rc;
    rc.loadFromFile("rigconfig_core_pullup.xml");

    CameraParameters::Ptr cameraParameters(new CameraParameters(rc));
    cameraParameters->print();

    FocusMotorTwoPolys focusMotor(rc);

    PoseTrackerKinfu poseTracker(rc);
    poseTracker.start();

    CloudProvider<pcl::PointXYZ>::Ptr rangeimageProvider = poseTracker.getKinfu();

    RangeFinder<pcl::PointXYZ>::Ptr rangeFinder(new RangeFinder<pcl::PointXYZ>(rc));
    rangeFinder->setCloudSource(rangeimageProvider);

    ViewFinderRangeImage<pcl::PointXYZ> viewFinder(1.0);
    viewFinder.setRangeFinder(rangeFinder);
    viewFinder.setCameraParameters(cameraParameters);

    /* simulate a main loop */
    for (int i=0; i<50; i++) {
        poseTracker.getPose();
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
    //viewFinder.getMiddlePoint();
    
}

