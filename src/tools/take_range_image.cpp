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

#include "rig_config.h"
#include "cameraparameters.h"
#include "rangefinder.h"
#include "posetrackerkinfu.h"
#include "viewfinder_rangeimage.h"
#include "rangeimagewriter.h"
#include "basicappoptions.h"


void print_usage() {
    std::cout << "--rigconfig <file> [-o <outputfile>] [-m] [-s scale]" << std::endl;
}


int main(int argc, char **argv) {

    /*** command line arguments ***/
    BasicAppOptions appopt(argc, argv);

    if (!appopt.gotRigConfigFile) {
        pcl::console::print_error(
                "No rigconfig provided. --rigconfig <path>\n");
        exit(1);
    }

    /* ozutput file to write the range image to */
    std::string outputFile = "range_image.png"; 
    pcl::console::parse(argc, argv, "-o", outputFile);

    /* scale the range image (when sampling) */
    float scale = 0.5;
    pcl::console::parse(argc, argv, "-s", scale);

    /* wait for user input to take the range image */
    bool manualRelease = false;
    manualRelease = pcl::console::find_switch(argc, argv, "-m");

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

    /* pose tracking for the camera*/
    PoseTrackerKinfu::Ptr poseTracker(new PoseTrackerKinfu(rc));
    poseTracker->start();

    /* range finder with kinfu cloud provider */
    CloudProvider<pcl::PointXYZ>::Ptr rangeimageProvider = 
        poseTracker->getKinfu();
    RangeFinder<pcl::PointXYZ>::Ptr rangeFinder
        (new RangeFinder<pcl::PointXYZ>(rc));
    rangeFinder->setCloudSource(rangeimageProvider);

    /* viewfinder to take the range image */
    ViewFinderRangeImage<pcl::PointXYZ> viewFinder(scale);
    viewFinder.setRangeFinder(rangeFinder);
    viewFinder.setCameraParameters(cameraParameters);

    if (manualRelease) {
        std::cout << "Type and enter for release:";
        std::string inp;
        std::cin >> inp;
    } else {
        sleep(1);
    }

    /* take range image and store it */
    viewFinder.compute();
    pcl::RangeImagePlanar::Ptr rangeImage = viewFinder.getRangeImage();
    RangeImageWriter riWriter(rangeImage);
    riWriter.save(outputFile);
    viewFinder.getMiddlePoint();
    
}

