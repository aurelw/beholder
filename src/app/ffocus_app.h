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

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>

#include "cameraparameters.h"
#include "rig_config.h"
#include "rangefinder.h"
#include "viewfinder.h"
#include "rangeimagewriter.h"
#include "ffocus_messure.h"
#include "transfer1d1d_constantpoly.h"
#include "focusmotor.h"
#include "focusmotor_twopolys.h"
#include "posetrackerkinfu.h"
#include "centralfocuscontrol.h"
#include "focuscontroller_singlepoint.h"
#include "focuscontroller_multinearest.h"
#include "focuscontroller_interpolate.h"
#include "poicollection.h"

#include "focustracker_single.h"
#include "focustracker_multi.h"
#include "focustracker_nearest.h"
#include "focustracker_interpolate.h"

#include "ffocus_visualizer.h"



typedef pcl::PointXYZ PointType;


class FFocusApp {

    public:
       
        FFocusApp(RigConfig::Ptr rigConf, std::string trackerType="default", bool driveFocus="false", std::string motorDevicePath="/dev/ttyUSB0",
                float volumeSize=3.0f, bool setPosition=false, float xpos=0.0, float ypos=0.0, float zpos=0.0);

        void spinOnce();
        void pickFocusPoint();

    private:

        void updateViewfinder();
        void doFocusPlane();

        RigConfig::Ptr rigConfig;
        PoseTrackerKinfu::Ptr poseTracker;
        RangeFinder<pcl::PointXYZ>::Ptr rangeFinder;
        ViewFinderRangeImage<pcl::PointXYZ>::Ptr viewFinder;
        CameraParameters::Ptr camParameters;
        CloudProvider<pcl::PointXYZ>::Ptr cloudProvider;

        FFocusVisualizer visualizer;

        /* new focus control */
        CentralFocusControl fControl;

        /* old focus trackers */
        FocusTrackerMulti *focusTracker;
        //FocusTrackerMulti focusTracker;
        //FocusTrackerNearest focusTracker;
        //FocusTrackerInterpolate focusTracker;

        pcl::RangeImagePlanar::Ptr rangeImage_ptr;
        pcl::PointCloud<PointType>::Ptr point_cloud_ptr;

        PointType focusPoint;
        float fPlaneDistance;

        /* motor */
        FocusMotor::Ptr fmotor;
        bool doDriveFocus;

        /* POIs */
        POICollection poiCollection;
        PointOfInterest::Ptr poi;

};

