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
#include "viewfinder.h"
#include "range_image_to_file.h"
#include "ffocus_visualizer.h"
#include "ffocus_messure.h"

#include "focustracker_single.h"
#include "focustracker_multi.h"
#include "focustracker_nearest.h"
#include "focustracker_interpolate.h"

#include "focusmotor.h"
#include "transfer1d1d_constantpoly.h"

#define WITH_KINFU

#ifdef WITH_KINFU
    #include "openni_capture.h"
    #include "kinfutrackerwrap.h"
#endif

typedef pcl::PointXYZ PointType;


class FFocusApp {

    public:
       
        FFocusApp(std::string trackerType="default", bool driveFocus="false", std::string motorDevicePath="/dev/ttyUSB0",
                float volumeSize=3.0f, bool setPosition=false, float xpos=0.0, float ypos=0.0, float zpos=0.0) :
            //transferF(),
            //motor(transferF, motorDevicePath),
            doDriveFocus(driveFocus),
            kinfu(volumeSize, setPosition, xpos, ypos, zpos)
        {
            /* open kinect device 0 for capturing */
            capture.open(0);
            kinfu.setCaptureSource(capture);
            kinfu.init();

            /* set camera parameters */
            cam.loadIntrinsicFromYAML("intrinsic.yaml");
            cam.loadExtrinsicFromXML("extrinsic.xml");

            /* setup viewfinder */
            viewFinder.setCamera(&cam);

            /* visualizer setup */
            visualizer.register_callbacks();
            visualizer.setDSLRExtrinsic(cam.getStaticExtrinsic());

            /* setup focus tracking */
            if (trackerType=="multi") {
                focusTracker = new FocusTrackerMulti();
            } else if (trackerType=="nearest") {
                focusTracker = new FocusTrackerNearest();
            } else if (trackerType=="interpolate") {
                focusTracker = new FocusTrackerInterpolate();
            } else { // default
                focusTracker = new FocusTrackerNearest();
            }

            focusTracker->setCameraParameters(&cam);
            focusTracker->setKinfu(&kinfu);
            focusTracker->init();

            /* init the motor interface */
            if (doDriveFocus) {
                //motor.connect();
            }

        }

        void spinOnce();
        void pickFocusPoint();

    private:

        void updateViewfinder();
        void doFocusPlane();

        pcl::gpu::CaptureOpenNI capture;
        KinfuTrackerWrap kinfu;
        ViewFinder viewFinder;
        CameraParameters cam;
        FFocusVisualizer visualizer;

        FocusTrackerMulti *focusTracker;
        //FocusTrackerMulti focusTracker;
        //FocusTrackerNearest focusTracker;
        //FocusTrackerInterpolate focusTracker;

        pcl::RangeImagePlanar::Ptr rangeImage_ptr;
        pcl::PointCloud<PointType>::Ptr point_cloud_ptr;

        PointType focusPoint;
        float fPlaneDistance;

        /*motor*/
        //Transfer1d1dConstantPoly transferF;
        //FocusMotor motor;
        bool doDriveFocus;

};

