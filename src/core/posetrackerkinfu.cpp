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

#include "posetrackerkinfu.h"

#include "mathutils.h"


PoseTrackerKinfu::PoseTrackerKinfu(const RigConfig &rigConfig) :
    threadRunning(false),
    stopThread(false)
{

    /* setup from rig config */
    if (rigConfig.hasTrackingCamera) { // load a seperate tracking camera
        //FIXME
        //kinectFuId = std::stoi(rigConfig.trackingCameraDeviceID); 
        kinectFuId = 1;
        exTranslation = rigConfig.trackingCameraExTranslation;
        exRotationVec = rigConfig.trackingCameraExRotationVec;
        staticExtrinsic = transRotVecToAffine3f(exTranslation, exRotationVec);
    } else { // use range finder for tracking
        //FIXME
        //kinectFuId = std::stoi(rigConfig.rangefinderDeviceID); 
        kinectFuId = 0;
        exTranslation = rigConfig.rangefinderExTranslation;
        exRotationVec = rigConfig.rangefinderExRotationVec;
        staticExtrinsic = transRotVecToAffine3f(exTranslation, exRotationVec);
    }

    /* setup capture interface */
    capture.open(kinectFuId);
    // register to rgb image
    capture.setRegistration(true);
    //FIXME this should be read from device interface
    capture.depth_focal_length_VGA = 525;

    /* setup kinfu tracking */
    kinfuWrapper.reset( new KinfuWrapper );
    kinfuWrapper->setCaptureSource(capture);
    kinfuWrapper->init();
}


PoseTrackerKinfu::~PoseTrackerKinfu() {
    stop();
}


void PoseTrackerKinfu::start() {
    if (!threadRunning) {
        threadRunning = true;
        thread = new boost::thread(
                boost::bind( &PoseTrackerKinfu::runTracking, this ));
    }
}


void PoseTrackerKinfu::stop() {
    /* terminate the working thread */
    if (threadRunning) {
        stopThread = true;
        thread->join();
        delete thread;
        stopThread = false;
        threadRunning = false;
    }
}


void PoseTrackerKinfu::runTracking() {
    while (!stopThread) {
        kinfuWrapper->spinOnce();
        //FIXME mutex
        /* The pose is in the tracker coordinate frame.
         * The extrinsic marks the transformation from
         * the camera frame to the tracking frame.
         * Transform with the inverse to get the pose
         * of the camera. 
         * */
        currentPose = kinfuWrapper->getLastPose() *
                staticExtrinsic.inverse();
    }
}


void PoseTrackerKinfu::reset() {
    stop();
    //FIXME propper reset
    start();
}


KinfuWrapper::Ptr PoseTrackerKinfu::getKinfu() {
    return kinfuWrapper;
};

