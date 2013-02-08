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
        kinectFuId = rigConfig.trackingCameraDeviceID; 
        exTranslation = rigConfig.trackingCameraExTranslation;
        exRotationVec = rigConfig.trackingCameraExRotationVec;
        staticExtrinsic = transRotVecToAffine3f(exTranslation, exRotationVec);
    } else { // use range finder for tracking
        kinectFuId = rigConfig.rangefinderDeviceID; 
        exTranslation = rigConfig.rangefinderExTranslation;
        exRotationVec = rigConfig.rangefinderExRotationVec;
        staticExtrinsic = transRotVecToAffine3f(exTranslation, exRotationVec);
    }

    /* setup kinfu tracking */
    capture.open(kinectFuId);
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
    stopThread = true;
    thread->join();
    delete thread;
    stopThread = false;
    threadRunning = false;
}


void PoseTrackerKinfu::runTracking() {
    while (!stopThread) {
        kinfuWrapper->spinOnce();
        //FIXME mutex
        currentPose = staticExtrinsic * kinfuWrapper->getLastPose();
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

