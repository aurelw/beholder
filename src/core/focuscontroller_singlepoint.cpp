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

#include "focuscontroller_singlepoint.h"


FocusControllerSinglePoint::FocusControllerSinglePoint(
        CameraParameters::Ptr camPar,
        PoseTracker::Ptr pTracker)
{
    camParameters = camPar;
    poseTracker = pTracker; 
    focusMessure.setCameraParameters(camParameters);
}


FocusControllerSinglePoint::~FocusControllerSinglePoint() {
    stop();
}


void FocusControllerSinglePoint::start() {
    if (!threadRunning) {
        threadRunning = true;
        thread = new boost::thread(
                boost::bind( &FocusControllerSinglePoint::runThread, this ));
    }
}


void FocusControllerSinglePoint::stop() {
    if (threadRunning) {
        /* stop the thread */
        stopThread = true;
        thread->join();
        delete thread;
        threadRunning = false;
        stopThread = false;

        /* delegate focus control */
        activeState = false;
    }
}


void FocusControllerSinglePoint::reset() {
    boost::unique_lock<boost::shared_mutex> lock(poiMutex);
    poi.reset();
}


void FocusControllerSinglePoint::addPOI(PointOfInterest::Ptr p) {
    boost::unique_lock<boost::shared_mutex> lock(poiMutex);
    poi = p;
}


void FocusControllerSinglePoint::runThread() {
    while (!stopThread) {
        doTracking();
        boost::this_thread::sleep(
                boost::posix_time::milliseconds(threadSleepMillis));
    }
}


void FocusControllerSinglePoint::doTracking() {

    bool canTrack = false;
    float distance = 1.0;

    Eigen::Affine3f cameraPose = poseTracker->getPose();
    focusMessure.setPose(cameraPose);

    /* get the point to track safely */
    pcl::PointXYZ trackedPoint;
    bool gotPoint = false;
    {
        boost::shared_lock<boost::shared_mutex> lock(poiMutex);
        if (PointOfInterest::Ptr p = poi.lock()) {
            trackedPoint = p->getPoint();
            gotPoint = true;
        }
    }

    if (gotPoint) {
        Eigen::Vector3f tp(trackedPoint.x, trackedPoint.y, trackedPoint.z);
        if (focusMessure.isVisible(tp)) {
            distance = focusMessure.getFocalPlaneDistance(tp);
            canTrack = true;
        }
    }

    // write the result
    setFocusData(canTrack, distance, trackedPoint);
}

