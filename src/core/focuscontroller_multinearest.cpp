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

#include "focuscontroller_multinearest.h"

FocusControllerMultiNearest::FocusControllerMultiNearest(
    CameraParameters::Ptr camPar,
    PoseTracker::Ptr pTracker) :
        FocusControllerSinglePoint(camPar, pTracker)
{
}


void FocusControllerMultiNearest::addPOI(PointOfInterest::Ptr p) {
    boost::unique_lock<boost::shared_mutex> lock(poiMutex);
    pois.push_back(p);
}


void FocusControllerMultiNearest::reset() {
    boost::unique_lock<boost::shared_mutex> lock(poiMutex);
    pois.clear();
}


void FocusControllerMultiNearest::doTracking() {

    bool canTrack = false;
    float distance = 1.0;
    pcl::PointXYZ trackedPoint;

    // to find the closest poi to the viewing vector
    float cDistanceToView = 424242;

    Eigen::Affine3f cameraPose = poseTracker->getPose();
    focusMessure.setPose(cameraPose);

    {
        boost::shared_lock<boost::shared_mutex> lock(poiMutex);
        for (PointOfInterest::Ptr poi : pois) {
            pcl::PointXYZ ctp = poi->getPoint();
            Eigen::Vector3f ctp_v(ctp.x, ctp.y, ctp.z);

            if (focusMessure.isVisible(ctp_v)) {
                Eigen::Vector3f focalPoint = focusMessure.getFocalPoint(ctp_v);
                float distanceToView = focusMessure.distancePP(ctp_v, focalPoint);
                if (distanceToView < cDistanceToView) {
                    cDistanceToView = distanceToView;
                    /* set focus data */
                    canTrack = true;
                    distance = focusMessure.getFocalPlaneDistance(ctp_v);
                    trackedPoint = ctp;
                } // new closest
            } // visible
        } // for
    }

    /* only set focus data when updated */
    if (canTrack || activeState) {
        setFocusData(canTrack, distance, trackedPoint);
    }

}

