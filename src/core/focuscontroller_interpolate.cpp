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

#include "focuscontroller_interpolate.h"


FocusControllerInterpolate::FocusControllerInterpolate(
    CameraParameters::Ptr camPar,
    PoseTracker::Ptr pTracker) :
        FocusControllerMultiPoint(camPar, pTracker)
{
}


void FocusControllerInterpolate::doTracking() {
    /* results */
    bool canTrack = false;
    float distance = 1.0;
    pcl::PointXYZ trackedPoint;

    /* update focus messure */
    focusMessure.setPose(poseTracker->getPose());

    { // lock pois

    boost::shared_lock<boost::shared_mutex> lock(poiMutex);
    /* check if there are enough pois to interpolate,
     * or only one to focus directly. */
    if (pois.size() == 1) { // only one

        trackedPoint = pois[0]->getPoint();
        Eigen::Vector3f tpv(trackedPoint.x, trackedPoint.y, trackedPoint.z);
        if (focusMessure.isVisible(tpv)) {
            canTrack = true;
            distance = focusMessure.getFocalPlaneDistance(tpv);
        }

    } else if (pois.size() > 1) { // interpolate
        /*** find the two closest pois to the viewing axis ***/
        pcl::PointXYZ pOne, pTwo;
        float dOne = 4242424242; // in a universe far, far away...
        float dTwo = 4242424242;
        for (PointOfInterest::Ptr poi : pois) {
            /* distance to viewing axis */
            pcl::PointXYZ p = poi->getPoint();
            Eigen::Vector3f pvec(p.x, p.y, p.z);
            Eigen::Vector3f focalPoint = focusMessure.getFocalPoint(pvec);
            float cDist = focusMessure.distancePP(focalPoint, pvec);

            /* sort into results */
            if (cDist < dOne) {
                dTwo = dOne;
                pTwo = pOne;
                dOne = cDist;
                pOne = p;
            } else if (cDist < dTwo) {
                dTwo = cDist;
                pTwo = p;
            }
        } // for

        /*** interpolation between the two pois is equal 
         *   to interpolate between the two focal points. ***/
        Eigen::Vector3f pOneVec(pOne.x, pOne.y, pOne.z);
        Eigen::Vector3f pTwoVec(pTwo.x, pTwo.y, pTwo.z);

        /* the ratio, essentially a linear function
         * between poiOne and poiTwo depending on viewing
         * angle */
        float ratio = dTwo/(dOne+dTwo);
        /* get the two focal points on the viewing vector */
        Eigen::Vector3f focalPointOne = 
            focusMessure.getFocalPoint(pOneVec);
        Eigen::Vector3f focalPointTwo = 
            focusMessure.getFocalPoint(pTwoVec);

        /* check the projected point */
        //TODO project, check region,..
        //TODO non linear interpolation by modulating function on ration (sin)
        //
        // example f(x) = x + (sin(2*pi*x) * 0.1)
        //                     ^ 0-1 period  ^ shape parameter
        //  this goes out parameter with a too high shape parameter
        //  look at sqrt() function modulation

        Eigen::Vector3f focalPointInterpolated =
            focalPointTwo + (focalPointOne - focalPointTwo)*ratio;

        /* final focus data */
        canTrack = true;
        distance = focusMessure.getFocalPlaneDistance(focalPointInterpolated);
        //FIXME maybe the real interpolated point and not closest?
        trackedPoint = pOne; 
    }

    } // lock pois

    /* only set focus data when updated */
    if (canTrack || activeState) {
        setFocusData(canTrack, distance, trackedPoint);
    }
}

