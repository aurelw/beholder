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

#include "mathutils.h"


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

        /*** Interpolate between the two pois by interpolating 
         * between the projectes point in image space. Also
         * check bounds and only project on a valid line segment.
         ***/
        Eigen::Vector3f pOneVec(pOne.x, pOne.y, pOne.z);
        Eigen::Vector3f pTwoVec(pTwo.x, pTwo.y, pTwo.z);

        /* project the two points into the image frame */
        Eigen::Vector2f projOne = focusMessure.projectPoint(pOneVec);
        Eigen::Vector2f projTwo = focusMessure.projectPoint(pTwoVec);

        /* the interpolation line between two projtected pois in image space */
        Eigen::ParametrizedLine<float, 2> projInterLine =
            Eigen::ParametrizedLine<float,2>::Through(projOne, projTwo);

        //FIXME check why point is wrongly projected at high angles of view!
        /* project the middle point of the camera frame onto the interline */
        Eigen::Vector2f frameMiddle(0.5, 0.5);
        Eigen::Vector2f projFPoint = projInterLine.projection(frameMiddle);

        /* projected from 2d image point to 3d vector in world space 
         * and intersect with the original line between the pois */
        Eigen::Vector3f projViewVec = focusMessure.projectPoint(projFPoint);
        Eigen::Vector3f interPoint = 
            focusMessure.getClosestOnSegment(pOneVec, pTwoVec, projViewVec);

        //TODO non linear interpolation by modulating function on ration (sin)
        //
        // example f(x) = x + (sin(2*pi*x) * 0.1)
        //                     ^ 0-1 period  ^ shape parameter
        //  this goes out parameter with a too high shape parameter
        //  look at sqrt() function modulation

        /* get final focal point from tracked point */
        Eigen::Vector3f focalPointInterpolated = 
            focusMessure.getFocalPoint(interPoint);

        /* final focus data */
        canTrack = true;
        distance = focusMessure.getFocalPlaneDistance(focalPointInterpolated);
        trackedPoint = vecToPoint(interPoint); 
    }

    } // lock pois

    /* only set focus data when updated */
    if (canTrack || activeState) {
        setFocusData(canTrack, distance, trackedPoint);
    }
}

