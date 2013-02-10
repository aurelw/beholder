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

#include "focustracker_interpolate.h"


void FocusTrackerInterpolate::init() {
    FocusTrackerMulti::init();
}


float FocusTrackerInterpolate::getDistance() {
    Eigen::Affine3f dslrPose = poseTracker->getPose();
    messure.setPose(dslrPose);

    trackedPointVisible = false;

    /* find first and second nearest track point */
    int numFound = 0;
    float p0_d = 4200023; //distance to current p0 (neirest track point)
    float p1_d = 4200023; //^^^^
    pcl::PointXYZ p0, p1;

    for (int i=0; i<trackedPoints.size(); i++) {
        pcl::PointXYZ ctp = trackedPoints[i];
        Eigen::Vector3f ctp_v(ctp.x, ctp.y, ctp.z);

        /* at least one point visible but use all points for interpolation */
        if (messure.isVisible(ctp_v)) {
            trackedPointVisible = true;

            Eigen::Vector3f cfp_v = messure.getFocalPoint(ctp_v);
            float ctp_distance = messure.distancePP(ctp_v, cfp_v);

            /* this point is the nearest so far*/
            if (p0_d > ctp_distance) {
                numFound++;
                p1_d = p0_d;
                p0_d = ctp_distance;
                p1 = p0;
                p0 = ctp;
            } else if (p1_d > ctp_distance) {    /* this point is the second nearest so far */
                numFound++;
                p1_d = ctp_distance; 
                p1 = ctp;
            }
        }//if visible
    }//for

    if (numFound == 0) { //no tracked point visible, keep current status
        return currentFPlane;
    } else if (numFound == 1) { //only one point, don't interpolate
        trackedPoint = p0;
        Eigen::Vector3f tp_v(trackedPoint.x, trackedPoint.y, trackedPoint.z);
        currentFPlane = messure.getFocalPlaneDistance(tp_v);
        return currentFPlane; 
    } else { // interpolate between the two nearest neighbours
        Eigen::Vector3f p0_v(p0.x, p0.y, p0.z);
        Eigen::Vector3f p1_v(p1.x, p1.y, p1.z);
        Eigen::Vector3f p0_focusPoint = messure.getFocalPoint(p0_v);
        Eigen::Vector3f p1_focusPoint = messure.getFocalPoint(p1_v);

        /* interpolate */
        float ratio = p1_d/(p1_d+p0_d);
        std::cout << "p0_d: " << p0_d << " p1_d: " << p1_d << " ratio: " << ratio << std::endl;
        Eigen::Vector3f fp_ipl_v = p1_focusPoint + (p0_focusPoint - p1_focusPoint)*ratio;

        /* set the nearest neighbor as tracked point */
        trackedPoint = p0;

        currentFPlane = messure.getFocalPlaneDistance(fp_ipl_v);
        return currentFPlane; 
    }

}

