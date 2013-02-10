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

#include "focustracker_nearest.h"


void FocusTrackerNearest::init() {
    FocusTrackerMulti::init();
}


float FocusTrackerNearest::getDistance() {
    Eigen::Affine3f dslrPose = poseTracker->getPose();
    messure.setPose(dslrPose);

    float cMinDistance = 4200023;
    trackedPointVisible = false;

    for (int i=0; i<trackedPoints.size(); i++) {
        pcl::PointXYZ ctp = trackedPoints[i];
        Eigen::Vector3f ctp_v(ctp.x, ctp.y, ctp.z);

        if (messure.isVisible(ctp_v)) {
            trackedPointVisible = true;
            Eigen::Vector3f cfp_v = messure.getFocalPoint(ctp_v);
            float ctp_distance = messure.distancePP(ctp_v, cfp_v);

            // if this point is closer than those before
            if (cMinDistance > ctp_distance) {
                cMinDistance = ctp_distance;
                trackedPoint = ctp;
            }
        }
    }

    /* compute the focal plane distance again */
    if (trackedPointVisible) {
        Eigen::Vector3f tp_v(trackedPoint.x, trackedPoint.y, trackedPoint.z);
        currentFPlane = messure.getFocalPlaneDistance(tp_v);
    }

    return currentFPlane;

}

