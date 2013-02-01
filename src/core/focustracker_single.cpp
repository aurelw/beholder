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

#include "focustracker_single.h"


void FocusTrackerSingle::init() {
    FocusTracker::init();
    viewFinder.setCamera(camPara);
    messure.setCamera(camPara);

    currentFPlane = 1.0f;
    trackedPointVisible = false;

    trackedPoint.x = -5;
    trackedPoint.y = -5;
    trackedPoint.z = -5;

}


void FocusTrackerSingle::reset() {
    currentFPlane = 1.0f;
    trackedPointVisible = false;
    trackedPoint.x = -5;
    trackedPoint.y = -5;
    trackedPoint.z = -5;
}


float FocusTrackerSingle::getDistance() {
    Eigen::Affine3f dslrPose = kinfu->getLastPose() * staticExtrinsic;
    Eigen::Vector3f tp(trackedPoint.x, trackedPoint.y, trackedPoint.z);

    messure.setPose(dslrPose);
    if (messure.isVisible(tp)) {
        currentFPlane = messure.getFocalPlaneDistance(tp);
        trackedPointVisible = true;
    } else {
        trackedPointVisible = false;
    }

    return currentFPlane;
}


bool FocusTrackerSingle::isVisible() {
    return trackedPointVisible;
}


pcl::PointXYZ FocusTrackerSingle::pick() {
    point_cloud_ptr = kinfu->getLastFrameCloud();

    viewFinder.setInputCloud(point_cloud_ptr);
    viewFinder.setTransform(kinfu->getLastPose());
    viewFinder.compute();
    trackedPoint = viewFinder.getMiddlePoint();

    std::cout << trackedPoint.x << "," << trackedPoint.y << "," << trackedPoint.z << std::endl;

    return trackedPoint;
}


