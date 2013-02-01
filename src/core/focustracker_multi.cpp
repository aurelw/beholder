#include "focustracker_multi.h"

void FocusTrackerMulti::init() {
    FocusTrackerSingle::init();
}


pcl::PointXYZ FocusTrackerMulti::pick() {
    pcl::PointXYZ tp = FocusTrackerSingle::pick();
    trackedPoints.push_back(tp);
    return tp;
}


float FocusTrackerMulti::getDistance() {
    Eigen::Affine3f dslrPose = kinfu->getLastPose() * staticExtrinsic;
    messure.setPose(dslrPose);

    for (int i=0; i<trackedPoints.size(); i++) {
        pcl::PointXYZ ctp = trackedPoints[i];
        Eigen::Vector3f ctp_v(ctp.x, ctp.y, ctp.z);

        if (messure.isVisible(ctp_v)) {
            trackedPointVisible = true; 
            currentFPlane = messure.getFocalPlaneDistance(ctp_v);
            // set the currently tracked point
            trackedPoint = ctp;
            return currentFPlane;
        }
    }

    trackedPointVisible = false; 
    return currentFPlane;
}


void FocusTrackerMulti::reset() {
    trackedPoint.x = -1;
    trackedPoint.y = -1;
    trackedPoint.z = -1;
    trackedPoints.clear();
}


pcl::PointXYZ FocusTrackerMulti::getTrackedPoint() {
    return trackedPoint;
}


