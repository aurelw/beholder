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

#include "ffocus_visualizer.h"

#include <iostream>
#include <string>

using namespace std;
using namespace pcl;


void FFocusVisualizer::setEnvironmentCloud(PointCloudPtr cloud) {
    envCloud = cloud;
    if (envColorHandler_ptr == NULL) {
        envColorHandler_ptr = new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange>(vfRangeImg, 220, 220, 220);
    }
    toggleUpdate = true;
}


void FFocusVisualizer::setViewFinderRangeImage(RangeImagePtr ri) {
    vfRangeImg = ri;
    if (vfColorHandler_ptr == NULL) {
        vfColorHandler_ptr = new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange>(vfRangeImg, 0, 0, 255);
    }
    toggleUpdate = true;
}


void FFocusVisualizer::setCameraPose(const Eigen::Affine3f& pose) {
    cameraPose = pose;
    toggleUpdateCoordinates = true;
}


void FFocusVisualizer::setRangeFinderExtrinsic(const Eigen::Affine3f& ex) {
    // the extrinsic marks the transformation from the kinect to the
    // nain camera coordinate frame. Invert the transformation here
    // to be easier to use and downard compatible.
    //FIXME keep ^^^ in mind!
    rangeFinderExtrinsic = ex.inverse();
    toggleUpdateCoordinates = true;
}


void FFocusVisualizer::setFocusPlaneDistance(float d) {
    fplaneDistance = d;
    toggleUpdateCoordinates = true;
}


void FFocusVisualizer::updateCoordinates() {

    /* add camera visualization */
    //FIXME bug in addCoordinateSystem, pitch/yaw/roll is wrong there
    //visualizer.removeCoordinateSystem();
    //visualizer.addCoordinateSystem(0.5, kinectPose*dslrExtrinsic);
    
    /* draw focal plane */
    Eigen::Vector3f focusPoint = cameraPose * 
        (Eigen::Vector3f(0,0,1) * fplaneDistance);
    Eigen::Vector3f dslrPoint =  cameraPose * Eigen::Vector3f(0,0,0);
    Eigen::Vector3f cordZ = cameraPose * (Eigen::Vector3f(0,0,1) * 0.3);
    PointXYZ focusXYZ, dslrXYZ, cordPZ;
    focusXYZ.getVector3fMap() = focusPoint;    
    dslrXYZ.getVector3fMap() = dslrPoint;    
    cordPZ.getVector3fMap() = cordZ;    

    visualizer.removeShape("focusPlane");
    visualizer.removeShape("focusPlaneLine");
    if (focusPointVisible) {
        visualizer.addSphere(focusXYZ, 0.02, 0, 255, 0, "focusPlane");
        visualizer.addLine(focusTrackPoint, focusXYZ, 0, 255, 255, "focusPlaneLine");
    } else {
        visualizer.addSphere(focusXYZ, 0.02, 200, 255, 0, "focusPlane");
    }

    /* draw line between camera coordinate system and focal plane */
    visualizer.removeShape("focusLine");
    visualizer.addLine(cordPZ, focusXYZ, 0, 255, 255, "focusLine");

    /* add kinect coordinate system */
    removeCoordinateSystem("kinectCoord");
    if (displayKinectCoordFrame) {
        addCoordinateSystem(0.3, cameraPose * rangeFinderExtrinsic, "kinectCoord");
    }

    /* add DSLR coordinate system */
    removeCoordinateSystem("dslrCoord");
    addCoordinateSystem(0.3, cameraPose, "dslrCoord");

    /* display the distance in meters */
    stringstream sstr;
    sstr.precision(3);
    sstr.setf(ios::fixed, ios::floatfield);
    sstr << fplaneDistance << "m";
    visualizer.removeText3D("distanceText");
    visualizer.addText3D(sstr.str(), cordPZ, 0.03, 1.0, 0.45, 0, "distanceText");
    
    toggleUpdateCoordinates = false;
}


void FFocusVisualizer::addCoordinateSystem(float scale, Eigen::Affine3f pose, string id) {
    Eigen::Vector3f cordX = pose * (Eigen::Vector3f(1,0,0) * scale);
    Eigen::Vector3f cordY = pose * (Eigen::Vector3f(0,1,0) * scale);
    Eigen::Vector3f cordZ = pose * (Eigen::Vector3f(0,0,1) * scale);
    Eigen::Vector3f origin = pose * Eigen::Vector3f(0,0,0);
    PointXYZ cordPX, cordPY, cordPZ, originXYZ;
    cordPX.getVector3fMap() = cordX;    
    cordPY.getVector3fMap() = cordY;    
    cordPZ.getVector3fMap() = cordZ;    
    originXYZ.getVector3fMap() = origin;    

    visualizer.addLine(originXYZ, cordPX, 255, 0, 0, "cordX" + id);
    visualizer.addLine(originXYZ, cordPY, 0, 255, 0, "cordY" + id);
    visualizer.addLine(originXYZ, cordPZ, 0, 0, 255, "cordZ" + id);
}


void FFocusVisualizer::removeCoordinateSystem(string id) {
    visualizer.removeShape("cordX" + id);
    visualizer.removeShape("cordY" + id );
    visualizer.removeShape("cordZ" + id);
}


void FFocusVisualizer::setFocusPoint(const PointT& point) {
    visualizer.removeShape("focusPoint");
    visualizer.addSphere(point, 0.05, 255, 0, 0, "focusPoint");
    focusTrackPoint = point;
}


void FFocusVisualizer::addSecondaryFocusPoint(const PointT& point) {
    visualizer.addSphere(point, 0.035, 255, 0, 0, "focusPoint_secondary_" + numSecondaryFocusPoints++);
}

void FFocusVisualizer::removeSecondaryFocusPoints() {
    for (;numSecondaryFocusPoints>0;) {
        visualizer.removeShape("focusPoint_secondary_" + --numSecondaryFocusPoints);
    }
}


void FFocusVisualizer::spinOnce() {

    if (toggleUpdate) {
        update();
    }

    if (toggleUpdateCoordinates) {
        updateCoordinates();
    }

    visualizer.spinOnce(25);
}


void FFocusVisualizer::update() {

    switch (displayMode) {

        case ENV_CLOUD:
            if (!visualizer.updatePointCloud(envCloud, "envCloud")) {
                visualizer.addPointCloud(envCloud, "envCloud");
            }
            visualizer.removePointCloud("dslrCloud");
            break;

        case DSLR_CLOUD:
            if (!visualizer.updatePointCloud(vfRangeImg, *vfColorHandler_ptr, "dslrCloud")) {
                visualizer.addPointCloud(vfRangeImg, *vfColorHandler_ptr, "dslrCloud");
            }
            visualizer.removePointCloud("envCloud");
            break;

        case ENV_DSLR_CLOUD:
            if (!visualizer.updatePointCloud(envCloud, "envCloud")) {
                visualizer.addPointCloud(envCloud, "envCloud");
            }
            if (!visualizer.updatePointCloud(vfRangeImg, *vfColorHandler_ptr, "dslrCloud")) {
                visualizer.addPointCloud(vfRangeImg, *vfColorHandler_ptr, "dslrCloud");
            }
            break;

        case NO_CLOUD:
            visualizer.removePointCloud("envCloud");
            visualizer.removePointCloud("dslrCloud");
            break;
    }

    toggleUpdate = false;
}


void FFocusVisualizer::setFocusPointVisibility(bool visible) {
    focusPointVisible = visible;
    toggleUpdateCoordinates = true;
}



void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* visualizer_void) {

    FFocusVisualizer* vis = (FFocusVisualizer*) visualizer_void;
    //cout << "keyboard event" << endl;

    if (event.getKeySym () == "t" && event.keyDown ()) {
        vis->toggleUpdate = true;
        vis->displayMode = (FFocusVisualizer::DisplayMode) ((vis->displayMode+1) % FFocusVisualizer::DISPLAY_MODE_SIZE);
    }

    if (event.getKeySym () == "c" && event.keyDown ()) {
        vis->capCloudFlag = true;
    }

    if (event.getKeySym () == "s" && event.keyDown ()) {
        vis->capStreamFlag = !vis->capStreamFlag;
    }

    if (event.getKeySym () == "a" && event.keyDown ()) {
        vis->capStreamAndCastFlag = !vis->capStreamAndCastFlag;
    }

    if (event.getKeySym () == "i" && event.keyDown ()) {
        vis->resetFlag = true;
    }

    if (event.getKeySym () == "v" && event.keyDown ()) {
        vis->pickFocusPointFlag = true;
    }

    if (event.getKeySym () == "b" && event.keyDown ()) {
        vis->multiTrackingResetFlag = true;
    }

    if (event.getKeySym () == "k" && event.keyDown ()) {
        vis->displayKinectCoordFrame = !vis->displayKinectCoordFrame;
    }
}

