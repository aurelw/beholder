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

#include "basic_visualizer.h"

#include "console_utils.h"


BasicVisualizer::BasicVisualizer() {
}


BasicVisualizer::~BasicVisualizer() {
    stop();
}


void BasicVisualizer::initVisualizer() {
    if (visualizer == NULL) {
        return;
    }

    /* set some basic visualizer properties */
    visualizer->setBackgroundColor(0.1, 0.1, 0.1);

    registerCallbacks();
}


void keyboardCallback(const pcl::visualization::KeyboardEvent &event, void* cvis) {
}

void BasicVisualizer::registerCallbacks() {
    visualizer->registerKeyboardCallback(keyboardCallback, (void*) this);
}


void BasicVisualizer::spinOnce() {
    /* if there is no visualizer present, create on in this thread */
    if (visualizer == NULL) {
        visualizer.reset (new pcl::visualization::PCLVisualizer);
        initVisualizer();
    }

    if (!threadRunning) {
        updateAllProperties();
        visualizer->spinOnce(10, false);
    }
}


void BasicVisualizer::start(bool waitInit) {
    if (!threadRunning) {
        threadRunning = true;
        thread = new boost::thread(
            boost::bind( &BasicVisualizer::runVisualizer, this ));

        if (waitInit) {
            while(!isInitialized);
        }
    }
}


void BasicVisualizer::stop() {
    if (threadRunning) {
        stopThread = true;
        thread->join();
        delete thread;
        stopThread = false;
        threadRunning = false;
        isInitialized = false;
    }  
}


void BasicVisualizer::setMainCloud(RGBCloud::ConstPtr cloud) {
    boost::unique_lock<boost::shared_mutex> lock(mutex);
    mainCloud = cloud;
    flagUpdateMainCloud = true;
}


void BasicVisualizer::updateMainCloud() {
    if (!flagUpdateMainCloud) return;

    if (mainCloudAdded) {
        if (drawMainCloud && mainCloud != NULL) {
            visualizer->updatePointCloud(mainCloud, "mainCloud");
        } else {
            visualizer->removePointCloud("mainCloud");
            mainCloudAdded = false;
        }
    } else if (drawMainCloud && mainCloud != NULL) {
        visualizer->addPointCloud(mainCloud, "mainCloud");
        mainCloudAdded = true;
    }
}


void BasicVisualizer::updateAllProperties() {
    updateMainCloud();
}


void BasicVisualizer::runVisualizer() {

    visualizer.reset (new pcl::visualization::PCLVisualizer);
    initVisualizer();
    isInitialized = true;

    while (!stopThread) {
        {
            boost::shared_lock<boost::shared_mutex> lock(mutex);
            
            /* update visualizer properties */
            updateAllProperties();

            visualizer->spinOnce(10, false);
        }
        // give setters the chance to acquire a lock
        usleep(5);
    }
}


void BasicVisualizer::addCoordinateSystem( 
        const Eigen::Affine3f &pose, 
        const std::string &id, float scale,
        bool drawMarker, float mRed, float mGreen, float mBlue) 
{
    Eigen::Vector3f cordX = pose * (Eigen::Vector3f(1,0,0) * scale);
    Eigen::Vector3f cordY = pose * (Eigen::Vector3f(0,1,0) * scale);
    Eigen::Vector3f cordZ = pose * (Eigen::Vector3f(0,0,1) * scale);
    Eigen::Vector3f origin = pose * Eigen::Vector3f(0,0,0);
    pcl::PointXYZ cordPX, cordPY, cordPZ, originXYZ;
    cordPX.getVector3fMap() = cordX;    
    cordPY.getVector3fMap() = cordY;    
    cordPZ.getVector3fMap() = cordZ;    
    originXYZ.getVector3fMap() = origin;    

    visualizer->addLine(originXYZ, cordPX, 255, 0, 0, "cordX" + id);
    visualizer->addLine(originXYZ, cordPY, 0, 255, 0, "cordY" + id);
    visualizer->addLine(originXYZ, cordPZ, 0, 0, 255, "cordZ" + id);

    /* draw a marker for the coordinate frame*/
    float mScale = 0;
    if (drawMarker) {
        mScale = 0.02;
    }
    visualizer->addSphere(originXYZ, mScale, 
            mRed, mGreen, mBlue,
            "cordMarker" + id);
}


void BasicVisualizer::removeCoordinateSystem(
        const std::string &id) 
{
    visualizer->removeShape("cordX" + id);
    visualizer->removeShape("cordY" + id );
    visualizer->removeShape("cordZ" + id);
    visualizer->removeShape("cordMarker" + id);
}

