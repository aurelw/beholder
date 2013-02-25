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

#include "calib_visualizer.h"

#include "console_utils.h"


CalibVisualizer::CalibVisualizer() {
}


CalibVisualizer::~CalibVisualizer() {
    stop();
}


void CalibVisualizer::initVisualizer() {
    if (visualizer == NULL) {
        return;
    }

    /* set some basic visualizer properties */
    visualizer->addCoordinateSystem(0.5);
    visualizer->setBackgroundColor(0.1, 0.1, 0.1);

    /* other properties */
    //visualizer->addText("", 10, 10, "markerText");

    registerCallbacks();
}


void keyboardCallback(const pcl::visualization::KeyboardEvent &event, void* cvis) {
}

void CalibVisualizer::registerCallbacks() {
    visualizer->registerKeyboardCallback(keyboardCallback, (void*) this);
}


void CalibVisualizer::spinOnce() {
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


void CalibVisualizer::start() {
    if (!threadRunning) {
        threadRunning = true;
        thread = new boost::thread(
            boost::bind( &CalibVisualizer::runVisualizer, this ));
    }
}


void CalibVisualizer::stop() {
    if (threadRunning) {
        stopThread = true;
        thread->join();
        delete thread;
        stopThread = false;
        threadRunning = false;
    }  
}


void CalibVisualizer::setMainCloud(RGBCloud::Ptr cloud) {
    boost::unique_lock<boost::shared_mutex> lock(mutex);
    mainCloud = cloud;
    flagUpdateMainCloud = true;
}


void CalibVisualizer::updateMainCloud() {
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


void CalibVisualizer::setMarkerCenter(pcl::PointXYZ center, bool found) {
    boost::unique_lock<boost::shared_mutex> lock(mutex);
    markerCenter = center;
    foundMarker = found;
    flagUpdateMarker = true;
}


void CalibVisualizer::updateMarker() {
    if (!flagUpdateMarker) return;

    if (markerAdded) {
        visualizer->removeShape("markerCenter");
    }

    if (drawMarker) {
        if (foundMarker) {
            visualizer->addSphere(markerCenter, 0.01, 
                    1.0, 0.64, 0.0, //orange
                    "markerCenter");
            markerAdded = true;
            //visualizer->updateText("", 10, 10, "markerText");
        } else {
            //visualizer->updateText("Marker not found.", 10, 10, "markerText");
        }
    } else {
        //visualizer->updateText("", 10, 10, "markerText");
    }
}


void CalibVisualizer::updateAllProperties() {
    updateMainCloud();
    updateMarker();
}


void CalibVisualizer::runVisualizer() {

    visualizer.reset (new pcl::visualization::PCLVisualizer);
    initVisualizer();

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

