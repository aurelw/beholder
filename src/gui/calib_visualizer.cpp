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

CalibVisualizer::CalibVisualizer() {
    registerCallbacks();

    /* set some basic visualizer properties */
    visualizer.addCoordinateSystem(1.0);
    visualizer.setBackgroundColor(0.1, 0.1, 0.1);
}


CalibVisualizer::~CalibVisualizer() {
    stop();
}


void keyboardCallback(const pcl::visualization::KeyboardEvent &event, void* cvis) {
}

void CalibVisualizer::registerCallbacks() {
    visualizer.registerKeyboardCallback(keyboardCallback, (void*) this);
}


void CalibVisualizer::spinOnce() {
    if (!threadRunning) {
        visualizer.spinOnce(10, false);
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
    updateMainCloud();
}


void CalibVisualizer::updateMainCloud() {
    if (mainCloudAdded) {
        if (drawMainCloud && mainCloud != NULL) {
            visualizer.updatePointCloud(mainCloud, "mainCloud");
        } else {
            visualizer.removePointCloud("mainCloud");
            mainCloudAdded = false;
        }
    } else if (drawMainCloud && mainCloud != NULL) {
        visualizer.addPointCloud(mainCloud, "mainCloud");
        mainCloudAdded = true;
    }
}


void CalibVisualizer::runVisualizer() {
    while (!stopThread) {
        {
            boost::shared_lock<boost::shared_mutex> lock(mutex);
            visualizer.spinOnce(20, false);
        }
        // give setters the chance to acquire a lock
        usleep(5);
    }
}

