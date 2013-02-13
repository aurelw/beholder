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

#include "focuscontroller.h"

void FocusController::setPriority(int pr) {
    priority = pr;
}


int FocusController::getPriority() {
    return priority;
}


void FocusController::setIdentifier(const std::string &id) {
    identifier = id;
}


std::string FocusController::getIdentifier() {
    return identifier;
}


void FocusController::addPOI(PointOfInterest::Ptr poi) {
    // per default, don't handle any POIs
};


bool FocusController::canFocus() {
    boost::shared_lock<boost::shared_mutex> lock(focusMutex);
    return activeState;
}


float FocusController::getFocusDistance() {
    boost::shared_lock<boost::shared_mutex> lock(focusMutex);
    return focusDistance;
}


pcl::PointXYZ FocusController::getFocusedPoint() {
    boost::shared_lock<boost::shared_mutex> lock(focusMutex);
    return focusedPoint;
}


void FocusController::setFocusData(bool active, float distance, 
        const pcl::PointXYZ& point)
{
    boost::unique_lock<boost::shared_mutex> lock(focusMutex);
    activeState = active;
    focusDistance = distance;
    focusedPoint = point;
}
