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

#include "centralfocuscontrol.h"


CentralFocusControl::CentralFocusControl() {
}


CentralFocusControl::~CentralFocusControl() {
    stop();
}


void CentralFocusControl::setFocusMotor(FocusMotor::Ptr fMotor) {
    focusMotor = fMotor;
}


float CentralFocusControl::focusDistance() {
    boost::shared_lock<boost::shared_mutex> lock(focusInfoMutex);
    return currentDistance;
}


bool CentralFocusControl::activlyControlled() {
    boost::shared_lock<boost::shared_mutex> lock(focusInfoMutex);
    return isActivlyControlled;
}


pcl::PointXYZ CentralFocusControl::getFocusedPoint() {
    boost::shared_lock<boost::shared_mutex> lock(focusInfoMutex);
    return focusedPoint;
}


void CentralFocusControl::addFocusController(FocusController::Ptr fController) {
    auto mit = focusControllers.find(fController->getIdentifier());
    if (mit == focusControllers.end()) { // checks if the key is already present
        // lock focus controllers and priorities
        boost::unique_lock<boost::shared_mutex> lock(focusControllersMutex);

        /* insert the controller */
        focusControllers[fController->getIdentifier()] = fController;
        rebuildPriorities();

        /* run the controller */
        fController->start();
    }
}


void CentralFocusControl::removeFocusController(const std::string &fCtrId) {
    boost::unique_lock<boost::shared_mutex> lock(focusControllersMutex);
    focusControllers.erase(fCtrId);
    rebuildPriorities();
}


void CentralFocusControl::rebuildPriorities() {
    priorities.clear();

    for (auto &controllersEntry : focusControllers) {
        /* create a new priority entry from the focusController */
        PriorityPair priority;
        // the priority
        priority.first = controllersEntry.second->getPriority();
        // the identifier
        priority.second = controllersEntry.first;
        priorities.push_back(priority);
    }

    std::sort(priorities.begin(), priorities.end());
}


void CentralFocusControl::addPOI(PointOfInterest::Ptr poi) {
    boost::shared_lock<boost::shared_mutex> lock(focusControllersMutex);

    /* add POI to selected or most prioritized controller */
    auto mit = focusControllers.find(selectedControllerId);
    if (mit != focusControllers.end()) { // checks if the key is already present
        focusControllers[selectedControllerId]->addPOI(poi);
    } else {
        if (priorities.size() > 0) {
            std::string fCtrID = priorities[0].second;
            focusControllers[fCtrID]->addPOI(poi);
        }
    }
}


void CentralFocusControl::addPOI(PointOfInterest::Ptr poi, 
        const std::string &fCtrId)
{
    boost::shared_lock<boost::shared_mutex> lock(focusControllersMutex);
    //FIXME check existence
    focusControllers[fCtrId]->addPOI(poi);
}


void CentralFocusControl::selectController(const std::string &fCtrId) {
    boost::unique_lock<boost::shared_mutex> lock(focusControllersMutex);
    selectedControllerId = fCtrId;
}


void CentralFocusControl::deselectController() {
    boost::unique_lock<boost::shared_mutex> lock(focusControllersMutex);
    selectedControllerId = "None";
}


void CentralFocusControl::doFocusControllers() {
    bool activeState = false;

    //lock priorities and focus controllers
    boost::shared_lock<boost::shared_mutex> lock(focusControllersMutex);

    /* if there is a selected focus controller use it */
    if (focusControllers.find(selectedControllerId) != focusControllers.end()) {

        FocusController::Ptr fCtr = focusControllers[selectedControllerId];
        if (fCtr->canFocus()) { // is the controller active?
            boost::unique_lock<boost::shared_mutex> lock(focusInfoMutex);
            activeState = true;
            currentDistance = fCtr->getFocusDistance();
            focusedPoint = fCtr->getFocusedPoint();
            isActivlyControlled = true;
        }

    } else { // select from priorities 

        for (PriorityPair &pr : priorities) {
            // get the focus controller with the highest priority
            FocusController::Ptr fCtr = focusControllers[pr.second];

            if (fCtr->canFocus()) { // is the controller active?
                boost::unique_lock<boost::shared_mutex> lock(focusInfoMutex);
                activeState = true;
                currentDistance = fCtr->getFocusDistance();
                focusedPoint = fCtr->getFocusedPoint();
                isActivlyControlled = true;
                break;
            }
        }

    } // else, no controller

    /* switchting to non-active focusing */
    if (activeState != isActivlyControlled) {
        boost::unique_lock<boost::shared_mutex> lock(focusInfoMutex);
        isActivlyControlled = activeState;
    }

}


void CentralFocusControl::doMotor() {
    if (focusMotor && isActivlyControlled) {
        focusMotor->setDistance(currentDistance);
    }
}


void CentralFocusControl::doFocusControl() {
    doFocusControllers();
    doMotor();
}


void CentralFocusControl::start() {
    if (!threadRunning) {
        //FIXME start focus controllers
        threadRunning = true;
        thread = new boost::thread(
                boost::bind( &CentralFocusControl::runControl, this ));
    }
}


void CentralFocusControl::stop() {
    if (threadRunning) {
        stopThread = true;
        thread->join();
        delete thread;
        stopThread = false;
        threadRunning = false;
        //FIXME stop focus controllers
    }
}


void CentralFocusControl::runControl() {
    while (!stopThread) {
        doFocusControl();
        boost::this_thread::sleep(
                boost::posix_time::milliseconds(threadSleepMillis));
    }
}

