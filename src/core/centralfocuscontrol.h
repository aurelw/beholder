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

#ifndef __CENTRAL_FOCUS_CONTROL_H__
#define __CENTRAL_FOCUS_CONTROL_H__

#include <vector>
#include <map>

#include <pcl/common/common_headers.h>

#include <boost/thread.hpp>

#include "focuscontroller.h"
#include "focusmotor.h"


class CentralFocusControl {

    public:

        typedef typename boost::shared_ptr<CentralFocusControl> Ptr;

        CentralFocusControl();
        ~CentralFocusControl();
        
        /* init and setup */
        void setFocusMotor(FocusMotor::Ptr fMotor);

        void addFocusController(FocusController::Ptr fController);
        void removeFocusController(const std::string &fCtrId);
        void addPOI(PointOfInterest::Ptr poi);
        void addPOI(PointOfInterest::Ptr poi, const std::string &fCtrId);

        /* selection of specific FocusController */
        void selectController(const std::string &fCtrId);
        /* deselect all FocusControllers, fall back to priorities */
        void deselectController();

        /* depatch focus control */
        void start();
        void stop();

        /**** information about current focuse *****/
        float focusDistance();
        pcl::PointXYZ getFocusedPoint();
        /* is focus acitly controlled or just defaults to a value */
        bool activlyControlled();

    private:

        /* spin method */
        void doFocusControl();

        void doFocusControllers();
        void doMotor();

        /* focus information */
        boost::shared_mutex focusInfoMutex;
        bool isActivlyControlled = false;
        float currentDistance = 1.0; // meter
        pcl::PointXYZ focusedPoint;

        /* FocusControllers */
        boost::shared_mutex focusControllersMutex;
        std::map<std::string, FocusController::Ptr> focusControllers;
        typedef std::pair<int, std::string> PriorityPair;
        std::vector<PriorityPair> prioritities;
        void rebuildPriorities();

        /* selected controller id */
        std::string selectedControllerId = "None";

        /* Motor */
        FocusMotor::Ptr focusMotor;

        /* threading */
        boost::thread* thread;
        bool threadRunning = false;
        bool stopThread = false;
        void runControl();
        int threadSleepMillis = 10;

};

#endif

