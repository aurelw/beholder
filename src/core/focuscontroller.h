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

#ifndef __FOCUS_CONTROLLER_H__
#define __FOCUS_CONTROLLER_H__

#include <string>

#include <pcl/common/common_headers.h>

#include <boost/thread.hpp>

#include "pointofinterest.h"

class FocusController {

    public:

        typedef typename boost::shared_ptr<FocusController> Ptr;

        /* priority to other FocusControllers */
        virtual void setPriority(int pr);
        virtual int getPriority();
        virtual void setIdentifier(const std::string &id);
        virtual std::string getIdentifier();

        /* run, resume, pause and reset focus control */
        virtual void start() = 0;
        virtual void stop() = 0;
        virtual void reset() = 0;

        /* is the controller currently able to control the focus */
        virtual bool canFocus();

        /* output of focus control */
        virtual float getFocusDistance();
        /* the point which gets actually focused.
         * not used for control, but for visualization.
         * may be invalid */
        virtual pcl::PointXYZ getFocusedPoint();

        /* additional, a FocusController may use POIs in the scene
         * to control and track focus. */
        virtual void addPOI(PointOfInterest::Ptr poi);

    protected:

        /* focusing data and info */
        void setFocusData(bool active, float distance, const pcl::PointXYZ& point);
        boost::shared_mutex focusMutex;

        std::string identifier;
        int priority;

        /* us setFocusData to set those safely */
        bool activeState = false;
        float focusDistance = 1.0;
        pcl::PointXYZ focusedPoint;

};

#endif

