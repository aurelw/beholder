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

#ifndef __FOCUS_CONTROLLER_SINGLE_POINT_H__
#define __FOCUS_CONTROLLER_SINGLE_POINT_H__

#include "focuscontroller.h"
#include "cameraparameters.h"
#include "posetracker.h"
#include "ffocus_messure.h"


class FocusControllerSinglePoint : public FocusController {

    public:

        FocusControllerSinglePoint(CameraParameters::Ptr camPar,
                PoseTracker::Ptr pTracker);

        ~FocusControllerSinglePoint();

        /* FocusController implementation */
        virtual void start();
        virtual void stop();
        virtual void reset();
        virtual void addPOI(PointOfInterest::Ptr p);

    protected:

        /* implementation of actual focus tracking */
        void doTracking();

        /* threading */
        boost::thread *thread;
        bool threadRunning = false;
        bool stopThread = false;
        virtual void runThread();
        float threadSleepMillis = 10;

        /* POI storage */
        boost::shared_mutex poiMutex;
        PointOfInterest::WeakPtr poi;

        /* helpers */
        CameraParameters::Ptr camParameters;
        PoseTracker::Ptr poseTracker;
        FFocusMessure focusMessure;
};

#endif
