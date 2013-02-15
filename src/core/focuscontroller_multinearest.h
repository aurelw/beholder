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

#ifndef __FOCUS_CONTROLLER_MULTI_NEAREST_H__
#define __FOCUS_CONTROLLER_MULTI_NEAREST_H__

#include "focuscontroller_singlepoint.h"


class FocusControllerMultiNearest : public FocusControllerSinglePoint {

    public:

        FocusControllerMultiNearest(CameraParameters::Ptr camPar,
                PoseTracker::Ptr pTracker);

        void addPOI(PointOfInterest::Ptr p) override;
        void reset() override;
        void doTracking() override;

    private:

        /* all active point of interests */
        std::vector<PointOfInterest::Ptr> pois;

};

#endif

