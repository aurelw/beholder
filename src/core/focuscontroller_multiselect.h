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

#ifndef __FOCUS_CONTROLLER_MULTI_SELECT_H__
#define __FOCUS_CONTROLLER_MULTI_SELECT_H__

#include "focuscontroller_multipoint.h"


class FocusControllerMutliSelect : public FocusControllerMultiPoint {

    public:

        FocusControllerMutliSelect(CameraParameters::Ptr camPar,
                PoseTracker::Ptr pTracker);

        virtual void next();
        virtual void prev();
        virtual void select(const std::string &poiID);

    protected:

        int selectedPoiIndex = 0;

};

#endif

