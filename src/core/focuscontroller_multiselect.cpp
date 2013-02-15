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

#include "focuscontroller_multiselect.h"


FocusControllerMutliSelect::FocusControllerMutliSelect(
    CameraParameters::Ptr camPar,
    PoseTracker::Ptr pTracker) :
        FocusControllerMultiPoint(camPar, pTracker)
{
}


void FocusControllerMutliSelect::next() {
    boost::unique_lock<boost::shared_mutex> lock(poiMutex);
    selectedPoiIndex++;
    if (!pois.empty()) {
        selectedPoiIndex %= pois.size();
        poi = pois[selectedPoiIndex];
    }
}


void FocusControllerMutliSelect::prev() {
    boost::unique_lock<boost::shared_mutex> lock(poiMutex);
    selectedPoiIndex--;
    if (!pois.empty()) {
        selectedPoiIndex %= pois.size();
        poi = pois[selectedPoiIndex];
    }
}


void FocusControllerMutliSelect::select(const std::string &poiID) {
    boost::unique_lock<boost::shared_mutex> lock(poiMutex);
    for (PointOfInterest::Ptr p : pois) {
        if (p->getIdentifier() == poiID) {
            poi = p;
            break;
        }
    }
}
