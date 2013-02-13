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

#include "poicollection.h"


void POICollection::addPOI(PointOfInterest::Ptr poi) {
    //FIXME mutex
    {
        boost::unique_lock<boost::shared_mutex> lock(mutex);
        pois.push_back(poi);
    }
    signal(poi, EventT::NEW);
    poi->connectToChange(boost::bind(&POICollection::updateEventSlot, this, _1));
}


void POICollection::removePOI(PointOfInterest::Ptr poi) {
    {
        boost::unique_lock<boost::shared_mutex> lock(mutex);
        pois.erase(std::find(pois.begin(), pois.end(), poi));
    }
    signal(poi, EventT::DELETED);
}


void POICollection::updateEventSlot(PointOfInterest::Ptr poi) {
    signal(poi, EventT::CHANGE);
}

