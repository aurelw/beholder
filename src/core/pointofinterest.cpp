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

#include "pointofinterest.h"


PointOfInterest::PointOfInterest(const std::string &id) :
    identifier(id)
{
}


void PointOfInterest::setPoint(const pcl::PointXYZ &p) {
    {
        boost::unique_lock<boost::shared_mutex> lock(mutex);
        point = p;
    }
}


void PointOfInterest::setPoint(const pcl::PointXYZ &p, 
        PointOfInterest::Ptr eventOrigin) 
{
    {
        boost::unique_lock<boost::shared_mutex> lock(mutex);
        point = p;
    }
    changeSignal(eventOrigin);
}


pcl::PointXYZ PointOfInterest::getPoint() {
    boost::shared_lock<boost::shared_mutex> lock(mutex);
    return point;
}


std::string PointOfInterest::getIdentifier() {
    return identifier;
}


PointOfInterest::connection_t PointOfInterest::connectToChange(
        changeSigT::slot_function_type slot) 
{
    return changeSignal.connect(slot);
}

