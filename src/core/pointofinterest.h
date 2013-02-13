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

#ifndef __POINT_OF_INTEREST_H__
#define __POINT_OF_INTEREST_H__

#include <pcl/common/common_headers.h>

#include <boost/thread.hpp>

#include "update_signal.h"


class PointOfInterest : public UpdateSignal {
    
    public:

        typedef typename boost::shared_ptr<PointOfInterest> Ptr;
        typedef typename boost::weak_ptr<PointOfInterest> WeakPtr;

        PointOfInterest();

        void setPoint(const pcl::PointXYZ &p);
        pcl::PointXYZ getPoint();

    private:

        boost::shared_mutex mutex;
        pcl::PointXYZ point;

};


#endif

