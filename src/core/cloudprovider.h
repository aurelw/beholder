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

#ifndef __CLOUD_PROVIDER_H__
#define __CLOUD_PROVIDER_H__

#include <pcl/common/common_headers.h>


template <class PointType>
class CloudProvider {

    public:

        typedef pcl::PointCloud<PointType> Cloud;
        typedef typename pcl::PointCloud<PointType>::Ptr CloudPtr;
        typedef typename pcl::PointCloud<PointType>::ConstPtr CloudConstPtr;
        typedef typename boost::shared_ptr<CloudProvider<PointType> > Ptr;

        virtual CloudConstPtr getLastCloud() = 0;
        virtual Eigen::Affine3f getCloudPose() = 0;

};

#endif

