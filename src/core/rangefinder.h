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

#ifndef __RANGE_FINDER_H__
#define __RANGE_FINDER_H__

#include "rig_config.h"
#include "cloudprovider.h"
#include "mathutils.h"


template <class PointType>
class RangeFinder : public CloudProvider<PointType> {

    public:

        typedef typename CloudProvider<PointType>::Ptr ProviderPtr;
        typedef typename CloudProvider<PointType>::CloudConstPtr CloudConstPtr;
        typedef typename CloudProvider<PointType>::CloudPtr CloudPtr;
        typedef typename boost::shared_ptr<RangeFinder<PointType> > Ptr;

        RangeFinder(const RigConfig &rigConfig) {
            exTranslation = rigConfig.rangefinderExTranslation;
            exRotationVec = rigConfig.rangefinderExRotationVec;
            staticExtrinsic = transRotVecToAffine3f(exTranslation, exRotationVec);
        }
        
        void setCloudSource(const ProviderPtr &source) {
            cloudSource = source;
        }

        /* CloudProvider implementation */
        CloudConstPtr getLastCloud() {
            return cloudSource->getLastCloud();
        }

        CloudPtr getCloudCopy() {
            return cloudSource->getCloudCopy();
        }

        /* The pose of the point cloud in world space */
        Eigen::Affine3f getCloudPose() {
            return cloudSource->getCloudPose();
        }

        /* the pose of the range finder in respect to the camera */
        Eigen::Affine3f getStaticExtrinsic() {
            return staticExtrinsic;
        }

    private:

        cv::Mat exTranslation, exRotationVec;
        Eigen::Affine3f staticExtrinsic;

        ProviderPtr cloudSource;

};

class RangeFinderRGB : RangeFinder<pcl::PointXYZRGBA> {};
class RangeFinderDepth : RangeFinder<pcl::PointXYZ> {};


#endif

