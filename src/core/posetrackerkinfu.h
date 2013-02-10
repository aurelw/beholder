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

#ifndef __POSE_TRACKER_KINFU_H__
#define __POSE_TRACKER_KINFU_H__

#include <boost/thread.hpp>

#include "posetracker.h"
#include "rig_config.h"
#include "kinfuwrapper.h"

class PoseTrackerKinfu : public PoseTracker {

    public:

        typedef typename boost::shared_ptr<PoseTrackerKinfu> Ptr;
       
        PoseTrackerKinfu(const RigConfig &rigConfig);
        ~PoseTrackerKinfu();

        virtual void start();
        virtual void stop();
        virtual void reset();

        virtual KinfuWrapper::Ptr getKinfu();

    protected:

        KinfuWrapper::Ptr kinfuWrapper;
        pcl::gpu::CaptureOpenNI capture;

        /* kinect properties */
        int kinectFuId;
        cv::Mat exTranslation, exRotationVec;
        Eigen::Affine3f staticExtrinsic;

        /* tracking thread */
        void runTracking();
        boost::thread* thread;
        bool threadRunning;
        bool stopThread;

};

#endif

