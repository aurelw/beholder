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

#ifndef __VIDEO_STREAM_H__
#define __VIDEO_STREAM_H__

#include <boost/thread.hpp>

#include "opencv2/opencv.hpp"

#include "update_signal.h"


class VideoStream : public UpdateSignal {

    public:

        typedef typename boost::shared_ptr<VideoStream> Ptr;

    public:

        VideoStream(int captureDevice);
        ~VideoStream();

        /* set some addition capture properties */
        virtual void setCropRegion(float x, float y, float xx, float yy);
        virtual void setResolution(int width, int height);

        virtual cv::Mat getFrame();

        void start();
        void stop();
        /* returns true if there are frames available */
        virtual bool waitForFirstFrame();

    protected:

        /* targeted resolution */
        int frameWidth = 640;
        int frameHeight = 480;

        /* roi */
        bool useROI = false;
        float roi_x, roi_y, roi_xx, roi_yy;

        /* capturing */
        int capDevice;
        cv::VideoCapture vCap;
        cv::Mat currentFrame;
        boost::shared_mutex mutex;
        bool gotFrame = false;

        /* threading */
        boost::thread *thread;
        bool threadRunning = false;
        bool stopThread = false;
        virtual void runCapture();
};

#endif

