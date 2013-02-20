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

#include "videostream.h"

#include "console_utils.h"


VideoStream::VideoStream(int captureDevice) {
    capDevice = captureDevice;
}


VideoStream::~VideoStream() {
    stop();
}


void VideoStream::setCropRegion(float x, float y, float xx, float yy) {
    roi_x = x;
    roi_y = y;
    roi_xx = xx;
    roi_yy = yy;
    useROI = true;
}


void VideoStream::setResolution(int width, int height) {
    frameWidth = width;
    frameHeight = height;
}


cv::Mat VideoStream::getFrame() {
    boost::shared_lock<boost::shared_mutex> lock(mutex);
    return currentFrame.clone();
}


void VideoStream::start() {
    if (!threadRunning) {

        std::stringstream msg;
        msg << " Streaming from camera device " << capDevice << std::endl;
        printSimpleInfo("[VideoStream]", msg.str()); 

        /* set up streaming */
        vCap.open(capDevice);
        if (!vCap.isOpened()) {
            pcl::console::print_error("[VideoStream] Can't capture from camera!\n");
            return;
        }

        /* set additional camera parameters */
        vCap.set(CV_CAP_PROP_FRAME_WIDTH, frameWidth);
        vCap.set(CV_CAP_PROP_FRAME_HEIGHT, frameHeight);

        /* start capturing thread */
        threadRunning = true;
        thread = new boost::thread(
                boost::bind( &VideoStream::runCapture, this ));
    }
}


void VideoStream::stop() {
    if (threadRunning) {
        /* stop thread */
        stopThread = true;
        thread->join();
        delete thread;
        stopThread = false;
        gotFrame = false;
        threadRunning = false;

        /* close streaming */
        vCap.release();
    }
}


bool VideoStream::waitForFirstFrame() {
    while (!gotFrame) {
        if (!threadRunning) {
            return false;
        }
        usleep(10);
    }
    return true;
}


void VideoStream::runCapture() {
    // used for double buffering
    cv::Mat localFrame;
    while (!stopThread) {
        vCap >> localFrame;
        gotFrame = true;
        {
            boost::unique_lock<boost::shared_mutex> lock(mutex);
            if (useROI) {
                cv::Size size = localFrame.size();
                cv::Rect cropROI(roi_x*size.width, roi_y*size.height,
                               roi_xx*size.width, roi_yy*size.height);
                currentFrame = localFrame(cropROI).clone();
            } else {
                currentFrame = localFrame.clone();
            }
        }
        update();
    }
}

