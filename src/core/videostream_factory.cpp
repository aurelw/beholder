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

#include "videostream_factory.h"


VideoStream::Ptr createMainCameraVideoStream(const RigConfig &rc) {
    VideoStream::Ptr vStream;

    if (rc.hasStreaming) {
        if (rc.streamingType == "v4l") {
            /* get the streaming device */
            std::string deviceId = rc.streamingDevice;
            int deviceNum;
            std::istringstream(deviceId) >> deviceNum;
            vStream.reset( new VideoStream(deviceNum) );

            /* set crop region */
            if (rc.streamingDoCrop) {
                vStream->setCropRegion(
                        rc.streamingCropX, rc.streamingCropY,
                        rc.streamingCropXX, rc.streamingCropYY);
            }
            
            /* additional parameters */
            vStream->setResolution(rc.streamingWidth, rc.streamingHeight);
        } // v4l
    }

    return vStream;
}
