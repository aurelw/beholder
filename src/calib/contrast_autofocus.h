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

#ifndef __CONTRAST_AUTOFOCUS_H__
#define __CONTRAST_AUTOFOCUS_H__

#include "videostream.h"
#include "motor.h"

#include "contrast_messure.h"


class ContrastAutofocus {

    public:

        typedef boost::shared_ptr<ContrastAutofocus> Ptr;

    public:

        ContrastAutofocus(VideoStream::Ptr stream, Motor::Ptr motor);

        float currentContrastLevel();

        /* trys to focus with contrast,
         *  returns true if successful.
         *  parameter focusDistance returns the valid distance */
        bool focus(float &focusPosition, const bool reverse=false);

        /* a focus position for one direction is known.
         * find the position for the other direction */
        bool focusOtherDirection(const float knownPosition, 
            const bool isKnownDown, float &otherDirection);

        /* finds focus position for both directions */
        bool focusBothDirections(float &upPosition, float &downPosition);

        ContrastMessure cMessure;

    private:

        bool searchLocalMaximum(bool doReverse, float &position);
        void doBacklashGap(const bool doReverse);
        float messureContrast(int nsamples=1);

        VideoStream::Ptr vStream;
        Motor::Ptr fMotor;

        /* focus parameters */
        float messureDelay = 0.2;
        float contrastThreshold = 0.01;
        int numImagesPerSample = 5;

};

#endif

