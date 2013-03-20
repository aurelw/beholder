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

#include "contrast_autofocus.h"


ContrastAutofocus::ContrastAutofocus(
        VideoStream::Ptr stream, Motor::Ptr motor)
{
    vStream = stream;
    fMotor = motor;
}


float ContrastAutofocus::currentContrastLevel() {

    //FIXME propper sampling
    /* sleep for some time*/
    sleep(0.1);

    /* get the messureing region */
    cv::Mat img = vStream->getFrame();

    /* messure contrast */
    return cMessure.messure(img);

}


float ContrastAutofocus::focus(const bool doReverse) {

    float fPosition = 0.0;

    /* determine the initial search direction */
    bool reverseInitDir =
        (doReverse && fMotor->isLowerLimit()) ||
        (!doReverse && fMotor->isUpperLimit());
    bool initDir = reverseInitDir ? !doReverse : doReverse;

    /* try to find initial local maximum */
    bool foundInit = searchLocalMaximum(initDir, fPosition);

    /* focus point found */
    if (foundInit && !reverseInitDir) {
        return fPosition;
    }

    /* focus point found but wrong direction */
    if (foundInit && reverseInitDir) {
        doBacklashGap(initDir);
        if (searchLocalMaximum(doReverse, fPosition)) {
            return fPosition;
        }
    }

    /* focus point not found, search other direction */
    bool otherDir = !initDir;
    bool foundOther = searchLocalMaximum(otherDir, fPosition);

    /* focus point found */
    if (foundOther && reverseInitDir) {
        return fPosition;
    }

    /* focus point found but wrong direction */
    if (foundOther && !reverseInitDir) {
        doBacklashGap(otherDir);
        if (searchLocalMaximum(doReverse, fPosition)) {
            return fPosition;
        }
    }

    /* now this feels hopeless */
    return 0.0;
}


void ContrastAutofocus::doBacklashGap(const bool doReverse) {
    /* proceed some steps before searching in the opposite direction */
    const int num_steps = 5;

    for (int i=0; i<num_steps; i++) {
        if (doReverse) {
            fMotor->stepDown();
        } else {
            fMotor->stepUp();
        }
    }
}


bool ContrastAutofocus::searchLocalMaximum(bool doReverse, float &position) {
    /* flag if a positive gradient was observed,
     * a local maximum is detected by a subsequent
     * negative gradient */
    bool positiveGradient = false;

    /* if a negative gradient is observed for
     * #searchLimit steps, abort early */
    const int searchLimit = 4;
    int searchLimitCounter = 0;

    /* theshold for a gradient difference */
    float gradientThresh = 0.01; // factor to lastConstrast

    float lastContrast = messureContrast();
    float lastPosition = fMotor->getPosition();

    while (true) {
        if (doReverse) {
            if (fMotor->isLowerLimit()) {
                return false;
            }
            fMotor->stepDown();
        } else {
            if (fMotor->isUpperLimit()) {
                return false;
            }
            fMotor->stepUp();
        }

        // FIXME threshhold for gradient
        float cContrast = messureContrast();
        if (!positiveGradient) {
           positiveGradient = (cContrast > lastContrast*(1+gradientThresh)); 
        /* negative gradient after positive */
        } else if (cContrast < lastContrast*(1-gradientThresh)) { 
            /* found local maximum */
            position = lastPosition;
            return true;
        }

        /* early exit if there is no positive gradient */
        if (!positiveGradient) {
            searchLimitCounter++;
            if (searchLimitCounter >= searchLimit) {
                return false;
            }
        }

        lastPosition = fMotor->getPosition();
        lastContrast = cContrast;
    }
}


float ContrastAutofocus::messureContrast() {
    return cMessure.messure(vStream->getFrame());
}

