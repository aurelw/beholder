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

#include "console_utils.h"


ContrastAutofocus::ContrastAutofocus(
        VideoStream::Ptr stream, Motor::Ptr motor)
{
    vStream = stream;
    fMotor = motor;
}


float ContrastAutofocus::currentContrastLevel() {
    return messureContrast(1);
}


bool ContrastAutofocus::focus(float &focusPosition, const bool doReverse) {

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
        focusPosition = fPosition;
        return true;
    }

    /* focus point found but wrong direction */
    if (foundInit && reverseInitDir) {
        doBacklashGap(initDir);
        if (searchLocalMaximum(doReverse, fPosition)) {
            focusPosition = fPosition;
            return true;
        }
    }

    /* focus point not found, search other direction */
    bool otherDir = !initDir;
    bool foundOther = searchLocalMaximum(otherDir, fPosition);

    /* focus point found */
    if (foundOther && reverseInitDir) {
        focusPosition = fPosition;
        return true;
    }

    /* focus point found but wrong direction */
    if (foundOther && !reverseInitDir) {
        doBacklashGap(otherDir);
        if (searchLocalMaximum(doReverse, fPosition)) {
            focusPosition = fPosition;
            return true;
        }
    }

    /* now this feels hopeless */
    return false;
}


bool ContrastAutofocus::focusOtherDirection(const float knownPosition,
        const bool isKnownReverse, float &otherDirection)
{
    fMotor->setPosition(knownPosition); 
    doBacklashGap(isKnownReverse);
    return searchLocalMaximum(!isKnownReverse, otherDirection);
}


bool ContrastAutofocus::focusBothDirections(
        float &upPosition, float &downPosition)
{
    bool foundDown = false;
    bool foundUp = searchLocalMaximum(false, upPosition); 

    if (foundUp) {
        doBacklashGap(false);
    }

    foundDown = searchLocalMaximum(true, downPosition);

    if (foundDown && !foundUp) {
        doBacklashGap(true);
        foundUp = searchLocalMaximum(false, upPosition);
    }

    return (foundUp && foundDown);
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
    const int searchLimit = 10;
    int searchLimitCounter = 0;

    /* theshold for a gradient difference */
    float gradientThresh = contrastThreshold; // factor to lastConstrast

    float maxContrast = messureContrast(numImagesPerSample);
    float maxPosition = fMotor->getPosition();

    while (true) {
        if (doReverse) {
            if (fMotor->isLowerLimit()) {
                break;
            }
            fMotor->stepDown();
        } else {
            if (fMotor->isUpperLimit()) {
                break;
            }
            fMotor->stepUp();
        }

        float cContrast = messureContrast(numImagesPerSample);
        float cPosition = fMotor->getPosition();

        /* new maximum? -> positve gradient, new focus point */
        if (cContrast > maxContrast*(1+gradientThresh)) {
            maxContrast = cContrast;
            maxPosition = cPosition;
            positiveGradient = true;
            searchLimitCounter = 0;
        /* negative gradient */
        } else { 
            searchLimitCounter++;
        }

        /* no new local maximum in #searchLimit steps */
        if (searchLimitCounter >= searchLimit) {
            break;
        }
    }

    if (positiveGradient) {
        position = maxPosition;
        return true;
    } else {
        position = 0;
        return false;
    }
}


float ContrastAutofocus::messureContrast(int nsamples) {
    if (nsamples < 1) {
        nsamples = 1;
    }

    std::vector<float> samples;

    sleep(messureDelay);

    /* capture n samples */
    while (samples.size() != nsamples) {
        cv::Mat frame = vStream->getFrame();
        frame = cMessure.preprocess(frame);
        float sample = cMessure.messure(frame);
        /* make sure that the new sample is unique */
        if (samples.size() == 0 || 
            (samples.back() != sample))
        {
            samples.push_back(sample);
        }
    }

    /* return the mean of all samples */
    float mean = std::accumulate(samples.begin(), samples.end(), 0.0f)
                    / samples.size();
    return mean;
}

