#include "focustracker_single.h"


#ifndef __FOCUS_TRACKER_MULTI_H__
#define __FOCUS_TRACKER_MULTI_H__

class FocusTrackerMulti : public FocusTrackerSingle {

    public:

        FocusTrackerMulti() {
        }

        virtual pcl::PointXYZ pick();
        virtual void reset();
        virtual pcl::PointXYZ getTrackedPoint();

        virtual void init();
        virtual float getDistance();


    protected:
        std::vector<pcl::PointXYZ> trackedPoints;

};

#endif
