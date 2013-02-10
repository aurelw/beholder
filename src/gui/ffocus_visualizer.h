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

#include <stdio.h>
#include <iostream>

#include <pcl/common/common_headers.h>

#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/range_image/range_image_planar.h>

#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>


//#include "cameraparameters.h"

#ifndef __FFOCUS__VISUALIZER_H__
#define __FFOCUS__VISUALIZER_H__


// forward declaration for registering the callback
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* visualizer_void);


class FFocusVisualizer {

    public:

        typedef enum DisplayMode {
            ENV_CLOUD,
            DSLR_CLOUD,
            ENV_DSLR_CLOUD,
            NO_CLOUD,
            DISPLAY_MODE_SIZE
        } DisplayCloud;

        friend void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* visualizer_void);

        typedef pcl::PointXYZ PointT;
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef boost::shared_ptr<PointCloud> PointCloudPtr;
        typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;
        //typedef boost::shared_ptr<pcl::RangeImagePlanar> RangeImagePtr;
        typedef pcl::RangeImagePlanar::Ptr RangeImagePtr;

        FFocusVisualizer() :
            visualizer("FFocus Visualizer"),
            toggleUpdate(false), 
            toggleUpdateCoordinates(false), 
            displayMode(ENV_CLOUD),
            capCloudFlag(false),
            capStreamFlag(false),
            capStreamAndCastFlag(false),
            pickFocusPointFlag(false),
            displayKinectCoordFrame(false),
            resetFlag(false),
            multiTrackingResetFlag(false),
            numSecondaryFocusPoints(0)
        {
            vfColorHandler_ptr = NULL;
            envColorHandler_ptr = NULL;
            visualizer.addCoordinateSystem(1.0);
        }

        void setEnvironmentCloud(PointCloudPtr cloud);
        void setViewFinderRangeImage(RangeImagePtr ri);
        void setCameraPose(const Eigen::Affine3f& pose);
        void setRangeFinderExtrinsic(const Eigen::Affine3f& pose);
        void setFocusPoint(const PointT& point);
        void setFocusPlaneDistance(float d);
        void setFocusPointVisibility(bool visible);


        int numSecondaryFocusPoints;
        void addSecondaryFocusPoint(const PointT& point);
        void removeSecondaryFocusPoints();

        void register_callbacks() {
            visualizer.registerKeyboardCallback(keyboardEventOccurred, (void*) this);
        }

        void spinOnce();

        bool capCloudFlag;
        bool capStreamFlag;
        bool capStreamAndCastFlag;
        bool pickFocusPointFlag;
        bool resetFlag;
        bool multiTrackingResetFlag;

    private:
        pcl::visualization::PCLVisualizer visualizer;
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange>* vfColorHandler_ptr;
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange>* envColorHandler_ptr;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr envCloud;
        RangeImagePtr vfRangeImg;

        Eigen::Affine3f cameraPose, rangeFinderExtrinsic;

        PointT focusTrackPoint;
        float fplaneDistance;
        bool focusPointVisible;

        bool toggleUpdate;
        bool toggleUpdateCoordinates;

        bool displayKinectCoordFrame;
        DisplayMode displayMode;

        void update();
        void updateCoordinates();
        void addCoordinateSystem(float scale, Eigen::Affine3f pose, std::string id);
        void removeCoordinateSystem(std::string id);

};



#endif


