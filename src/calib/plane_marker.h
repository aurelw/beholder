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

#include <iostream>
#include <cmath>

#include <boost/thread/thread.hpp>
#include <boost/foreach.hpp> 

#include "pcl/common/common_headers.h"

#include <pcl/filters/voxel_grid.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>

#define DEBUG_MARKER

#ifdef DEBUG_MARKER
    #define DEBUG(s) printf("[PlaneMarker] "); printf(s); printf("\n")
    #define DEBUG_P(s,p) printf("[PlaneMarker] " ); printf(s,p); printf("\n")
#else
    #define DEBUG(s)
    #define DEBUG_P(s,p)
#endif

template<typename PointT> 
class PlaneMarker {

    public:
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef boost::shared_ptr<PointCloud> PointCloudPtr;
        typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;


        PlaneMarker (float length, float width, float sigma) :
            pointA(),
            pointB()
        {
            if (width > length) {
                marker_length = width;
                marker_width = length;
            } else {
                marker_length = length;
                marker_width = width;
            }
            marker_sigma = sigma;
            marker_diagonal = std::sqrt(length*length + width*width);

            maxSACIterations = 1000;
            sacDistanceThresh = 0.06;
            clusterTolerance = 0.05;
            minClusterSize = 2000;
        }


        bool computeMarkerCenter(const PointCloudConstPtr& c, PointT& center);

        PointCloudConstPtr cloud_ptr;
        PointT pointA, pointB, centerPoint;
        PointT pointC, pointD;
        PointT pointR;
       
    private:
        /* marker properties */ 
        float marker_length, marker_width, marker_diagonal;
        float marker_sigma;

        /* search properties */
        int maxSACIterations;
        float sacDistanceThresh;
        float clusterTolerance;
        int minClusterSize;

        bool checkCandidate(const PointCloudPtr& pc_ptr, 
                const pcl::PointIndices::Ptr& indices_ptr);
        bool findMaxDistandPoint(const PointCloudPtr& pc_ptr, 
                const pcl::PointIndices::Ptr& indices_ptr, const float limit,
                const PointT& start_point, PointT& max_point);
        bool mainDiagonal(const PointCloudPtr& pc_ptr, 
                const pcl::PointIndices::Ptr& indices_ptr, const float limit);
        bool secondDiagonal(const PointCloudPtr& pc_ptr, 
                const pcl::PointIndices::Ptr& indices_ptr, const float limit);
        void centerBetweenPoints(const PointT& a, const PointT& b, PointT& c);
        void centerFromMainDiagonal();
        void centerFromBothDiagonals();

};

