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

    public:

        PlaneMarker (float length, float width, float sigma);

        bool computeMarkerCenter(const PointCloudConstPtr& c, PointT& center);

        /* some points describing the marker */
        PointT pointA, pointB, centerPoint;
        PointT pointC, pointD;
        PointT pointR;

        /* the cloud of the marker */
        PointCloudConstPtr markerCloud;
       
    private:

        /* marker properties */ 
        float marker_length, marker_width, marker_diagonal;
        float marker_sigma;

        /* search properties */
        int maxSACIterations;
        float sacDistanceThresh;
        float clusterTolerance;
        int minClusterSize;

        /* classification methods */
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


template <typename PointT> PlaneMarker<PointT>::PlaneMarker 
    (float length, float width, float sigma) :
    pointA(),
    pointB()
{
    /* length should be greateter then width */
    if (width > length) {
        marker_length = width;
        marker_width = length;
    } else {
        marker_length = length;
        marker_width = width;
    }
    marker_diagonal = std::sqrt(length*length + width*width);

    /* the valid delta for the found diagonal */
    marker_sigma = sigma;

    /* general search properties */
    maxSACIterations = 1000;
    sacDistanceThresh = 0.06;
    clusterTolerance = 0.05; //5cm
    minClusterSize = 2000;
}


template <typename PointT> bool PlaneMarker<PointT>::computeMarkerCenter(
        const PointCloudConstPtr& c, 
        PointT& center) 
{
    //FIXME clusters before planes!
    PointCloudPtr filtered_cloud_ptr (new PointCloud(*c));
/*          
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud (c);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*filtered_cloud_ptr);
*/
    pcl::PointIndices::Ptr inliers_ptr (new pcl::PointIndices);
    pcl::PointIndices& inliers = *inliers_ptr;

    pcl::ModelCoefficients coefficients;

    //Create the filtering object
    pcl::ExtractIndices<PointT> extract;

    /* segment for candidate planes */
    pcl::SACSegmentation<PointT> seg;

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxSACIterations);
    seg.setDistanceThreshold (sacDistanceThresh);

    int plane_counter = 0;
    int num_total_points = filtered_cloud_ptr->size();
    while (filtered_cloud_ptr->size() > 0.001*num_total_points) {

        DEBUG_P("filtered_cloud.size(): %d", filtered_cloud_ptr->size());

        seg.setInputCloud (filtered_cloud_ptr);
        seg.segment (inliers, coefficients); 

        DEBUG("segmented a plane");

        /* can't fit plane */
        if (inliers.indices.size() == 0) {
            DEBUG("can't fit to plane!");
            return false;
        }
        plane_counter++;

        DEBUG_P("number of inliers: %d", inliers_ptr->indices.size()); 

        //TODO project !!!!

        /* get the euclidian clusters from this plane 
         * this is slooooOw */
        std::vector<pcl::PointIndices> cluster_indices;

        pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree 
            (new pcl::search::KdTree<pcl::PointXYZRGBA>);
        tree->setInputCloud (filtered_cloud_ptr);

        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance (clusterTolerance);
        ec.setMinClusterSize(minClusterSize);
        ec.setSearchMethod (tree);
        ec.setInputCloud (filtered_cloud_ptr);
        ec.setIndices (inliers_ptr);
        ec.extract (cluster_indices);

        DEBUG_P("number of clusters extracted: %d", cluster_indices.size());

        for (size_t i=0; i<cluster_indices.size(); i++) {
            
            // convert to a boost shared pointer
            pcl::PointIndices::Ptr cluster_indices_ptr 
                (new pcl::PointIndices(cluster_indices[i]));

            DEBUG_P("cluster size: %d", cluster_indices_ptr->indices.size()); 

            if (checkCandidate(filtered_cloud_ptr, cluster_indices_ptr)) {
                DEBUG("found marker");

                /* extract the marker cloud */
                PointCloudPtr marker_cloud(new PointCloud);
                pcl::PCDWriter writer;
                pcl::ExtractIndices<PointT> seg_extract;
                seg_extract.setInputCloud (filtered_cloud_ptr);
                seg_extract.setIndices (cluster_indices_ptr);
                seg_extract.setNegative (false);
                seg_extract.filter(*marker_cloud);
                markerCloud = marker_cloud;
                    
                /* determine the center point */
                centerFromBothDiagonals(); 
                center = centerPoint;

                return true;
            }
        }

        /* extract the candidate plane from the cloud */
        //FIXME this might remove points from the marker
        extract.setInputCloud (filtered_cloud_ptr);
        extract.setIndices (inliers_ptr);
        extract.setNegative (true);
        extract.filter (*filtered_cloud_ptr);
    }

    return false;
}


template <typename PointT> bool PlaneMarker<PointT>::checkCandidate(
        const PointCloudPtr& pc_ptr, 
        const pcl::PointIndices::Ptr& indices_ptr) 
{

    if (!mainDiagonal(pc_ptr, indices_ptr, marker_diagonal+marker_sigma)) {
        DEBUG("early exit, cluster too big");
        return false;
    }

    if (!(std::abs( pcl::euclideanDistance(pointA, pointB) - marker_diagonal)               <= marker_sigma)) 
    {
        return false;
    }

    if (!secondDiagonal(pc_ptr, indices_ptr, marker_diagonal+marker_sigma)) {
        DEBUG("no second diagonal");
        return false;
    }

    float short_side = (marker_length > marker_width) ? 
        marker_width : marker_length;

    // check marker dimensions. length > width
    if (pcl::euclideanDistance(pointA, pointD) < (marker_width - marker_sigma)) 
        return false;
    if (pcl::euclideanDistance(pointA, pointC) < (marker_width - marker_sigma)) 
        return false;
    if (pcl::euclideanDistance(pointB, pointD) < (marker_width - marker_sigma)) 
        return false;
    if (pcl::euclideanDistance(pointB, pointC) < (marker_width - marker_sigma)) 
        return false;
    if (pcl::euclideanDistance(pointA, pointD) >= (marker_length + marker_sigma)) 
        return false;
    if (pcl::euclideanDistance(pointA, pointC) >= (marker_length + marker_sigma)) 
        return false;
    if (pcl::euclideanDistance(pointB, pointD) >= (marker_length + marker_sigma)) 
        return false;
    if (pcl::euclideanDistance(pointB, pointC) >= (marker_length + marker_sigma)) 
        return false;
    return true;
}


template <typename PointT> bool PlaneMarker<PointT>::findMaxDistandPoint(
        const PointCloudPtr& pc_ptr, 
        const pcl::PointIndices::Ptr& indices_ptr, 
        const float limit,
        const PointT& start_point, PointT& max_point) 
{

    pcl::PointIndices& indices = *indices_ptr;

    // find the first corner
    float max_distance = 0.0f;
    BOOST_FOREACH( int index,  indices.indices) {
        float c_distance = 
            pcl::euclideanDistance(pc_ptr->points[index], start_point);
        if (c_distance > max_distance)  {
            max_distance = c_distance;
            max_point = pc_ptr->points[index];
            // early exit, if cluster is too big
            if (max_distance > limit) {
                DEBUG_P("max distance: %f", max_distance);
                return false;
            }
        }
    }

    DEBUG_P("max distance: %f", max_distance);
    return true;
}


template <typename PointT> bool PlaneMarker<PointT>::mainDiagonal(
        const PointCloudPtr& pc_ptr, 
        const pcl::PointIndices::Ptr& indices_ptr, 
        const float limit) 
{
    // start with a 'random' point
    PointT point_r = pc_ptr->points[indices_ptr->indices[0]];
    pointR = point_r;

    if (! findMaxDistandPoint(pc_ptr, indices_ptr, limit, point_r, pointA)) 
        return false;
    if (! findMaxDistandPoint(pc_ptr, indices_ptr, limit, pointA, pointB)) 
        return false; 

    return true;
} 


template <typename PointT> bool PlaneMarker<PointT>::secondDiagonal(
        const PointCloudPtr& pc_ptr, 
        const pcl::PointIndices::Ptr& indices_ptr, 
        const float limit) 
{

    pcl::PointIndices& indices = *indices_ptr;
    PointT point_r;

    float distCenter_thres = marker_diagonal / 3.0;
    float distAB_thresh = distCenter_thres / 2.0;

    /* search for a candidate point to start */
    bool found_candidate = false;
    BOOST_FOREACH(int index, indices.indices) {
        if (pcl::euclideanDistance(pc_ptr->points[index], centerPoint) 
                > distCenter_thres) 
        {
            float distA = pcl::euclideanDistance(pc_ptr->points[index], pointA);
            float distB = pcl::euclideanDistance(pc_ptr->points[index], pointB);
            if (std::abs(distA - distB) < distAB_thresh) {
                found_candidate = true;
                point_r =  pc_ptr->points[index];
                break;
            }
        } 
    }

    // no candidate found, not a valid marker
    if (!found_candidate) {
        return false;
    }

    // not likely to fail on limit
    findMaxDistandPoint(pc_ptr, indices_ptr, limit, point_r, pointC);
    findMaxDistandPoint(pc_ptr, indices_ptr, limit, pointC, pointD);

    return true;
}


template <typename PointT> void PlaneMarker<PointT>::centerBetweenPoints(
        const PointT& a, 
        const PointT& b, PointT& c) 
{
    Eigen::Vector3f pA(a.x, a.y, a.z);
    Eigen::Vector3f pB(b.x, b.y, b.z);
    Eigen::Vector3f middle = pA + ((pB - pA) /2.0);
    c.getVector3fMap() = middle;
}


template <typename PointT> void PlaneMarker<PointT>::centerFromMainDiagonal() {
    centerBetweenPoints(pointA, pointB, centerPoint);
}


template <typename PointT> void PlaneMarker<PointT>::centerFromBothDiagonals() {
    PointT p0, p1;
    centerBetweenPoints(pointA, pointB, p0);
    centerBetweenPoints(pointC, pointD, p1);
    centerBetweenPoints(p0, p1, centerPoint);
}

