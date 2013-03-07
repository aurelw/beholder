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

#include "calib_visualizer.h"

#include "console_utils.h"


void CalibVisualizer::initVisualizer() {
    BasicVisualizer::initVisualizer();

    Eigen::Affine3f idm;
    idm.setIdentity();
    addCoordinateSystem(idm, "world", 1.0);
}


void CalibVisualizer::setMarkerCenter(pcl::PointXYZ center, bool found) {
    boost::unique_lock<boost::shared_mutex> lock(mutex);
    markerCenter = center;
    foundMarker = found;
    flagUpdateMarker = true;
}


void CalibVisualizer::setCorrespondence(PlainCloud::ConstPtr cloud0,
        PlainCloud::ConstPtr cloud1)
{
    boost::unique_lock<boost::shared_mutex> lock(mutex);
    cpCloudOne = cloud0;
    cpCloudTwo = cloud1;
    flagUpdateCorrespondence = true;
}


void CalibVisualizer::setDrawMarker(bool doDraw) {
    drawMarker = doDraw;
    flagUpdateMarker = true;
}


void CalibVisualizer::setDrawCorrespondence(bool doDraw, bool arrow) {
    drawCorrespondence = doDraw;
    drawCorrespondenceArrows = arrow;
    flagUpdateCorrespondence = true;
}


void CalibVisualizer::updateMarker() {
    if (!flagUpdateMarker) {
        return;
    } else {
        flagUpdateMarker = false;
    }

    if (markerAdded) {
        visualizer->removeShape("markerCenter");
    }

    if (drawMarker) {
        if (foundMarker) {
            visualizer->addSphere(markerCenter, 0.01, 
                    1.0, 0.64, 0.0, //orange
                    "markerCenter");
            markerAdded = true;
        } else {
            //visualizer->updateText("Marker not found.", 10, 10, "markerText");
        }
    } else {
        //visualizer->updateText("", 10, 10, "markerText");
    }
}


void CalibVisualizer::updateCorrespondence() {
    if (!flagUpdateCorrespondence) {
        return;
    } else {
        flagUpdateCorrespondence = false;
    }

    if (correspondenceAdded) {
        //visualizer->removeCorrespondences("cal_corr0");
        //
        /* remove clouds */
        visualizer->removePointCloud("cpCloudOne");
        visualizer->removePointCloud("cpCloudTwo");

        /* remove arrows */
        for (int i=0; i<numCorrespondences; i++) {
            std::stringstream ss;
            ss << "crsArrow" << i;
            visualizer->removeShape(ss.str());
        }
    }

    if (drawCorrespondence) {
        /* specify correspondences */
        std::vector<int> crs;
        for (int i=0; i<cpCloudTwo->points.size(); i++) {
            crs.push_back(i);
        }
        numCorrespondences = crs.size();

        //visualizer->addCorrespondences<pcl::PointXYZ>(
        //        cpCloudOne, cpCloudTwo, crs, "cal_corr0");

        /* draw clouds */
        visualizer->addPointCloud(cpCloudOne, "cpCloudOne");
        visualizer->addPointCloud(cpCloudTwo, "cpCloudTwo");

        /* draw arrows */
        for (int i=0; i<numCorrespondences; i++) {
            /* fancy pancy colors */
            float cP0 = 1.0 - (i/(float)numCorrespondences);
            float cP1 = i/(float)numCorrespondences;
            float cP2 = (2*cP0 + cP1)/2;
            std::stringstream ss;
            ss << "crsArrow" << i;
            if (drawCorrespondenceArrows) {
                visualizer->addArrow(
                        cpCloudOne->points[i],
                        cpCloudTwo->points[i],
                        cP0, cP1, cP2,
                        false, // no length
                        ss.str());
            } else {
                visualizer->addLine(
                        cpCloudOne->points[i],
                        cpCloudTwo->points[i],
                        cP0, cP1, cP2,
                        ss.str());
            }
        }

        correspondenceAdded = true;
    }
}


void CalibVisualizer::updateAllProperties() {
    BasicVisualizer::updateAllProperties();
    updateMarker();
    updateCorrespondence();
}

