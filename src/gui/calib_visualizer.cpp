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


void CalibVisualizer::setPlainCloud(PlainCloud::ConstPtr cloud) {
    boost::unique_lock<boost::shared_mutex> lock(mutex);
    plainCloud = cloud;
    flagUpdatePlainCloud = true;
}


void CalibVisualizer::setRegistration(
        PlainCloud::ConstPtr cloud0,
        PlainCloud::ConstPtr cloud1)
{
    boost::unique_lock<boost::shared_mutex> lock(mutex);
    regCloud0 = cloud0;
    regCloud1 = cloud1;

    targetColorHandler.reset( new ColorHandler(regCloud0, 255, 0, 0) );
    regColorHandler.reset( new ColorHandler(regCloud0, 0, 0, 255) );

    flagUpdateRegistration = true;
}


void CalibVisualizer::setDrawRegistration(bool doDraw) {
    drawRegistration = doDraw;
    flagUpdateRegistration = true;
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


void CalibVisualizer::setDrawPlainCloud(bool doDraw) {
    drawPlainCloud = doDraw;
    flagUpdatePlainCloud = true;
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


void CalibVisualizer::updatePlainCloud() {
    if (!flagUpdatePlainCloud) {
        return;
    } else {
        flagUpdatePlainCloud = false;
    }

    if (plainCloudAdded) {
        if (drawPlainCloud && plainCloud != NULL) {
            visualizer->updatePointCloud(plainCloud, "plainCloud");
        } else {
            visualizer->removePointCloud("plainCloud");
            plainCloudAdded = false;
        }
    } else if (drawPlainCloud && plainCloud != NULL) {
        visualizer->addPointCloud(plainCloud, "plainCloud");
        plainCloudAdded = true;
    }
}


void CalibVisualizer::updateRegistration() {
    if (!flagUpdateRegistration) {
        return;
    } else {
        flagUpdateRegistration = false;
    }

    if (registrationCloudAdded) {
        if (drawRegistration && regCloud0 != NULL && regCloud1 != NULL) {
            visualizer->updatePointCloud(regCloud0, 
                    *targetColorHandler, "regCloud0");
            visualizer->updatePointCloud(regCloud1, 
                    *regColorHandler, "regCloud1");
        } else {
            visualizer->removePointCloud("regCloud0");
            visualizer->removePointCloud("regCloud1");
            registrationCloudAdded = false;
        }
    } else if (drawRegistration && regCloud0 != NULL && regCloud1 != NULL) {
        visualizer->addPointCloud(regCloud0, *targetColorHandler, "regCloud0");
        visualizer->addPointCloud(regCloud1, *regColorHandler, "regCloud1");
        registrationCloudAdded = true;
    }
}



void CalibVisualizer::updateAllProperties() {
    BasicVisualizer::updateAllProperties();
    updateMarker();
    updateCorrespondence();
    updatePlainCloud();
    updateRegistration();
}

