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

#include "mathutils.h"

Eigen::Affine3f transRotVecToAffine3f(
        const cv::Mat &translationVec, 
        const cv::Mat &rotationVec)
{
    Eigen::Affine3f pose, rot;

    //TODO check if M_PI is legit * M_PI (propably not) should be in r
    rot = Eigen::AngleAxisf(rotationVec.at<float>(0,0), Eigen::Vector3f::UnitX())
      * Eigen::AngleAxisf(rotationVec.at<float>(1,0),  Eigen::Vector3f::UnitY())
      * Eigen::AngleAxisf(rotationVec.at<float>(2,0), Eigen::Vector3f::UnitZ());

    pose = Eigen::Affine3f (Eigen::Translation3f (
                translationVec.at<float>(0, 0),
                translationVec.at<float>(1, 0),
                translationVec.at<float>(2, 0))) *
            Eigen::Affine3f(rot);
    
    return pose;
}


/* http://geomalgorithms.com/a07-_distance.html */
void intersectLines(const Eigen::Vector3f &p0, const Eigen::Vector3f &p1,
                    const Eigen::Vector3f &q0, const Eigen::Vector3f &q1,
                    Eigen::Vector3f &pointOnP, Eigen::Vector3f &pointOnQ)
{
    Eigen::Vector3f w0 = p0 - q0;

    /* the two vectors on the lines */
    Eigen::Vector3f u = p1 - p0;
    u.normalize();
    Eigen::Vector3f v = q0 - q1;
    v.normalize();

    float a = u.dot(u);
    float b = u.dot(v);
    float c = v.dot(v);
    float d = u.dot(w0);
    float e = v.dot(w0);

    float normFactor = a*c - b*b;
    float sc = (b*e - c*d) / normFactor;
    float tc = (a*e - b*d) / normFactor;

    /* the two nearest points on the lines */
    Eigen::ParametrizedLine<float, 3> lineP(p0, u);
    pointOnP = lineP.pointAt(sc);
    Eigen::ParametrizedLine<float, 3> lineQ(q0, v);
    pointOnQ = lineQ.pointAt(tc);
}

