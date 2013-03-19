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

    /* Copies the axis angle rotation
     * and the translation to an
     * Affine3f transformation matrix
     */

    /* axis angle roation */
#if 1
    Eigen::Vector3f axis(
            rotationVec.at<float>(0,0),
            rotationVec.at<float>(1,0),
            rotationVec.at<float>(2,0));
    float angle = axis.norm(); // length of the vector 
    axis.normalize();
    Eigen::AngleAxisf rot(angle, axis);
#endif


#if 0
    /* do euler angle rotation */
    Eigen::Affine3f rot;
    rot = Eigen::AngleAxisf(rotationVec.at<float>(0,0), Eigen::Vector3f::UnitX())
      * Eigen::AngleAxisf(rotationVec.at<float>(1,0), Eigen::Vector3f::UnitY())
      * Eigen::AngleAxisf(rotationVec.at<float>(2,0), Eigen::Vector3f::UnitZ());
#endif

    /* compose new pose */
    Eigen::Affine3f pose;
    pose = Eigen::Affine3f (Eigen::Translation3f (
                translationVec.at<float>(0, 0),
                translationVec.at<float>(1, 0),
                translationVec.at<float>(2, 0)));
    pose.rotate(rot);

    return pose;
}


void affine3fToTransRotVec(const Eigen::Affine3f &aff,
        cv::Mat &tvec, cv::Mat &rvec)
{   
    /* decompose matrix */
    Eigen::Vector3f pos = aff.translation();

    Eigen::AngleAxisf rot(aff.rotation());
    Eigen::Vector3f rotVec = rot.axis();
    float angle = rot.angle();
    rotVec *= angle;

    /* copy translation vector */
    tvec = cv::Mat::zeros(3, 1, CV_32F);
    tvec.at<float>(0,0) = pos[0];
    tvec.at<float>(1,0) = pos[1];
    tvec.at<float>(2,0) = pos[2];

    /* copy axis angle rotation */
    rvec = cv::Mat::zeros(3, 1, CV_32F);
    rvec.at<float>(0,0) = rotVec[0];
    rvec.at<float>(1,0) = rotVec[1];
    rvec.at<float>(2,0) = rotVec[2];
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


Eigen::Affine3f interpolateAffine(const Eigen::Affine3f &pose0, 
        const Eigen::Affine3f &pose1, float blend)
{
    /* interpolate translation */
    Eigen::Vector3f t0 = pose0.translation();
    Eigen::Vector3f t1 = pose1.translation();
    Eigen::Vector3f tIP = (t1 - t0)*blend;

    /* interpolate rotation */
    Eigen::Quaternionf r0(pose0.rotation());
    Eigen::Quaternionf r1(pose1.rotation());
    Eigen::Quaternionf rIP(r1.slerp(blend, r0));

    /* compose resulting pose */
    Eigen::Affine3f ipAff = pose0;
    ipAff.rotate(rIP);
    ipAff.translate(tIP);
    return ipAff;
}



void printAffine3f(const Eigen::Affine3f m) {
    std::cout << "[" << m(0,0) << ", " << m(0,1) << ", " 
                     << m(0,2) << ", " << m(0,3) << ", "
                     << std::endl
                     << m(1,0) << ", " << m(1,1) << ", " 
                     << m(1,2) << ", " << m(1,3) << ", "
                     << std::endl
                     << m(2,0) << ", " << m(2,1) << ", " 
                     << m(2,2) << ", " << m(2,3) << ", "
                     << std::endl 
                     << m(3,0) << ", " << m(3,1) << ", " 
                     << m(3,2) << ", " << m(3,3) 
                     << "]" << std::endl;
}


pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZtoRGBA(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr inCloud)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr newCloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>);

    int numPoints = inCloud->size();
    newCloud->resize(numPoints);

    for (int i; i<numPoints; i++) {
        pcl::PointXYZRGBA &np = newCloud->at(i);
        const pcl::PointXYZ &op = inCloud->at(i);

        np.x = op.x;
        np.y = op.y;
        np.z = op.z;
    }

    return newCloud;
}

