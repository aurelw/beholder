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

#ifndef __CALIB_STORAGE_CONTRACT_H__
#define __CALIB_STORAGE_CONTRACT_H__

#include <pcl/common/common_headers.h>

#include "opencv2/opencv.hpp"

#include "console_utils.h"


class CalibStorageContract {

    public:

        typedef pcl::PointCloud<pcl::PointXYZ> PlainCloud;
        typedef pcl::PointCloud<pcl::PointXYZRGBA> RGBCloud;

        /* pair: <img, cloud> */
        typedef std::pair<std::string, std::string> FilePair;

        /* point pair <3d-cloud-point, 2d-marker-point> */
        typedef std::pair<cv::Point3f, cv::Point2f> PointPair3d2d;
        typedef std::pair<cv::Point3f, cv::Point3f> PointPair3d3d;

        typedef struct FocusSample {
            float distance;
            float position;
            bool isReverse;
        } FocusSample;

    public:

        CalibStorageContract(const std::string &dir);
        ~CalibStorageContract();

        /* intrinsic sample images */
        void addMainIntrinsic(cv::Mat img);
        std::vector<std::string> getMainIntrinsicFiles();

        /* extrinsic sample images/clouds */
        void addExtrinsicPairXYZ(cv::Mat img, PlainCloud::Ptr cloud); 
        void addExtrinsicPairRGB(cv::Mat img, RGBCloud::Ptr cloud); 
        std::vector<FilePair> getExtrinsicFiles();

        /* 3d <-> 2d correspondece extrinsic */
        void addExtrinsicPointPair(cv::Point3f p3d, cv::Point2f p2d);
        void saveExtrinsicPointPairs();
        std::vector<PointPair3d2d> getExtrinsicPoints();
        std::pair<cv::Mat, cv::Mat> getExtrinsicPointsMatrices();

        /* 3d-marker point, 2d-marker point */
        void addExtrinsicPointPair3d(cv::Point3f p0, cv::Point3f p1);
        void saveExtrinsicPointPairs3d();
        std::vector<PointPair3d3d> getExtrinsicPoints3d();
        std::pair<cv::Mat, cv::Mat> getExtrinsicPoints3dMatrices();

        /* focus distance samples */
        void addFocusSample(float distance, float position, bool isReverse);
        void saveFocusSamples();
        std::vector<FocusSample> getFocusSamples();

    protected:

        std::string rootDir;
        bool gotValidRoot = false;

        /* path names */
        std::string mainIntrinsicDir = "/main_intrinsic/";
        std::string rangeFinderExtrinsicDir =  "/rangefinder_extrinsic/";
        std::string rangeFinderExtrinsicCamDir =  "/rangefinder_extrinsic/cam/";
        std::string rangeFinderExtrinsicCloudDir =  "/rangefinder_extrinsic/cloud/";
        std::string rangeFinderExtrinsicPointFile =  
           rangeFinderExtrinsicDir + "pointpairs.xml";
        std::string rangeFinderExtrinsicPoint3dFile =  
           rangeFinderExtrinsicDir + "pointpairs_3d.xml";
        std::string focusSamplesFile = "focusSamples.csv";

        /* new file names */
        int findNextFile(
                const std::string &dir, const std::string &extension);
        int findNextFilePair(
                const std::string &dir0, const std::string &extension0,
                const std::string &dir1, const std::string &extension1);
        std::string numToFileName(int n, const std::string &extension);

        /* file counters */
        void initCounters();
        int mainIntrinsicCounter = 0;
        int rangeFinderExtrinsicCounter = 0;

        /* search */
        std::vector<std::string> getFilesFromDir(
                const std::string &dir, const std::string &extension);

        /* point pairs for rangefinder extrinsic */
        std::vector<PointPair3d2d> exPointPairs;
        cv::Mat exPointPairsObject;
        cv::Mat exPointPairsImage;
        bool exPointsUpdated = false;
        void loadExPointPairs();
        void writeExPointPairFile();

        /* point pairs for rangefinder extrinsic */
        std::vector<PointPair3d3d> exPointPairs3d;
        cv::Mat exPointPairs3d_0;
        cv::Mat exPointPairs3d_1;
        bool exPoints3dUpdated = false;
        void loadExPointPairs3d();
        void writeExPointPair3dFile();

        /* samples for focus distance */
        std::vector<FocusSample> focusSamples;
        bool focusSamplesUpdated = false;
        void loadFocusSamples();
        void writeFocusSamples();

};

#endif

