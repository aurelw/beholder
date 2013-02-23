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

        typedef std::pair<std::string, std::string> FilePair;

    public:

        CalibStorageContract(const std::string &dir);

        void addMainIntrinsic(cv::Mat img);
        void addExtrinsicPairXYZ(cv::Mat img, PlainCloud::Ptr cloud); 
        void addExtrinsicPairRGB(cv::Mat img, RGBCloud::Ptr cloud); 

        std::vector<std::string> getMainIntrinsicFiles();
        std::vector<FilePair> getExtrinsicFiles();

    protected:

        std::string rootDir;
        bool gotValidRoot = false;

        /* path names */
        std::string mainIntrinsicDir = "/main_intrinsic/";
        std::string rangeFinderExtrinsicDir =  "/rangefinder_extrinsic/";
        std::string rangeFinderExtrinsicCamDir =  "/rangefinder_extrinsic/cam/";
        std::string rangeFinderExtrinsicCloudDir =  "/rangefinder_extrinsic/cloud/";

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

};

#endif

