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

#include "calibstorage_contract.h"

#include <iomanip>
#include <sstream>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/range/adaptors.hpp>

#include <pcl/io/pcd_io.h>

#include "console_utils.h"

using namespace boost::filesystem;


CalibStorageContract::CalibStorageContract(const std::string &dir) {
    path rootp(dir);
    if (is_directory(rootp)) {
        gotValidRoot = true;
        rootDir = dir;

        /* main intrinsic path */
        path intrinsicp(rootDir + mainIntrinsicDir);
        if (!exists(intrinsicp)) {
            create_directory(intrinsicp);
        }

        /* rangefinder extrinsic pair */
        path extrinsicp(rootDir + rangeFinderExtrinsicDir);
        if (!exists(extrinsicp)) {
            create_directory(extrinsicp);
            /* subpaths */
            create_directory(path(rootDir + rangeFinderExtrinsicCamDir));
            create_directory(path(rootDir + rangeFinderExtrinsicCloudDir));
        }

        /* check which files are already there */
        initCounters();
    }
}


void CalibStorageContract::addMainIntrinsic(cv::Mat img) {
    std::string fname = numToFileName(mainIntrinsicCounter++, ".jpg");
    cv::imwrite(rootDir + mainIntrinsicDir + fname, img);
    printSimpleInfo("[CalibStorage] ", "stored intrinsic sample: " + fname + "\n");
}


void CalibStorageContract::addExtrinsicPairXYZ(cv::Mat img, PlainCloud::Ptr cloud) {
    /* get file names */
    std::string imgfname = numToFileName(rangeFinderExtrinsicCounter, ".jpg");
    std::string cloudfname = numToFileName(rangeFinderExtrinsicCounter, ".pcd");
    rangeFinderExtrinsicCounter++;

    /* write pair */
    cv::imwrite(rootDir + rangeFinderExtrinsicCamDir + imgfname, img);
    pcl::io::savePCDFileASCII(
            rootDir + rangeFinderExtrinsicCloudDir + cloudfname, *cloud);
    printSimpleInfo("[CalibStorage] ", "stored extrinsic pair: " + cloudfname + "\n");
}


void CalibStorageContract::addExtrinsicPairRGB(cv::Mat img, RGBCloud::Ptr cloud) {
    /* get file names */
    std::string imgfname = numToFileName(rangeFinderExtrinsicCounter, ".jpg");
    std::string cloudfname = numToFileName(rangeFinderExtrinsicCounter, ".pcd");
    rangeFinderExtrinsicCounter++;

    /* write pair */
    cv::imwrite(rootDir + rangeFinderExtrinsicCamDir + imgfname, img);
    pcl::io::savePCDFileASCII(
            rootDir + rangeFinderExtrinsicCloudDir + cloudfname, *cloud);
    printSimpleInfo("[CalibStorage] ", "stored extrinsic pair: " + 
            cloudfname + "," + imgfname + "\n");
}


std::vector<std::string> CalibStorageContract::getMainIntrinsicFiles() {
     return getFilesFromDir(rootDir + mainIntrinsicDir, ".jpg");
}


std::vector<CalibStorageContract::FilePair> 
CalibStorageContract::getExtrinsicFiles() {

    // pair of <imgfile, pcdfile>
    std::vector<FilePair> result;

    std::vector<std::string> imgFiles = 
        getFilesFromDir(rootDir + rangeFinderExtrinsicCamDir, ".jgg");
    std::vector<std::string> cloudFiles = 
        getFilesFromDir(rootDir + rangeFinderExtrinsicCloudDir, ".pcd");

    for (size_t i=0; i<imgFiles.size(); i++) {
        //FIXME check if file numbers are equal
        FilePair pair(imgFiles[i], cloudFiles[i]); 
        result.push_back(pair);
    }

    return result;
}


void CalibStorageContract::initCounters() {
    mainIntrinsicCounter = findNextFile(rootDir + mainIntrinsicDir, ".jpg");
    rangeFinderExtrinsicCounter = findNextFilePair(
            rootDir + rangeFinderExtrinsicCamDir, ".jpg",
            rootDir + rangeFinderExtrinsicCloudDir, ".pcd");
}


int CalibStorageContract::findNextFile(
        const std::string &dir, const std::string &extension)
{
    /* prepare vector of file names */
    path p(dir);
    std::vector<path> files;
    std::copy(directory_iterator(p), directory_iterator(), back_inserter(files));
    std::sort(files.begin(), files.end());

    if (files.size() > 0) {
        /* highest file number first */
        for (auto file : boost::adaptors::reverse(files)) {
            // get only the filename from the path
            std::string fname = file.filename().string();

            /* check if file is valid */
            size_t extPos = fname.find(extension);
            if (extPos != std::string::npos) {
                // remove extension
                std::string fnumber = fname.substr(0, extPos);
                return std::stoi(fnumber) + 1;
            }
        }
    } else {
        return 0;
    }
}


int CalibStorageContract::findNextFilePair(
        const std::string &dir0, const std::string &extension0,
        const std::string &dir1, const std::string &extension1)
{
    int n0 = findNextFile(dir0, extension0);
    int n1 = findNextFile(dir1, extension1);

    return (n0 > n1) ? n0 : n1;
}


std::string CalibStorageContract::numToFileName(
        int num, const std::string &extension) 
{
    std::ostringstream ss;
    ss << std::setw(3) << std::setfill('0') << num;
    return ss.str() + extension;
}


std::vector<std::string> CalibStorageContract::getFilesFromDir(
        const std::string &dir, const std::string &extension)
{

    std::vector<std::string> resultFiles;

    /* get all files in the directory sorted */
    path p(dir);
    std::vector<path> files;
    copy(directory_iterator(p), directory_iterator(), back_inserter(files));
    sort(files.begin(), files.end());

    for (path file : files) {
        std::string fname = file.string();
        if (std::string::npos != fname.find(extension)) {
            resultFiles.push_back(fname);
        }
    }

    return resultFiles;
}

