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
#include <algorithm>

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

        /* point pair files */
        loadExPointPairs();
        loadExPointPairs3d();

        loadFocusSamples();

        /* check which files are already there */
        initCounters();
    }
}


CalibStorageContract::~CalibStorageContract() {
    saveExtrinsicPointPairs();
    saveExtrinsicPointPairs3d();
    saveFocusSamples();
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


void CalibStorageContract::addExtrinsicPairRGB(
        cv::Mat img, RGBCloud::Ptr cloud) 
{
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


void CalibStorageContract::addExtrinsicPointPair(
        cv::Point3f p3d, cv::Point2f p2d) 
{
    PointPair3d2d ppair(p3d, p2d);
    exPointPairs.push_back(ppair);
    exPointsUpdated = true;
    
    /* print some output */
    std::stringstream ss;
    ss << "p3d: " << p3d << " p2d: " << p2d;
    printSimpleInfo("[CalibStorage] ", 
            "added extrinsic point pair. " + ss.str() + "\n"); 
}


void CalibStorageContract::saveExtrinsicPointPairs() {
    if (exPointsUpdated) {
        writeExPointPairFile();
        exPointsUpdated = false;
    }
}


/* 3d marker, 2d marker */
void CalibStorageContract::addExtrinsicPointPair3d(
        cv::Point3f p0, cv::Point3f p1) 
{
    PointPair3d3d ppair(p0, p1);
    exPointPairs3d.push_back(ppair);
    exPoints3dUpdated = true;
    
    /* print some output */
    std::stringstream ss;
    ss << "p0: " << p0 << " p1: " << p1;
    printSimpleInfo("[CalibStorage] ", 
            "added extrinsic 3d point pair. " + ss.str() + "\n"); 
}


void CalibStorageContract::saveExtrinsicPointPairs3d() {
    if (exPoints3dUpdated) {
        writeExPointPair3dFile();
        exPoints3dUpdated = false;
    }
}


std::vector<std::string> CalibStorageContract::getMainIntrinsicFiles() {
     return getFilesFromDir(rootDir + mainIntrinsicDir, ".jpg");
}


std::vector<CalibStorageContract::FilePair> 
CalibStorageContract::getExtrinsicFiles() {

    // pair of <imgfile, pcdfile>
    std::vector<FilePair> result;

    std::vector<std::string> imgFiles = 
        getFilesFromDir(rootDir + rangeFinderExtrinsicCamDir, ".jpg");
    std::vector<std::string> cloudFiles = 
        getFilesFromDir(rootDir + rangeFinderExtrinsicCloudDir, ".pcd");

    for (size_t i=0; i<imgFiles.size(); i++) {
        //FIXME check if file numbers are equal
        FilePair pair(imgFiles[i], cloudFiles[i]); 
        result.push_back(pair);
    }

    return result;
}


std::vector<CalibStorageContract::PointPair3d2d> 
CalibStorageContract::getExtrinsicPoints() {
    return exPointPairs;
}


std::pair<cv::Mat, cv::Mat> 
CalibStorageContract::getExtrinsicPointsMatrices() {
    std::pair<cv::Mat, cv::Mat> mp;
    mp.first = exPointPairsObject.clone();
    mp.second = exPointPairsImage.clone();
    return mp;
}


std::vector<CalibStorageContract::PointPair3d3d> 
CalibStorageContract::getExtrinsicPoints3d() {
    return exPointPairs3d;
}


std::pair<cv::Mat, cv::Mat> 
CalibStorageContract::getExtrinsicPoints3dMatrices() {
    std::pair<cv::Mat, cv::Mat> mp;
    mp.first = exPointPairs3d_0.clone();
    mp.second = exPointPairs3d_1.clone();
    return mp;
}


void CalibStorageContract::addFocusSample(
        float distance, float position, bool isReverse)
{
    FocusSample sample = {distance, position, isReverse};
    focusSamples.push_back(sample);
    focusSamplesUpdated = true;
    
    std::stringstream ss;
    ss << "pos: " << position << " distance: " << distance <<
        (isReverse ? " down" : " up");
    printSimpleInfo("[CalibStorage] ", 
            "focus sampled added. " + ss.str() + "\n"); 
}


void CalibStorageContract::saveFocusSamples() {
    if (focusSamplesUpdated) {
        focusSamplesUpdated = false;
        writeFocusSamples();
    }
}


std::vector<CalibStorageContract::FocusSample> 
CalibStorageContract::getFocusSamples() {
    return focusSamples;
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


void CalibStorageContract::loadExPointPairs() {
    cv::FileStorage fs(rootDir + rangeFinderExtrinsicPointFile, 
            cv::FileStorage::READ);
    if (fs.isOpened()) {

        cv::Mat points3d;
        cv::Mat points2d;

        fs["points3d"] >> points3d;
        fs["points2d"] >> points2d;

        /* extract points from matrices and store as pair */
        for (int row=0; row < points3d.rows; row++) {
            cv::Point3f p3d;
            p3d.x = points3d.at<float>(row, 0);
            p3d.y = points3d.at<float>(row, 1);
            p3d.y = points3d.at<float>(row, 2);

            cv::Point2f p2d;
            p2d.x = points2d.at<float>(row, 0);
            p2d.y = points2d.at<float>(row, 1);

            PointPair3d2d ppair(p3d, p2d);
            exPointPairs.push_back(ppair);
        }

        /* alose save matrices */
        exPointPairsObject = points3d;
        exPointPairsImage = points2d;
    }
}


void CalibStorageContract::writeExPointPairFile() {

    cv::FileStorage fs(rootDir + rangeFinderExtrinsicPointFile, 
            cv::FileStorage::WRITE);
    if (fs.isOpened()) {

        cv::Mat points3d((int)exPointPairs.size(), 3, cv::DataType<float>::type);
        cv::Mat points2d((int)exPointPairs.size(), 2, cv::DataType<float>::type);

        /* copy points to matrices */
        int row = 0;
        for (PointPair3d2d ppair : exPointPairs) {
            cv::Point3f p3d = ppair.first;
            cv::Point2f p2d = ppair.second;

            points3d.at<float>(row, 0) = p3d.x;
            points3d.at<float>(row, 1) = p3d.y;
            points3d.at<float>(row, 2) = p3d.z;

            points2d.at<float>(row, 0) = p2d.x;
            points2d.at<float>(row, 1) = p2d.y;

            row++;
        }

        /* store matrices in file */
        fs << "points3d" << points3d;
        fs << "points2d" << points2d;

        printSimpleInfo("[CalibStorage] ", "stored extrinsic point pairs.\n"); 
    }
}


void CalibStorageContract::loadExPointPairs3d() {
    cv::FileStorage fs(rootDir + rangeFinderExtrinsicPoint3dFile, 
            cv::FileStorage::READ);
    if (fs.isOpened()) {

        cv::Mat points0;
        cv::Mat points1;

        fs["points_0"] >> points0;
        fs["points_1"] >> points1;

        /* extract points from matrices and store as pair */
        for (int row=0; row < points0.rows; row++) {
            cv::Point3f p0;
            p0.x = points0.at<float>(row, 0);
            p0.y = points0.at<float>(row, 1);
            p0.z = points0.at<float>(row, 2);

            cv::Point3f p1;
            p1.x = points1.at<float>(row, 0);
            p1.y = points1.at<float>(row, 1);
            p1.z = points1.at<float>(row, 2);

            PointPair3d3d ppair(p0, p1);
            exPointPairs3d.push_back(ppair);
        }

        /* alose save matrices */
        exPointPairs3d_0 = points0;
        exPointPairs3d_1 = points1;
    }
}


void CalibStorageContract::writeExPointPair3dFile() {

    cv::FileStorage fs(rootDir + rangeFinderExtrinsicPoint3dFile, 
            cv::FileStorage::WRITE);
    if (fs.isOpened()) {

        cv::Mat points0((int)exPointPairs3d.size(), 3, 
                cv::DataType<float>::type);
        cv::Mat points1((int)exPointPairs3d.size(), 3, 
                cv::DataType<float>::type);

        /* copy points to matrices */
        int row = 0;
        for (PointPair3d3d ppair : exPointPairs3d) {
            cv::Point3f p0 = ppair.first;
            cv::Point3f p1 = ppair.second;

            points0.at<float>(row, 0) = p0.x;
            points0.at<float>(row, 1) = p0.y;
            points0.at<float>(row, 2) = p0.z;

            points1.at<float>(row, 0) = p1.x;
            points1.at<float>(row, 1) = p1.y;
            points1.at<float>(row, 2) = p1.z;

            row++;
        }

        /* store matrices in file */
        fs << "points_0" << points0;
        fs << "points_1" << points1;

        printSimpleInfo("[CalibStorage] ", "stored extrinsic 3d point pairs.\n"); 
    }
}


void CalibStorageContract::loadFocusSamples() {
    std::ifstream csvfile;
    csvfile.open((rootDir + focusSamplesFile).c_str());

    /* check if file exists */
    if (!csvfile.is_open()) {
        return;
    }
    
    std::string line;
    while (std::getline(csvfile, line)) {
        /* tokenize */
        std::stringstream lineStream(line);
        std::string cell;
        std::stringstream cellStream;
        while (std::getline(lineStream, cell, ',')) {
            cellStream << cell << " ";
        }

        /* read the cells */
        float distance = 0.0;
        float position = 0.0;
        bool isReverse = false;
        cellStream >> distance;
        cellStream >> position;
        cellStream >> isReverse;

        /* add sample */
        FocusSample sample = {distance, position, isReverse};
        focusSamples.push_back(sample); 
    }
}


void CalibStorageContract::writeFocusSamples() {
    std::ofstream csvfile;
    csvfile.open((rootDir + focusSamplesFile).c_str());

    for (auto &sample : focusSamples) {
        csvfile << sample.distance << ", " 
                << sample.position << ", "
                << sample.isReverse << std::endl;
    }

    csvfile.close();
}


