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

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>

/* calib */
#include "calibstorage_contract.h"

/* core */
#include "rig_config.h"
#include "basicappoptions.h"
#include "console_utils.h"
#include "mathutils.h"
#include "kinfuwrapper.h"

/* gui */
#include "calib_visualizer.h"


class CalibrateTrackerExtrinsic {

    public:

        typedef boost::shared_ptr<pcl::gpu::CaptureOpenNI> CapturePtr;
        typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

    public:

        CalibrateTrackerExtrinsic(const RigConfig &rc);
        void run();
        void captureClouds();
        void registerClouds();
        void saveClouds();
        void loadClouds();
        void displayClouds();

        Eigen::Affine3f trackerExtrinsic;

    private:

        int rangeFingerKinectId;
        int trackerKinectId;

        void initTracker(const int id);
        CapturePtr capture;
        KinfuWrapper::Ptr kinfu;

        /* visualization */
        CalibVisualizer visualizer;

        void captureScene();
        Eigen::Affine3f registerScenes(Cloud::ConstPtr cloud0,
                Cloud::ConstPtr cloud1);
        Cloud::Ptr postProcessScene(Cloud::ConstPtr inCloud);

        Cloud::Ptr targetCloud;
        Cloud::Ptr regCloud;

        pcl::PCDWriter writer;
};


void CalibrateTrackerExtrinsic::saveClouds() {
    writer.writeBinaryCompressed<pcl::PointXYZ>("targetCloud.pcd", *targetCloud); 
    writer.writeBinaryCompressed<pcl::PointXYZ>("regCloud.pcd", *regCloud); 
}


void CalibrateTrackerExtrinsic::loadClouds() {
    std::cout << "Loading clouds" << std::endl;
    targetCloud.reset(new Cloud);
    regCloud.reset(new Cloud);
    pcl::io::loadPCDFile<pcl::PointXYZ>("targetCloud.pcd", *targetCloud);
    pcl::io::loadPCDFile<pcl::PointXYZ>("regCloud.pcd", *regCloud);
}


void CalibrateTrackerExtrinsic::displayClouds() {
    visualizer.setRegistration(targetCloud, regCloud);
    visualizer.setDrawRegistration(true);
}


CalibrateTrackerExtrinsic::CalibrateTrackerExtrinsic(const RigConfig& rc) {
    /* get the device ids of both kinects */
    rangeFingerKinectId = std::stoi(rc.rangefinderDeviceID);
    trackerKinectId = std::stoi(rc.trackingCameraDeviceID);

    //FIXME don't override
    rangeFingerKinectId = 1;
    trackerKinectId = 0;

    /* start tracking */
    initTracker(rangeFingerKinectId);

    /* init visualizer */
    visualizer.start();
    visualizer.setDrawPlainCloud(true);
}


void CalibrateTrackerExtrinsic::initTracker(const int id) {
    capture.reset(new pcl::gpu::CaptureOpenNI);

    capture->open(id);
    capture->setRegistration(true);
    // FIXME get this from the openni interface
    capture->depth_focal_length_VGA = 525;

    /* if there is a kinfu running, close it and wait */
    if (kinfu) {
        kinfu.reset();
        sleep(5);
    }

    kinfu.reset(new KinfuWrapper);
    kinfu->setCaptureSource(*capture);
    kinfu->init();
}


void CalibrateTrackerExtrinsic::captureClouds() {

    /* capture cloud for the range finder */
    printSimpleInfo("[Capturing]", " range finder kinect.\n");
    Eigen::Affine3f firstRfPose = kinfu->getLastPose();
    captureScene();
    Eigen::Affine3f lastRfPose = kinfu->getLastPose();
    Cloud::Ptr rfCloud = kinfu->getFullCloud();
    rfCloud = postProcessScene(rfCloud);


    /* relative pose for movement */
    Eigen::Affine3f relPose = firstRfPose.inverse() * lastRfPose;

    /* display result and query to continue */
    visualizer.setPlainCloud(rfCloud);
    char key;
    while (!visualizer.getKeyEvent(key) || key != 'c') {
        sleep(0.1);
    }

    /* capture cloud for the tracker */
    printSimpleInfo("[Capturing]", " tracking kinect.\n");
    initTracker(trackerKinectId);
    Eigen::Affine3f firstTrPose = kinfu->getLastPose();
    captureScene();
    Cloud::Ptr trCloud = kinfu->getFullCloud();
    trCloud = postProcessScene(trCloud);

    /* display result and query to continue */
    visualizer.setPlainCloud(trCloud);

    /* rearange both pointclouds into global coordinate system */
    targetCloud.reset(new Cloud(*rfCloud));
    regCloud.reset(new Cloud(*trCloud));
    pcl::transformPointCloud(*targetCloud, *targetCloud, lastRfPose.inverse());
    //pcl::transformPointCloud(*regCloud, *regCloud, relPose);
    pcl::transformPointCloud(*regCloud, *regCloud, firstTrPose.inverse());

}


void CalibrateTrackerExtrinsic::registerClouds() {

    /* do a guess for extrinsic */
    Eigen::Vector3f axis(1.0, 0.0, 0.0);
    float angle = 1.57; // 90 degrees
    Eigen::AngleAxisf exRotGuess(angle, axis);
    Eigen::Affine3f exGuess(exRotGuess); 
    pcl::transformPointCloud(*regCloud, *regCloud, exGuess);

    /* register clouds */
#if 1
    printSimpleInfo("[Registration]", " registering sensors.... ");
    std::cout.flush();
    Eigen::Affine3f sceneEx = registerScenes(targetCloud, regCloud);
    std::cout << "done." << std::endl;
#endif

    char key;
    while (!visualizer.getKeyEvent(key) || key != 'c') {
        sleep(0.1);
    }

    Cloud::Ptr tRegCloud(new Cloud);
    pcl::transformPointCloud(*regCloud, *tRegCloud, sceneEx.inverse());

    /* display registration result */
    visualizer.setDrawPlainCloud(false);
    visualizer.setDrawRegistration(true);
    visualizer.setRegistration(targetCloud, tRegCloud);

    /* compensate for initial guess */
    sceneEx = exGuess.inverse() * sceneEx;
    std::cout << affineToString(sceneEx) << std::endl;
    visualizer.addCoordinateSystem(sceneEx.inverse(), "ext", 0.5, true);

    while (!visualizer.getKeyEvent(key) || key != 'c') {
        sleep(0.1);
    }

    trackerExtrinsic = sceneEx;
}


void CalibrateTrackerExtrinsic::run() {
    captureScene();
    registerClouds();
}


Eigen::Affine3f CalibrateTrackerExtrinsic::registerScenes(
        Cloud::ConstPtr cloud0, Cloud::ConstPtr cloud1)
{
    /* align clouds */

    //pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; 
    icp.setInputCloud(cloud0);
    icp.setInputTarget(cloud1);

    //icp.setMaxCorrespondenceDistance (1);
    //icp.setMaximumIterations(3000);
    //icp.setTransformationEpsilon(1e-16);
    //icp.setEuclideanFitnessEpsilon(5);

    pcl::PointCloud<pcl::PointXYZ> alignCloud;
    icp.align(alignCloud);
    Eigen::Matrix4f mat;
    mat = icp.getFinalTransformation();
    Eigen::Affine3f icpTrans(mat);
 
    return icpTrans;
}


CalibrateTrackerExtrinsic::Cloud::Ptr CalibrateTrackerExtrinsic::postProcessScene(
        Cloud::ConstPtr inCloud) 
{

    Cloud::Ptr outCloud(new Cloud);

    /* remove outliers */
//FIXME broken, just scatters points
/*
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(inCloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(0.3);
    sor.filter(*outCloud);
*/
    /* down sample */
    float voxelSize = 0.04;
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setInputCloud(inCloud);
    grid.setLeafSize(voxelSize, voxelSize, voxelSize);
    grid.filter(*outCloud);

    return outCloud;
}


void CalibrateTrackerExtrinsic::captureScene() {

    while (true) {
        kinfu->spinOnce();
        visualizer.setPlainCloud(kinfu->getLastCloud());
        visualizer.removeCoordinateSystem("kinfu_pos");
        visualizer.addCoordinateSystem(kinfu->getLastPose(), "kinfu_pos");

        /* handle key events */
        char key;
        if (visualizer.getKeyEvent(key)) {
            if (key == 'v') {
                break;
            }
        }

    } //while

    visualizer.removeCoordinateSystem("kinfu_pos");
}


void print_usage() {
    std::cout << "--rigconfig <file>" << std::endl;
}


int main(int argc, char **argv) {

    /*** command line arguments ***/
    BasicAppOptions appopt(argc, argv);

    /* help */
    if (pcl::console::find_switch(argc, argv, "-h") ||
        pcl::console::find_switch(argc, argv, "--help")) 
    {
        print_usage();
        exit(0);
    }

    bool doSave = pcl::console::find_switch(argc, argv, "-s");
    bool doLoad = pcl::console::find_switch(argc, argv, "-l");

    if (!appopt.gotRigConfigFile) {
        pcl::console::print_error(
                "No rigconfig provided. --rigconfig <path>\n");
        exit(1);
    }

    /*****************************/

    /* the rig config */
    RigConfig rigConfig;
    rigConfig.loadFromFile(appopt.rigConfigFile);

    CalibrateTrackerExtrinsic calibTrackerEx(rigConfig);

    if (doLoad) {
        calibTrackerEx.loadClouds();
        calibTrackerEx.displayClouds();
    } else {
        calibTrackerEx.captureClouds();
        calibTrackerEx.displayClouds();
        if (doSave) {
            calibTrackerEx.saveClouds();
        }
    }

    calibTrackerEx.registerClouds();

    /* save extrinsic to rigconfig */
    cv::Mat exRot, exTrans;
    affine3fToTransRotVec(calibTrackerEx.trackerExtrinsic, exTrans, exRot);
    rigConfig.trackingCameraExTranslation = exTrans;
    rigConfig.trackingCameraExRotationVec = exRot;
    rigConfig.saveToFile(appopt.rigConfigFile);

}

