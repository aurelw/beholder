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
#include <fstream>

/* core */
#include "rig_config.h"
#include "basicappoptions.h"

/* calib */
#include "calib_utils.h"
#include "calibstorage_contract.h"

void print_usage() {
    std::cout << "--calibstorage <path> --rigconfig <file>" << std::endl;
}


int main(int argc, char **argv) {

    /*** command line arguments ***/
    /* help */
    if (pcl::console::find_switch(argc, argv, "-h") ||
        pcl::console::find_switch(argc, argv, "--help")) 
    {
        print_usage();
        exit(0);
    }

    BasicAppOptions appopt(argc, argv);

    if (!appopt.gotCalibStorageDir) {
        pcl::console::print_error(
                "No calibstorage provided. --calibstorage <path>\n");
        exit(1);
    }

    if (!appopt.gotRigConfigFile) {
        pcl::console::print_error(
                "No rigconfig provided. --rigconfig <file>\n");
        exit(1);
    }

    float upper_limit = 5.0;
    pcl::console::parse(argc, argv, "-l", upper_limit);
    /*****************************/

    /* calibration storage */
    CalibStorageContract calibStorage(appopt.calibStorageDir);

    /* the basic rig config */
    RigConfig rc;
    rc.loadFromFile(appopt.rigConfigFile);

    /* get samples for calib storage */
    std::vector<CalibStorageContract::FocusSample> samples;
    samples = calibStorage.getFocusSamples();

    /* decompose into two vectors */
    std::vector<std::pair<float, float>> upSamples, downSamples;
    for (auto &sample : samples) {
        if (sample.isReverse) {
            downSamples.push_back(std::make_pair(
                        sample.distance, sample.position));
        } else {
            upSamples.push_back(std::make_pair(
                        sample.distance, sample.position));
        }
    }

    /* compose plotting file */
    std::string up_s = samplesToOctaveString(upSamples, "up");
    std::string down_s = samplesToOctaveString(upSamples, "down");

    std::stringstream ss_polyup;
    ss_polyup << "poly_up = [";
    ss_polyup << rc.fMotorHa << ",";
    ss_polyup << rc.fMotorHb << ",";
    ss_polyup << rc.fMotorHc << ",";
    ss_polyup << rc.fMotorHd << ",";
    ss_polyup << rc.fMotorHe << ",";
    ss_polyup << rc.fMotorHf << ",";
    ss_polyup << rc.fMotorHg << ",";
    ss_polyup << "];" << std::endl;

    std::stringstream ss_polydown;
    ss_polydown << "poly_up = [";
    ss_polydown << rc.fMotorLa << ",";
    ss_polydown << rc.fMotorLb << ",";
    ss_polydown << rc.fMotorLc << ",";
    ss_polydown << rc.fMotorLd << ",";
    ss_polydown << rc.fMotorLe << ",";
    ss_polydown << rc.fMotorLf << ",";
    ss_polydown << rc.fMotorLg << ",";
    ss_polydown << "];" << std::endl;

    std::ofstream ofs;
    ofs.open("plot_up_points.m");
    ofs << up_s << down_s; 
    ofs << ss_polyup.str() <<  ss_polydown.str();
    ofs << "xvalues = linspace(0," << upper_limit << ", 400);" << std::endl;
    ofs << "yvalues = polyval(poly, xvalues);" << std::endl;
    ofs << "plot(upx, upy, \"@1\");" << std::endl;
    ofs << "hold on;" << std::endl;
    ofs << "plot(xvalues, yvalues);" << std::endl;

    ofs.close();
}

