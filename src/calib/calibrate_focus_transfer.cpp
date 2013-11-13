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

#include <algorithm>

/* calib */
#include "calibstorage_contract.h"
#include "calib_utils.h"
#include "octave_interface.h"

/* core */
#include "rig_config.h"
#include "basicappoptions.h"
#include "console_utils.h"
#include "mathutils.h"
#include "bsplineinterpolation.h"


void print_usage() {
    std::cout << "--calibstorage <path> --rigconfig <file> -d <degree> -k <splineK>" << std::endl;
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

    if (!appopt.gotCalibStorageDir) {
        pcl::console::print_error(
            "No calibration storage directory provided. --calibstorage <path>\n");
        exit(1);
    }

    if (!appopt.gotRigConfigFile) {
        pcl::console::print_error(
                "No rigconfig provided. --rigconfig <path>\n");
        exit(1);
    }

    int poly_degree = 4;
    pcl::console::parse(argc, argv, "-d", poly_degree);

    int splineK = 4;
    pcl::console::parse(argc, argv, "-k", splineK);
    /*****************************/


    /* the calibration storage */
    CalibStorageContract calibStorage(appopt.calibStorageDir);

    /* the rig config */
    RigConfig rigConfig;
    rigConfig.loadFromFile(appopt.rigConfigFile);

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
    std::sort(upSamples.begin(), upSamples.end());
    std::sort(downSamples.begin(), downSamples.end());

    /* print some sample info */
    std::stringstream ss;
    ss << "total: " << samples.size()
       << "  up: " << upSamples.size()
       << "  down: " << downSamples.size() << std::endl;
    printSimpleInfo("[SAMPLES] ", ss.str());

    /* fit bspline */
    BSplineInterpolation upSpline(poly_degree, splineK);
    upSpline.fitFromData(upSamples);
    BSplineInterpolation::Samples upFitSamples;
    upFitSamples = upSpline.evaluateRange(0, 6, 0.05);
    
    /* print result */
    std::cout << samplesToOctaveString(upSamples, "up_");
    std::cout << samplesToOctaveString(upFitSamples, "upfit_");

    /* fit bspline */
    BSplineInterpolation downSpline(poly_degree, splineK);
    downSpline.fitFromData(downSamples);
    BSplineInterpolation::Samples downFitSamples;
    downFitSamples = downSpline.evaluateRange(0, 6, 0.05);
    
    /* print result */
    std::cout << samplesToOctaveString(downSamples, "down_");
    std::cout << samplesToOctaveString(downFitSamples, "downfit_");

#if 0
    /* fit polynomes */
    std::vector<float> upPoly, downPoly;
    OctaveInterface &octif = OctaveInterface::getInstance();
    upPoly = octif.polyfit(upSamples, poly_degree);
    downPoly = octif.polyfit(downSamples, poly_degree);

    /* check if polyfit was successful */
    if (upPoly.size() != poly_degree+1 || downPoly.size() != poly_degree+1) {
        printError("[PolyFit] ", "failed.\n");
        exit(1);
    } else {
        printBrightInfo("[PolyFit] ", "done.\n");
    }

    /* pad polynomes */
    for (int i=upPoly.size(); i<7; i++) {
        upPoly.insert(upPoly.begin(), 0.0f);
    }
    for (int i=downPoly.size(); i<7; i++) {
        downPoly.insert(downPoly.begin(), 0.0f);
    }

    /* store results */
    rigConfig.fMotorHa = upPoly[0];
    rigConfig.fMotorHb = upPoly[1];
    rigConfig.fMotorHc = upPoly[2];
    rigConfig.fMotorHd = upPoly[3];
    rigConfig.fMotorHe = upPoly[4];
    rigConfig.fMotorHf = upPoly[5];
    rigConfig.fMotorHg = upPoly[6];

    rigConfig.fMotorLa = downPoly[0];
    rigConfig.fMotorLb = downPoly[1];
    rigConfig.fMotorLc = downPoly[2];
    rigConfig.fMotorLd = downPoly[3];
    rigConfig.fMotorLe = downPoly[4];
    rigConfig.fMotorLf = downPoly[5];
    rigConfig.fMotorLg = downPoly[6];
#endif
    rigConfig.saveToFile(appopt.rigConfigFile);

}

