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

#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>

#include "rig_config.h"
#include "basicappoptions.h"


void print_usage() {
    std::cout << "[--rigconfig <file> | -g]  [-o <file>]" << std::endl;
}


int main(int argc, char **argv) {

    /*** command line arguments ***/
    BasicAppOptions appopt(argc, argv);

    bool onlyGenerate = false;
    onlyGenerate = pcl::console::find_switch(argc, argv, "-g");

    if (!onlyGenerate && !appopt.gotRigConfigFile) {
        pcl::console::print_error(
                "No rigconfig provided. --rigconfig <path>\n");
        exit(1);
    }

    /* output file for new rigconfig */
    std::string outputFile = "new_config.xml"; 
    bool gotOutputFile = (pcl::console::parse(argc, argv, "-o", outputFile) != -1);

    /**************************/

    /* the basic rig config */
    RigConfig rc;
    if (!onlyGenerate) {
        rc.loadFromFile(appopt.rigConfigFile);
    }


    /* store result */
    if (gotOutputFile) {
        rc.saveToFile(outputFile);
    } else {
        rc.saveToFile(appopt.rigConfigFile);
    }


}

