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

/* core */
#include "rig_config.h"
#include "basicappoptions.h"
#include "motor_factory.h"


void print_usage() {
    std::cout << "--rigconfig <file>" << std::endl;
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

    if (!appopt.gotRigConfigFile) {
        pcl::console::print_error(
                "No rigconfig provided. --rigconfig <file>\n");
        exit(1);
    }
    /*****************************/

    /* the basic rig config */
    RigConfig rc;
    rc.loadFromFile(appopt.rigConfigFile);

    /* motor */
    Motor::Ptr fMotor = createMotor(rc, "default");
    if (!fMotor) {
        pcl::console::print_error("Can't instantiate Motor Driver!\n");
        exit(1);
    }
    fMotor->connect();

    std::cout << "'1.0'-position; [u]-up; [d]-down; [e]-exit;" << std::endl;
    while (true) {
        std::cout << "~# ";
        std::string input;
        std::cin >> input;

        if (input == "e") {
            break;
        } else if (input == "u") {
            fMotor->stepUp();
        } else if (input == "d") {
            fMotor->stepDown();
        } else {
            std::stringstream ss;
            ss << input;
            float position;
            if (!(ss >> position).fail()) {
                fMotor->setPosition(position);
            }
        }
    }


}

