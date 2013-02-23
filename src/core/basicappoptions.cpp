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

#include "basicappoptions.h"

#include <boost/filesystem.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>


BasicAppOptions::BasicAppOptions(int argc, char **argv) {

    /* basic config files and paths */
    pcl::console::parse(argc, argv, "--rigconfig", rigConfigFile);
    gotRigConfigFile = gotFile(rigConfigFile);
    pcl::console::parse(argc, argv, "--calibstorage", calibStorageDir);
    gotCalibStorageDir = gotDir(calibStorageDir);

}


void BasicAppOptions::overloadRigConfig(const RigConfig &rc) {
}


bool BasicAppOptions::gotFile(const std::string &fname) {
    boost::filesystem::path p(fname);
    return boost::filesystem::is_regular_file(p);
}


bool BasicAppOptions::gotDir(const std::string &dir) {
    boost::filesystem::path p(dir);
    return boost::filesystem::is_directory(p);
}

