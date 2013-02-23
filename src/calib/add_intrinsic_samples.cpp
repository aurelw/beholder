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

#include "opencv2/opencv.hpp"

#include <pcl/console/parse.h>

#include "calibstorage_contract.h"
#include "basicappoptions.h"


int main(int argc, char **argv) {

    BasicAppOptions appopt(argc, argv);

    if (!appopt.gotCalibStorageDir) {
        pcl::console::print_error(
            "No calibration storage directory provided. --calibstorage <path>\n");
        return 1;
    }

    CalibStorageContract calibStorage(appopt.calibStorageDir);

    /* get a list of all .jpg provided on the command line */
    std::vector<int> fileindices = 
        pcl::console::parse_file_extension_argument(argc, argv, ".jpg");

    for (int i : fileindices) {
        cv::Mat img = cv::imread(argv[i], CV_LOAD_IMAGE_COLOR);
        calibStorage.addMainIntrinsic(img);
    }

}

