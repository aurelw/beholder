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

#include <pcl/io/pcd_io.h>

#include "rig_config.h"
#include "cameraparameters.h"
#include "camerainterface_factory.h"

int main() {

    /* the basic rig config */
    RigConfig rc;
    rc.loadFromFile("rigconfig_camerainterface.xml");

    CameraInterface::Ptr cif = createCameraInterface(rc);
    cv::Mat img = cif->captureImage();

    cv::imshow("capture", img);
    cv::imwrite("test_capture.jpg", img);

    while (true) {
        int key = cv::waitKey(10);
        if (key == 27) { // ESC
            break;
        } else if (key > 0) {
            img = cif->captureImage();
            cv::imshow("capture", img);
        }
    }

}

