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

#ifndef __BASIC_APP_OPTIONS_H__
#define __BASIC_APP_OPTIONS_H__

#include "rig_config.h"


class BasicAppOptions {

    public:

        BasicAppOptions(int argc, char **argv);

        virtual void overloadRigConfig(const RigConfig &rc);

    public:

        std::string rigConfigFile = "";
        bool gotRigConfigFile = false;
        std::string calibStorageDir = "";
        bool gotCalibStorageDir = false;

    protected:

        bool gotFile(const std::string &fname);
        bool gotDir(const std::string &dir);

};

#endif

