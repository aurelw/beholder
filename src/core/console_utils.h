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

#ifndef __CONSOLE_UTILS_H__
#define __CONSOLE_UTILS_H__

#include <pcl/common/common_headers.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

void printSimpleInfo(const std::string &str);
void printSimpleInfo(const std::string &label, const std::string &text);

void printBrightInfo(const std::string &str);
void printBrightInfo(const std::string &label, const std::string &text);

void printWarning(const std::string &str);
void printWarning(const std::string &label, const std::string &text);

std::string affineToString(const Eigen::Affine3f aff);

#define DEBUG_PRINT(a) std::cout << "[DEBUG] " << __FILE__ << ":" <<__LINE__ << "  " #a << " -> " << a << std::endl

#endif

