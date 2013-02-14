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

#include "console_utils.h"

void printSimpleInfo(const std::string &str) {
    pcl::console::print_color(stdout, 
            pcl::console::TT_BRIGHT, 
            pcl::console::TT_BLUE, 
            str.c_str());
}


void printSimpleInfo(const std::string &label, const std::string &text) {
    printSimpleInfo(label);
    pcl::console::print_info(text.c_str());
}


void printBrightInfo(const std::string &str) {
    pcl::console::print_color(stdout, 
            pcl::console::TT_BRIGHT, 
            pcl::console::TT_GREEN, 
            str.c_str());
}


void printBrightInfo(const std::string &label, const std::string &text) {
    printBrightInfo(label);
    pcl::console::print_info(text.c_str());
}


void printWarning(const std::string &str) {
    pcl::console::print_warn(str.c_str());
}


void printWarning(const std::string &label, const std::string &text) {
    printWarning(label);
    pcl::console::print_info(text.c_str());
}
