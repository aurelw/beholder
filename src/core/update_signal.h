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
   * GNU General Public License for more details
   * You should have received a copy of the GNU General Public License
   * along with Beholder. If not, see <http://www.gnu.org/licenses/>. */

#ifndef __UPDATE_SIGNAL_H__
#define __UPDATE_SIGNAL_H__

#include <iostream>

#include <boost/signals2.hpp>
#include <boost/bind.hpp>
#include <boost/signals2/connection.hpp>

#include <pcl/common/common_headers.h>


class UpdateSignal {

    public: 
        typedef boost::signals2::signal<void ()>  signal_t;
        typedef boost::signals2::connection  connection_t;

    public:
        connection_t connect(signal_t::slot_function_type subscriber);
        void disconnect(connection_t subscriber);

    protected:
        signal_t m_sig;
        virtual void update();

};

#endif

