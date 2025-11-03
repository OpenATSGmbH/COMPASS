/*
 * This file is part of OpenATS COMPASS.
 *
 * COMPASS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * COMPASS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with COMPASS. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "datasourcelineinfo.h"

#include <boost/asio.hpp>

class UDPReceiver
{
public:
    UDPReceiver(boost::asio::io_context& io_context, std::shared_ptr<DataSourceLineInfo> line_info,
                std::function<void(const char*, unsigned int)> data_callback, unsigned int max_read_size);


    void handle_receive_from(const boost::system::error_code& error,
                             size_t bytes_recvd);

private:
    std::shared_ptr<DataSourceLineInfo> line_info_;

    boost::asio::ip::udp::endpoint socket_endpoint_;
    boost::asio::ip::udp::endpoint sender_endpoint_;
    boost::asio::ip::udp::socket socket_;

    bool has_sender_address_ {false};
    boost::asio::ip::address sender_addr_;

    std::function<void(const char*, unsigned int)> data_callback_; // const std::string&,

    unsigned int max_read_size_ {0};
    char* data_ {nullptr};
};
