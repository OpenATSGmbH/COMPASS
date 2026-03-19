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

#include "tcpserver.h"
#include "logger.h"
#include "traced_assert.h"

#include <istream>

using namespace std;

TCPSession::TCPSession(boost::asio::ip::tcp::socket socket)
    : socket_(std::move(socket))
{
    loginf;
}

void TCPSession::start()
{
    do_read();
}

bool TCPSession::hasStrData()
{
    boost::mutex::scoped_lock lock(str_data_mutex_);
    return str_data_.size();
}

std::vector<std::string> TCPSession::getStrData()
{
    boost::mutex::scoped_lock lock(str_data_mutex_);

    traced_assert(str_data_.size());
    return move(str_data_);
}

void TCPSession::sendStrData(const std::string& str)
{
    auto self(shared_from_this());
    auto buf = std::make_shared<std::string>(str);

    boost::asio::async_write(socket_, boost::asio::buffer(*buf),
                             [self, buf](boost::system::error_code ec, std::size_t /*length*/)
    {
        traced_assert(!ec);
    });
}

void TCPSession::do_read()
{
    auto self(shared_from_this());

    boost::asio::async_read_until(socket_, read_buf_, '\n',
                                  [this, self](boost::system::error_code ec, std::size_t /*bytes_transferred*/)
    {
        if (!ec)
        {
            std::string line;
            std::istream is(&read_buf_);
            std::getline(is, line);
            // any data beyond '\n' stays in read_buf_ for the next read

            {
                boost::mutex::scoped_lock lock(str_data_mutex_);
                str_data_.push_back(std::move(line));
            }

            do_read();
        }
    });
}

TCPServer::TCPServer(boost::asio::io_context& io_context, short port)
    : acceptor_(io_context, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port))
{
}

TCPServer::~TCPServer()
{

}

void TCPServer::start()
{
    traced_assert(!started_);

    started_ = true;

    int one = 1;
    setsockopt(acceptor_.native_handle(), SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &one, sizeof(one));

    do_accept();
}

void TCPServer::do_accept()
{
    acceptor_.async_accept(
                [this](boost::system::error_code ec, boost::asio::ip::tcp::socket socket)
    {
        if (!ec)
        {
            session_ = std::make_shared<TCPSession>(std::move(socket));
            session_->start();
        }

        do_accept();
    });
}

bool TCPServer::hasSession()
{
    return session_ != nullptr;
}

bool TCPServer::hasStrData()
{
    traced_assert(session_);
    return session_->hasStrData();
}

std::vector<std::string> TCPServer::getStrData()
{
    traced_assert(session_);
    return session_->getStrData();
}

void TCPServer::sendStrData(const std::string& str)
{
    traced_assert(session_);
    session_->sendStrData(str);
}
