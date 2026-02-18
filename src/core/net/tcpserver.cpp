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
#include "stringconv.h"
#include "traced_assert.h"

using namespace std;
using namespace Utils;

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
    unsigned int required_size = str.size() + 1;

    if (!data_) // init if required
    {
        if (required_size > data_size_)
            data_size_ = required_size;

        data_.reset(new char [data_size_]);
    }
    else if (required_size > data_size_) // check size
    {
        data_size_ = required_size;
        data_.reset(new char [data_size_]);
    }

    traced_assert(data_);
    str.copy(data_.get(), str.size());
    do_write(str.size());
}

void TCPSession::do_read()
{
    auto self(shared_from_this());

    if (!data_)
        data_.reset(new char [data_size_]);

    traced_assert(data_);
    socket_.async_read_some(boost::asio::buffer(data_.get(), data_size_),
                            [this, self](boost::system::error_code ec, std::size_t length)
    {
        if (!ec)
        {
            traced_assert(data_);

            std::string tmp;
            tmp.assign(data_.get(), length);

            vector<string> string_vec = String::split(tmp, '\n');

            //loginf << "SOCKET GOT MSG '" << tmp << "' LEN " << length << " num sub strings " << string_vec.size();

            boost::mutex::scoped_lock lock(str_data_mutex_);
            str_data_.insert(str_data_.end(), string_vec.begin(), string_vec.end());

            do_read();
        }
    });
}

void TCPSession::do_write(std::size_t length)
{
    traced_assert(data_);

    auto self(shared_from_this());
    boost::asio::async_write(socket_, boost::asio::buffer(data_.get(), length),
                             [this, self](boost::system::error_code ec, std::size_t /*length*/)
    {
        traced_assert(!ec);
    });

    //do_read();
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
