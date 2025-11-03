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

#include "asterixdecoderbase.h"

#include "datasourcelineinfo.h"

#include <boost/array.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/mutex.hpp>

#include <cstddef>
#include <map>

#define MAX_UDP_READ_SIZE_VALUE 1024*1024
#define MAX_ALL_RECEIVE_SIZE_VALUE 100*1024*1024

/**
 * Decode ASTERIX from network lines.
*/
class ASTERIXNetworkDecoder : public ASTERIXDecoderBase
{
public:
    ASTERIXNetworkDecoder(ASTERIXImportSource& source, 
                          const ASTERIXImportTaskSettings* settings = nullptr);
    virtual ~ASTERIXNetworkDecoder();

    std::string name() const override final { return "ASTERIXNetworkDecoder"; }

    virtual std::string currentDataSourceName() const override { return "Network"; }

    static constexpr unsigned int MAX_UDP_READ_SIZE    = MAX_UDP_READ_SIZE_VALUE;
    static constexpr unsigned int MAX_ALL_RECEIVE_SIZE = MAX_ALL_RECEIVE_SIZE_VALUE;

    static const unsigned int MaxUDPReadSize;
    static const unsigned int MaxAllReceiveSize;

protected:
    void start_impl() override final;
    void stop_impl() override final;

    bool canRun_impl() const override final;
    bool canDecode_impl() const override final;

private:
    std::map<unsigned int, std::map<std::string, std::shared_ptr<DataSourceLineInfo>>> ds_lines_;
    // ds_id -> line str ->(ip, port)

    boost::interprocess::interprocess_semaphore receive_semaphore_;
    std::map<unsigned int, std::unique_ptr<boost::array<char, MAX_ALL_RECEIVE_SIZE>>> receive_buffers_copy_; // line->buf
    std::map<unsigned int, size_t> receive_copy_buffer_sizes_; // line -> len

    boost::mutex receive_buffers_mutex_;
    std::map<unsigned int, std::unique_ptr<boost::array<char, MAX_ALL_RECEIVE_SIZE>>> receive_buffers_; // line -> buf
    std::map<unsigned int, size_t> receive_buffer_sizes_; // line -> len

    boost::posix_time::ptime last_receive_decode_time_;

    void storeReceivedData (unsigned int line, const char* data, unsigned int length);
};
