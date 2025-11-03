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

#include "asterixnetworkdecoder.h"
#include "asteriximporttask.h"
#include "stringconv.h"
#include "compass.h"
#include "datasourcemanager.h"
#include "udpreceiver.h"

#include <jasterix/jasterix.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

using namespace nlohmann;
using namespace Utils;
using namespace std;

const unsigned int ASTERIXNetworkDecoder::MaxUDPReadSize    = MAX_UDP_READ_SIZE;
const unsigned int ASTERIXNetworkDecoder::MaxAllReceiveSize = MAX_ALL_RECEIVE_SIZE;

/**
 * @param source Import source to retrieve data from.
 * @param settings If set, external settings will be applied, otherwise settings will be retrieved from the import task.
*/
ASTERIXNetworkDecoder::ASTERIXNetworkDecoder(ASTERIXImportSource& source, 
                                             const ASTERIXImportTaskSettings* settings)
:   ASTERIXDecoderBase(source, settings)
,   receive_semaphore_((unsigned int)0)
{
    traced_assert(source.isNetworkType());

    ds_lines_ = COMPASS::instance().dataSourceManager().getNetworkLines();

    for (auto& ds_it : ds_lines_)
    {
        loginf << ds_it.first << ":";

        for (auto& line_it : ds_it.second)
            loginf << "\t" << line_it.first << " " << line_it.second->asString();
    }
}

/**
*/
ASTERIXNetworkDecoder::~ASTERIXNetworkDecoder() = default;

/**
*/
bool ASTERIXNetworkDecoder::canDecode_impl() const
{
    //@TODO: try to retrieve some data from network lines?
    return true;
}

/**
*/
bool ASTERIXNetworkDecoder::canRun_impl() const
{
    return COMPASS::instance().dataSourceManager().getNetworkLines().size(); // there are network lines defined
}

/**
*/
void ASTERIXNetworkDecoder::start_impl()
{
    boost::asio::io_context io_context;

    unsigned int line;

    vector<unique_ptr<UDPReceiver>> udp_receivers;

    int max_lines = settings().max_network_lines_;

    loginf << "max lines " << max_lines;

    for (auto& ds_it : ds_lines_)
    {
        //loginf << ds_it.first << ":";

        unsigned int line_cnt = 0;

        for (auto& line_it : ds_it.second)
        {
            line = String::getAppendedInt(line_it.first);
            traced_assert(line >= 1 && line <= 4);
            line--; // technical counting starts at 0

            loginf << "setting up ds_id " << ds_it.first
                   << " line " << line << " info " << line_it.second->asString();

            auto data_callback = [this,line](const char* data, unsigned int length) {
                this->storeReceivedData(line, data, length);
            };

            udp_receivers.emplace_back(new UDPReceiver(io_context, line_it.second, data_callback, MaxUDPReadSize));

            ++line_cnt;

            if (max_lines != -1 && line_cnt == (unsigned int) max_lines)
                break; // HACK only do first line
        }
    }

    loginf << "running iocontext";

    boost::thread t(boost::bind(&boost::asio::io_context::run, &io_context));
    t.detach();

    last_receive_decode_time_ = boost::posix_time::microsec_clock::local_time();

    unsigned int line_id = 0;

    while (isRunning())
    {
        receive_semaphore_.wait();

        if (!isRunning())
            break;

        {
            boost::mutex::scoped_lock lock(receive_buffers_mutex_);

            if (receive_buffer_sizes_.size() // not paused, any data received, 1sec passed
                    && (boost::posix_time::microsec_clock::local_time()
                        - last_receive_decode_time_).total_milliseconds() > 1000)
            {
                logdbg << "copying data "
                       << receive_buffer_sizes_.size() << " buffers  max " << MaxAllReceiveSize;

                // copy data
                for (auto& size_it : receive_buffer_sizes_)
                {
                    line_id = size_it.first;

                    traced_assert(receive_buffers_.count(line_id));

                    traced_assert(receive_buffer_sizes_.at(line_id) <= MaxAllReceiveSize);

                    if (!receive_buffers_copy_.count(line_id))
                        receive_buffers_copy_[line_id].reset(new boost::array<char, MaxAllReceiveSize>());

                    *receive_buffers_copy_.at(line_id) = *receive_buffers_.at(line_id);
                    receive_copy_buffer_sizes_[line_id] = size_it.second;

                }

                receive_buffer_sizes_.clear();

                lock.unlock();

                last_receive_decode_time_ = boost::posix_time::microsec_clock::local_time();

                logdbg << "processing copied data";

                for (auto& size_it : receive_copy_buffer_sizes_)
                {
                    line_id = size_it.first;

                    traced_assert(receive_buffers_copy_.count(line_id));

                    auto callback = [this, line_id](std::unique_ptr<nlohmann::json> data, size_t num_frames,
                            size_t num_records, size_t num_errors) {

                        if (job() && !job()->obsolete())
                            job()->netJasterixCallback(std::move(data), line_id, num_frames, num_records, num_errors);
                    };

                    task().jASTERIX()->decodeData((char*) receive_buffers_copy_.at(line_id)->data(),
                                                   size_it.second, callback);
                }

                logdbg << "done";

                receive_copy_buffer_sizes_.clear();

                if (job() && !job()->obsolete())
                    job()->forceBlockingDataProcessing();
            }
        }
    }

    loginf << "shutting down iocontext";

    io_context.stop();
    traced_assert(io_context.stopped());

    t.timed_join(100);

    //done_ = true; // done set in outer run function

    loginf << "done";
}

/**
*/
void ASTERIXNetworkDecoder::stop_impl()
{
    // stop decoding
    receive_semaphore_.post(); // wake up loop
}

/**
*/
void ASTERIXNetworkDecoder::storeReceivedData(unsigned int line, 
                                              const char* data, 
                                              unsigned int length)
{
    if (!isRunning())
        return;

    //loginf << "sender " << sender_id;

    boost::mutex::scoped_lock lock(receive_buffers_mutex_);

    if (length + receive_buffer_sizes_[line] >= MaxAllReceiveSize)
    {
        logerr << "overload, too much data in buffer";
        return;
    }

    if (!receive_buffers_.count(line))
        receive_buffers_[line].reset(new boost::array<char, MaxAllReceiveSize>());

    traced_assert(receive_buffers_[line]);

    for (unsigned int cnt=0; cnt < length; ++cnt)
        receive_buffers_[line]->at(receive_buffer_sizes_[line]+cnt) = data[cnt];

    receive_buffer_sizes_[line] += length;

    lock.unlock();

    receive_semaphore_.post();
}
