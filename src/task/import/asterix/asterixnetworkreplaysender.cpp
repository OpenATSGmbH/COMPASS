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

#include "asterixnetworkreplaysender.h"
#include "logger.h"
#include "traced_assert.h"

#include <boost/asio.hpp>

#include <chrono>
#include <fstream>
#include <vector>

using namespace std;

const unsigned int ASTERIXNetworkReplaySender::IOSSHeaderSize      = 8;
const unsigned int ASTERIXNetworkReplaySender::IOSSFrameOverhead   = 12; // 8 header + 4 padding
const double       ASTERIXNetworkReplaySender::StartupDelaySeconds = 1.0;

namespace
{
const double tod_24h_secs   = 24.0 * 60.0 * 60.0;
const double frame_time_lsb = 0.01;

// reads one IOSS frame header, returns false on eof/truncation
bool readFrameHeader(ifstream& file, unsigned int& frame_length, double& frame_time)
{
    unsigned char header[ASTERIXNetworkReplaySender::IOSSHeaderSize];

    file.read(reinterpret_cast<char*>(header), sizeof(header));

    if (file.gcount() == 0)
        return false; // clean eof

    if (file.gcount() != (streamsize)sizeof(header))
        return false; // truncated header

    frame_length = ((unsigned int)header[0] << 8) | (unsigned int)header[1];
    frame_time   = (((unsigned int)header[5] << 16) | ((unsigned int)header[6] << 8)
                    | (unsigned int)header[7]) * frame_time_lsb;

    return true;
}
}

/**
 * @param filename IOSS-framed ASTERIX recording to replay.
 * @param speed Replay speed factor, 1.0 replays in recorded time.
 * @param target_ip Target IP address the datagrams are sent to.
 * @param target_port Target UDP port.
 * @param reference_time Common pacing base in seconds of day for several senders
 * replaying simultaneously; unset means the file's own first frame time.
*/
ASTERIXNetworkReplaySender::ASTERIXNetworkReplaySender(const std::string& filename, float speed,
                                                       const std::string& target_ip,
                                                       unsigned int target_port,
                                                       const boost::optional<double>& reference_time)
    : filename_(filename), speed_(speed), target_ip_(target_ip), target_port_(target_port),
      reference_time_(reference_time)
{
    traced_assert(speed_ > 0.0f);
}

/**
*/
ASTERIXNetworkReplaySender::~ASTERIXNetworkReplaySender()
{
    stop();
}

/**
*/
void ASTERIXNetworkReplaySender::start()
{
    traced_assert(!thread_.joinable());

    loginf << "file '" << filename_ << "' speed " << speed_
           << " target " << target_ip_ << ":" << target_port_;

    stop_requested_ = false;
    running_        = true;

    thread_ = std::thread(&ASTERIXNetworkReplaySender::run, this);
}

/**
*/
void ASTERIXNetworkReplaySender::stop()
{
    stop_requested_ = true;

    if (thread_.joinable())
        thread_.join();

    running_ = false;
}

/**
 * Parses the leading IOSS frames of the file, validating frame lengths and the
 * contained data block envelopes. Returns the first frame time in seconds of day.
*/
boost::optional<double> ASTERIXNetworkReplaySender::firstFrameTime(const std::string& filename,
                                                                   std::string* error)
{
    ifstream file(filename, ios::binary);

    if (!file.is_open())
    {
        if (error)
            *error = "file '" + filename + "' could not be opened";
        return {};
    }

    boost::optional<double> first_time;

    const unsigned int num_check_frames = 5;

    for (unsigned int cnt = 0; cnt < num_check_frames; ++cnt)
    {
        unsigned int frame_length {0};
        double       frame_time   {0.0};

        if (!readFrameHeader(file, frame_length, frame_time))
        {
            if (cnt == 0)
            {
                if (error)
                    *error = "file '" + filename + "' contains no complete IOSS frame";
                return {};
            }
            break; // fewer frames than checked, already validated ones suffice
        }

        if (frame_length < IOSSFrameOverhead + 3) // content must hold a CAT/LEN envelope
        {
            if (error)
                *error = "frame " + to_string(cnt) + " length " + to_string(frame_length)
                         + " too small, not IOSS framing";
            return {};
        }

        unsigned int content_length = frame_length - IOSSFrameOverhead;

        vector<char> content(content_length);
        file.read(content.data(), content_length);

        if (file.gcount() != (streamsize)content_length)
        {
            if (error)
                *error = "frame " + to_string(cnt) + " truncated, not IOSS framing";
            return {};
        }

        // content must start with a plausible ASTERIX data block: CAT byte plus
        // big-endian length not exceeding the content
        unsigned int cat = (unsigned char)content[0];
        unsigned int data_block_length =
            ((unsigned int)(unsigned char)content[1] << 8) | (unsigned int)(unsigned char)content[2];

        if (cat == 0 || data_block_length < 3 || data_block_length > content_length)
        {
            if (error)
                *error = "frame " + to_string(cnt) + " content invalid, cat " + to_string(cat)
                         + " data block length " + to_string(data_block_length)
                         + " content length " + to_string(content_length) + ", not IOSS framing";
            return {};
        }

        file.seekg(4, ios::cur); // skip padding

        if (!first_time)
            first_time = frame_time;
    }

    return first_time;
}

/**
*/
std::string ASTERIXNetworkReplaySender::effectiveTargetIP(const std::string& configured_ip)
{
    boost::system::error_code ec;
    boost::asio::ip::address addr = boost::asio::ip::make_address(configured_ip, ec);

    if (!ec && addr.is_multicast())
        return configured_ip;

    return "127.0.0.1";
}

/**
 * Worker thread: walks the IOSS frames, paces by frame time deltas and sends
 * each frame's content as one UDP datagram.
*/
void ASTERIXNetworkReplaySender::run()
{
    using namespace std::chrono;

    loginf << "started";

    size_t num_frames_sent {0};
    size_t num_bytes_sent  {0};
    size_t num_send_errors {0};

    try
    {
        ifstream file(filename_, ios::binary);

        if (!file.is_open())
            throw runtime_error("file '" + filename_ + "' could not be opened");

        boost::asio::io_context io_context;
        boost::asio::ip::udp::socket socket(io_context);
        boost::asio::ip::address target_addr = boost::asio::ip::make_address(target_ip_);
        boost::asio::ip::udp::endpoint target(target_addr, (unsigned short)target_port_);

        socket.open(boost::asio::ip::udp::v4());

        if (target_addr.is_multicast())
        {
            // deliver to local group members only: loop back on this host, and
            // hops 0 so the replayed recording is never transmitted onto a real
            // network - required to replay against unchanged operational contexts
            socket.set_option(boost::asio::ip::multicast::enable_loopback(true));
            socket.set_option(boost::asio::ip::multicast::hops(0));
        }

        // receivers are opened asynchronously by the decode job, wait for them to bind
        steady_clock::time_point wall_start =
            steady_clock::now() + duration_cast<steady_clock::duration>(
                                      duration<double>(StartupDelaySeconds));

        bool   first_frame       {true};
        double first_frame_time  {0.0};
        double prev_frame_time   {0.0};
        double time_accumulated  {0.0}; // 24h wrap unrolling

        vector<char> content;

        while (!stop_requested_)
        {
            unsigned int frame_length {0};
            double       frame_time   {0.0};

            if (!readFrameHeader(file, frame_length, frame_time))
                break; // eof or truncation

            if (frame_length < IOSSFrameOverhead)
            {
                logwrn << "frame " << num_frames_sent << " length " << frame_length
                       << " too small, stopping replay";
                break;
            }

            unsigned int content_length = frame_length - IOSSFrameOverhead;

            content.resize(content_length);
            file.read(content.data(), content_length);

            if (file.gcount() != (streamsize)content_length)
            {
                logwrn << "frame " << num_frames_sent << " truncated, stopping replay";
                break;
            }

            file.seekg(4, ios::cur); // skip padding

            if (first_frame)
            {
                first_frame       = false;
                first_frame_time  = reference_time_.has_value() ? reference_time_.value() : frame_time;
                prev_frame_time   = frame_time;
            }

            // unroll midnight wrap
            if (frame_time + time_accumulated < prev_frame_time - tod_24h_secs / 2.0)
                time_accumulated += tod_24h_secs;

            frame_time += time_accumulated;
            prev_frame_time = frame_time;

            steady_clock::time_point send_time =
                wall_start + duration_cast<steady_clock::duration>(
                                 duration<double>((frame_time - first_frame_time) / speed_));

            // sleep in small slices to stay responsive to stop()
            while (!stop_requested_ && steady_clock::now() < send_time)
                this_thread::sleep_for(milliseconds(min<long long>(
                    50, duration_cast<milliseconds>(send_time - steady_clock::now()).count() + 1)));

            if (stop_requested_)
                break;

            boost::system::error_code ec;
            socket.send_to(boost::asio::buffer(content.data(), content_length), target, 0, ec);

            if (ec)
            {
                ++num_send_errors;

                if (num_send_errors == 1)
                    logwrn << "send failed with '" << ec.message() << "'";
            }
            else
            {
                ++num_frames_sent;
                num_bytes_sent += content_length;
            }
        }
    }
    catch (const exception& e)
    {
        logerr << "replay failed with '" << e.what() << "'";
    }

    loginf << "done, sent " << num_frames_sent << " frames " << num_bytes_sent
           << " bytes, send errors " << num_send_errors
           << (stop_requested_ ? ", stopped" : ", end of file");

    running_ = false;

    emit doneSignal();
}
