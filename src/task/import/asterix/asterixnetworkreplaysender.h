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

#include <QObject>

#include <boost/optional.hpp>

#include <atomic>
#include <string>
#include <thread>

/**
 * Replays an IOSS-framed ASTERIX recording as UDP datagrams, simulating live
 * network input. Each IOSS frame's content (raw CAT/LEN data block) is sent as
 * one datagram to the given target endpoint, paced by the frame header times,
 * so the receiving side (the normal ASTERIXNetworkDecoder UDP receivers) sees
 * the same traffic pattern as from a real sensor feed.
 *
 * The sender is passive regarding time-of-day handling: the records' ToD stays
 * untouched on the wire. Aligning the recording time to the current wall clock
 * is done via the existing ToD override in the import task settings (see
 * RTCommandImportASTERIXNetworkStart).
 *
 * IOSS frame layout (see data/jasterix_definitions/framings/ioss.json):
 * 2 bytes total frame length, 1 byte unknown, 1 byte board number,
 * 1 byte recording day, 3 bytes frame time (LSB 0.01 s),
 * content (frame length - 12 bytes), 4 bytes padding.
 */
class ASTERIXNetworkReplaySender : public QObject
{
    Q_OBJECT

signals:
    void doneSignal(); // emitted from the worker thread, connect queued

public:
    ASTERIXNetworkReplaySender(const std::string& filename, float speed,
                               const std::string& target_ip, unsigned int target_port,
                               const boost::optional<double>& reference_time = {});
    virtual ~ASTERIXNetworkReplaySender();

    void start();
    void stop(); // signals the worker thread to stop and joins it

    bool isRunning() const { return running_; }

    /// Parses the first IOSS frame and returns its frame time in seconds of day.
    /// Walks a few frames as sanity check, so this also validates that the file
    /// is IOSS-framed. Returns unset and fills error on failure.
    static boost::optional<double> firstFrameTime(const std::string& filename,
                                                  std::string* error = nullptr);

    /// Maps a configured line address to a usable replay target: multicast
    /// addresses are kept (sent host-local, see run()), everything else
    /// (listen-any "0.0.0.0", unicast listen addresses) becomes 127.0.0.1,
    /// which reaches a local receiver bound to that port.
    static std::string effectiveTargetIP(const std::string& configured_ip);

    static const unsigned int IOSSHeaderSize;     // bytes before the frame content
    static const unsigned int IOSSFrameOverhead;  // header + trailing padding
    static const double       StartupDelaySeconds; // wait before first send, receivers must bind

private:
    void run();

    std::string  filename_;
    float        speed_       {1.0f};
    std::string  target_ip_;
    unsigned int target_port_ {0};

    // common pacing base for several senders replaying simultaneously, so the
    // relative timing between their files is kept; unset = own first frame time
    boost::optional<double> reference_time_;

    std::atomic_bool  stop_requested_ {false};
    std::atomic_bool  running_        {false};
    std::thread       thread_;
};
