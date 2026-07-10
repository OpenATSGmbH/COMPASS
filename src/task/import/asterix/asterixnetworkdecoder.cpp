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
#include "asynctask.h"
#include "stringconv.h"
#include "compass.h"
#include "datasourcelineinfo.h"
#include "db_context_manager.h"
#include "udpreceiver.h"

#include <jasterix/jasterix.h>

#include <QSet>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <chrono>
#include <set>
#include <thread>

using namespace nlohmann;
using namespace Utils;
using namespace std;

const unsigned int ASTERIXNetworkDecoder::MaxUDPReadSize    = MAX_UDP_READ_SIZE;
const unsigned int ASTERIXNetworkDecoder::MaxAllReceiveSize = MAX_ALL_RECEIVE_SIZE;

const int    ASTERIXNetworkDecoder::ProbeDurationSeconds = 10;
const size_t ASTERIXNetworkDecoder::ProbeMaxBytesPerLine = 10 * 1024 * 1024;

/**
 * @param source Import source to retrieve data from.
 * @param settings If set, external settings will be applied, otherwise settings will be retrieved from the import task.
*/
ASTERIXNetworkDecoder::ASTERIXNetworkDecoder(ASTERIXImportTask& task,
                                             ASTERIXImportSource& source,
                                             const ASTERIXImportTaskSettings* settings)
:   ASTERIXDecoderBase(task, source, settings)
,   receive_semaphore_((unsigned int)0)
{
    traced_assert(source.isNetworkType());

    // build DataSourceLineInfo objects from context data sources' network line JSON
    auto& ctx_man = task.compass().dbContextManager();
    for (auto& [ds_id, ds] : ctx_man.activeContext().dataSources())
    {
        if (!ds.hasNetworkLines())
            continue;

        ds_names_[ds.id()] = ds.name();

        auto& nl = ds.info()["network_lines"];
        for (auto it = nl.begin(); it != nl.end(); ++it)
        {
            auto line_info = std::make_shared<DataSourceLineInfo>(it.key(), it.value());
            ds_lines_[ds.id()][it.key()] = line_info;
        }
    }

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
    // probe must have produced at least one section that decoded without error
    for (const auto& fi : source_.files())
    {
        for (const auto& sec : fi.sections)
            if (!sec.error.hasError() && !sec.records_per_category.empty())
                return true;
    }
    return false;
}

/**
*/
bool ASTERIXNetworkDecoder::canRun_impl() const
{
    return task().compass().dbContextManager().getNetworkLines().size(); // there are network lines defined
}

/**
 * Probes all network lines simultaneously for ProbeDurationSeconds, then runs
 * jASTERIX::analyzeData on each accumulated buffer. Synthesizes one
 * ASTERIXImportFileInfo per data source and one ASTERIXImportFileSection per line
 * so the same tree UI used for files/PCAP renders network probe results.
*/
void ASTERIXNetworkDecoder::checkDecoding_impl(bool force_recompute,
                                               AsyncTaskProgressWrapper* progress) const
{
    // canDecode(false) is called from canRun() during widget construction (e.g. to
    // decide whether the Import button should be enabled). Without this guard, every
    // such call would re-trigger a full 10s UDP probe + analyze on the GUI thread
    // without a progress dialog. If we already produced probe results, reuse them.
    if (!force_recompute && !source_.files().empty())
    {
        loginf << "skipping probe, reusing cached results from previous probe";
        return;
    }

    loginf << "starting network probe";

    using line_key_t = std::pair<unsigned int, std::string>; // (ds_id, line_key)

    // build synthetic file infos from cached ds_lines_ (populated in ctor): one FileInfo per
    // data source with network lines, one Section per line
    ASTERIXImportSource::FileInfos infos;
    std::map<line_key_t, std::pair<size_t, size_t>>           ix;       // (ds_id, line) -> (fi_idx, sec_idx)
    std::map<line_key_t, std::shared_ptr<DataSourceLineInfo>> targets;  // (ds_id, line) -> line info

    for (const auto& ds_it : ds_lines_)
    {
        unsigned int ds_id = ds_it.first;

        ASTERIXImportFileInfo fi;
        auto name_it       = ds_names_.find(ds_id);
        fi.filename        = name_it != ds_names_.end() ? name_it->second
                                                        : "ds " + std::to_string(ds_id);
        fi.decoding_tested = true;

        size_t fi_idx = infos.size();

        for (const auto& line_it : ds_it.second)
        {
            const std::string& line_key  = line_it.first;
            auto               line_info = line_it.second;

            ASTERIXImportFileSection sec;
            sec.id          = line_key;
            sec.description = line_key + ": " + line_info->asString();
            sec.used        = true;

            ix[{ds_id, line_key}]      = {fi_idx, fi.sections.size()};
            targets[{ds_id, line_key}] = line_info;

            fi.sections.push_back(std::move(sec));
        }

        infos.push_back(std::move(fi));
    }

    loginf << targets.size() << " network lines across " << infos.size() << " data sources";

    if (targets.empty())
    {
        loginf << "no network lines configured, skipping probe";
        const_cast<ASTERIXImportSource&>(source_).setNetworkProbeResults(std::move(infos));
        return;
    }

    if (progress)
    {
        progress->setMessage("Probing network for " + std::to_string(ProbeDurationSeconds) + " seconds ...",
                             false, true);
        progress->setSteps(0, ProbeDurationSeconds, true, true);
    }

    loginf << "opening receivers and collecting UDP traffic for " << ProbeDurationSeconds << " seconds";

    // open receivers and collect bytes per line
    boost::asio::io_context io_context;
    boost::mutex                                  buf_mutex;
    std::map<line_key_t, std::vector<char>>       buffers;
    std::map<line_key_t, size_t>                  packet_counts;
    std::set<line_key_t>                          first_packet_logged;
    std::vector<std::unique_ptr<UDPReceiver>>     receivers;
    std::map<line_key_t, std::string>             open_errors;

    for (auto& kv : targets)
    {
        const auto& key       = kv.first;
        const auto& line_info = kv.second;

        buffers[key].reserve(64 * 1024);

        auto cb = [&buf_mutex, &buffers, &packet_counts, &first_packet_logged, key]
                  (const char* data, unsigned int len)
        {
            boost::mutex::scoped_lock lk(buf_mutex);

            // log the very first packet for each line, so the data flow is visible
            // and obvious framing issues (junk leading bytes) are easy to spot.
            if (!first_packet_logged.count(key))
            {
                first_packet_logged.insert(key);
                loginf << "ds " << key.first << " " << key.second
                       << ": first packet " << len << " bytes, hex: "
                       << String::bytesToHex(data, len);
            }

            auto& b = buffers[key];
            if (b.size() + len > ProbeMaxBytesPerLine)
                return;
            b.insert(b.end(), data, data + len);
            ++packet_counts[key];
        };

        try
        {
            receivers.emplace_back(new UDPReceiver(io_context, line_info, cb, MaxUDPReadSize));
            logdbg << "opened receiver for ds " << key.first << " " << key.second
                   << " " << line_info->asString();
        }
        catch (const std::exception& e)
        {
            open_errors[key] = e.what();
            logwrn << "could not open receiver for ds " << key.first
                   << " line " << key.second << ": " << e.what();
        }
    }

    // run io_context on a worker thread for the probe duration
    boost::thread t([&io_context]() {
        try { io_context.run(); }
        catch (const std::exception& e) { logwrn << "io_context exception: " << e.what(); }
    });

    for (int sec = 0; sec < ProbeDurationSeconds; ++sec)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (progress)
            progress->increment(1, true);
    }

    // closing the io_context and joining the receiver thread can take a moment;
    // surface that step so the dialog isn't blank between probe and analyze.
    if (progress)
        progress->setMessage("Stopping receivers ...", false, true);

    io_context.stop();
    t.join();
    receivers.clear();

    loginf << "probe complete, analyzing " << targets.size() << " line buffers";

    if (progress)
        progress->setSteps(0, (int)targets.size(), true, true);

    int line_idx = 0;
    const int line_total = (int)targets.size();

    for (auto& kv : targets)
    {
        const auto& key = kv.first;
        auto&       sec = infos[ix[key].first].sections[ix[key].second];

        ++line_idx;

        // jASTERIX refresh per line is ~1s/line, so per-line status is no longer
        // a flicker - show concrete progress instead of one stable message.
        if (progress)
            progress->setMessage("Analyzing line " + std::to_string(line_idx) + "/"
                                 + std::to_string(line_total) + " ...", false, true);

        // refresh jASTERIX per line: the instance keeps cumulative num_records_/num_errors_
        // members, so once one buffer produces decode errors every subsequent analyzeData
        // call short-circuits via `if (num_errors_) break;` and returns 0/0. Re-creating
        // the instance per line gives each buffer a clean run.
        auto jasterix = task().jASTERIX(true);

        size_t bytes;
        size_t packets;
        {
            boost::mutex::scoped_lock lk(buf_mutex);
            bytes   = buffers[key].size();
            packets = packet_counts[key];
        }

        sec.total_size_bytes = bytes;
        sec.info             = std::to_string(packets) + " packets, " +
                               std::to_string(bytes)   + " bytes";

        if (!open_errors[key].empty())
        {
            loginf << "ds " << key.first << " " << key.second
                   << ": skipping analysis, receiver open failed: " << open_errors[key];
            sec.error.errtype = ASTERIXImportFileError::ErrorType::Invalid;
            sec.error.errinfo = open_errors[key];
            if (progress) progress->increment(1, true);
            continue;
        }

        if (bytes == 0)
        {
            loginf << "ds " << key.first << " " << key.second << ": no data received";
            sec.info = "no data received";
            sec.used = false;
            if (progress) progress->increment(1, true);
            continue;
        }

        // pre-analyze diagnostic: total bytes, packet count, and the buffer prefix
        // as hex. The first byte should be a valid ASTERIX CAT and bytes 1..2 the
        // big-endian data block length - anything else points to a framing/alignment
        // issue before jASTERIX even gets a chance.
        {
            boost::mutex::scoped_lock lk(buf_mutex);
            const auto& b = buffers[key];

            std::string first_block_info;
            if (b.size() >= 3)
            {
                unsigned cat = static_cast<unsigned char>(b[0]);
                unsigned len = (static_cast<unsigned char>(b[1]) << 8) |
                                static_cast<unsigned char>(b[2]);
                first_block_info = " first-block CAT=" + std::to_string(cat)
                                 + " declared-len=" + std::to_string(len);
            }

            loginf << "ds " << key.first << " " << key.second
                   << ": analyzing " << packets << " packets / " << bytes << " bytes;"
                   << first_block_info;
                   //<< " hex: " << String::bytesToHex(b.data(), b.size());
        }

        std::unique_ptr<nlohmann::json> analysis_info;
        try
        {
            boost::mutex::scoped_lock lk(buf_mutex);
            analysis_info = jasterix->analyzeData(buffers[key].data(), buffers[key].size(),
                                                  10000 /*record limit*/);
        }
        catch (const std::exception& e)
        {
            loginf << "ds " << key.first << " " << key.second
                   << ": analyze threw: " << e.what();
            sec.error.errtype = ASTERIXImportFileError::ErrorType::DecodingFailed;
            sec.error.errinfo = std::string("analyze failed: ") + e.what();
            if (progress) progress->increment(1, true);
            continue;
        }

        if (!analysis_info || !analysis_info->is_object()
            || !analysis_info->contains("num_records")
            || !analysis_info->contains("num_errors"))
        {
            sec.error.errtype = ASTERIXImportFileError::ErrorType::DecodingFailed;
            sec.error.errinfo = "Decoding failed";
            if (progress) progress->increment(1, true);
            continue;
        }

        sec.error.analysis_info = *analysis_info;

        unsigned int num_errors  = analysis_info->at("num_errors");
        unsigned int num_records = analysis_info->at("num_records");

        loginf << "ds " << key.first << " " << key.second
               << ": analyze result records=" << num_records << " errors=" << num_errors;

        if (num_errors)
        {
            sec.error.errtype = ASTERIXImportFileError::ErrorType::DecodingFailed;
            sec.error.errinfo = "Decoding errors: " + std::to_string(num_errors);
            if (progress) progress->increment(1, true);
            continue;
        }

        //store categories found in the stream but skipped during decoding
        sec.skipped_categories = ASTERIXSkippedCategoryInfo::fromAnalysisInfo(*analysis_info);

        if (!num_records)
        {
            sec.error.errtype = ASTERIXImportFileError::ErrorType::DecodingFailed;
            if (!sec.skipped_categories.empty())
                sec.error.errinfo = "Only skipped categories: "
                                    + ASTERIXSkippedCategoryInfo::asString(sec.skipped_categories);
            else
                sec.error.errinfo = "No records decoded";
            if (progress) progress->increment(1, true);
            continue;
        }

        // collect categories and per-category counts (mirror file/pcap decoders)
        std::set<std::string> categories;
        sec.records_per_category.clear();

        for (const auto& sac_sic : analysis_info->items())
        {
            if (!sac_sic.value().is_object())
                continue;
            if (sac_sic.key() == "skipped_categories")
                continue;
            for (const auto& category : sac_sic.value().items())
            {
                bool cat_ok;
                auto cat = QString::fromStdString(category.key()).toInt(&cat_ok);
                if (!cat_ok)
                    continue;
                categories.insert(String::categoryString(cat));
                if (category.value().is_object() && category.value().contains("count"))
                    sec.records_per_category[(unsigned int)cat] +=
                        category.value().at("count").get<size_t>();
            }
        }

        std::string content;
        for (const auto& c : categories)
            content += (content.empty() ? "" : ", ") + c;

        for (const auto& cat_it : sec.skipped_categories)
            content += (content.empty() ? "" : ", ")
                       + String::categoryString(cat_it.first) + " (skipped)";

        sec.contentinfo = content;

        if (progress) progress->increment(1, true);
    }

    const_cast<ASTERIXImportSource&>(source_).setNetworkProbeResults(std::move(infos));
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

                    auto callback = [this, line_id](std::unique_ptr<nlohmann::json> data, size_t total_num_bytes,
                            size_t num_frames, size_t num_records, size_t num_errors) {

                        if (job() && !job()->obsolete())
                            job()->netJasterixCallback(std::move(data), line_id, num_frames, num_records, num_errors);
                    };

                    task().jASTERIX()->decodeData((char*) receive_buffers_copy_.at(line_id)->data(),
                                                   size_it.second, callback, false, true);
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
