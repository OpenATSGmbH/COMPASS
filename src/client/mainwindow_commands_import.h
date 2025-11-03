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

#include "rtcommand.h"
#include "rtcommand_macros.h"

namespace main_window
{
// import_view_points
struct RTCommandImportViewPointsFile : public rtcommand::RTCommand
{
    std::string filename_;

    virtual rtcommand::IsValid valid() const override;

    RTCommandImportViewPointsFile();

protected:
    virtual bool run_impl() override;

    DECLARE_RTCOMMAND(import_view_points, "imports view points JSON file with given filename, e.g. '/data/file1.json'")
    DECLARE_RTCOMMAND_OPTIONS
};

// import_asterix_file
struct RTCommandImportASTERIXFile : public rtcommand::RTCommand
{
    std::string filename_;
    std::string framing_;
    std::string line_id_;
    std::string date_str_;
    std::string time_offset_str_;
    bool        ignore_time_jumps_ {false};

    virtual rtcommand::IsValid valid() const override;

    RTCommandImportASTERIXFile();

protected:
    virtual bool run_impl() override;
    virtual bool checkResult_impl() override;

    DECLARE_RTCOMMAND(import_asterix_file,
                      "imports ASTERIX file with given filename, e.g. '/data/file1.ff'")
    DECLARE_RTCOMMAND_OPTIONS
};

// import_asterix_files
struct RTCommandImportASTERIXFiles : public rtcommand::RTCommand
{
    std::string filenames_;
    std::vector<std::string> split_filenames_;
    std::string framing_;
    std::string line_id_;
    std::string date_str_;
    std::string time_offset_str_;
    bool ignore_time_jumps_ {false};

    virtual rtcommand::IsValid valid() const override;

    RTCommandImportASTERIXFiles();

protected:
    virtual bool run_impl() override;
    virtual bool checkResult_impl() override;

    DECLARE_RTCOMMAND(import_asterix_files,
                      "imports multiple ASTERIX files with given filenames, e.g. '/data/file1.ff;/data/file2.ff'")
    DECLARE_RTCOMMAND_OPTIONS
};

// import_pcap_file
struct RTCommandImportASTERIXPCAPFile : public rtcommand::RTCommand
{
    std::string filename_;
    std::string line_id_;
    std::string date_str_;
    std::string time_offset_str_;
    bool ignore_time_jumps_ {false};

    virtual rtcommand::IsValid valid() const override;

    RTCommandImportASTERIXPCAPFile();

protected:
    virtual bool run_impl() override;
    virtual bool checkResult_impl() override;

    DECLARE_RTCOMMAND(import_asterix_pcap_file,
                      "imports ASTERIX PCAP file with given filename, e.g. '/data/file1.pcap'")
    DECLARE_RTCOMMAND_OPTIONS
};

// import_pcap_files
struct RTCommandImportASTERIXPCAPFiles : public rtcommand::RTCommand
{
    std::string filenames_;
    std::vector<std::string> split_filenames_;
    std::string line_id_;
    std::string date_str_;
    std::string time_offset_str_;
    bool ignore_time_jumps_ {false};

    virtual rtcommand::IsValid valid() const override;

    RTCommandImportASTERIXPCAPFiles();

protected:
    virtual bool run_impl() override;
    virtual bool checkResult_impl() override;

    DECLARE_RTCOMMAND(import_asterix_pcap_files,
                      "imports multiple ASTERIX PCAP files with given filenames, e.g. '/data/file1.pcap;/data/file2.pcap'")
    DECLARE_RTCOMMAND_OPTIONS
};

// import_asterix_network
struct RTCommandImportASTERIXNetworkStart : public rtcommand::RTCommand
{
    std::string time_offset_str_;
    int max_lines_ {-1};
    bool ignore_future_ts_ {false};

    virtual rtcommand::IsValid valid() const override;

    RTCommandImportASTERIXNetworkStart();

protected:
    virtual bool run_impl() override;

    DECLARE_RTCOMMAND(import_asterix_network, "imports ASTERIX from defined network UDP streams")
    DECLARE_RTCOMMAND_OPTIONS
};

// import_asterix_network_stop
struct RTCommandImportASTERIXNetworkStop : public rtcommand::RTCommand
{
    RTCommandImportASTERIXNetworkStop();

protected:
    virtual bool run_impl() override;

    DECLARE_RTCOMMAND(import_asterix_network_stop, "stops import ASTERIX from network")
    DECLARE_RTCOMMAND_NOOPTIONS
};

// import_json
struct RTCommandImportJSONFile : public rtcommand::RTCommand
{
    std::string filename_;

    virtual rtcommand::IsValid valid() const override;

    RTCommandImportJSONFile();

protected:
    virtual bool run_impl() override;

    DECLARE_RTCOMMAND(import_json, "imports JSON file with given filename, e.g. ’/data/file1.json’")
    DECLARE_RTCOMMAND_OPTIONS
};

// import_gps
struct RTCommandImportGPSTrail : public rtcommand::RTCommand
{
    std::string  filename_;
    std::string  name_;
    int          sac_;
    int          sic_;
    bool         has_tod_offset_ = false;
    double       tod_offset_     =  0.0;
    std::string  date_;
    std::string  mode_3a_code_;
    std::string  aircraft_address_;
    std::string  aircraft_id_;

    virtual rtcommand::IsValid valid() const override;

    RTCommandImportGPSTrail();

protected:
    virtual bool run_impl() override;

    DECLARE_RTCOMMAND(import_gps_trail, "imports gps trail NMEA with given filename, e.g. ’/data/file2.txt’")
    DECLARE_RTCOMMAND_OPTIONS
};

}
