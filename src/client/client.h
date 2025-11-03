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

#include <QApplication>

class Client : public QApplication
{
public:
    Client(int& argc, char** argv);
    virtual ~Client();

    virtual bool notify(QObject* receiver, QEvent* event);

    bool quitRequested() const;

    bool run ();

private:

    std::string system_install_path_;
    bool quit_requested_{false};

    bool home_subdir_deletion_wanted_{false};
    bool config_and_data_copy_wanted_{false};

    std::string override_cfg_path_;

    std::string create_new_db_filename_;
    std::string open_db_filename_;

    std::string import_data_sources_filename_;

    std::string import_asterix_filename_;
    std::string import_asterix_filenames_;

    std::string import_asterix_pcap_filename_;
    std::string import_asterix_pcap_filenames_;

    std::string import_asterix_file_line_;
    std::string import_asterix_date_;
    std::string import_asterix_file_time_offset_;
    bool import_asterix_ignore_time_jumps_ {false};

    bool import_asterix_network_ {false};
    std::string import_asterix_network_time_offset_;
    int import_asterix_network_max_lines_ {-1};
    bool import_asterix_network_ignore_future_ts_ {false};
    std::string asterix_framing;
    std::string asterix_decoder_cfg;
    std::string import_asterix_parameters_;

    std::string import_json_filename_;

    std::string import_view_points_filename_;
    std::string import_gps_trail_filename_;
    std::string import_gps_parameters_;
    std::string import_sectors_filename_;

    bool calculate_radar_plot_positions_ {false};
    bool calculate_artas_tr_usage_ {false};

    bool reconstruct_references_ {false};

    bool load_data_ {false};

    std::string export_view_points_report_filename_;

    std::string evaluation_parameters_;
    bool evaluate_run_filter_ {false};
    bool evaluate_ {false};
    
    std::string export_report_name_; // 'EUROCAE ED-87E Evaluation'
    std::string export_report_directory_;
    std::string export_report_mode_{"PDF"}; // JSON','Latex','PDF'

    bool no_config_save_ {false};

    bool open_rt_cmd_port_ {false};
    bool enable_event_log_ {false};
    bool quit_ {false};

    bool expert_mode_ {false};

    bool do_assert_ {false};
    bool do_throw_ {false};
    bool do_numerical_crash_ {false};
    bool do_segfault_ {false};

    void checkAndSetupConfig();

    void checkNeededActions();
    void performNeededActions();

    void deleteCompleteHomeSubDir();
    void copyConfigurationAndData();
};

