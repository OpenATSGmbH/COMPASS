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

#include "client.h"

#include "compass.h"
#include "config.h"
#include "configurationmanager.h"
#include "files.h"
#include "global.h"
#include "logger.h"
#include "stringconv.h"
#include "taskmanager.h"
#include "asteriximporttask.h"
#include "mainwindow.h"
#include "rtcommand_manager.h"
#include "projectionmanager.h"
#include "util/system.h"

#include "json.hpp"
#include "util/tbbhack.h"

#include "traced_assert.h"

#include <QApplication>
#include <QMessageBox>
#include <QSurfaceFormat>
#include <QSplashScreen>
#include <QStyleFactory>
#include <QThreadPool>
#include <QFontDatabase>

#include "util/tbbhack.h"

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"

#include <string>
#include <locale.h>
#include <thread>

#if USE_EXPERIMENTAL_SOURCE == true
#include <osgDB/Registry>

#include "cpl_conv.h"
#endif

using namespace std;
using namespace Utils;

namespace po = boost::program_options;

std::string APP_FILENAME;

Client::Client(int& argc, char** argv) : QApplication(argc, argv)
{
    setlocale(LC_ALL, "C");

    APP_FILENAME = argv[0];

    QSurfaceFormat format = QSurfaceFormat::defaultFormat();

 #ifdef OSG_GL3_AVAILABLE
    cout << "COMPASSClient: OSG_GL3_AVAILABLE true, version "
         << format.version().first << "." << format.version().second << endl;

//     format.setVersion(3, 3);
//     format.setProfile(QSurfaceFormat::CoreProfile);
//     format.setRenderableType(QSurfaceFormat::OpenGL);

//     osg::DisplaySettings::instance()->setGLContextVersion("3.3");
//     osg::DisplaySettings::instance()->setShaderHint(osg::DisplaySettings::SHADER_GL3);

 #else
    cout << "COMPASSClient: OSG_GL3_AVAILABLE false, version "
         << format.version().first << "." << format.version().second << endl;
    // format.setVersion(2, 0);
    // format.setProfile(QSurfaceFormat::CompatibilityProfile);
    // format.setRenderableType(QSurfaceFormat::OpenGL);
    //format.setOption(QSurfaceFormat::DebugContext); // scatterplot stops working if active

    // osg::DisplaySettings::instance()->setGLContextVersion("2.0");
    // osg::DisplaySettings::instance()->setShaderHint(osg::DisplaySettings::SHADER_GL2);
#endif
    //format.setDepthBufferSize(32); // scatterplot stops working if active
    //format.setAlphaBufferSize(8);

    format.setSamples(8);
    format.setStencilBufferSize(8);
    format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
    QSurfaceFormat::setDefaultFormat(format);

    //    QApplication::setAttribute(Qt::AA_ShareOpenGLContexts);

    //    QSurfaceFormat format;
    //    format.setRenderableType(QSurfaceFormat::OpenGL);
    //    format.setProfile(QSurfaceFormat::CoreProfile);
    //    format.setVersion(3,3);

    //    QSurfaceFormat format;
    //    format.setSwapBehavior(QSurfaceFormat::SwapBehavior::SingleBuffer); //DoubleBuffer
    //    format.setRedBufferSize(8);
    //    format.setGreenBufferSize(8);
    //    format.setBlueBufferSize(8);
    //    format.setAlphaBufferSize(8);
    //    format.setDepthBufferSize(32); //24
    //    format.setStencilBufferSize(8);
    //    format.setSwapInterval(0);
    //    format.setSamples(8);
    // QSurfaceFormat::setDefaultFormat(format);

    //    std::string import_json_filename;
    //    std::string import_json_schema;

    //    bool load_data {false};

    //    bool quit {false};

    cout << "COMPASSClient: qt platform in use " << QGuiApplication::platformName().toStdString() << endl;

    po::options_description desc("Allowed options");
    po::options_description hidden_options("Hidden options");

    desc.add_options()("help", "produce help message")
        ("reset,r", po::bool_switch(&config_and_data_copy_wanted_) ,"reset user configuration and data")
        ("override_cfg_path", po::value<std::string>(&override_cfg_path_),
         "overrides 'default' config subfolder to other value, e.g. 'org'")
        ("expert_mode", po::bool_switch(&expert_mode_) ,"set expert mode")
        ("create_db", po::value<std::string>(&create_new_db_filename_),
         "creates and opens new DuckDB database with given filename, e.g. '/data/file1.db'")
        ("open_db", po::value<std::string>(&open_db_filename_),
         "opens existing DuckDB database with given filename, e.g. '/data/file1.db'")
        ("import_data_sources_file", po::value<std::string>(&import_data_sources_filename_),
         "imports data sources JSON file with given filename, e.g. '/data/ds1.json'")
        ("import_view_points", po::value<std::string>(&import_view_points_filename_),
         "imports view points JSON file with given filename, e.g. '/data/file1.json'")
        ("import_asterix_file", po::value<std::string>(&import_asterix_filename_),
         "imports ASTERIX file with given filename, e.g. '/data/file1.ff'")
        ("import_asterix_files", po::value<std::string>(&import_asterix_filenames_),
         "imports multiple ASTERIX files with given filenames, e.g. '/data/file1.ff;/data/file2.ff'")
        ("import_asterix_pcap_file", po::value<std::string>(&import_asterix_pcap_filename_),
         "imports ASTERIX PCAP file with given filename, e.g. '/data/file1.pcap'")
        ("import_asterix_pcap_files", po::value<std::string>(&import_asterix_pcap_filenames_),
         "imports multiple ASTERIX PCAP files with given filenames, e.g. '/data/file1.pcap;/data/file2.pcap'")
        ("import_asterix_file_line", po::value<std::string>(&import_asterix_file_line_),
         "imports ASTERIX file with given line, e.g. 'L2'")
        ("import_asterix_date", po::value<std::string>(&import_asterix_date_),
         "imports ASTERIX file with given date, in YYYY-MM-DD format e.g. '2020-04-20'")
        ("import_asterix_file_time_offset", po::value<std::string>(&import_asterix_file_time_offset_),
         "used time offset during ASTERIX file import Time of Day override, in HH:MM:SS.ZZZ'")
        ("import_asterix_ignore_time_jumps", po::bool_switch(&import_asterix_ignore_time_jumps_),
         "imports ASTERIX file ignoring 24h time jumps")
        ("import_asterix_network", po::bool_switch(&import_asterix_network_),
         "imports ASTERIX from defined network UDP streams")
        ("import_asterix_network_time_offset", po::value<std::string>(&import_asterix_network_time_offset_),
         "additive time offset during ASTERIX network import, in HH:MM:SS.ZZZ'")
        ("import_asterix_network_max_lines", po::value<int>(&import_asterix_network_max_lines_),
         "maximum number of lines per data source during ASTERIX network import, 1..4")
        ("import_asterix_network_ignore_future_ts", po::bool_switch(&import_asterix_network_ignore_future_ts_),
         "ignore future timestamps during ASTERIX network import'")
        ("asterix_framing", po::value<std::string>(&asterix_framing),
         "sets ASTERIX framing, e.g. 'none', 'ioss', 'ioss_seq', 'rff'. if not set configuration value is used")
        ("asterix_decoder_cfg", po::value<std::string>(&asterix_decoder_cfg),
         "sets ASTERIX decoder config using JSON string, e.g. ''{\"10\":{\"edition\":\"0.31\"}}''"
         " (including one pair of single quotes)")
        ("import_asterix_parameters", po::value<std::string>(&import_asterix_parameters_),
         "ASTERIX import parameters as JSON string, e.g. ''{\"filter_modec_active\": true,\"filter_modec_max\": 50000.0,\"filter_modec_min\": -10000.0}'' (including one pair of single quotes)")

        ("import_json", po::value<std::string>(&import_json_filename_),
         "imports JSON file with given filename, e.g. '/data/file1.json'")
        //            ("json_schema", po::value<std::string>(&import_json_schema),
        //             "JSON file import schema, e.g. 'jASTERIX', 'OpenSkyNetwork', 'ADSBExchange', 'SDDL'")
        ("import_gps_trail", po::value<std::string>(&import_gps_trail_filename_),
         "imports gps trail NMEA with given filename, e.g. '/data/file2.txt'")
        ("import_gps_parameters", po::value<std::string>(&import_gps_parameters_),
         "import GPS parameters as JSON string, e.g. ''{\"callsign\": \"ENTRPRSE\", \"ds_name\": \"GPS Trail\", \"ds_sac\": 0, \"ds_sic\": 0, \"mode_3a_code\": 961, \"set_callsign\": true, \"set_mode_3a_code\": true, \"set_target_address\": true, \"target_address\": 16702992, \"tod_offset\": 0.0}'' (including one pair of single quotes)")
        ("import_sectors_json", po::value<std::string>(&import_sectors_filename_),
         "imports exported sectors JSON with given filename, e.g. '/data/sectors.json'")
        ("calculate_radar_plot_positions", po::bool_switch(&calculate_radar_plot_positions_),
         "calculate radar plot positions")
        ("calculate_artas_tr_usage", po::bool_switch(&calculate_artas_tr_usage_), "associate target reports based on ARTAS usage")
        ("reconstruct_references", po::bool_switch(&reconstruct_references_),
         "reconstruct references from sensor and tracker data")
        ("load_data", po::bool_switch(&load_data_), "load data after start")
        ("export_view_points_report", po::value<std::string>(&export_view_points_report_filename_),
         "export view points report after start with given filename, e.g. '/data/db2/report.tex")
        ("evaluate", po::bool_switch(&evaluate_), "run evaluation")
        ("evaluation_parameters", po::value<std::string>(&evaluation_parameters_),
         "evaluation parameters as JSON string, e.g. ''{\"current_standard\": \"test\", \"dbcontent_name_ref\": \"CAT062\", \"dbcontent_name_tst\": \"CAT020\"}'' (including one pair of single quotes)")
        ("evaluate_run_filter", po::bool_switch(&evaluate_run_filter_), "run evaluation filter before evaluation")
        ("export_eval_report", po::value<std::string>(&export_eval_report_filename_),
         "export evaluation report after start with given filename, e.g. '/data/eval_db2/report.tex")
        ("no_cfg_save", po::bool_switch(&no_config_save_), "do not save configuration upon quitting")
        ("open_rt_cmd_port", po::bool_switch(&open_rt_cmd_port_), "open runtime command port (default at 27960)")
        ("enable_event_log", po::bool_switch(&enable_event_log_), "collect warnings and errors in the event log")
        ("quit", po::bool_switch(&quit_), "quit after finishing all previous steps")
        ;

    // add hidden options
    hidden_options.add_options()
        ("assert", po::bool_switch(&do_assert_), "")
        ("throw", po::bool_switch(&do_throw_), "")
        ("numerical_crash", po::bool_switch(&do_numerical_crash_), "")
        ("segfault", po::bool_switch(&do_segfault_), "")
        ;

    // Print full command line for debugging
    cout << "COMPASSClient: command line: ";
    for (int i = 0; i < argc; ++i) {
        cout << "'" << argv[i] << "'";
        if (i < argc - 1) cout << " ";
    }
    cout << endl;

    try
    {
        po::options_description all_options;
        all_options.add(desc).add(hidden_options);

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, all_options), vm);
        po::notify(vm);

        if (vm.count("help"))
        {
            cout << desc << "\n";
            quit_requested_ = true;
            return;
        }
    }
    catch (exception& e)
    {
        throw runtime_error("COMPASSClient: unable to parse command line parameters: " +
                            string(e.what()));
    }

    checkAndSetupConfig();

    // check if more than 1 ASTERIX import operations are defined
    unsigned int import_count = 0;

    if (do_assert_)
        traced_assert(!do_assert_);

    if (do_throw_)
        throw std::runtime_error("error of interest");

    if (do_numerical_crash_)
    {
        int crash = 0/0;  // Integer division by zero
        double crash2 = std::sqrt(-1.0);  // NaN
        double crash3 = std::log(0.0);    // -Infinity
    }

    if (do_segfault_)
    {
        int* ptr = nullptr;
        *ptr = 42;  // Classic segfault
    }

    if (import_asterix_network_) import_count++;
    if (!import_asterix_filename_.empty()) import_count++;
    if (!import_asterix_filenames_.empty()) import_count++;
    if (!import_asterix_pcap_filename_.empty()) import_count++;
    if (!import_asterix_pcap_filenames_.empty()) import_count++;

    if (import_count > 1)
    {
        logerr << "unable run multiple ASTERIX import operations at the same time";
        return;
    }

    //    if (quit_requested_)
    //        return;

    //    if (import_json_filename.size() && !import_json_schema.size())
    //    {
    //        loginf << "schema name must be set for JSON import";
    //        return;
    //    }

}

bool Client::run ()
{
    // #define TBB_VERSION_MAJOR 4

    int num_threads = 0;

#if TBB_VERSION_MAJOR <= 2020

    // in appimage

    num_threads = tbb::task_scheduler_init::default_num_threads();;

    loginf << "started with " << num_threads << " threads (tbb old)";
    tbb::task_scheduler_init init {num_threads};

#else
    num_threads = oneapi::tbb::info::default_concurrency();

    tbb::global_control global_limit(tbb::global_control::max_allowed_parallelism, num_threads);

    loginf << "started with " << num_threads << " threads";
#endif

    loginf << "qt ideal thread count " << QThread::idealThreadCount()
           << " max thread count " << QThreadPool::globalInstance()->maxThreadCount()
           << " setting num_threads " << num_threads;

    QThreadPool::globalInstance()->setMaxThreadCount(num_threads);

    //axThreadCount() is QThread::idealThreadCount().

    //    unsigned int data_size = 10e6;
    //    tbb::parallel_for(uint(0), data_size, [&](unsigned int cnt) {
    //        double x = 0;

    //        for (unsigned int cnt2=1; cnt2 < data_size * data_size; ++ cnt2)
    //            x += (data_size * cnt) % cnt2;

    //        loginf << x;
    //    });

    // Enable High DPI support
    QGuiApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);

    // Set the "Fusion" style for better cross-platform results
    //QApplication::setStyle(QStyleFactory::create("Fusion"));

    QPixmap pixmap(Files::getImageFilepath("logo.png").c_str());
    QSplashScreen splash(pixmap);
    splash.show();

    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();

    while ((boost::posix_time::microsec_clock::local_time() - start_time).total_milliseconds() < 50)
    {
        QCoreApplication::processEvents();
    }

    if (open_rt_cmd_port_)
    {
        RTCommandManager::open_port_ = true; // has to be done before COMPASS ctor is called
    }

    loginf << "creating COMPASS instance...";

    //!this should be the first call to COMPASS instance!
    try
    {
        if (COMPASS::instance().darkMode()) // also needed as first call
        {
            // Define a simple dark mode stylesheet
            QPalette dark_pal;
            dark_pal.setColor(QPalette::Window, QColor(53, 53, 53));
            dark_pal.setColor(QPalette::WindowText, Qt::white);
            dark_pal.setColor(QPalette::Base, QColor(25, 25, 25));
            dark_pal.setColor(QPalette::AlternateBase, QColor(53, 53, 53));
            dark_pal.setColor(QPalette::ToolTipBase, Qt::white);
            dark_pal.setColor(QPalette::ToolTipText, Qt::white);
            dark_pal.setColor(QPalette::Text, Qt::white);
            dark_pal.setColor(QPalette::Button, QColor(75, 75, 75));
            dark_pal.setColor(QPalette::ButtonText, Qt::white);
            dark_pal.setColor(QPalette::BrightText, Qt::red);
            dark_pal.setColor(QPalette::Link, QColor(42, 130, 218));
            dark_pal.setColor(QPalette::Highlight, QColor(42, 130, 218));
            dark_pal.setColor(QPalette::HighlightedText, Qt::black);

            dark_pal.setColor(QPalette::Disabled, QPalette::Window, dark_pal.window().color().darker());
            dark_pal.setColor(QPalette::Disabled, QPalette::WindowText, dark_pal.windowText().color().darker());
            dark_pal.setColor(QPalette::Disabled, QPalette::Base, dark_pal.base().color().darker());
            dark_pal.setColor(QPalette::Disabled, QPalette::AlternateBase, dark_pal.alternateBase().color().darker());
            dark_pal.setColor(QPalette::Disabled, QPalette::Text, dark_pal.text().color().darker());
            dark_pal.setColor(QPalette::Disabled, QPalette::Button, dark_pal.button().color().darker());
            dark_pal.setColor(QPalette::Disabled, QPalette::ButtonText, dark_pal.buttonText().color().darker());
            dark_pal.setColor(QPalette::Disabled, QPalette::BrightText, dark_pal.brightText().color().darker());
            dark_pal.setColor(QPalette::Disabled, QPalette::Link, dark_pal.link().color().darker());
            dark_pal.setColor(QPalette::Disabled, QPalette::Highlight, dark_pal.highlight().color().darker());
            dark_pal.setColor(QPalette::Disabled, QPalette::HighlightedText, dark_pal.highlightedText().color().darker());

            // Apply the palette
            QApplication::setPalette(dark_pal);

            // (Optional) Apply a global stylesheet for further refinement
            QApplication::setStyleSheet("QToolTip { color: #ffffff; background-color: #2a2a2a; border: 1px solid white; }");
        }

        // make system your application font (applies to all widgets)
        if (getenv("APPDIR") != nullptr)
        {
            QFont system_font = QFontDatabase::systemFont(QFontDatabase::GeneralFont);

            system_font.setPointSizeF(system_font.pointSizeF() * COMPASS::instance().appFontScale());
            setFont(system_font);
        }

        COMPASS::instance().init(); //here everything created in compass instance should be available

        ProjectionManager::instance().test();
    }
    catch(const std::exception& e)
    {
        logerr << "creating COMPASS instance failed: " << e.what();
        quit_requested_ = true;
        System::printBacktrace();

        return false;
    }
    catch(...)
    {
        logerr << "creating COMPASS instance failed: unknown error";
        quit_requested_ = true;
        return false;
    }
    
    loginf << "created COMPASS instance";

    if (expert_mode_)
        COMPASS::instance().expertMode(true);

    MainWindow& main_window = COMPASS::instance().mainWindow();
    splash.raise();

    start_time = boost::posix_time::microsec_clock::local_time();
    while ((boost::posix_time::microsec_clock::local_time() - start_time).total_milliseconds()
            < 10)
    {
        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
        QThread::msleep(1);
    }

    main_window.show();
    splash.raise();

    start_time = boost::posix_time::microsec_clock::local_time();
    while ((boost::posix_time::microsec_clock::local_time() - start_time).total_milliseconds()
            < 10)
    {
        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
        QThread::msleep(1);
    }

    splash.finish(&main_window);

    RTCommandManager& rt_man = RTCommandManager::instance();

    if (no_config_save_)
        main_window.disableConfigurationSaving();

    try
    {
        if (create_new_db_filename_.size())
            rt_man.addCommandFromConsole("create_db " + create_new_db_filename_);

        if (open_db_filename_.size())
            rt_man.addCommandFromConsole("open_db " + open_db_filename_);

        if (import_data_sources_filename_.size())
            rt_man.addCommandFromConsole("import_data_sources " + import_data_sources_filename_);

        if (import_view_points_filename_.size())
            rt_man.addCommandFromConsole("import_view_points " + import_view_points_filename_);

        TaskManager& task_man = COMPASS::instance().taskManager();

        if (asterix_decoder_cfg.size())
            task_man.asterixImporterTask().asterixDecoderConfig(asterix_decoder_cfg);

        if (import_asterix_filename_.size())
        {
            string cmd = "import_asterix_file " + import_asterix_filename_;

            if (asterix_framing.size() && asterix_framing != "none")
                cmd += " --framing " + asterix_framing;

            if (import_asterix_file_line_.size())
                cmd += " --line " + import_asterix_file_line_;
            else
                cmd += " --line L1";

            if (import_asterix_date_.size())
                cmd += " --date " + import_asterix_date_;

            if (import_asterix_file_time_offset_.size())
                cmd += " --time_offset " + import_asterix_file_time_offset_;

            if (import_asterix_ignore_time_jumps_)
                cmd += " --ignore_time_jumps";

            rt_man.addCommandFromConsole(cmd);
        }

        if (import_asterix_filenames_.size())
        {
            string cmd = "import_asterix_files '" + import_asterix_filenames_ + "'";

            if (asterix_framing.size())
                cmd += " --framing " + asterix_framing;

            if (import_asterix_file_line_.size())
                cmd += " --line " + import_asterix_file_line_;
            else
                cmd += " --line L1";

            if (import_asterix_date_.size())
                cmd += " --date " + import_asterix_date_;

            if (import_asterix_file_time_offset_.size())
                cmd += " --time_offset " + import_asterix_file_time_offset_;

            if (import_asterix_ignore_time_jumps_)
                cmd += " --ignore_time_jumps";

            rt_man.addCommandFromConsole(cmd);
        }

        if (import_asterix_pcap_filename_.size())
        {
            string cmd = "import_asterix_pcap_file " + import_asterix_pcap_filename_;

            if (import_asterix_file_line_.size())
                cmd += " --line " + import_asterix_file_line_;
            else
                cmd += " --line L1";

            if (import_asterix_date_.size())
                cmd += " --date " + import_asterix_date_;

            if (import_asterix_file_time_offset_.size())
                cmd += " --time_offset " + import_asterix_file_time_offset_;

            if (import_asterix_ignore_time_jumps_)
                cmd += " --ignore_time_jumps";

            rt_man.addCommandFromConsole(cmd);
        }

        if (import_asterix_pcap_filenames_.size())
        {
            string cmd = "import_asterix_pcap_files '" + import_asterix_pcap_filenames_ + "'";

            if (import_asterix_file_line_.size())
                cmd += " --line " + import_asterix_file_line_;
            else
                cmd += " --line L1";

            if (import_asterix_date_.size())
                cmd += " --date " + import_asterix_date_;

            if (import_asterix_file_time_offset_.size())
                cmd += " --time_offset " + import_asterix_file_time_offset_;

            if (import_asterix_ignore_time_jumps_)
                cmd += " --ignore_time_jumps";

            rt_man.addCommandFromConsole(cmd);
        }

        if (import_asterix_network_)
        {
            string cmd = "import_asterix_network";

            if (import_asterix_network_time_offset_.size())
                cmd += " --time_offset " + import_asterix_network_time_offset_;

            if (import_asterix_network_max_lines_ != -1)
            {
                if (import_asterix_network_max_lines_ < 1 || import_asterix_network_max_lines_ > 4)
                    throw runtime_error(
                        "COMPASSClient: number of maximum network lines must be between 1 and 4");

                cmd += " --max_lines " + to_string(import_asterix_network_max_lines_);
            }

            if (import_asterix_network_ignore_future_ts_)
                cmd += " --ignore_future_ts";

            rt_man.addCommandFromConsole(cmd);
        }

        if (import_json_filename_.size())
            rt_man.addCommandFromConsole("import_json " + import_json_filename_);

        if (import_gps_trail_filename_.size())
            rt_man.addCommandFromConsole("import_gps_trail " + import_gps_trail_filename_);

        if (import_sectors_filename_.size())
            rt_man.addCommandFromConsole("import_sectors_json " + import_sectors_filename_);

        if (calculate_radar_plot_positions_)
            rt_man.addCommandFromConsole("calculate_radar_plot_positions");

        if (calculate_artas_tr_usage_)
            rt_man.addCommandFromConsole("calculate_artas_tr_usage");

        if (reconstruct_references_)
            rt_man.addCommandFromConsole("reconstruct_references");

        if (load_data_)
            rt_man.addCommandFromConsole("load_data");

        if (export_view_points_report_filename_.size())
            rt_man.addCommandFromConsole("export_view_points_report " +
                                         export_view_points_report_filename_);

        if (evaluate_)
        {
            string cmd = "evaluate";

            if (evaluate_run_filter_)
                cmd += " --run_filter";

            rt_man.addCommandFromConsole(cmd);
        }

        if (export_eval_report_filename_.size())
            rt_man.addCommandFromConsole("export_eval_report " + export_eval_report_filename_);

        if (quit_)
            rt_man.addCommandFromConsole("quit");

        rt_man.startCommandProcessing();

        // finally => set compass as running
        COMPASS::instance().setAppState(AppState::Running);

        return true;
    }
    catch (exception& e)
    {
        logerr << "error: " << e.what();
        return false;
    }
}

Client::~Client()
{
    logdbg;
}

bool Client::notify(QObject* receiver, QEvent* event)
{
    try
    {
        return QApplication::notify(receiver, event);
    }
    catch (exception& e)
    {
        std::string msg = "Unhandled exception '" + std::string(e.what()) + "'";
        traced_assert_msg(false, msg.c_str());

        // traced_assert(false);
        //QMessageBox::critical(nullptr, "COMPASSClient: notify: exception", QString(e.what()));
    }
    catch (...)
    {
        std::string msg = "Unhandled exception";
        traced_assert_msg(false, msg.c_str());

        // traced_assert(false);
        //QMessageBox::critical(nullptr, "COMPASSClient: notify: exception", "Unknown exception");
    }
    return false;
}

bool Client::quitRequested() const { return quit_requested_; }

void Client::checkAndSetupConfig()
{
    // check if basic configuration works
    try
    {
        cout << "COMPASSClient: setting directory paths" << endl;

        system_install_path_ = SYSTEM_INSTALL_PATH;

#if USE_EXPERIMENTAL_SOURCE == true
        cout << "COMPASSClient: includes experimental features" << endl;

        const char* appdir = getenv("APPDIR");
        if (appdir)
        {
            cout << "COMPASSClient: assuming fuse environment in '" << appdir << "'" << endl;
            assert(appdir);
            assert(Files::directoryExists(appdir));

            system_install_path_ = string(appdir) + "/compass/";

            cout << "COMPASSClient: set install path to '" << system_install_path_ << "'" << endl;
            assert(Files::directoryExists(system_install_path_));

            osgDB::FilePathList path_list;

            //path_list.push_back("$ORIGIN/appdir/lib");
            path_list.push_back("$ORIGIN/lib");
            path_list.push_back("lib");

            osgDB::Registry::instance()->setLibraryFilePathList(string(appdir) + "/lib");

            string gdal_path = string(appdir) + "/compass/data/gdal";
            CPLSetConfigOption("GDAL_DATA", gdal_path.c_str());
        }
#endif

        checkNeededActions();

        performNeededActions();

        cout << "COMPASSClient: opening simple config file at '" << HOME_CONF_DIRECTORY + "main.conf'"
             << endl;

        SimpleConfig config("config.json");
        assert(config.existsId("version"));
        assert(config.existsId("configuration_path"));
        assert(config.existsId("save_config_on_exit"));
        assert(config.existsId("log_properties_file"));
        assert(config.existsId("save_config_on_exit"));

        if (override_cfg_path_.size())
        {
            cout << "COMPASSClient: overriding config path to '" << override_cfg_path_ + "'" << endl;

            CURRENT_CONF_DIRECTORY = HOME_CONF_DIRECTORY + override_cfg_path_ + "/";
        }
        else
            CURRENT_CONF_DIRECTORY = HOME_CONF_DIRECTORY + config.getString("configuration_path") + "/";

        cout << "COMPASSClient: current configuration path is '" << CURRENT_CONF_DIRECTORY + "'"
             << endl;

        string log_config_path = HOME_CONF_DIRECTORY + config.getString("log_properties_file");
        Files::verifyFileExists(log_config_path);

        cout << "COMPASSClient: initializing logger using '" << log_config_path << "'" << endl;
        Logger::getInstance().init(log_config_path, enable_event_log_);

        loginf << "startup version " << VERSION;
        string config_version = config.getString("version");
        loginf << "configuration version " << config_version;

        ConfigurationManager::getInstance().init(config.getString("main_configuration_file"));

        if (import_asterix_parameters_.size())
        {
            loginf << "overriding ASTERIX import parameters";
            using namespace nlohmann;

            try {
                json json_config = json::parse(import_asterix_parameters_);

                traced_assert(ConfigurationManager::getInstance().hasRootConfiguration(
                    "COMPASS", "COMPASS0"));
                Configuration& compass_config = ConfigurationManager::getInstance().getRootConfiguration(
                    "COMPASS", "COMPASS0");

                traced_assert(compass_config.hasSubConfiguration("TaskManager", "TaskManager0"));
                Configuration& task_man_config = compass_config.getOrCreateSubConfiguration(
                    "TaskManager", "TaskManager0");

                traced_assert(task_man_config.hasSubConfiguration("ASTERIXImportTask", "ASTERIXImportTask0"));
                Configuration& task_config = task_man_config.getOrCreateSubConfiguration(
                    "ASTERIXImportTask", "ASTERIXImportTask0");

                task_config.overrideJSONParameters(json_config);
            }
            catch (exception& e)
            {
                logerr << "JSON parse error in '" << import_asterix_parameters_ << "'";
                throw e;
            }
        }

        if (import_gps_parameters_.size())
        {
            loginf << "overriding gps import parameters";
            using namespace nlohmann;

            try {
                json json_config = json::parse(import_gps_parameters_);

                traced_assert(ConfigurationManager::getInstance().hasRootConfiguration(
                    "COMPASS", "COMPASS0"));
                Configuration& compass_config = ConfigurationManager::getInstance().getRootConfiguration(
                    "COMPASS", "COMPASS0");

                traced_assert(compass_config.hasSubConfiguration("TaskManager", "TaskManager0"));
                Configuration& task_man_config = compass_config.getOrCreateSubConfiguration(
                    "TaskManager", "TaskManager0");

                traced_assert(task_man_config.hasSubConfiguration("GPSTrailImportTask", "GPSTrailImportTask0"));
                Configuration& task_config = task_man_config.getOrCreateSubConfiguration(
                    "GPSTrailImportTask", "GPSTrailImportTask0");

                task_config.overrideJSONParameters(json_config);
            }
            catch (exception& e)
            {
                logerr << "JSON parse error in '" << import_gps_parameters_ << "'";
                throw e;
            }
        }

        if (evaluation_parameters_.size())
        {
            loginf << "overriding evaluation parameters";
            using namespace nlohmann;

            try {
                json json_config = json::parse(evaluation_parameters_);

                traced_assert(ConfigurationManager::getInstance().hasRootConfiguration(
                    "COMPASS", "COMPASS0"));
                Configuration& compass_config = ConfigurationManager::getInstance().getRootConfiguration(
                    "COMPASS", "COMPASS0");

                traced_assert(compass_config.hasSubConfiguration("EvaluationManager", "EvaluationManager0"));
                Configuration& eval_man_config = compass_config.getOrCreateSubConfiguration(
                    "EvaluationManager", "EvaluationManager0");

                traced_assert(eval_man_config.hasSubConfiguration("EvaluationCalculator", "EvaluationCalculator0"));
                Configuration& eval_calc_config = eval_man_config.getOrCreateSubConfiguration(
                    "EvaluationCalculator", "EvaluationCalculator0");

                eval_calc_config.overrideJSONParameters(json_config);
            }
            catch (exception& e)
            {
                logerr << "JSON parse error in '" << evaluation_parameters_ << "'";
                throw e;
            }
        }
    }
    catch (exception& ex)
    {
        logerr << "caught exception '" << ex.what() << "'";
        //logerr.flush();
        // traced_assert(false);

        quit_requested_ = true;
        return;
    }
    catch (...)
    {
        logerr << "caught exception";
        //logerr.flush();
        // traced_assert(false);

        quit_requested_ = true;
        return;
    }
}

void Client::checkNeededActions()
{
    cout << "COMPASSClient: SAVE CONFIG: " << (no_config_save_ ? "NO" : "YES") << std::endl;
    cout << "COMPASSClient: checking if compass home directory exists ... ";

    bool home_subdir_exists = Files::directoryExists(HOME_SUBDIRECTORY);

    if (home_subdir_exists)
        cout << " yes" << endl;
    else
    {
        cout << " no" << endl;

        cout << "COMPASSClient: complete copy into compass home directory needed";
        config_and_data_copy_wanted_ = true;

        return;
    }

    // home subdir exists
    cout << "COMPASSClient: checking if old compass config exists ... ";

    bool old_cfg_exists = Files::fileExists(HOME_SUBDIRECTORY + "conf/config.json");

    if (old_cfg_exists) // complete delete needed
    {
        cout << " yes" << endl;

        cout << "COMPASSClient: complete delete of compass home directory needed";

        home_subdir_deletion_wanted_ = true;
        config_and_data_copy_wanted_ = true;

        return;
    }
    else
        cout << " no" << endl;

    // home subdir exists, no old config exists
    cout << "COMPASSClient: checking if current compass config exists ... ";

    bool current_cfg_subdir_exists = Files::directoryExists(HOME_SUBDIRECTORY)
                                     && Files::directoryExists(HOME_VERSION_SUBDIRECTORY);

    if (current_cfg_subdir_exists)
    {
        cout << " yes" << endl;

        SimpleConfig config("config.json");
        string config_version;

        if (config.existsId("version"))
            config_version = config.getString("version");

        if (String::compareVersions(VERSION, config_version) != 0)
            cerr << "COMPASSClient: app version '" << VERSION << "' config version " << config_version << "'" << endl;

        assert(String::compareVersions(VERSION, config_version) == 0);  // must be same
        return; // nothing to do
    }
    else
    {
        cout << " no" << endl;

        cout << "COMPASSClient: complete copy into compass home directory needed";
        config_and_data_copy_wanted_ = true;

        return;
    }
}

void Client::performNeededActions()
{
    if (home_subdir_deletion_wanted_)  // version so old it should be deleted before
    {
        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(
            nullptr, "Delete Previous Configuration & Data",
            "Complete deletion of the previous configuration and data is required. This will delete"
            " the folder '~/.compass'. Do you want to continue?",
            QMessageBox::Yes | QMessageBox::No);

        if (reply == QMessageBox::Yes)
        {
            cout << "COMPASSClient: config & data delete confirmed, deleting" << endl;

            deleteCompleteHomeSubDir();
        }
        else
        {
            cout << "COMPASSClient: required config & data delete denied" << endl;
            quit_requested_ = true;
            return;
        }
    }

    if (config_and_data_copy_wanted_)
    {
        cout << "COMPASSClient: copying current config & data" << endl;

        copyConfigurationAndData();
    }

}

void Client::deleteCompleteHomeSubDir()
{
    if (!Files::directoryExists(HOME_SUBDIRECTORY))
        throw runtime_error("COMPASSClient: unable to delete directory'" + HOME_SUBDIRECTORY +
                            "'");

    Files::deleteFolder(HOME_SUBDIRECTORY);

    assert(!Files::directoryExists(HOME_SUBDIRECTORY));
}

void Client::copyConfigurationAndData()
{
    if (!Files::directoryExists(system_install_path_))
        throw runtime_error("COMPASSClient: unable to locate system installation files at '" +
                            system_install_path_ + "'");

    if (!Files::directoryExists(OSGEARTH_CACHE_SUBDIRECTORY))
    {
        cout << "COMPASSClient: creating GDAL cache directory '" << OSGEARTH_CACHE_SUBDIRECTORY
             << "'";
        Files::createMissingDirectories(OSGEARTH_CACHE_SUBDIRECTORY);
        assert(Files::directoryExists(OSGEARTH_CACHE_SUBDIRECTORY));
    }

    cout << "COMPASSClient: copying files from system installation from '" << system_install_path_
         << "' to '" << HOME_VERSION_SUBDIRECTORY << "' ... ";

    if (!Files::copyRecursively(system_install_path_, HOME_VERSION_SUBDIRECTORY))
        throw runtime_error("COMPASSClient: copying files from system installation from '" +
                            system_install_path_ + "' to '" + HOME_VERSION_SUBDIRECTORY + " failed");

    cout << " done" << endl;
}

//void Client::copyConfiguration()
//{
//    string system_conf_path = system_install_path_ + "conf/";
//    string home_conf_path = HOME_SUBDIRECTORY + "conf/";

//    cout << "COMPASSClient: reset config from from '" << system_conf_path << "' to '"
//         << home_conf_path << "' ... ";

//    if (!Files::copyRecursively(system_conf_path, home_conf_path))
//        throw runtime_error("COMPASSClient: reset config from from '" + system_conf_path + "' to '" +
//                            home_conf_path + "' failed");

//    cout << " done" << endl;
//}
