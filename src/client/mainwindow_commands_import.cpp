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

#include "mainwindow_commands_import.h"
#include "mainwindow.h"
#include "compass.h"
#include "taskmanager.h"
#include "viewpointsimporttask.h"
#include "asteriximporttask.h"
#include "jsonimporttask.h"
#include "gpstrailimporttask.h"
#include "viewpointsimporttask.h"
#include "logger.h"
#include "util/files.h"
#include "util/stringconv.h"
#include "util/timeconv.h"

#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>

using namespace std;
using namespace Utils;

namespace main_window
{

// import vp

RTCommandImportViewPointsFile::RTCommandImportViewPointsFile()
    : rtcommand::RTCommand()
{
}

rtcommand::IsValid  RTCommandImportViewPointsFile::valid() const
{
    CHECK_RTCOMMAND_INVALID_CONDITION(!filename_.size(), "Filename empty")
    CHECK_RTCOMMAND_INVALID_CONDITION(!Files::fileExists(filename_), string("File '")+filename_+"' does not exist")

    return RTCommand::valid();
}

bool RTCommandImportViewPointsFile::run_impl()
{
    if (!filename_.size())
    {
        setResultMessage("Filename empty");
        return false;
    }

    if (!Files::fileExists(filename_))
    {
        setResultMessage("File '"+filename_+"' does not exist");
        return false;
    }

    if (!COMPASS::instance().dbOpened())
    {
        setResultMessage("Database not opened");
        return false;
    }

    if (COMPASS::instance().appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+COMPASS::instance().appModeStr());
        return false;
    }
    ViewPointsImportTask& vp_import_task = COMPASS::instance().taskManager().viewPointsImportTask();

    vp_import_task.importFilename(filename_);

    if (!vp_import_task.canRun())
    {
        setResultMessage("Import error '"+vp_import_task.currentError()+"'");
        return false;
    }

    vp_import_task.allowUserInteractions(false);

    vp_import_task.run();
    traced_assert(vp_import_task.done());

    // if shitty
    //setResultMessage("VP error case 3");
    //return false;

    // if ok
    return true;
}

void RTCommandImportViewPointsFile::collectOptions_impl(OptionsDescription& options,
                                                        PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
    ("filename,f", po::value<std::string>()->required(), "given filename, e.g. ’/data/file1.json’");

    ADD_RTCOMMAND_POS_OPTION(positional, "filename") // give position
}

void RTCommandImportViewPointsFile::assignVariables_impl(const VariablesMap& variables)
    {
        RTCOMMAND_GET_VAR_OR_THROW(variables, "filename", std::string, filename_)
    }

// import asterix file

namespace helpers
{
    void addImportErrorFiles(nlohmann::json& j,
                             ASTERIXImportTask& import_task)
    {
        traced_assert(import_task.source().isFileType());

        auto err_files = nlohmann::json::array();

        for (const auto& f : import_task.source().files())
        {
            if (f.hasError())
            {
                nlohmann::json jfile;
                jfile[ "filename" ] = f.filename;
                jfile[ "error"    ] = f.error.errinfo;

                err_files.push_back(jfile);
            }
        }

        j[ "file_errors"     ] = err_files;
        j[ "has_file_errors" ] = err_files.size() > 0;
    }
}

RTCommandImportASTERIXFile::RTCommandImportASTERIXFile()
    : rtcommand::RTCommand()
{
    condition.setSignal("compass.taskmanager.asteriximporttask.doneSignal", -1); // think about max duration
}

rtcommand::IsValid  RTCommandImportASTERIXFile::valid() const
{
    CHECK_RTCOMMAND_INVALID_CONDITION(!filename_.size(), "Filename empty")
    CHECK_RTCOMMAND_INVALID_CONDITION(!Files::fileExists(filename_), "File '"+filename_+"' does not exist")

    if (framing_.size()) // ’none’, ’ioss’, ’ioss_seq’, ’rff’
    {
        CHECK_RTCOMMAND_INVALID_CONDITION(
            !(framing_ == "none" || framing_ == "ioss" || framing_ == "ioss_seq" || framing_ == "rff"),
            string("Framing '")+framing_+"' does not exist")
    }

    if (line_id_.size())
    {
        CHECK_RTCOMMAND_INVALID_CONDITION(
            !(line_id_ == "L1" || line_id_ == "L2" || line_id_ == "L3" || line_id_ == "L4"),
            "Line '"+line_id_+"' does not exist")
    }

    if (date_str_.size())
    {
        try
        {
            boost::posix_time::ptime date = Time::fromDateString(date_str_);
            CHECK_RTCOMMAND_INVALID_CONDITION(date.is_not_a_date_time(), "Given date '"+date_str_+"' invalid")
        }
        catch (const boost::bad_lexical_cast& e)
        {
            CHECK_RTCOMMAND_INVALID_CONDITION(true, "Given date '"+date_str_+"' has invalid format: " + string(e.what()))
        }
        catch (const std::exception& e)
        {
            CHECK_RTCOMMAND_INVALID_CONDITION(true, "Given date '"+date_str_+"' parsing failed: " + string(e.what()))
        }
    }

    if (time_offset_str_.size())
    {
        bool ok {true};

        String::timeFromString(time_offset_str_, &ok);

        CHECK_RTCOMMAND_INVALID_CONDITION(!ok, "Given time offset '"+time_offset_str_+"' invalid")
    }

    return RTCommand::valid();
}

bool RTCommandImportASTERIXFile::run_impl()
{
    if (!filename_.size())
    {
        setResultMessage("Filename empty");
        return false;
    }

    if (!Files::fileExists(filename_))
    {
        setResultMessage("File '"+filename_+"' does not exist");
        return false;
    }

    if (!COMPASS::instance().dbOpened())
    {
        setResultMessage("Database not opened");
        return false;
    }

    if (COMPASS::instance().appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+COMPASS::instance().appModeStr());
        return false;
    }

    ASTERIXImportTask& import_task = COMPASS::instance().taskManager().asterixImporterTask();
    unsigned int file_line {0};

    try
    {
        if (framing_.size())
        {
            if (framing_ == "none")
                import_task.asterixFileFraming("");
            else
                import_task.asterixFileFraming(framing_);
        }

        if (line_id_.size())
        {
            file_line = String::lineFromStr(line_id_);

            //@TODO: remove if per-file line id works as expected
            import_task.settings().file_line_id_ = file_line;
        }

        if (date_str_.size())
        {
            import_task.settings().date_ = Time::fromDateString(date_str_);
        }

        if (time_offset_str_.size())
        {
            bool ok {true};

            double time_offset = String::timeFromString(time_offset_str_, &ok);
            traced_assert(ok); // was checked in valid

            import_task.settings().override_tod_active_ = true;
            import_task.settings().override_tod_offset_ = time_offset;
        }

        import_task.settings().ignore_time_jumps_ = ignore_time_jumps_;
    }
    catch (exception& e)
    {
        logerr << "setting ASTERIX options resulted in error: " << e.what();
        setResultMessage(string("Setting ASTERIX options resulted in error: ")+e.what());
        return false;
    }

    traced_assert(filename_.size());
    traced_assert(Files::fileExists(filename_));

    import_task.source().setSourceType(ASTERIXImportSource::SourceType::FileASTERIX, {filename_});//, file_line);

    if (!import_task.canRun())
    {
        setResultMessage("ASTERIX task can not be run"); // should never happen, checked before
        return false;
    }

    import_task.allowUserInteractions(false);
    import_task.run();

    return true;
}

bool RTCommandImportASTERIXFile::checkResult_impl()
{
    ASTERIXImportTask& import_task = COMPASS::instance().taskManager().asterixImporterTask();

    if (import_task.hasError())
    {
        setResultMessage(import_task.error());
        return false;
    }

    nlohmann::json j;
    helpers::addImportErrorFiles(j, import_task);
    setJSONReply(j);

    return true;
}

void RTCommandImportASTERIXFile::collectOptions_impl(OptionsDescription& options,
                                                     PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
    ("filename", po::value<std::string>()->required(), "given filename, e.g. ’/data/file1.json’");
    ADD_RTCOMMAND_OPTIONS(options)
    ("framing,f", po::value<std::string>()->default_value(""),
     "imports ASTERIX file with given  ASTERIX framing, e.g. ’none’, ’ioss’, ’ioss_seq’, ’rff’");
    ADD_RTCOMMAND_OPTIONS(options)
    ("line,l", po::value<std::string>()->default_value(""), "imports ASTERIX file with given line e.g. ’L2’");
    ADD_RTCOMMAND_OPTIONS(options)
    ("date,d", po::value<std::string>()->default_value(""),
     "imports ASTERIX file with given date, in YYYY-MM-DD format e.g. ’2020-04-20’");
    ADD_RTCOMMAND_OPTIONS(options)
    ("time_offset,t", po::value<std::string>()->default_value(""),
     "imports ASTERIX file with given Time of Day override, in HH:MM:SS.ZZZ’");
    ADD_RTCOMMAND_OPTIONS(options)("ignore_time_jumps,i", "ignore 24h time jumps");

    ADD_RTCOMMAND_POS_OPTION(positional, "filename") // give position
}

void RTCommandImportASTERIXFile::assignVariables_impl(const VariablesMap& variables)
    {
        RTCOMMAND_GET_VAR_OR_THROW(variables, "filename", std::string, filename_)
        RTCOMMAND_GET_VAR_OR_THROW(variables, "framing", std::string, framing_)
        RTCOMMAND_GET_VAR_OR_THROW(variables, "line", std::string, line_id_)
        RTCOMMAND_GET_VAR_OR_THROW(variables, "date", std::string, date_str_)
        RTCOMMAND_GET_VAR_OR_THROW(variables, "time_offset", std::string, time_offset_str_)
        RTCOMMAND_CHECK_VAR(variables, "ignore_time_jumps", ignore_time_jumps_)
    }

// import asterix files

RTCommandImportASTERIXFiles::RTCommandImportASTERIXFiles()
    : rtcommand::RTCommand()
{
    condition.setSignal("compass.taskmanager.asteriximporttask.doneSignal", -1); // think about max duration
}

rtcommand::IsValid RTCommandImportASTERIXFiles::valid() const
{
    CHECK_RTCOMMAND_INVALID_CONDITION(!filenames_.size(), "Filenames empty")

    for (const auto& filename : split_filenames_)
        CHECK_RTCOMMAND_INVALID_CONDITION(!Files::fileExists(filename), "File '"+filename+"' does not exist")

    if (framing_.size()) // ’none’, ’ioss’, ’ioss_seq’, ’rff’
    {
        CHECK_RTCOMMAND_INVALID_CONDITION(
            !(framing_ == "none" || framing_ == "ioss" || framing_ == "ioss_seq" || framing_ == "rff"),
            string("Framing '")+framing_+"' does not exist")
    }

    if (line_id_.size())
    {
        CHECK_RTCOMMAND_INVALID_CONDITION(
            !(line_id_ == "L1" || line_id_ == "L2" || line_id_ == "L3" || line_id_ == "L4"),
            "Line '"+line_id_+"' does not exist")
    }

    if (date_str_.size())
    {
        try
        {
            boost::posix_time::ptime date = Time::fromDateString(date_str_);
            CHECK_RTCOMMAND_INVALID_CONDITION(date.is_not_a_date_time(), "Given date '"+date_str_+"' invalid")
        }
        catch (const boost::bad_lexical_cast& e)
        {
            CHECK_RTCOMMAND_INVALID_CONDITION(true, "Given date '"+date_str_+"' has invalid format: " + string(e.what()))
        }
        catch (const std::exception& e)
        {
            CHECK_RTCOMMAND_INVALID_CONDITION(true, "Given date '"+date_str_+"' parsing failed: " + string(e.what()))
        }
    }

    if (time_offset_str_.size())
    {
        bool ok {true};

        String::timeFromString(time_offset_str_, &ok);

        CHECK_RTCOMMAND_INVALID_CONDITION(!ok, "Given time offset '"+time_offset_str_+"' invalid")
    }

    return RTCommand::valid();
}

bool RTCommandImportASTERIXFiles::run_impl()
{
    if (!filenames_.size())
    {
        setResultMessage("Filenames empty");
        return false;
    }

    for (const auto& filename : split_filenames_)
    {
        if (!Files::fileExists(filename))
        {
            setResultMessage("File '"+filename+"' does not exist");
            return false;
        }
    }

    if (!COMPASS::instance().dbOpened())
    {
        setResultMessage("Database not opened");
        return false;
    }

    if (COMPASS::instance().appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+COMPASS::instance().appModeStr());
        return false;
    }

    ASTERIXImportTask& import_task = COMPASS::instance().taskManager().asterixImporterTask();

    unsigned int file_line {0};

    try
    {
        if (framing_.size())
        {
            if (framing_ == "none")
                import_task.asterixFileFraming("");
            else
                import_task.asterixFileFraming(framing_);
        }

        if (line_id_.size())
        {
            file_line = String::lineFromStr(line_id_);

            //@TODO: remove if per-file line id works as expected
            import_task.settings().file_line_id_ = file_line;
        }

        if (date_str_.size())
        {
            import_task.settings().date_ = Time::fromDateString(date_str_);
        }

        if (time_offset_str_.size())
        {
            bool ok {true};

            double time_offset = String::timeFromString(time_offset_str_, &ok);
            traced_assert(ok); // was checked in valid

            import_task.settings().override_tod_active_ = true;
            import_task.settings().override_tod_offset_ = time_offset;
        }

        import_task.settings().ignore_time_jumps_ = ignore_time_jumps_;
    }
    catch (exception& e)
    {
        logerr << "setting ASTERIX options resulted in error: " << e.what();
        setResultMessage(string("Setting ASTERIX options resulted in error: ")+e.what());
        return false;
    }

    traced_assert(filenames_.size());

    for (const auto& filename : split_filenames_)
    {
        loginf << "file '" << filename << "'";

        traced_assert(Files::fileExists(filename));
    }

    import_task.source().setSourceType(ASTERIXImportSource::SourceType::FileASTERIX, split_filenames_);//, file_line);

    if (!import_task.canRun())
    {
        setResultMessage("ASTERIX task can not be run"); // should never happen, checked before
        return false;
    }

    import_task.allowUserInteractions(false);
    import_task.run();

    // handle errors

    // if shitty
    //setResultMessage("VP error case 3");
    //return false;

    // if ok
    return true;
}

bool RTCommandImportASTERIXFiles::checkResult_impl()
{
    ASTERIXImportTask& import_task = COMPASS::instance().taskManager().asterixImporterTask();

    if (import_task.hasError())
    {
        setResultMessage(import_task.error());
        return false;
    }

    nlohmann::json j;
    helpers::addImportErrorFiles(j, import_task);
    setJSONReply(j);

    return true;
}

void RTCommandImportASTERIXFiles::collectOptions_impl(OptionsDescription& options,
                                                      PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
    ("filenames", po::value<std::string>()->required(), "given filenames, e.g. '/data/file1.ff;/data/file2.ff'");
    ADD_RTCOMMAND_OPTIONS(options)
    ("framing,f", po::value<std::string>()->default_value(""),
     "imports ASTERIX file with given  ASTERIX framing, e.g. ’none’, ’ioss’, ’ioss_seq’, ’rff’");
    ADD_RTCOMMAND_OPTIONS(options)
    ("line,l", po::value<std::string>()->default_value(""), "imports ASTERIX file with given line e.g. ’L2’");
    ADD_RTCOMMAND_OPTIONS(options)
    ("date,d", po::value<std::string>()->default_value(""),
     "imports ASTERIX file with given date, in YYYY-MM-DD format e.g. ’2020-04-20’");
    ADD_RTCOMMAND_OPTIONS(options)
    ("time_offset,t", po::value<std::string>()->default_value(""),
     "imports ASTERIX file with given Time of Day override, in HH:MM:SS.ZZZ’");
    ADD_RTCOMMAND_OPTIONS(options)("ignore_time_jumps,i", "ignore 24h time jumps");

    ADD_RTCOMMAND_POS_OPTION(positional, "filenames") // give position
}

void RTCommandImportASTERIXFiles::assignVariables_impl(const VariablesMap& variables)
{
    RTCOMMAND_GET_VAR_OR_THROW(variables, "filenames", std::string, filenames_)

    for (string filename : String::split(filenames_, ';'))
    {
        if(filename.rfind("~", 0) == 0)  // change tilde to home path
            filename.replace(0, 1, QDir::homePath().toStdString());

        split_filenames_.push_back(filename);
    }

    RTCOMMAND_GET_VAR_OR_THROW(variables, "framing", std::string, framing_)
    RTCOMMAND_GET_VAR_OR_THROW(variables, "line", std::string, line_id_)
    RTCOMMAND_GET_VAR_OR_THROW(variables, "date", std::string, date_str_)
    RTCOMMAND_GET_VAR_OR_THROW(variables, "time_offset", std::string, time_offset_str_)
    RTCOMMAND_CHECK_VAR(variables, "ignore_time_jumps", ignore_time_jumps_)
}

// import asterix pcap file

RTCommandImportASTERIXPCAPFile::RTCommandImportASTERIXPCAPFile()
    : rtcommand::RTCommand()
{
    condition.setSignal("compass.taskmanager.asteriximporttask.doneSignal", -1); // think about max duration
}

rtcommand::IsValid  RTCommandImportASTERIXPCAPFile::valid() const
{
    CHECK_RTCOMMAND_INVALID_CONDITION(!filename_.size(), "Filename empty")
    CHECK_RTCOMMAND_INVALID_CONDITION(!Files::fileExists(filename_), "File '"+filename_+"' does not exist")

    if (line_id_.size())
    {
        CHECK_RTCOMMAND_INVALID_CONDITION(
            !(line_id_ == "L1" || line_id_ == "L2" || line_id_ == "L3" || line_id_ == "L4"),
            "Line '"+line_id_+"' does not exist")
    }

    if (date_str_.size())
    {
        boost::posix_time::ptime date = Time::fromDateString(date_str_);
        CHECK_RTCOMMAND_INVALID_CONDITION(date.is_not_a_date_time(), "Given date '"+date_str_+"' invalid")
    }

    if (time_offset_str_.size())
    {
        bool ok {true};

        String::timeFromString(time_offset_str_, &ok);

        CHECK_RTCOMMAND_INVALID_CONDITION(!ok, "Given time offset '"+time_offset_str_+"' invalid")
    }

    return RTCommand::valid();
}

bool RTCommandImportASTERIXPCAPFile::run_impl()
{
    if (!filename_.size())
    {
        setResultMessage("Filename empty");
        return false;
    }

    if (!Files::fileExists(filename_))
    {
        setResultMessage("File '"+filename_+"' does not exist");
        return false;
    }

    if (!COMPASS::instance().dbOpened())
    {
        setResultMessage("Database not opened");
        return false;
    }

    if (COMPASS::instance().appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+COMPASS::instance().appModeStr());
        return false;
    }

    ASTERIXImportTask& import_task = COMPASS::instance().taskManager().asterixImporterTask();
    unsigned int file_line {0};

    try
    {
        if (line_id_.size())
        {
            file_line = String::lineFromStr(line_id_);

            //@TODO: remove if per-file line id works as expected
            import_task.settings().file_line_id_ = file_line;
        }

        if (date_str_.size())
        {
            import_task.settings().date_ = Time::fromDateString(date_str_);
        }

        if (time_offset_str_.size())
        {
            bool ok {true};

            double time_offset = String::timeFromString(time_offset_str_, &ok);
            traced_assert(ok); // was checked in valid

            import_task.settings().override_tod_active_ = true;
            import_task.settings().override_tod_offset_ = time_offset;
        }

        import_task.settings().ignore_time_jumps_ = ignore_time_jumps_;
    }
    catch (exception& e)
    {
        logerr << "setting ASTERIX options resulted in error: " << e.what();
        setResultMessage(string("Setting ASTERIX options resulted in error: ")+e.what());
        return false;
    }

    traced_assert(filename_.size());
    traced_assert(Files::fileExists(filename_));

    import_task.source().setSourceType(ASTERIXImportSource::SourceType::FilePCAP, {filename_});//, file_line);

    if (!import_task.canRun())
    {
        setResultMessage("ASTERIX task can not be run"); // should never happen, checked before
        return false;
    }

    import_task.allowUserInteractions(false);
    import_task.run();

    return true;
}

bool RTCommandImportASTERIXPCAPFile::checkResult_impl()
{
    ASTERIXImportTask& import_task = COMPASS::instance().taskManager().asterixImporterTask();

    if (import_task.hasError())
    {
        setResultMessage(import_task.error());
        return false;
    }

    nlohmann::json j;
    helpers::addImportErrorFiles(j, import_task);
    setJSONReply(j);

    return true;
}

void RTCommandImportASTERIXPCAPFile::collectOptions_impl(OptionsDescription& options,
                                                         PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
    ("filename", po::value<std::string>()->required(), "given filename, e.g. ’/data/file1.json’");
    ADD_RTCOMMAND_OPTIONS(options)
    ("line,l", po::value<std::string>()->default_value(""), "imports ASTERIX file with given line e.g. ’L2’");
    ADD_RTCOMMAND_OPTIONS(options)
    ("date,d", po::value<std::string>()->default_value(""),
     "imports ASTERIX file with given date, in YYYY-MM-DD format e.g. ’2020-04-20’");
    ADD_RTCOMMAND_OPTIONS(options)
    ("time_offset,t", po::value<std::string>()->default_value(""),
     "imports ASTERIX file with given Time of Day override, in HH:MM:SS.ZZZ’");
    ADD_RTCOMMAND_OPTIONS(options)("ignore_time_jumps,i", "ignore 24h time jumps");

    ADD_RTCOMMAND_POS_OPTION(positional, "filename") // give position
}

void RTCommandImportASTERIXPCAPFile::assignVariables_impl(const VariablesMap& variables)
    {
        RTCOMMAND_GET_VAR_OR_THROW(variables, "filename", std::string, filename_)
        RTCOMMAND_GET_VAR_OR_THROW(variables, "line", std::string, line_id_)
        RTCOMMAND_GET_VAR_OR_THROW(variables, "date", std::string, date_str_)
        RTCOMMAND_GET_VAR_OR_THROW(variables, "time_offset", std::string, time_offset_str_)
        RTCOMMAND_CHECK_VAR(variables, "ignore_time_jumps", ignore_time_jumps_)
    }

// import asterix files

RTCommandImportASTERIXPCAPFiles::RTCommandImportASTERIXPCAPFiles()
    : rtcommand::RTCommand()
{
    condition.setSignal("compass.taskmanager.asteriximporttask.doneSignal", -1); // think about max duration
}

rtcommand::IsValid RTCommandImportASTERIXPCAPFiles::valid() const
{
    CHECK_RTCOMMAND_INVALID_CONDITION(!filenames_.size(), "Filenames empty")

    for (const auto& filename : split_filenames_)
        CHECK_RTCOMMAND_INVALID_CONDITION(!Files::fileExists(filename), "File '"+filename+"' does not exist")

    if (line_id_.size())
    {
        CHECK_RTCOMMAND_INVALID_CONDITION(
            !(line_id_ == "L1" || line_id_ == "L2" || line_id_ == "L3" || line_id_ == "L4"),
            "Line '"+line_id_+"' does not exist")
    }

    if (date_str_.size())
    {
        boost::posix_time::ptime date = Time::fromDateString(date_str_);
        CHECK_RTCOMMAND_INVALID_CONDITION(date.is_not_a_date_time(), "Given date '"+date_str_+"' invalid")
    }

    if (time_offset_str_.size())
    {
        bool ok {true};

        String::timeFromString(time_offset_str_, &ok);

        CHECK_RTCOMMAND_INVALID_CONDITION(!ok, "Given time offset '"+time_offset_str_+"' invalid")
    }

    return RTCommand::valid();
}

bool RTCommandImportASTERIXPCAPFiles::run_impl()
{
    if (!filenames_.size())
    {
        setResultMessage("Filenames empty");
        return false;
    }

    for (const auto& filename : split_filenames_)
    {
        if (!Files::fileExists(filename))
        {
            setResultMessage("File '"+filename+"' does not exist");
            return false;
        }
    }

    if (!COMPASS::instance().dbOpened())
    {
        setResultMessage("Database not opened");
        return false;
    }

    if (COMPASS::instance().appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+COMPASS::instance().appModeStr());
        return false;
    }

    ASTERIXImportTask& import_task = COMPASS::instance().taskManager().asterixImporterTask();

    unsigned int file_line {0};

    try
    {
        if (line_id_.size())
        {
            file_line = String::lineFromStr(line_id_);

            //@TODO: remove if per-file line id works as expected
            import_task.settings().file_line_id_ = file_line;
        }

        if (date_str_.size())
        {
            import_task.settings().date_ = Time::fromDateString(date_str_);
        }

        if (time_offset_str_.size())
        {
            bool ok {true};

            double time_offset = String::timeFromString(time_offset_str_, &ok);
            traced_assert(ok); // was checked in valid

            import_task.settings().override_tod_active_ = true;
            import_task.settings().override_tod_offset_ = time_offset;
        }

        import_task.settings().ignore_time_jumps_ = ignore_time_jumps_;
    }
    catch (exception& e)
    {
        logerr << "setting ASTERIX options resulted in error: " << e.what();
        setResultMessage(string("Setting ASTERIX options resulted in error: ")+e.what());
        return false;
    }

    traced_assert(filenames_.size());

    for (const auto& filename : split_filenames_)
    {
        loginf << "file '" << filename << "'";

        traced_assert(Files::fileExists(filename));
    }

    import_task.source().setSourceType(ASTERIXImportSource::SourceType::FilePCAP, split_filenames_);//, file_line);

    if (!import_task.canRun())
    {
        setResultMessage("ASTERIX task can not be run"); // should never happen, checked before
        return false;
    }

    import_task.allowUserInteractions(false);
    import_task.run();

    return true;
}

bool RTCommandImportASTERIXPCAPFiles::checkResult_impl()
{
    ASTERIXImportTask& import_task = COMPASS::instance().taskManager().asterixImporterTask();

    if (import_task.hasError())
    {
        setResultMessage(import_task.error());
        return false;
    }

    nlohmann::json j;
    helpers::addImportErrorFiles(j, import_task);
    setJSONReply(j);

    return true;
}

void RTCommandImportASTERIXPCAPFiles::collectOptions_impl(OptionsDescription& options,
                                                          PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
    ("filenames", po::value<std::string>()->required(), "given filenames, e.g. '/data/file1.ff;/data/file2.ff'");
    ADD_RTCOMMAND_OPTIONS(options)
    ("line,l", po::value<std::string>()->default_value(""), "imports ASTERIX file with given line e.g. ’L2’");
    ADD_RTCOMMAND_OPTIONS(options)
    ("date,d", po::value<std::string>()->default_value(""),
     "imports ASTERIX file with given date, in YYYY-MM-DD format e.g. ’2020-04-20’");
    ADD_RTCOMMAND_OPTIONS(options)
    ("time_offset,t", po::value<std::string>()->default_value(""),
     "imports ASTERIX file with given Time of Day override, in HH:MM:SS.ZZZ’");
    ADD_RTCOMMAND_OPTIONS(options)("ignore_time_jumps,i", "ignore 24h time jumps");

    ADD_RTCOMMAND_POS_OPTION(positional, "filenames") // give position
}

void RTCommandImportASTERIXPCAPFiles::assignVariables_impl(const VariablesMap& variables)
{
    RTCOMMAND_GET_VAR_OR_THROW(variables, "filenames", std::string, filenames_)

    for (string filename : String::split(filenames_, ';'))
    {
        if(filename.rfind("~", 0) == 0)  // change tilde to home path
            filename.replace(0, 1, QDir::homePath().toStdString());

        split_filenames_.push_back(filename);
    }

    RTCOMMAND_GET_VAR_OR_THROW(variables, "line", std::string, line_id_)
    RTCOMMAND_GET_VAR_OR_THROW(variables, "date", std::string, date_str_)
    RTCOMMAND_GET_VAR_OR_THROW(variables, "time_offset", std::string, time_offset_str_)
    RTCOMMAND_CHECK_VAR(variables, "ignore_time_jumps", ignore_time_jumps_)
}

// import asterix network

RTCommandImportASTERIXNetworkStart::RTCommandImportASTERIXNetworkStart()
    : rtcommand::RTCommand()
{
    condition.setDelay(500); // think about max duration
}

rtcommand::IsValid  RTCommandImportASTERIXNetworkStart::valid() const
{
    if (time_offset_str_.size())
    {
        bool ok {true};

        String::timeFromString(time_offset_str_, &ok);

        CHECK_RTCOMMAND_INVALID_CONDITION(!ok, "Given time offset '"+time_offset_str_+"' invalid")
    }

    return RTCommand::valid();
}

bool RTCommandImportASTERIXNetworkStart::run_impl()
{
    if (COMPASS::instance().appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+COMPASS::instance().appModeStr());
        return false;
    }

    bool db_opened = COMPASS::instance().dbOpened();
    bool db_inmem  = COMPASS::instance().dbInMem();

    std::string current_db_filename = (db_opened && !db_inmem) ? COMPASS::instance().lastDbFilename() : "";

    //close current db
    if (db_opened)
        COMPASS::instance().mainWindow().closeDBSlot();

    //create in-memory db for live mode
    COMPASS::instance().mainWindow().createInMemoryDB(current_db_filename);

    ASTERIXImportTask& import_task = COMPASS::instance().taskManager().asterixImporterTask();

    try
    {
        if (time_offset_str_.size())
        {
            bool ok {true};

            double time_offset = String::timeFromString(time_offset_str_, &ok);
            traced_assert(ok); // was checked in valid

            import_task.settings().override_tod_active_ = true;
            import_task.settings().override_tod_offset_ = time_offset;
        }

        if (max_lines_ != -1)
            import_task.settings().max_network_lines_ = max_lines_;
    }
    catch (exception& e)
    {
        logerr << "setting ASTERIX options resulted in error: " << e.what();
        setResultMessage(string("Setting ASTERIX options resulted in error: ")+e.what());
        return false;
    }

    import_task.source().setSourceType(ASTERIXImportSource::SourceType::NetASTERIX);

    if (!import_task.canRun())
    {
        setResultMessage("ASTERIX task can not be run"); // should never happen, checked before
        return false;
    }

    import_task.allowUserInteractions(false);
    import_task.run();

    // handle errors

    // if shitty
    //setResultMessage("VP error case 3");
    //return false;

    // if ok
    return true;
}

void RTCommandImportASTERIXNetworkStart::collectOptions_impl(OptionsDescription& options,
                                                             PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
    ("time_offset,t", po::value<std::string>()->default_value(""),
     "imports ASTERIX file with given Time of Day override, in HH:MM:SS.ZZZ’");

    ADD_RTCOMMAND_OPTIONS(options)
    ("max_lines,m", po::value<int>()->default_value(-1),
     "maximum number of lines per data source during ASTERIX network import, 1..4");

    ADD_RTCOMMAND_OPTIONS(options)("ignore_future_ts,i", "ignore future timestamps");
}

void RTCommandImportASTERIXNetworkStart::assignVariables_impl(const VariablesMap& variables)
    {
        RTCOMMAND_GET_VAR_OR_THROW(variables, "time_offset", std::string, time_offset_str_)
        RTCOMMAND_GET_VAR_OR_THROW(variables, "max_lines", int, max_lines_)
        RTCOMMAND_CHECK_VAR(variables, "ignore_future_ts", ignore_future_ts_)
    }

// import asterix network stop

RTCommandImportASTERIXNetworkStop::RTCommandImportASTERIXNetworkStop()
    : rtcommand::RTCommand()
{
    condition.setDelay(500); // think about max duration
}

bool RTCommandImportASTERIXNetworkStop::run_impl()
{
    if (!COMPASS::instance().dbOpened())
    {
        setResultMessage("Database not opened");
        return false;
    }

    if (COMPASS::instance().appMode() == AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+COMPASS::instance().appModeStr());
        return false;
    }

    ASTERIXImportTask& import_task = COMPASS::instance().taskManager().asterixImporterTask();

    if (!import_task.isRunning() || !import_task.source().isNetworkType())
    {
        setResultMessage("No ASTERIX network import running");
        return false;
    }

    import_task.allowUserInteractions(false);

    MainWindow* main_window = dynamic_cast<MainWindow*> (rtcommand::mainWindow());
    traced_assert(main_window);

    main_window->liveStopSlot();

    // handle errors

    // if shitty
    //setResultMessage("VP error case 3");
    //return false;

    // if ok
    return true;
}

// import json
RTCommandImportJSONFile::RTCommandImportJSONFile()
    : rtcommand::RTCommand()
{
    condition.setSignal("compass.taskmanager.jsonimporttask.doneSignal", -1);  // think about max duration
}

rtcommand::IsValid  RTCommandImportJSONFile::valid() const
{
    CHECK_RTCOMMAND_INVALID_CONDITION(!filename_.size(), "Filename empty")
    CHECK_RTCOMMAND_INVALID_CONDITION(!Files::fileExists(filename_), string("File '")+filename_+"' does not exist")

    return RTCommand::valid();
}

bool RTCommandImportJSONFile::run_impl()
{
    if (!filename_.size())
    {
        setResultMessage("Filename empty");
        return false;
    }

    if (!Files::fileExists(filename_))
    {
        setResultMessage("File '"+filename_+"' does not exist");
        return false;
    }

    if (!COMPASS::instance().dbOpened())
    {
        setResultMessage("Database not opened");
        return false;
    }

    if (COMPASS::instance().appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+COMPASS::instance().appModeStr());
        return false;
    }

    JSONImportTask& import_task = COMPASS::instance().taskManager().jsonImporterTask();

    import_task.importFilename(filename_);

    if (!import_task.canRun())
    {
        setResultMessage("JSON import task can not be run");
        return false;
    }

    import_task.allowUserInteractions(false);
    import_task.run();

    return true;
}

void RTCommandImportJSONFile::collectOptions_impl(OptionsDescription& options,
                                                  PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
    ("filename,f", po::value<std::string>()->required(), "given filename, e.g. ’/data/file1.json’");

    ADD_RTCOMMAND_POS_OPTION(positional, "filename") // give position
}

void RTCommandImportJSONFile::assignVariables_impl(const VariablesMap& variables)
    {
        RTCOMMAND_GET_VAR_OR_THROW(variables, "filename", std::string, filename_)
    }

// import gps trail
RTCommandImportGPSTrail::RTCommandImportGPSTrail()
    : rtcommand::RTCommand()
{
    condition.setSignal("compass.taskmanager.gpstrailimporttask.doneSignal", -1); // think about max duration
}

rtcommand::IsValid  RTCommandImportGPSTrail::valid() const
{
    CHECK_RTCOMMAND_INVALID_CONDITION(!filename_.size(), "Filename empty")
    CHECK_RTCOMMAND_INVALID_CONDITION(!Files::fileExists(filename_), string("File '")+filename_+"' does not exist")

    return RTCommand::valid();
}

bool RTCommandImportGPSTrail::run_impl()
{
    if (!filename_.size())
    {
        setResultMessage("Filename empty");
        return false;
    }

    if (!Files::fileExists(filename_))
    {
        setResultMessage("File '"+filename_+"' does not exist");
        return false;
    }

    if (!COMPASS::instance().dbOpened())
    {
        setResultMessage("Database not opened");
        return false;
    }

    if (COMPASS::instance().appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+COMPASS::instance().appModeStr());
        return false;
    }

    GPSTrailImportTask& import_task = COMPASS::instance().taskManager().gpsTrailImportTask();

    //deactivate extra information
    import_task.useTodOffset(false);
    import_task.useOverrideDate(false);
    import_task.setMode3aCode(false);
    import_task.setTargetAddress(false);
    import_task.setCallsign(false);

    //reenable extra information based on whats provided
    if (has_tod_offset_)
    {
        import_task.useTodOffset(true);
        import_task.todOffset(tod_offset_);
    }

    if (!date_.empty())
    {
        import_task.useOverrideDate(true);
        import_task.overrideDate(boost::gregorian::from_string(date_));
    }

    if (!mode_3a_code_.empty())
    {
        import_task.setMode3aCode(true);
        import_task.mode3aCode(String::intFromOctalString(mode_3a_code_));
    }

    if (!aircraft_address_.empty())
    {
        import_task.setTargetAddress(true);
        import_task.targetAddress(String::intFromHexString(aircraft_address_));
    }

    if (!aircraft_id_.empty())
    {
        import_task.setCallsign(true);
        import_task.callsign(aircraft_id_);
    }

    if (!name_.empty())
    {
        import_task.dsName(name_);
    }

    if (sac_ >=  0)
    {
        import_task.dsSAC((unsigned int)sac_);
    }

    if (sic_ >=  0)
    {
        import_task.dsSIC((unsigned int)sic_);
    }

    //will parse the file
    import_task.importFilename(filename_);

    if (!import_task.canRun())
    {
        setResultMessage("GPS trail import task can not be run");
        return false;
    }

    import_task.allowUserInteractions(false);
    import_task.run();

    return true;
}

void RTCommandImportGPSTrail::collectOptions_impl(OptionsDescription& options,
                                                  PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
    ("filename,f", po::value<std::string>()->required(), "given filename, e.g. ’/data/file1.nmea'")
    ("name,n", po::value<std::string>()->default_value(""), "optional data source name, e.g. ’GPS Trail'")
    ("sac", po::value<int>()->default_value(-1), "optional sac, e.g. 255")
    ("sic", po::value<int>()->default_value(-1), "optional sic, e.g. 0")
    ("tod_offset,t", po::value<double>(), "optional time of day offset, e.g. -10000.0")
    ("date,d", po::value<std::string>()->default_value(""), "optional override date, e.g. ’2025-01-01'")
    ("mode3a,m", po::value<std::string>()->default_value(""), "optional mode3a code in octal, e.g. ")
    ("address,a", po::value<std::string>()->default_value(""), "optional aircraft address in hex, e.g. ’0xffffff'")
    ("id,i", po::value<std::string>()->default_value(""), "optional aircraft identification, e.g. ’ENTE'");

    ADD_RTCOMMAND_POS_OPTION(positional, "filename") // give position
}

void RTCommandImportGPSTrail::assignVariables_impl(const VariablesMap& variables)
{
    RTCOMMAND_GET_VAR_OR_THROW(variables, "filename", std::string, filename_)
    RTCOMMAND_GET_VAR_OR_THROW(variables, "name", std::string, name_)
    RTCOMMAND_GET_VAR_OR_THROW(variables, "sac", int, sac_)
    RTCOMMAND_GET_VAR_OR_THROW(variables, "sic", int, sic_)
    RTCOMMAND_CHECK_VAR(variables, "tod_offset", has_tod_offset_)
    RTCOMMAND_GET_VAR(variables, "tod_offset", double, tod_offset_)
    RTCOMMAND_GET_VAR_OR_THROW(variables, "date", std::string, date_)
    RTCOMMAND_GET_VAR_OR_THROW(variables, "mode3a", std::string, mode_3a_code_)
    RTCOMMAND_GET_VAR_OR_THROW(variables, "address", std::string, aircraft_address_)
    RTCOMMAND_GET_VAR_OR_THROW(variables, "id", std::string, aircraft_id_)
}

}
