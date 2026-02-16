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

#include "rtcommand.h"
#include "rtcommand_wait_condition.h"
#include "rtcommand_registry.h"
#include "rtcommand_string.h"
#include "rtcommand_helpers.h"
#include "stringconv.h"
#include "ui_test_find.h"
#include "compass.h"
#include "viewmanager.h"
#include "viewcontainerwidget.h"

#include "logger.h"

#include <iostream>

#include <boost/program_options.hpp>
#include <boost/optional.hpp>

#include <QObject>
#include <QMainWindow>
#include <QApplication>
#include <QWindow>
#include <QDialog>

REGISTER_RTCOMMAND(rtcommand::RTCommandEmpty)
REGISTER_RTCOMMAND(rtcommand::RTCommandHelp)

using namespace Utils;

namespace rtcommand
{

    /***************************************************************************************
     * RTCommandWaitCondition
     ***************************************************************************************/

    /**
     * Configures the wait condition as 'signal' type.
     */
    void RTCommandWaitCondition::setSignal(const QString& obj_name,
                                           const QString& sig_name,
                                           int timeout_in_ms)
    {
        *this = {};

        type = Type::Signal;

        signal_obj        = obj_name;
        signal_name       = sig_name;
        signal_timeout_ms = timeout_in_ms;
    }

    /**
     * Sets the signal object and name from the given signal path.
     */
    bool RTCommandWaitCondition::setSignalFromPath(const QString& path)
    {
        auto sig = signalFromObjectPath(path.toStdString());
        if (!sig.has_value())
            return false;

        signal_obj  = QString::fromStdString(sig.value().first);
        signal_name = QString::fromStdString(sig.value().second);

        return true;
    }

    /**
     * Configures the wait condition as 'signal' type using a config string.
     * 
     * "SIGNAL_PATH;TIMEOUT_IN_MS"
     * E.g. "mainwindow.histogramview2.dataLoaded;10000"
     */
    bool RTCommandWaitCondition::setSignal(const QString &config_string)
    {
        if (config_string.isEmpty())
            return false;

        QStringList parts = config_string.split(";");

        if (parts.count() < 1 || 
            parts.count() > 2 ||
            parts[ 0 ].isEmpty() ||
            (parts.count() == 2 && parts[ 1 ].isEmpty()))
            return false;

        //optional signal timeout
        int sig_timeout = -1;
        if (parts.count() == 2)
        {
            bool ok;
            sig_timeout = parts[ 1 ].toInt(&ok);
            if (!ok)
                return false;
        }

        //obtain signal path as object and signal name
        auto sig = signalFromObjectPath(parts[ 0 ].toStdString());
        if (!sig.has_value())
            return false;

        QString sig_obj  = QString::fromStdString(sig.value().first);
        QString sig_name = QString::fromStdString(sig.value().second);

        setSignal(sig_obj, sig_name, sig_timeout);

        return true;
    }

    /**
     * Configures the wait condition as 'signal' type using a signal path and a timeout.
     * 
     * Path is e.g. "mainwindow.geographicview1.dataLoaded"
     */
    bool RTCommandWaitCondition::setSignal(const QString& signal_path, int timeout_in_ms)
    {
        auto sig = signalFromObjectPath(signal_path.toStdString());
        if (!sig.has_value())
            return false;

        QString sig_obj  = QString::fromStdString(sig.value().first);
        QString sig_name = QString::fromStdString(sig.value().second);

        setSignal(sig_obj, sig_name, timeout_in_ms);

        return true;
    }

    /**
     * Configures the wait condition as 'delay' type.
     */
    void RTCommandWaitCondition::setDelay(int ms)
    {
        *this = {};

        type = Type::Delay;

        delay_ms = ms;
    }

    /**
     * Configures the wait condition from the given config string.
     *
     * "DELAY_IN_MS"
     * E.g. "5000"
     */
    bool RTCommandWaitCondition::setDelay(const QString &config_string)
    {
        bool ok;
        int delay_ms = config_string.toInt(&ok);
        if (!ok)
            return false;

        setDelay(delay_ms);

        return true;
    }

    /***************************************************************************************
     * RTCommand
     ***************************************************************************************/

    const std::string RTCommand::HelpOptionFull = "help";
    const std::string RTCommand::HelpOptionShort = "h";
    const std::string RTCommand::HelpOption = HelpOptionFull + "," + HelpOptionShort;
    const std::string RTCommand::HelpOptionCmdFull = "--" + HelpOptionFull;
    const std::string RTCommand::HelpOptionCmdShort = "-" + HelpOptionShort;

    const std::string RTCommand::ReplyStringIndentation = "   ";
    const char        RTCommand::ObjectPathSeparator    = '.';
    const char        RTCommand::ParameterListSeparator = '|';

    std::vector<std::string> RTCommand::DefaultOptions = { "wait", "wait_signal", "async", HelpOptionFull };

    /**
     */
    RTCommand::RTCommand() = default;

    /**
     */
    RTCommand::~RTCommand() = default;

    /**
     */
    void RTCommand::resetResult() const 
    { 
        result_.reset();
    }

    /**
     */
    const RTCommandResult& RTCommand::result() const
    { 
        //update command name
        result_.command = name().toStdString();

        return result_;
    }

    /**
     * Collects command option descriptions throughout the class hierarchy.
     */
    bool RTCommand::collectOptions(OptionsDescription &options,
                                   PosOptionsDescription &positional,
                                   QString* err_msg)
    {
        try
        {
            // !add basic command options here!
            ADD_RTCOMMAND_OPTIONS(options)
            ("wait", po::value<int>()->default_value(0), "delay to wait after command execution, specified in milliseconds")
            ("wait_signal", po::value<std::string>()->default_value(""), "signal to wait for after command execution, specified as a configuration string \"PATH_TO_OBJECT.SIGNAL;TIMEOUT_IN_MS\"")
            ("async", "enables asynchrous command execution, meaning execution will return immediately after the command has been deployed to the main thread")
            (HelpOption.c_str(), "show command help information");

            // collect from derived
            collectOptions_impl(options, positional);
        }
        catch (const std::exception &ex)
        {
            logerr << "error: " << ex.what();
            if (err_msg) *err_msg = QString(ex.what());
            return false;
        }
        catch (...)
        {
            logerr << "unknown error";
            if (err_msg) *err_msg = "Unknown error";
            return false;
        }

        return true;
    }

    /**
     * Assigns scanned variables to struct data throughout the class hierarchy.
     */
    namespace
    {
        void configureWaitCondition(RTCommandWaitCondition& condition,
                                    const boost::program_options::variables_map &variables)
        {
            int wait_delay_ms;
            RTCOMMAND_GET_VAR_OR_THROW(variables, "wait", int, wait_delay_ms)

            QString wait_signal_config_str;
            RTCOMMAND_GET_QSTRING_OR_THROW(variables, "wait_signal", wait_signal_config_str)

            bool has_wait_delay  = (wait_delay_ms > 0);
            bool has_wait_signal = (!wait_signal_config_str.isEmpty());

            //only one wait condition possible
            if (has_wait_delay && has_wait_signal)
                throw std::runtime_error("Multiple wait conditions specified");

            if (has_wait_delay)
            {
                condition.setDelay(wait_delay_ms);
            }
            else if (has_wait_signal)
            {
                if (!condition.setSignal(wait_signal_config_str))
                    throw std::runtime_error("Badly configured signal wait condition");
            }
        }
    }
    bool RTCommand::assignVariables(const boost::program_options::variables_map &variables,
                                    QString* err_msg)
    {
        try
        {
            // !assign basic options here!
            configureWaitCondition(condition, variables);

            RTCOMMAND_CHECK_VAR(variables, "async", execute_async)

            // assign in derived
            assignVariables_impl(variables);
        }
        catch (const std::exception &ex)
        {
            logerr << "error: " << ex.what();
            if (err_msg) *err_msg = QString(ex.what());
            return false;
        }
        catch (...)
        {
            logerr << "unknown error";
            if (err_msg) *err_msg = "Unknown error";
            return false;
        }

        return true;
    }

    /**
     * Sets a result message, e.g. additional error information.
     * This is most likely useful when reimplementing run_impl() or checkResult_impl().
     */
    void RTCommand::setResultMessage(const std::string& m) const 
    { 
        result_.error.message = m; 
    }

    /**
     * Sets the commands JSON reply and optionally a string-representation of it.
     * This is most likely useful when reimplementing run_impl().
     */
    void RTCommand::setJSONReply(const nlohmann::json& json_reply, const std::string& reply_as_string) const
    { 
        result_.json_reply        = json_reply;
        result_.json_reply_string = reply_as_string;
    }

    /**
     * Sets the commands current state.
     */
    void RTCommand::setState(CmdState state) const
    {
        state_ = state;
    }

    /**
     * Sets the commands current error state/information.
     */
    void RTCommand::setError(CmdErrorCode code, boost::optional<std::string> msg) const
    {
        result_.error.code = code;
        if (msg.has_value())
            result_.error.message = msg.value();
    }

    /**
     * Run the command and track state.
     */
    bool RTCommand::run()
    {
        try
        {
            if (result().hasError())
                return false;

            //command should be in configured state
            if (state() != CmdState::Configured)
            {
                setError(CmdErrorCode::Exec_Unconfigured);
                return false;
            }

            // check again: command configuration valid?
            auto valid_state = valid();
            if (!valid_state.is_valid)
            {
                setError(CmdErrorCode::Exec_InvalidConfig, valid_state.errorString());
                return false;
            }

            // run command
            if (!run_impl())
            {
                setError(CmdErrorCode::Exec_Failed);
                return false;
            }
        }
        catch (const std::exception& ex)
        {
            setError(CmdErrorCode::Exec_Crash, std::string(ex.what()));
            return false;
        }
        catch (...)
        {
            setError(CmdErrorCode::Exec_Crash, std::string("Unexpected crash"));
            return false;
        }

        return true;
    }

    /**
     * Run command result check and track state.
     */
    bool RTCommand::checkResult()
    {
        try
        {
            if (result().hasError())
                return false;

            //command should be in executed state
            if (state() != CmdState::Executed)
            {
                setError(CmdErrorCode::ResultCheck_NotExecuted);
                return false;
            }

            // run check
            if (!checkResult_impl())
            {
                setError(CmdErrorCode::ResultCheck_InvalidResult);
                return false;
            }
        }
        catch (const std::exception& ex)
        {
            setError(CmdErrorCode::ResultCheck_Crash, std::string(ex.what()));
            return false;
        }
        catch (...)
        {
            setError(CmdErrorCode::ResultCheck_Crash, std::string("Unexpected crash"));
            return false;
        }

        return true;
    }

    /**
     * Checks if the command in finished state.
    */
    bool RTCommand::isFinished() const
    {
        return (state() == CmdState::Finished);
    }

    /**
     * Checks if the command in configured state.
    */
    bool RTCommand::isConfigured() const
    {
        return (state() == CmdState::Configured);
    }

    /**
     * Checks configuration validity, and sets the commands state accordingly.
     */
    bool RTCommand::checkConfiguration() const
    {
        auto valid_state = valid();
        if (!valid_state.is_valid)
        {
            setError(CmdErrorCode::Config_Invalid, valid_state.errorString());
            return false;
        }

        setState(CmdState::Configured);
        return true;
    }

    /**
     * Configure the command struct using the given command string.
     */
    bool RTCommand::configure(const RTCommandString &cmd)
    {
        setState(CmdState::Unconfigured);

        if (!cmd.valid())
        {
            // std::cout << "RTCommand::configure: Passed command not valid" << std::endl;
            setError(CmdErrorCode::Config_CommandStringInvalid);
            return false;
        }

        // does the command even concern me?
        if (name() != cmd.cmdName())
        {
            // std::cout << "RTCommand::configure: Name '" << name().toStdString()
            //           << "' does not match command name '" << cmd.cmdName().toStdString() << "'" << std::endl;
            setError(CmdErrorCode::Config_CommandStringMismatch);
            return false;
        }

        QString err_msg;

        // collect options description
        namespace po = boost::program_options;
        OptionsDescription od;
        PosOptionsDescription pod;
        if (!collectOptions(od, pod, &err_msg))
        {
            // std::cout << "RTCommand::configure: Could not collect options" << std::endl;
            setError(CmdErrorCode::Config_CollectOptionsFailed, err_msg.toStdString());
            return false;
        }

        // parse command using collected options description
        po::variables_map vm;
        if (!cmd.parse(vm, od, pod, true, &err_msg))
        {
            // std::cout << "RTCommand::configure: Could not parse command" << std::endl;
            setError(CmdErrorCode::Config_ParseOptionsFailed, err_msg.toStdString());
            return false;
        }

        // store parsed variables
        if (!assignVariables(vm, &err_msg))
        {
            // std::cout << "RTCommand::configure: Could not assign vars" << std::endl;
            setError(CmdErrorCode::Config_AssignOptionsFailed, err_msg.toStdString());
            return false;
        }

        //finally, check parsed configuration (also handles state)
        return checkConfiguration();
    }

    /***************************************************************************************
     * RTCommandHelp
     ***************************************************************************************/

    /**
     */
    bool RTCommandHelp::run_impl()
    {
        nlohmann::json root;
        std::string    str;

        auto getCommandInfo = [ & ] (const QString& cmd_name)
        {
            std::string info;
            nlohmann::json info_json;

            auto cmd = RTCommandRegistry::instance().createCommandTemplate(cmd_name);
            traced_assert(cmd);

            info += cmd->name().toStdString() + "\n";
            info += ReplyStringIndentation + cmd->description().toStdString() + "\n";

            info_json[ "command_name"        ] = cmd->name().toStdString();
            info_json[ "command_description" ] = cmd->description().toStdString();

            if (details)
            {
                std::string    options_info;
                nlohmann::json options_info_json;

                rtcommand::RTCommand::OptionsDescription    opt;
                rtcommand::RTCommand::PosOptionsDescription pos;
                bool ok = cmd->collectOptions(opt, pos);
                if (ok)
                {
                    for (const auto& o : opt.options())
                    {
                        //skip default options
                        auto it = std::find(rtcommand::RTCommand::DefaultOptions.begin(), 
                                            rtcommand::RTCommand::DefaultOptions.end(), 
                                            o->long_name());
                        if (it != rtcommand::RTCommand::DefaultOptions.end())
                            continue;

                        options_info += ReplyStringIndentation
                                     +  ReplyStringIndentation
                                     +  o->long_name() + "\t\t" + o->description() 
                                     + "\n";

                        nlohmann::json option;
                        option[ "option_name"        ] = o->long_name();
                        option[ "option_name_format" ] = o->format_name();
                        option[ "option_description" ] = o->description();

                        options_info_json.push_back(option);
                    }
                }
                else
                {
                    options_info = "options info not available\n";
                }

                info += options_info;

                info_json[ "command_options" ] = options_info_json;
            }

            return std::make_pair(info, info_json);
        };

        if (command.isEmpty())
        {
            const auto &cmds = RTCommandRegistry::instance().availableCommands();

            for (const auto &elem : cmds)
            {
                auto info = getCommandInfo(elem.first);

                str += info.first + "\n";
                root.push_back(info.second);
            }
        }
        else
        {
            if (!RTCommandRegistry::instance().hasCommand(command))
            {
                setResultMessage("Command '" + command.toStdString() + "' not registered");
                return false;
            }

            auto info = getCommandInfo(command);

            str += info.first;
            root = info.second;
        }

        setJSONReply(root, str);

        return true;
    }

    /**
     */
    void RTCommandHelp::collectOptions_impl(OptionsDescription &options,
                                            PosOptionsDescription &positional)
    {
        ADD_RTCOMMAND_OPTIONS(options)
        ("command", po::value<std::string>()->default_value(""), "command to retrieve help information for")
        ("details", "obtain detailed information such as parameter lists");

        ADD_RTCOMMAND_POS_OPTION(positional, "command")
    }

    /**
     */
    void RTCommandHelp::assignVariables_impl(const VariablesMap &variables)
    {
        RTCOMMAND_GET_QSTRING_OR_THROW(variables, "command", command)
        RTCOMMAND_CHECK_VAR(variables, "details", details)
    }

} // namespace rtcommand
