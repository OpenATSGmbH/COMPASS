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

#include "rtcommand_string.h"
#include "rtcommand_registry.h"

#include "logger.h"

#include <iostream>

#include <boost/program_options.hpp>

#include <QStringList>
#include <QRegularExpression>
#include <QProcess>

using namespace std;

namespace rtcommand
{

namespace 
{
    QStringList splitCommand(const QString& command, bool keep_empty)
    {
        //return QProcess::splitCommand(cmd); // does only exist in Qt 5.15+

        QStringList args;
        QString tmp;
        int quoteCount = 0;
        bool inQuote = false;

        // handle quoting. tokens can be surrounded by double quotes
        // "hello world". three consecutive double quotes represent
        // the quote character itself.
        for (int i = 0; i < command.size(); ++i) 
        {
            if (command.at(i) == QLatin1Char('"')) 
            {
                ++quoteCount;
                if (quoteCount == 3) 
                {
                    // third consecutive quote
                    quoteCount = 0;
                    tmp += command.at(i);
                }
                continue;
            }
            if (quoteCount) 
            {
                if (quoteCount == 1)
                    inQuote = !inQuote;
                
                quoteCount = 0;
            }
            if (!inQuote && command.at(i).isSpace()) 
            {
                if (keep_empty || !tmp.isEmpty())
                {
                    args += tmp;
                    tmp.clear();
                }
            } 
            else 
            {
                tmp += command.at(i);
            }
        }

        if (keep_empty || !tmp.isEmpty())
            args += tmp;
        
        return args;
    }

    vector<string> splitCommand2 (const string& command)
    {
        //loginf << "UGA0 '" << command << "'";

        vector<string> parts = boost::program_options::split_unix(command);

//        for (auto& part : parts)
//            loginf << "UGA1 '" << part << "'";

        return parts;
    }

}

const bool RTCommandString::KeepEmptyCommands = false;

/**
*/
RTCommandString::RTCommandString(const QString& cmd)
:   cmd_(cmd.trimmed())
{
    //extract command name and store
    cmd_name_ = extractName();
}

/**
*/
RTCommandString::~RTCommandString() = default;

/**
 * Extracts the command name from the current command string.
 */
QString RTCommandString::extractName() const
{
    if (cmd_.isEmpty())
        return "";
    
    auto s = splitCommand(cmd_, KeepEmptyCommands);
    if (s.isEmpty())
        return "";

    return s[ 0 ];
}

/**
*/
QString RTCommandString::cmdName() const
{
    return cmd_name_;
}

/**
*/
QString RTCommandString::cmd() const
{ 
    return cmd_;
}

/**
 */
bool RTCommandString::valid() const
{
    return (!cmd_name_.isEmpty());
}

/**
 * Sets the given string under quotes. 
 */
QString RTCommandString::quoteString(const QString& s) const
{ 
    return "\"" + s + "\"";
}

/**
 * Generates a full format command string (--name=value).
 */
QString RTCommandString::paramFull(const QString& name, const QString& value, bool quote_value) const
{ 
    return "--" + name + "=" + (quote_value ? quoteString(value) : value); 
}

/**
 * Generates a short format command string (-name value).
 */
QString RTCommandString::paramShort(const QString& name, const QString& value, bool quote_value) const
{ 
    return "-" + name + " " + (quote_value ? quoteString(value) : value); 
}

/**
 * Appends a new command option to the command string.
 */
RTCommandString& RTCommandString::append(const QString& name, 
                                         const QString& value, 
                                         bool cmd_is_short_format, 
                                         bool quote_value)
{  
    cmd_ += " ";
    cmd_ += (cmd_is_short_format ? paramShort(name, value, quote_value) : 
                                   paramFull (name, value, quote_value));
    return (*this);
}

/**
 * Checks if the current command string obtains the help option (--help or -h).
 */
bool RTCommandString::hasHelpOption() const
{
    if (cmd_.isEmpty() || !valid())
        return false;

    QStringList parts = splitCommand(cmd_, KeepEmptyCommands);
    int argc = parts.count();

    if (argc < 1)
        return false;

    for (int i = 1; i < argc; ++i)
    {
        auto s = parts[ i ].trimmed().toStdString();
        if (s == RTCommand::HelpOptionCmdShort || s == RTCommand::HelpOptionCmdFull)
            return true;
    }

    return false;
}

/**
 * Parses the current command string for the provided program option description.
 */
bool RTCommandString::parse(boost::program_options::variables_map& vm, 
                            const boost::program_options::options_description& d,
                            const boost::program_options::positional_options_description& pod,
                            bool drop_quotes,
                            QString* err_msg) const
{
    namespace po = boost::program_options;

    auto setErrorMsg = [ & ] (const QString& msg) { if (err_msg) *err_msg = msg; };

    if (cmd_.isEmpty() || !valid())
    {
        setErrorMsg("Parse error: Invalid command string");
        return false;
    }

    //QStringList parts = splitCommand(cmd_);

    vector<string> parts = splitCommand2(cmd_.toStdString());

    int argc = parts.size();

    if (argc < 1)
    {
        setErrorMsg("Parse error: Empty command string");
        return false;
    }

    std::vector<std::string> strings(argc);
    std::vector<const char*> argv   (argc);
    for (int i = 0; i < argc; ++i)
    {
        strings[ i ] = parts  [ i ];
        argv   [ i ] = strings[ i ].c_str();
    }
    
    try
    {
        po::store(po::command_line_parser(argc, argv.data()).options(d).positional(pod).run(), vm);
        po::notify(vm);
    }
    catch (const std::exception& ex)
    {
        logerr << "error parsing command: " << ex.what();
        setErrorMsg("Parse error: " + QString(ex.what()));
        return false;
    }
    catch (...)
    {
        logerr << "error parsing command: Unknown error";
        setErrorMsg("Parse error: Unknown error");
        return false;
    }

    if (drop_quotes)
    {
        //chop quotes from all variable strings
        for (auto& elem : vm)
        {
            try
            {
                //try to cast var to string
                auto& str = elem.second.as<std::string>();

                QString s = QString::fromStdString(str);
                if (s.startsWith("\"") && 
                    s.endsWith("\"") && 
                    s.count() >= 2)
                {
                    s = s.mid(1, s.count() - 2);
                    str = s.toStdString();
                }
            }
            catch (...)
            {
                continue;
            }
        }
    }

    return true;
}

/**
 * Issues a pre-configured rtcommand object using the currently stored command string.
*/
RTCommandString::IssueResult RTCommandString::issue() const
{
    IssueResult result;
    result.second.command = cmdName().toStdString();

    auto setResultCode = [ & ] (CmdErrorCode code, const std::string& msg)
    {
        result.second.error.code = code;
        result.second.error.message  = msg;
    };

    //check command string validity
    if (!valid())
    {
        logdbg << "command string not valid";
        setResultCode(CmdErrorCode::Issue_CommandStringInvalid, "");
        return result;
    }

    const QString cmd_str  = cmd();
    const QString cmd_name = cmdName();

    logdbg << "command to process: " << cmd_str.toStdString();
    logdbg << "creating command template from name '" << cmd_name.toStdString() << "'";

    //check if command is even registered
    if (!rtcommand::RTCommandRegistry::instance().hasCommand(cmd_name))
    {
        logerr << "command not registered";
        setResultCode(CmdErrorCode::Issue_CommandNotFound, "");
        return result;
    }

    //preparse command for help option
    if (hasHelpOption())
    {
        //help option detected, return help command
        logdbg << "help option detected";

        auto cmd_help = new RTCommandHelp;
        cmd_help->command = cmd_name;

        result.first = std::unique_ptr<RTCommand>(cmd_help);
        result.second.issued = true;
        
        return result;
    }

    //create empty command template
    auto cmd_obj = rtcommand::RTCommandRegistry::instance().createCommandTemplate(cmd_name);
    if (!cmd_obj)
    {
        logerr << "registry returned nullptr";
        setResultCode(CmdErrorCode::Issue_CommandCreationFailed, "");
        return result;
    }

    //check if command name matches
    if (cmd_obj->name() != cmd_name)
    {
        logerr << "issued command name mismatch";
        setResultCode(CmdErrorCode::Issue_CommandStringMismatch, "");
        return result;
    }

    logdbg << "configuring command template...";

    //configure command
    if (!cmd_obj->configure(*this))
    {
        logerr << "command could not be configured";
        setResultCode(cmd_obj->result().error.code, cmd_obj->result().error.message);
        return result;
    }

    //store issued command
    result.first         = std::move(cmd_obj);
    result.second.issued = true;

    return result;
}

} // namespace rtcommand
