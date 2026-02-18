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

#include "rtcommand_defs.h"

#include <memory>

#include <QString>

namespace boost
{
    namespace program_options
    {
        class options_description;
        class positional_options_description;
        class variables_map;
    }
}

namespace rtcommand
{

/**
*/
class RTCommandString
{
public:
    typedef std::pair<std::unique_ptr<rtcommand::RTCommand>, IssueInfo> IssueResult;

    RTCommandString(const QString& cmd);
    virtual ~RTCommandString();

    QString cmdName() const;
    QString cmd() const;

    bool valid() const;

    RTCommandString& append(const QString& name, 
                            const QString& value, 
                            bool cmd_is_short_format = false, 
                            bool quote_value = false);

    bool hasHelpOption() const;
    bool parse(boost::program_options::variables_map& vm, 
               const boost::program_options::options_description& d,
               const boost::program_options::positional_options_description& pod,
               bool drop_quotes = true,
               QString* err_msg = nullptr) const;

    IssueResult issue() const;

    static const bool KeepEmptyCommands;

private:
    QString extractName() const;
    QString quoteString(const QString& s) const;
    QString paramFull(const QString& name, const QString& value, bool quote_value) const;
    QString paramShort(const QString& name, const QString& value, bool quote_value) const;

    QString cmd_name_;
    QString cmd_;
};

} // namespace rtcommand
