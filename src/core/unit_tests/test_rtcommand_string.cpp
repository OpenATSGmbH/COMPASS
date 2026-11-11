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

// Parsing of runtime command strings. On Windows, file paths contain backslashes, which
// must reach the command unchanged. On Linux, the backslash is an escape character, and an
// invalid escape sequence must give a parse error, not an uncaught exception.

#include "catch.hpp"
#include "rtcommand_string.h"

#include <boost/program_options.hpp>

#include <string>

namespace po = boost::program_options;

namespace
{
    bool parseFilename(const QString& cmd, std::string& filename, QString& err_msg)
    {
        po::options_description d;
        d.add_options()("filename", po::value<std::string>(), "file name");

        po::positional_options_description pod;
        pod.add("filename", 1);

        po::variables_map vm;
        rtcommand::RTCommandString cmd_str(cmd);

        //the command name is the first token, the option parser skips it
        if (!cmd_str.parse(vm, d, pod, true, &err_msg))
            return false;

        if (vm.count("filename"))
            filename = vm["filename"].as<std::string>();

        return true;
    }
}

TEST_CASE("RTCommandString parses a plain path", "[rtcommand]")
{
    std::string filename;
    QString err_msg;

    REQUIRE(parseFilename("create_db /data/test/at_20230422/temp/empty.db", filename, err_msg));
    REQUIRE(filename == "/data/test/at_20230422/temp/empty.db");
}

TEST_CASE("RTCommandString parses a quoted path with spaces", "[rtcommand]")
{
    std::string filename;
    QString err_msg;

    REQUIRE(parseFilename("create_db \"/data/test dir/empty.db\"", filename, err_msg));
    REQUIRE(filename == "/data/test dir/empty.db");
}

TEST_CASE("RTCommandString parses escaped quotes in a quoted JSON argument", "[rtcommand]")
{
    std::string filename;
    QString err_msg;

    REQUIRE(parseFilename("reconfigure \"{\\\"parameters\\\": {\\\"live_cleanup_time_min\\\": 0}}\"", filename, err_msg));
    REQUIRE(filename == "{\"parameters\": {\"live_cleanup_time_min\": 0}}");

    REQUIRE(parseFilename("reconfigure '{\\\"a\\\": \\\"b c\\\"}'", filename, err_msg));
    REQUIRE(filename == "{\"a\": \"b c\"}");
}

#ifdef _WIN32
TEST_CASE("RTCommandString parses a quoted Windows path with spaces", "[rtcommand]")
{
    std::string filename;
    QString err_msg;

    REQUIRE(parseFilename("create_db \"C:\\data\\test dir\\new.db\"", filename, err_msg));
    REQUIRE(filename == "C:\\data\\test dir\\new.db");

    REQUIRE(parseFilename("create_db \\\\server\\share\\test.db", filename, err_msg));
    REQUIRE(filename == "\\\\server\\share\\test.db");
}
#endif

TEST_CASE("RTCommandString with a Windows path", "[rtcommand]")
{
    std::string filename;
    QString err_msg;

    bool ok = false;
    REQUIRE_NOTHROW(ok = parseFilename("create_db C:\\data\\test\\temp\\empty.db", filename, err_msg));

#ifdef _WIN32
    REQUIRE(ok);
    REQUIRE(filename == "C:\\data\\test\\temp\\empty.db");
#else
    //backslash is the escape character, \d is no valid escape sequence
    REQUIRE(!ok);
    REQUIRE(!err_msg.isEmpty());
#endif
}
