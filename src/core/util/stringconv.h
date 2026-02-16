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

#include "json_fwd.hpp"

#include <sstream>

#include <vector>
#include <set>

namespace boost
{
namespace posix_time
{
class ptime;
}
}

namespace Utils
{
namespace String
{

extern bool isNumber(const std::string& number_str);
extern std::string intToString(int number, int width, char c);
extern std::string categoryString(unsigned int cat);
extern std::string doubleToStringPrecision(double number, unsigned int precision);
extern std::string doubleToStringNoScientific(double number);
extern std::string percentToString(double number, unsigned int precision=2);
extern std::string percentToStringProtected(double x, double y, unsigned int precision=2); // protected against y=0
extern std::string boolToString(bool value);
extern unsigned int intFromOctalString(std::string number);
extern unsigned int intFromHexString(std::string number);

extern std::vector<std::string>& split(const std::string& s, char delim, std::vector<std::string>& elems);
extern std::vector<std::string> split(const std::string& s, char delim);

extern std::string timeStringFromDouble(double seconds, bool milliseconds = true);
extern double timeFromString(std::string time_str, bool* ok=nullptr);

extern std::string octStringFromInt(int number);
extern std::string octStringFromInt(int number, int width, char c);
extern std::string hexStringFromInt(int number);
extern std::string hexStringFromInt(int number, int width, char c);

extern int getAppendedInt(std::string text);
extern unsigned int lineFromStr(const std::string& line_str);
extern std::string lineStrFrom(unsigned int line);

extern int getLeadingInt(std::string text);

extern double doubleFromLatitudeString(std::string& latitude_str);
extern double doubleFromLongitudeString(std::string& longitude_str);

extern unsigned int hash(const std::string& str);

extern std::string getValueString(const std::string& value);
extern std::string getValueString(const float& value);
extern std::string getValueString(const double& value);
extern std::string getValueString(const nlohmann::json& value);
extern std::string getValueString(const boost::posix_time::ptime& value);

template <typename T>
std::string getValueString(T value)
{
    return std::to_string(value);
}

extern std::string compress(const std::vector<std::string>& values, char seperator);

template <typename T>
std::string compress(const std::vector<T>& values, char seperator)
{
    std::ostringstream ss;

    bool first_val = true;
    for (auto& val_it : values)
    {
        if (!first_val)
            ss << seperator;

        ss << std::to_string(val_it);

        first_val = false;
    }

    return ss.str();
}

extern std::string compress(const std::set<std::string>& values, char seperator);

template <typename T>
std::string compress(const std::set<T>& values, char seperator)
{
    std::ostringstream ss;

    bool first_val = true;
    for (auto& val_it : values)
    {
        if (!first_val)
            ss << seperator;

        ss << std::to_string(val_it);

        first_val = false;
    }

    return ss.str();
}

extern bool hasEnding(std::string const& full_string, std::string const& ending);

extern bool replace(std::string& str, const std::string& from, const std::string& to);

// 0 if same, -1 if v1 > v2, 1 if v1 < v2
extern int compareVersions(const std::string& v1_str, const std::string& v2_str);

extern std::string latexString(std::string str);

extern std::string ipFromString(const std::string& name);
extern unsigned int portFromString(const std::string& name);

extern std::string trim(const std::string& name);

extern std::string ecatToString(unsigned int ecat);

}  // namespace String

// template <typename T> std::string formatBinaryString (T val)
//{
//    int length = sizeof(T) * 8;
//    std::string binary(length, '0');
//    for (int i = 0; i < length; i++)
//    {
//        if ((val >> i) & 1)
//            binary[length - 1 - i] = '1';
//    }

//    return binary;
//}

}  // namespace Utils

