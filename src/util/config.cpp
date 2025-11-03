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

#include "config.h"
#include "files.h"
#include "logger.h"
#include "traced_assert.h"

#include <stdlib.h>

#include <fstream>
#include <limits>


using namespace std;
using namespace Utils;
using namespace nlohmann;

SimpleConfig::SimpleConfig(const std::string& config_filename) : config_filename_(config_filename)
{
    try
    {
        loadFile();
        traced_assert(opened_);
    }
    catch (exception& e)
    {
        cout << "SimpleConfig: constructor: exception " << e.what() << '\n';
        opened_ = false;
    }
}

SimpleConfig::~SimpleConfig() { opened_ = false; }

void SimpleConfig::loadFile()
{
    traced_assert(!opened_);

    std::string config_path = HOME_CONF_DIRECTORY + config_filename_;

    Files::verifyFileExists(config_path);

    std::ifstream config_file(config_path, std::ifstream::in);

    try
    {
        config_ = json::parse(config_file);
    }
    catch (json::exception& e)
    {
        logerr << "could not load file '" << config_path << "'";
        throw e;
    }

    opened_ = true;
}

bool SimpleConfig::getBool(const std::string& id)
{
    if (!opened_)
        throw std::runtime_error("SimpleConfig: getBool: config file was not opened");

    if (!existsId(id))
        throw std::runtime_error("SimpleConfig: getBool: config id '" + id +
                                 "' not present in configuration");

    if (!config_.at(id).is_boolean())
        throw std::runtime_error("SimpleConfig: getBool: config id '" + id + "' is not boolean");

    return config_.at(id);
}

int SimpleConfig::getInt(const std::string& id)
{
    if (!opened_)
        throw std::runtime_error("SimpleConfig: getInt: config file was not opened");

    if (!existsId(id))
        throw std::runtime_error("SimpleConfig: getInt: config id '" + id +
                                 "' not present in configuration");

    if (!config_.at(id).is_number_integer())
        throw std::runtime_error("SimpleConfig: getBool: config id '" + id + "' is not integer");

    return config_.at(id);
}

unsigned int SimpleConfig::getUnsignedInt(const std::string& id)
{
    if (!opened_)
        throw std::runtime_error("SimpleConfig: getUnsignedInt: config file was not opened");

    if (!existsId(id))
        throw std::runtime_error("SimpleConfig: getUnsignedInt: config id '" + id +
                                 "' not present in configuration");

    if (!config_.at(id).is_number_unsigned())
        throw std::runtime_error("SimpleConfig: getBool: config id '" + id +
                                 "' is not unsigned integer");

    return config_.at(id);
}

double SimpleConfig::getDouble(const std::string& id)
{
    if (!opened_)
        throw std::runtime_error("SimpleConfig: getDouble: config file was not opened");

    if (!existsId(id))
        throw std::runtime_error("SimpleConfig: getDouble: config id '" + id +
                                 "' not present in configuration");

    if (!config_.at(id).is_number_float())
        throw std::runtime_error("SimpleConfig: getBool: config id '" + id + "' is not float");

    return config_.at(id);
}

std::string SimpleConfig::getString(const std::string& id)
{
    if (!opened_)
        throw std::runtime_error("SimpleConfig: getValue: config file was not opened");

    if (!existsId(id))
        throw std::runtime_error("SimpleConfig: getBool: config id '" + id +
                                 "' not present in configuration");

    if (!config_.at(id).is_string())
        throw std::runtime_error("SimpleConfig: getBool: config id '" + id + "' is not string");

    return config_.at(id);
}

bool SimpleConfig::existsId(const std::string& id)
{
    traced_assert(id.size() > 0);
    return config_.contains(id);
}
