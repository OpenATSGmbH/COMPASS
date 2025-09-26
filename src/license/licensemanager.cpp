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

#include "licensemanager.h"

#include "compass.h"
#include "files.h"
#include "logger.h"

#include "json.hpp"

#include <fstream>
#include <map>

const std::string LicenseManager::LicenseFileName = "licenses.json";

/**
*/
LicenseManager::LicenseManager(const std::string& class_id, 
                               const std::string& instance_id,
                               COMPASS* parent)
:   Configurable(class_id, instance_id, parent, "licenses.json")
{
    //read licenses on creation
    readLicenses();

    createSubConfigurables();
}

/**
*/
LicenseManager::~LicenseManager()
{
    //@TODO: final write of licenses on deletion?
}

/**
*/
std::string LicenseManager::licenseFilePath()
{
    return LICENSE_SUBDIRECTORY + LicenseFileName;
}

/**
*/
bool LicenseManager::readLicenses()
{
    licenses_.clear();

    auto p = licenseFilePath();

    //no license file? => should be ok, e.g. on first start?
    if (!Utils::Files::fileExists(p))
        return true;

    //open license file for reading
    std::ifstream f(p.c_str());
    if (!f.is_open())
    {
        logerr << "license file could not be opened for reading";
        return false;
    }

    nlohmann::json j;
    
    //try to parse license json
    try
    {
        j = nlohmann::json::parse(f);
    } 
    catch(const std::exception& ex)
    {
        logerr << "license file could not be parsed: " << ex.what();
        return false;
    }
    catch(...)
    {
        logerr << "license file could not be parsed";
        return false;
    }

    f.close();

    //needs to be a valid json object
    if (!j.is_object())
    {
        logerr << "license file invalid";
        return false;
    }

    //license array present?
    if (!j.contains("licenses"))
    {
        logerr << "license file invalid";
        return false;
    }

    const auto& jlicenses = j.at("licenses");
    if (!jlicenses.is_object())
    {
        logerr << "license file invalid";
        return false;
    }

    //read license array
    for (const auto& jl : jlicenses.items())
    {
        std::string id = jl.key();

        traced_assert(licenses_.find(id) == licenses_.end());

        auto& l = licenses_[ id ];

        l.read(jl.value(), &id);

        if (l.state != license::License::State::Read)
            logwrn << "license '" << id << "' could not be read: " << l.error;
        else if (!l.isComplete())
            logwrn << "license '" << id << "' is incomplete";
    }

    emit licensesChanged();
    emit changed();

    return true;
}

/**
*/
bool LicenseManager::writeLicenses() const
{
    //create missing license directory
    if (!Utils::Files::directoryExists(LICENSE_SUBDIRECTORY) &&
        !Utils::Files::createMissingDirectories(LICENSE_SUBDIRECTORY))
    {
        logerr << "license directory could not be created";
        return false;
    }

    auto p = licenseFilePath();

    //open license file for write
    std::ofstream f(p.c_str());
    if (!f.is_open())
    {
        logerr << "license file inaccessible";
        return false;
    }

    //store json licenses
    nlohmann::json j;

    j["licenses"] = nlohmann::json::object();

    auto& jlicenses = j["licenses"];

    bool ok = true;

    for (const auto& l : licenses_)
    {
        if (!l.second.isComplete())
            logwrn << "incomplete license of id '" << l.first << "' encountered";
        
        jlicenses[ l.first ] = l.second.json_blob.is_object() ? l.second.json_blob : nlohmann::json::object();
    }

    //write to license file
    auto str = j.dump(4);

    f << str;

    //write failed?
    if (!f)
    {
        logerr << "license file could not be written";
        return false;
    }

    f.close();

    return ok;
}

/**
*/
boost::optional<std::string> LicenseManager::activeLicenseID() const
{
    std::map<license::License::Type, std::vector<std::string>> valid_licenses;
    for (const auto& l : licenses_)
        if (l.second.validity().first == license::License::Validity::Valid)
            valid_licenses[ l.second.type ].push_back(l.first);

    if (valid_licenses.empty() || valid_licenses.rbegin()->second.empty())
        return {};

    std::string lid_newest;
    boost::optional<boost::posix_time::ptime> license_date;

    for (const auto& lid : valid_licenses.rbegin()->second)
    {
        const auto& l = getLicense(lid);

        if (!license_date.has_value() || l.created > license_date.value())
        {
            lid_newest   = lid;
            license_date = l.created;
        }
    }

    if (!license_date.has_value())
        return {};

    return lid_newest;
}

/**
*/
const license::License* LicenseManager::activeLicense() const
{
    auto id = activeLicenseID();
    if (!id.has_value())
        return nullptr;

    return &getLicense(id.value());
}

/**
*/
const license::License* LicenseManager::activeLicenseForComponent(license::License::Component c) const
{
    std::map<license::License::Type, std::vector<const license::License*>> valid_licenses;
    for (const auto& l : licenses_)
        if (l.second.validity().first == license::License::Validity::Valid && l.second.componentEnabled(c))
            valid_licenses[ l.second.type ].push_back(&l.second);

    if (valid_licenses.empty() || valid_licenses.rbegin()->second.empty())
        return nullptr;

    const license::License* l_newest;
    boost::optional<boost::posix_time::ptime> license_date;

    for (auto l: valid_licenses.rbegin()->second)
    {
        if (!license_date.has_value() || l->created > license_date.value())
        {
            l_newest     = l;
            license_date = l->created;
        }
    }

    if (!license_date.has_value())
        return nullptr;

    return l_newest;
}

/**
*/
bool LicenseManager::componentEnabled(license::License::Component c) const
{
    for (const auto& l : licenses_)
        if (l.second.validity().first == license::License::Validity::Valid && l.second.componentEnabled(c))
            return true;

    return false;
}

/**
*/
bool LicenseManager::hasLicense(const std::string& id) const
{
    return (licenses_.find(id) != licenses_.end());
}

/**
*/
const license::License& LicenseManager::getLicense(const std::string& id) const
{
    auto it = licenses_.find(id);
    traced_assert(it != licenses_.end());
    return it->second;
}

/**
*/
const std::map<std::string, license::License>& LicenseManager::getLicenses() const
{
    return licenses_;
}

/**
*/
bool LicenseManager::addLicense(const license::License& license, bool write_licenses)
{
    //adding a license shall not overwrite existing ids
    traced_assert(!hasLicense(license.id));

    licenses_[ license.id ] = license;

    //update license file?
    bool ok = write_licenses ? writeLicenses() : true;

    emit licensesChanged();
    emit changed();

    return ok;
}

/**
*/
bool LicenseManager::setLicense(const license::License& license, bool write_licenses)
{
    licenses_[ license.id ] = license;

    //update license file?
    bool ok = write_licenses ? writeLicenses() : true;

    emit licensesChanged();
    emit changed();

    return ok;
}

/**
*/
bool LicenseManager::removeLicense(const std::string& id, bool write_licenses)
{
    traced_assert(hasLicense(id));

    licenses_.erase(id);

    //update license file?
    bool ok =  write_licenses ? writeLicenses() : true;

    emit licensesChanged();
    emit changed();

    return ok;
}

/**
*/
size_t LicenseManager::numLicenses() const
{
    return licenses_.size();
}
