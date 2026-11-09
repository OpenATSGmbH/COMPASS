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

#include "configurable.h"
#include "license.h"

#include <map>
#include <vector>
#include <string>

#include <QObject>

#include <boost/optional.hpp>

#include "json_fwd.hpp"

/**
*/
class LicenseManager : public QObject, public Configurable
{
    Q_OBJECT
public:
    LicenseManager(nlohmann::json& config, Configurable& parent);
    virtual ~LicenseManager();

    bool readLicenses();
    bool writeLicenses() const;

    const license::License* activeLicense() const;
    std::string licenseeString(bool licensed_to = true) const; // licensee of the active license, empty if none
    std::string versionString(const std::string& app_version,
                              bool open_ats = true,
                              bool license_type = true) const; // e.g. "OpenATS COMPASS v1.1.0 Free"
    const license::License* activeLicenseForComponent(license::License::Component c) const;
    bool componentEnabled(license::License::Component c) const;
    
    bool hasLicense(const std::string& id) const;
    const license::License& getLicense(const std::string& id) const;
    const std::map<std::string, license::License>& getLicenses() const;
    bool addLicense(const license::License& license, bool write_licenses);
    bool setLicense(const license::License& license, bool write_licenses);
    bool removeLicense(const std::string& id, bool write_licenses);
    size_t numLicenses() const;

    void generateSubConfigurable(nlohmann::json& child_json) override {}

    static const std::string LicenseFileName;

signals:
    void changed();
    void licensesChanged();

protected:
    virtual void checkSubConfigurables() override {}

private:
    boost::optional<std::string> activeLicenseID() const;

    static std::string licenseFilePath();

    std::map<std::string, license::License> licenses_;
};
