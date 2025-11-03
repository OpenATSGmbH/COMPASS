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

#include "source/configurationdatasource.h"
#include "source/dbdatasource.h"
#include "datasourcemanager.h"
#include "logger.h"
#include "util/number.h"

#include <algorithm>

using namespace std;
using namespace nlohmann;
using namespace Utils;

namespace dbContent
{


ConfigurationDataSource::ConfigurationDataSource(const std::string& class_id, const std::string& instance_id,
                                                 DataSourceManager& ds_manager)
    : Configurable(class_id, instance_id, &ds_manager)
{
    registerParameter("ds_type", &ds_type_, std::string());
    registerParameter("sac", &sac_, 0u);
    registerParameter("sic", &sic_, 0u);

    traced_assert(ds_type_.size());

    if (find(DataSourceManager::data_source_types_.begin(),
             DataSourceManager::data_source_types_.end(), ds_type_)
        == DataSourceManager::data_source_types_.end())
    {
        logerr << "sac/sic " << sac_ << sic_ << " ds_type '" << ds_type_
               << "' wrong";
    }

    traced_assert(find(DataSourceManager::data_source_types_.begin(),
                 DataSourceManager::data_source_types_.end(), ds_type_)
            != DataSourceManager::data_source_types_.end());

    registerParameter("name", &name_, std::string());
    registerParameter("has_short_name", &has_short_name_, false);
    registerParameter("short_name", &short_name_, std::string());

    registerParameter("info", &info_, {});

    traced_assert(name_.size());

    if (has_short_name_ && !short_name_.size())
        has_short_name_ = false;

    parseNetworkLineInfo();

    logdbg << "start" << name()
           << " sac/sic " << sac() << "/" << sic();
}

ConfigurationDataSource::~ConfigurationDataSource()
{
}

void ConfigurationDataSource::setFromJSON(const json& j)
{
    logdbg << "'" << j.dump(4) << "'";

    traced_assert(j.contains("ds_type"));

    ds_type_ = j["ds_type"];

    traced_assert(j.contains("sac"));
    traced_assert(j.contains("sic"));
    sac_ = j["sac"];
    sic_ = j["sic"];

    traced_assert(j.contains("name"));
    name_ = j["name"];

    if (j.contains("short_name"))
    {
        has_short_name_ = true;
        short_name_ = j["short_name"];
    }
    else
        has_short_name_ = false;

    if (j.contains("info"))
        info_ = j["info"];
    else
        info_ = nullptr;

    parseNetworkLineInfo();
}

DBDataSource* ConfigurationDataSource::getAsNewDBDS()
{
    DBDataSource* new_ds = new DBDataSource();
    new_ds->id(Number::dsIdFrom(sac_, sic_));
    new_ds->dsType(ds_type_);
    new_ds->sac(sac_);
    new_ds->sic(sic_);
    new_ds->name(name_);

    if (has_short_name_)
        new_ds->shortName(short_name_);

    if (!info_.is_null())
        new_ds->info(info_.dump());

    loginf << "name " << new_ds->name()
            << " sac/sic " << new_ds->sac() << "/" << new_ds->sic();

    return new_ds;
}

}
