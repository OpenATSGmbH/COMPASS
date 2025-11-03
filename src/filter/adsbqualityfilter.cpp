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

#include "adsbqualityfilter.h"
#include "adsbqualityfilterwidget.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "compass.h"
#include "logger.h"
//#include "stringconv.h"

#include <sstream>

using namespace std;
using namespace Utils;
using namespace nlohmann;

ADSBQualityFilter::ADSBQualityFilter(const std::string& class_id, const std::string& instance_id,
                                     Configurable* parent)
    : DBFilter(class_id, instance_id, parent, false)
{
    registerParameter("use_v0", &use_v0_, true);
    registerParameter("use_v1", &use_v1_, true);
    registerParameter("use_v2", &use_v2_, true);

    registerParameter("use_min_nucp", &use_min_nucp_, true);
    registerParameter("min_nucp", &min_nucp_, 4u);

    registerParameter("use_min_nic", &use_min_nic_, true);
    registerParameter("min_nic", &min_nic_, 5u);

    registerParameter("use_min_nacp", &use_min_nacp_, true);
    registerParameter("min_nacp", &min_nacp_, 5u);

    registerParameter("use_min_sil_v1", &use_min_sil_v1_, true);
    registerParameter("min_sil_v1", &min_sil_v1_, 2u);

    registerParameter("use_min_sil_v2", &use_min_sil_v2_, true);
    registerParameter("min_sil_v2", &min_sil_v2_, 4u);

    registerParameter("use_max_nucp", &use_max_nucp_, true);
    registerParameter("max_nucp", &max_nucp_, 4u);

    registerParameter("use_max_nic", &use_max_nic_, true);
    registerParameter("max_nic", &max_nic_, 5u);

    registerParameter("use_max_nacp", &use_max_nacp_, true);
    registerParameter("max_nacp", &max_nacp_, 5u);

    registerParameter("use_max_sil_v1", &use_max_sil_v1_, true);
    registerParameter("max_sil_v1", &max_sil_v1_, 2u);

    registerParameter("use_max_sil_v2", &use_max_sil_v2_, true);
    registerParameter("max_sil_v2", &max_sil_v2_, 4u);

    name_ = "ADSB Quality";

    createSubConfigurables();
}

ADSBQualityFilter::~ADSBQualityFilter() {}

bool ADSBQualityFilter::filters(const std::string& dbcont_name)
{
    loginf << dbcont_name << " " << (dbcont_name == "CAT021");

    return dbcont_name == "CAT021";
}

std::string ADSBQualityFilter::getConditionString(const std::string& dbcontent_name, bool& first)
{
    loginf << "dbcont_name " << dbcontent_name << " active " << active_;

    if (dbcontent_name != "CAT021")
        return "";

    stringstream ss;

    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    string mops_col_name = dbcont_man.getVariable("CAT021", DBContent::var_cat021_mops_version_).dbColumnName();
    string nacp_col_name = dbcont_man.getVariable("CAT021", DBContent::var_cat021_nacp_).dbColumnName();
    string mucp_nic_col_name = dbcont_man.getVariable("CAT021", DBContent::var_cat021_nucp_nic_).dbColumnName();
    string sil_col_name = dbcont_man.getVariable("CAT021", DBContent::var_cat021_sil_).dbColumnName();

    if (active_)
    {
        if (!first)
        {
            ss << " AND";
        }

        first = false;

        ss << " " << mops_col_name << " IN (";

        if (use_v0_)
            ss << "0";

        if (use_v1_)
        {
            if (use_v0_)
                ss << ",1";
            else
                ss << "1";
        }

        if (use_v2_)
        {
            if (use_v0_ || use_v1_)
                ss << ",2";
            else
                ss << "2";
        }

        ss << ")";

        if (use_min_nucp_)
            ss << " AND ((" << mucp_nic_col_name << " >= " << min_nucp_
               << " AND " << mops_col_name << "=0) OR " << mops_col_name << " IN (1,2))";

        if (use_max_nucp_)
            ss << " AND ((" << mucp_nic_col_name << " <= " << max_nucp_
               << " AND " << mops_col_name << "=0) OR " << mops_col_name << " IN (1,2))";

        if (use_min_nic_)
            ss << " AND ((" << mucp_nic_col_name << " >= " << min_nic_
               << " AND " << mops_col_name << " IN (1,2)) OR " << mops_col_name << " = 0)";

        if (use_max_nic_)
            ss << " AND ((" << mucp_nic_col_name << " <= " << max_nic_
               << " AND " << mops_col_name << " IN (1,2)) OR " << mops_col_name << " = 0)";

        if (use_min_nacp_)
            ss << " AND ((" << nacp_col_name << " >= " << min_nacp_
               << " AND " << mops_col_name << " IN (1,2)) OR " << mops_col_name << " = 0)";

        if (use_max_nacp_)
            ss << " AND ((" << nacp_col_name << " <= " << max_nacp_
               << " AND " << mops_col_name << " IN (1,2)) OR " << mops_col_name << " = 0)";

        if (use_min_sil_v1_)
            ss << " AND ((" << sil_col_name << " >= " << min_sil_v1_
               << " AND " << mops_col_name << "=1) OR " << mops_col_name << " IN (0,2))";

        if (use_max_sil_v1_)
            ss << " AND ((" << sil_col_name << " <= " << max_sil_v1_
               << " AND " << mops_col_name << "=1) OR " << mops_col_name << " IN (0,2))";

        if (use_min_sil_v2_)
            ss << " AND ((" << sil_col_name << " >= " << min_sil_v2_
               << " AND " << mops_col_name << "=2) OR " << mops_col_name << " IN (0,1))";

        if (use_max_sil_v2_)
            ss << " AND ((" << sil_col_name << " <= " << max_sil_v2_
               << " AND " << mops_col_name << "=2) OR " << mops_col_name << " IN (0,1))";

    }

    loginf << "here '" << ss.str() << "'";

    return ss.str();
}


void ADSBQualityFilter::generateSubConfigurable(const std::string& class_id,
                                                const std::string& instance_id)
{
    logdbg << "class_id " << class_id;

    throw std::runtime_error("ADSBQualityFilter: generateSubConfigurable: unknown class_id " + class_id);
}


void ADSBQualityFilter::checkSubConfigurables()
{
    logdbg;
}

DBFilterWidget* ADSBQualityFilter::createWidget()
{
    return new ADSBQualityFilterWidget(*this);
}


void ADSBQualityFilter::reset()
{
    use_v0_ = true;
    use_v1_ = true;
    use_v2_ = true;

    use_min_nucp_ = true;
    min_nucp_ = 4;

    use_min_nic_ = true;
    min_nic_ = 5;

    use_min_nacp_ = true;
    min_nacp_ = 5;

    use_min_sil_v1_ = true;
    min_sil_v1_ = 2;

    use_min_sil_v2_ = true;
    min_sil_v2_ = 4;

    widget_->update();
}

void ADSBQualityFilter::saveViewPointConditions (nlohmann::json& filters)
{
    traced_assert(conditions_.size() == 0);

    traced_assert(!filters.contains(name_));
    filters[name_] = json::object();
    json& filter = filters.at(name_);

    filter["use_v0"] = use_v0_;
    filter["use_v1"] = use_v1_;
    filter["use_v2"] = use_v2_;

    // nucp
    filter["use_min_nucp"] = use_min_nucp_;
    filter["min_nucp"] = min_nucp_;
    filter["use_max_nucp"] = use_max_nucp_;
    filter["max_nucp"] = max_nucp_;

    // nic
    filter["use_min_nic"] = use_min_nic_;
    filter["min_nic"] = min_nic_;
    filter["use_max_nic"] = use_max_nic_;
    filter["max_nic"] = max_nic_;

    // nacp
    filter["use_min_nacp"] = use_min_nacp_;
    filter["min_nacp"] = min_nacp_;
    filter["use_max_nacp"] = use_max_nacp_;
    filter["max_nacp"] = max_nacp_;

    // sil v1
    filter["use_min_sil_v1"] = use_min_sil_v1_;
    filter["min_sil_v1"] = min_sil_v1_;
    filter["use_max_sil_v1"] = use_max_sil_v1_;
    filter["max_sil_v1"] = max_sil_v1_;

    // sil v2
    filter["use_min_sil_v2"] = use_min_sil_v2_;
    filter["min_sil_v2"] = min_sil_v2_;
    filter["use_max_sil_v2"] = use_max_sil_v2_;
    filter["max_sil_v2"] = max_sil_v2_;
}

void ADSBQualityFilter::loadViewPointConditions (const nlohmann::json& filters)
{
    traced_assert(conditions_.size() == 0);

    traced_assert(filters.contains(name_));
    const json& filter = filters.at(name_);

    traced_assert(filter.contains("use_v0"));
    use_v0_ = filter.at("use_v0");

    traced_assert(filter.contains("use_v1"));
    use_v1_ = filter.at("use_v1");

    traced_assert(filter.contains("use_v2"));
    use_v2_= filter.at("use_v2");

    // nucp
    traced_assert(filter.contains("use_min_nucp"));
    use_min_nucp_ = filter.at("use_min_nucp");
    traced_assert(filter.contains("min_nucp"));
    min_nucp_ = filter.at("min_nucp");

    traced_assert(filter.contains("use_max_nucp"));
    use_max_nucp_ = filter.at("use_max_nucp");
    traced_assert(filter.contains("max_nucp"));
    max_nucp_ = filter.at("max_nucp");

    // nic
    traced_assert(filter.contains("use_min_nic"));
    use_min_nic_ = filter.at("use_min_nic");
    traced_assert(filter.contains("min_nic"));
    min_nic_ = filter.at("min_nic");

    traced_assert(filter.contains("use_max_nic"));
    use_max_nic_ = filter.at("use_max_nic");
    traced_assert(filter.contains("max_nic"));
    max_nic_ = filter.at("max_nic");

    // nacp
    traced_assert(filter.contains("use_min_nacp"));
    use_min_nacp_ = filter.at("use_min_nacp");
    traced_assert(filter.contains("min_nacp"));
    min_nacp_ = filter.at("min_nacp");

    traced_assert(filter.contains("use_max_nacp"));
    use_max_nacp_ = filter.at("use_max_nacp");
    traced_assert(filter.contains("max_nacp"));
    max_nacp_ = filter.at("max_nacp");

    // sil v1
    traced_assert(filter.contains("use_min_sil_v1"));
    use_min_sil_v1_ = filter.at("use_min_sil_v1");
    traced_assert(filter.contains("min_sil_v1"));
    min_sil_v1_ = filter.at("min_sil_v1");

    traced_assert(filter.contains("use_max_sil_v1"));
    use_max_sil_v1_ = filter.at("use_max_sil_v1");
    traced_assert(filter.contains("max_sil_v1"));
    max_sil_v1_ = filter.at("max_sil_v1");

    // sil v2
    traced_assert(filter.contains("use_min_sil_v2"));
    use_min_sil_v2_ = filter.at("use_min_sil_v2");
    traced_assert(filter.contains("min_sil_v2"));
    min_sil_v2_ = filter.at("min_sil_v2");

    traced_assert(filter.contains("use_max_sil_v2"));
    use_max_sil_v2_ = filter.at("use_max_sil_v2");
    traced_assert(filter.contains("max_sil_v2"));
    max_sil_v2_ = filter.at("max_sil_v2");


    if (widget())
        widget()->update();
}

bool ADSBQualityFilter::useV0() const
{
    return use_v0_;
}

void ADSBQualityFilter::useV0(bool value)
{
    loginf << "value " << value;
    use_v0_ = value;
}

bool ADSBQualityFilter::useV1() const
{
    return use_v1_;
}

void ADSBQualityFilter::useV1(bool value)
{
    loginf << "value " << value;
    use_v1_ = value;
}

bool ADSBQualityFilter::useV2() const
{
    return use_v2_;
}

void ADSBQualityFilter::useV2(bool value)
{
    loginf << "value " << value;
    use_v2_ = value;
}

bool ADSBQualityFilter::useMinNUCP() const
{
    return use_min_nucp_;
}

void ADSBQualityFilter::useMinNUCP(bool value)
{
    loginf << "value " << value;
    use_min_nucp_ = value;
}

unsigned int ADSBQualityFilter::minNUCP() const
{
    return min_nucp_;
}

void ADSBQualityFilter::minNUCP(unsigned int value)
{
    loginf << "value " << value;
    min_nucp_ = value;
}

bool ADSBQualityFilter::useMinNIC() const
{
    return use_min_nic_;
}

void ADSBQualityFilter::useMinNIC(bool value)
{
    loginf << "value " << value;
    use_min_nic_ = value;
}

unsigned int ADSBQualityFilter::minNIC() const
{
    return min_nic_;
}

void ADSBQualityFilter::minNIC(unsigned int value)
{
    loginf << "value " << value;
    min_nic_ = value;
}

bool ADSBQualityFilter::useMinNACp() const
{
    return use_min_nacp_;
}

void ADSBQualityFilter::useMinNACp(bool value)
{
    loginf << "value " << value;
    use_min_nacp_ = value;
}

unsigned int ADSBQualityFilter::minNACp() const
{
    return min_nacp_;
}

void ADSBQualityFilter::minNACp(unsigned int value)
{
    loginf << "value " << value;
    min_nacp_ = value;
}

bool ADSBQualityFilter::useMinSILv1() const
{
    return use_min_sil_v1_;
}

void ADSBQualityFilter::useMinSILv1(bool value)
{
    loginf << "value " << value;
    use_min_sil_v1_ = value;
}

unsigned int ADSBQualityFilter::minSILv1() const
{
    return min_sil_v1_;
}

void ADSBQualityFilter::minSILv1(unsigned int value)
{
    loginf << "value " << value;
    min_sil_v1_ = value;
}

bool ADSBQualityFilter::useMinSILv2() const
{
    return use_min_sil_v2_;
}

void ADSBQualityFilter::useMinSILv2(bool value)
{
    loginf << "value " << value;
    use_min_sil_v2_ = value;
}

unsigned int ADSBQualityFilter::minSILv2() const
{
    return min_sil_v2_;
}

void ADSBQualityFilter::minSILv2(unsigned int value)
{
    loginf << "value " << value;
    min_sil_v2_ = value;
}

bool ADSBQualityFilter::useMaxNUCP() const
{
    return use_max_nucp_;
}

void ADSBQualityFilter::useMaxNUCP(bool value)
{
    loginf << "value " << value;
    use_max_nucp_ = value;
}

unsigned int ADSBQualityFilter::maxNUCP() const
{
    return max_nucp_;
}

void ADSBQualityFilter::maxNUCP(unsigned int value)
{
    loginf << "value " << value;
    max_nucp_ = value;
}

bool ADSBQualityFilter::useMaxNIC() const
{
    return use_max_nic_;
}

void ADSBQualityFilter::useMaxNIC(bool value)
{
    loginf << "value " << value;
    use_max_nic_ = value;
}

unsigned int ADSBQualityFilter::maxNIC() const
{
    return max_nic_;
}

void ADSBQualityFilter::maxNIC(unsigned int value)
{
    loginf << "value " << value;
    max_nic_ = value;
}

bool ADSBQualityFilter::useMaxNACp() const
{
    return use_max_nacp_;
}

void ADSBQualityFilter::useMaxNACp(bool value)
{
    loginf << "value " << value;
    use_max_nacp_ = value;
}

unsigned int ADSBQualityFilter::maxNACp() const
{
    return max_nacp_;
}

void ADSBQualityFilter::maxNACp(unsigned int value)
{
    loginf << "value " << value;
    max_nacp_ = value;
}

bool ADSBQualityFilter::useMaxSILv1() const
{
    return use_max_sil_v1_;
}

void ADSBQualityFilter::useMaxSILv1(bool value)
{
    loginf << "value " << value;
    use_max_sil_v1_ = value;
}

unsigned int ADSBQualityFilter::maxSILv1() const
{
    return max_sil_v1_;
}

void ADSBQualityFilter::maxSILv1(unsigned int value)
{
    loginf << "value " << value;
    max_sil_v1_ = value;
}

bool ADSBQualityFilter::useMaxSILv2() const
{
    return use_max_sil_v2_;
}

void ADSBQualityFilter::useMaxSILv2(bool value)
{
    loginf << "value " << value;
    use_max_sil_v2_ = value;
}

unsigned int ADSBQualityFilter::maxSILv2() const
{
    return max_sil_v2_;
}

void ADSBQualityFilter::maxSILv2(unsigned int value)
{
    loginf << "value " << value;
    max_sil_v2_ = value;
}
