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

#include "dbcontent/label/labelgenerator.h"
#include "compass.h"
#include "iviewprovider.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/dbcontent.h"
//#include "dbcontent/variable/metavariable.h"
#include "dbcontent/variable/reportvariable.h"
#include "dbcontent/variable/variableset.h"
//#include "dbcontent/label/labelgeneratorwidget.h"
#include "dbcontent/label/labelcontentdialog.h"
#include "db_context_manager.h"
#include "logger.h"
#include "stringconv.h"
#include "timeconv.h"
#include "number.h"

#include "global.h"

//#if USE_EXPERIMENTAL_SOURCE
//#include "geometryleafitemlabels.h"
//#endif

#include <QRect>

#include <osg/Vec4>

#include <algorithm>

using namespace std;
using namespace nlohmann;
using namespace Utils;

namespace dbContent
{

const int          LabelGenerator::LabelAnchorLengthPxDefault        = 20;
const unsigned int LabelGenerator::MaxDeclutterLabels                = 200;
const unsigned int LabelGenerator::MaxDeclutterLabelsConstraintBased = 500;
const float        LabelGenerator::MaxDeclutterScreenRatio           = 0.4f;

LabelGenerator::LabelGenerator(DBContentManager& manager)
    : dbcont_manager_(manager)
{
    // every setter emits labelConfigChanged => count config changes for caches of derived results
    connect(this, &LabelGenerator::labelConfigChanged, this, [ this ] () { ++config_generation_; });
}

LabelGenerator::~LabelGenerator()
{
}

void LabelGenerator::classifyIdentification(const std::string& raw_acid,
                                            std::string& usable_id, std::string& fallback_id) const
{
    string value = raw_acid;
    value.erase(std::remove(value.begin(), value.end(), ' '), value.end());

    if (dbcont_manager_.compass().isUnspecificACID(raw_acid))
    {
        if (fallback_id.empty())
            fallback_id = value;
    }
    else
    {
        if (usable_id.empty())
            usable_id = value;
    }
}

std::vector<std::string> LabelGenerator::getLabelTexts(const std::string& dbcontent_name,
                                                       const std::shared_ptr<Buffer>& buffer,
                                                       unsigned int buffer_index,
                                                       const std::optional<unsigned int>& override_lod)
{
    std::vector<std::string> tmp;

    if (!buffer)
    {
        logerr << "no buffer given for dbcontent '" << dbcontent_name << "'";
        return tmp;
    }

    unsigned int lod = override_lod.has_value() ? override_lod.value() : currentLOD();

    if (buffer_index >= buffer->size())
    {
        logerr << "buffer index of dbcontent '" << dbcontent_name << "' out of range";
        return tmp;
    }

    using namespace dbContent;

    Variable& utn_var = dbcont_manager_.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_utn_);

    // the identification Meta Variable, CAT062 is the selection "Aircraft Identification*"
    // over I062/245, I062/380 and the flight plan callsigns, no own fallback chain here
    Variable* acid_var {nullptr};
    if (dbcont_manager_.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_acid_))
        acid_var = &dbcont_manager_.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_acid_);

    Variable* acad_var {nullptr};
    if (dbcont_manager_.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_acad_))
        acad_var = &dbcont_manager_.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_acad_);

    Variable& m3a_var = dbcont_manager_.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_);

    // first row
    // 1x1
    {
        string main_id("?");

        if (config_.use_utn_as_id_ && buffer->has<unsigned int>(utn_var.name())
                && !buffer->get<unsigned int>(utn_var.name()).isNull(buffer_index))
        {
            main_id = to_string(buffer->get<unsigned int>(utn_var.name()).get(buffer_index));
        }
        else
        {
            // identification hierarchy: ACID > ACAD > Mode 3/A.
            // an unspecific ACID (e.g. "00000000") is demoted below ACAD/Mode 3/A,
            // but still shown if no other identification is available.

            string usable_id, fallback_id;

            if (acid_var && buffer->has<string>(acid_var->name())
                    && !buffer->get<string>(acid_var->name()).isNull(buffer_index))
                classifyIdentification(buffer->get<string>(acid_var->name()).get(buffer_index),
                                       usable_id, fallback_id);

            if (usable_id.size())
                main_id = usable_id;
            else if (acad_var && buffer->has<unsigned int>(acad_var->name()) &&
                     !buffer->get<unsigned int>(acad_var->name()).isNull(buffer_index))
                main_id = acad_var->getAsSpecialRepresentationString(
                            buffer->get<unsigned int>(acad_var->name()).get(buffer_index));
            else if (buffer->has<unsigned int>(m3a_var.name()) &&
                     !buffer->get<unsigned int>(m3a_var.name()).isNull(buffer_index))
                main_id = getMode3AText(dbcontent_name, buffer_index, buffer);
            else if (fallback_id.size()) // nothing else available, use the unspecific ACID
                main_id = fallback_id;
        }

        tmp.push_back(main_id);
    }

    if (lod == 1)
        return tmp;

    // 1,2
    string acid;

    if (acid_var && buffer->has<string>(acid_var->name())
            && !buffer->get<string>(acid_var->name()).isNull(buffer_index))
        acid = buffer->get<string>(acid_var->name()).get(buffer_index);

    acid.erase(std::remove(acid.begin(), acid.end(), ' '), acid.end());
    tmp.push_back(acid);

    if (lod == 3)
    {
        // 1,3
        tmp.push_back(getVariableValue(dbcontent_name, 0*3+2, buffer, buffer_index));
    }

    // row 2

    // 2,1
    string m3a;

    if (buffer->has<unsigned int>(m3a_var.name()) &&
            !buffer->get<unsigned int>(m3a_var.name()).isNull(buffer_index))
        m3a = getMode3AText(dbcontent_name, buffer_index, buffer);

    tmp.push_back(m3a);

    // 2,2

    tmp.push_back(getModeCText(dbcontent_name, buffer_index, buffer));

    if (lod == 2)
        return tmp;

    // 2,3
    tmp.push_back(getVariableValue(dbcontent_name, 1*3+2, buffer, buffer_index));

    // row 3

    // 3,1
    tmp.push_back(getVariableValue(dbcontent_name, 2*3+0, buffer, buffer_index));

    // 3,2
    tmp.push_back(getVariableValue(dbcontent_name, 2*3+1, buffer, buffer_index));

    // 3,3

    tmp.push_back(getVariableValue(dbcontent_name, 2*3+2, buffer, buffer_index));

    return tmp;
}

std::vector<std::string> LabelGenerator::getFullTexts(const std::string& dbcontent_name,
                                                      const std::shared_ptr<Buffer>& buffer,
                                                      unsigned int buffer_index)
{
    std::vector<std::string> tmp;

    if (!buffer)
    {
        logerr << "no buffer given for dbcontent '" << dbcontent_name << "'";
        return tmp;
    }

    DBContent& db_content = dbcont_manager_.dbContent(dbcontent_name);

    if (buffer_index >= buffer->size())
    {
        logerr << "buffer index of dbcontent '" << dbcontent_name << "' out of range";
        return tmp;
    }

    using namespace dbContent;

    string value_str;
    string property_name;
    bool null;
    bool use_presentation;

    tmp.push_back("Variable");
    tmp.push_back("Value");
    tmp.push_back("Unit");

    // do common label parts
    set<string> used_varnames;
    {
        using namespace dbContent;

        Variable& utn_var = dbcont_manager_.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_utn_);

        Variable* acid_var {nullptr};
        if (dbcont_manager_.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_acid_))
            acid_var = &dbcont_manager_.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_acid_);

        Variable* acad_var {nullptr};
        if (dbcont_manager_.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_acad_))
            acad_var = &dbcont_manager_.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_acad_);

        Variable& m3a_var = dbcont_manager_.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_);

        string varname, value, unit;

        // first row
        // 1x1
        {
            varname = "Best Available Identification";
            value = "?";
            unit = "";

            if (config_.use_utn_as_id_ && buffer->has<unsigned int>(utn_var.name())
                    && !buffer->get<unsigned int>(utn_var.name()).isNull(buffer_index))
            {
                value = to_string(buffer->get<unsigned int>(utn_var.name()).get(buffer_index));
                value += " ("+utn_var.name()+")";
            }
            else
            {
                // identification hierarchy: ACID > flight-plan ACID > ACAD > Mode 3/A.
                // an unspecific ACID (e.g. "00000000") is demoted below ACAD/Mode 3/A,
                // but still shown if no other identification is available.

                string usable_id, fallback_id, usable_name, fallback_name;

                auto apply = [&](const string& raw, const string& vname) {
                    bool had_usable   = !usable_id.empty();
                    bool had_fallback = !fallback_id.empty();
                    classifyIdentification(raw, usable_id, fallback_id);
                    if (!had_usable && !usable_id.empty())     usable_name   = vname;
                    if (!had_fallback && !fallback_id.empty()) fallback_name = vname;
                };

                if (acid_var && buffer->has<string>(acid_var->name())
                        && !buffer->get<string>(acid_var->name()).isNull(buffer_index))
                    apply(buffer->get<string>(acid_var->name()).get(buffer_index), acid_var->name());

                if (usable_id.size())
                    value = usable_id + " ("+usable_name+")";
                else if (acad_var && buffer->has<unsigned int>(acad_var->name()) &&
                         !buffer->get<unsigned int>(acad_var->name()).isNull(buffer_index))
                    value = acad_var->getAsSpecialRepresentationString(
                                buffer->get<unsigned int>(acad_var->name()).get(buffer_index))
                            + " ("+acad_var->name()+")";
                else if (buffer->has<unsigned int>(m3a_var.name()) &&
                         !buffer->get<unsigned int>(m3a_var.name()).isNull(buffer_index))
                    value = getMode3AText(dbcontent_name, buffer_index, buffer)
                            + " ("+m3a_var.name()+")";
                else if (fallback_id.size()) // nothing else available, use the unspecific ACID
                    value = fallback_id + " ("+fallback_name+")";
            }

            tmp.push_back(varname);
            tmp.push_back(value);
            tmp.push_back(unit);
        }

        // 1,2
        varname = "";
        value = "";
        unit = "";

        if (acid_var)
        {
            varname = acid_var->name();

            if (buffer->has<string>(acid_var->name())
                    && !buffer->get<string>(acid_var->name()).isNull(buffer_index))
            {

                value = buffer->get<string>(acid_var->name()).get(buffer_index);
                value.erase(std::remove(value.begin(), value.end(), ' '), value.end());
            }

            used_varnames.insert(varname);
        }
        tmp.push_back(varname);
        tmp.push_back(value);
        tmp.push_back(unit);

        unsigned int index;

        // 1,3
        {
            index = 0*3+2;
            varname = getVariableName(dbcontent_name, index);
            tmp.push_back(varname);
            tmp.push_back(getVariableValue(dbcontent_name, index, buffer, buffer_index, true));
            tmp.push_back(getVariableUnit(dbcontent_name, index));

            if (varname.size())
                used_varnames.insert(varname);
        }

        // row 2

        // 2,1
        varname = "";
        value = "";
        unit = "";

        varname = m3a_var.name();
        if (buffer->has<unsigned int>(m3a_var.name()) &&
                !buffer->get<unsigned int>(m3a_var.name()).isNull(buffer_index))
        {
            value = getMode3AText(dbcontent_name, buffer_index, buffer);
        }
        tmp.push_back(varname);
        tmp.push_back(value);
        tmp.push_back(unit);

        if (varname.size())
            used_varnames.insert(varname);

        // 2,2
        Variable& mc_var = dbcont_manager_.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_);
        varname = mc_var.name();
        value = "";
        unit = mc_var.dimensionUnitStr();

        if (buffer->has<float>(mc_var.name()) &&
                !buffer->get<float>(mc_var.name()).isNull(buffer_index))
        {
            value = getModeCText(dbcontent_name, buffer_index, buffer);
        }
        tmp.push_back(varname);
        tmp.push_back(value);
        tmp.push_back(unit);

        if (varname.size())
            used_varnames.insert(varname);


        // 2,3
        index = 1*3+2;
        varname = getVariableName(dbcontent_name, index);
        tmp.push_back(varname);
        tmp.push_back(getVariableValue(dbcontent_name, index, buffer, buffer_index, true));
        tmp.push_back(getVariableUnit(dbcontent_name, index));

        if (varname.size())
            used_varnames.insert(varname);

        // row 3

        // 3,1
        index = 2*3+0;
        varname = getVariableName(dbcontent_name, index);

        tmp.push_back(varname);
        tmp.push_back(getVariableValue(dbcontent_name, index, buffer, buffer_index, true));
        tmp.push_back(getVariableUnit(dbcontent_name, index));

        if (varname.size())
            used_varnames.insert(varname);

        // 3,2
        index = 2*3+1;
        varname = getVariableName(dbcontent_name, index);

        tmp.push_back(varname);
        tmp.push_back(getVariableValue(dbcontent_name, index, buffer, buffer_index, true));
        tmp.push_back(getVariableUnit(dbcontent_name, index));

        if (varname.size())
            used_varnames.insert(varname);

        // 3,3
        index = 2*3+2;
        varname = getVariableName(dbcontent_name, index);

        tmp.push_back(varname);
        tmp.push_back(getVariableValue(dbcontent_name, index, buffer, buffer_index, true));
        tmp.push_back(getVariableUnit(dbcontent_name, index));

        if (varname.size())
            used_varnames.insert(varname);
    }

    // do rest, first an empty line
    tmp.push_back("");
    tmp.push_back("");
    tmp.push_back("");

    for (auto& var_it : db_content.variables())
    {
        if (used_varnames.count(var_it.first)) // skip if used
            continue;

        if (buffer->hasProperty(*var_it.second.get()))
        {
            property_name = var_it.first;
            PropertyDataType data_type = var_it.second->dataType();

            use_presentation = var_it.second->representation() != Variable::Representation::STANDARD;

            value_str = NULL_STRING;

            if (data_type == PropertyDataType::BOOL)
            {
                traced_assert(buffer->has<bool>(property_name));
                null = buffer->get<bool>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (use_presentation)
                        value_str = var_it.second->getRepresentationStringFromValue(
                                    buffer->get<bool>(property_name).getAsString(buffer_index));
                    else
                        value_str = buffer->get<bool>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::CHAR)
            {
                traced_assert(buffer->has<char>(property_name));
                null = buffer->get<char>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (use_presentation)
                        value_str = var_it.second->getRepresentationStringFromValue(
                                    buffer->get<char>(property_name).getAsString(buffer_index));
                    else
                        value_str = buffer->get<char>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::UCHAR)
            {
                traced_assert(buffer->has<unsigned char>(property_name));
                null = buffer->get<unsigned char>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (use_presentation)
                        value_str = var_it.second->getRepresentationStringFromValue(
                                    buffer->get<unsigned char>(property_name).getAsString(buffer_index));
                    else
                        value_str =
                                buffer->get<unsigned char>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::INT)
            {
                traced_assert(buffer->has<int>(property_name));
                null = buffer->get<int>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (use_presentation)
                        value_str = var_it.second->getRepresentationStringFromValue(
                                    buffer->get<int>(property_name).getAsString(buffer_index));
                    else
                        value_str = buffer->get<int>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::UINT)
            {
                traced_assert(buffer->has<unsigned int>(property_name));
                null = buffer->get<unsigned int>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (use_presentation)
                        value_str = var_it.second->getRepresentationStringFromValue(
                                    buffer->get<unsigned int>(property_name).getAsString(buffer_index));
                    else
                        value_str =
                                buffer->get<unsigned int>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::LONGINT)
            {
                traced_assert(buffer->has<long int>(property_name));
                null = buffer->get<long int>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (use_presentation)
                        value_str = var_it.second->getRepresentationStringFromValue(
                                    buffer->get<long int>(property_name).getAsString(buffer_index));
                    else
                        value_str = buffer->get<long int>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::ULONGINT)
            {
                traced_assert(buffer->has<unsigned long int>(property_name));
                null = buffer->get<unsigned long int>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (use_presentation)
                        value_str = var_it.second->getRepresentationStringFromValue(
                                    buffer->get<unsigned long int>(property_name)
                                    .getAsString(buffer_index));
                    else
                        value_str =
                                buffer->get<unsigned long int>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::FLOAT)
            {
                traced_assert(buffer->has<float>(property_name));
                null = buffer->get<float>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (use_presentation)
                        value_str = var_it.second->getRepresentationStringFromValue(
                                    buffer->get<float>(property_name).getAsString(buffer_index));
                    else
                        value_str = buffer->get<float>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::DOUBLE)
            {
                traced_assert(buffer->has<double>(property_name));
                null = buffer->get<double>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (use_presentation)
                        value_str = var_it.second->getRepresentationStringFromValue(
                                    buffer->get<double>(property_name).getAsString(buffer_index));
                    else
                        value_str = buffer->get<double>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::STRING)
            {
                traced_assert(buffer->has<std::string>(property_name));
                null = buffer->get<std::string>(property_name).isNull(buffer_index);
                if (!null)
                {
                    // fixed length strings are padded with spaces => trimmed as elsewhere
                    value_str = String::trim(buffer->get<std::string>(property_name).getAsString(buffer_index));
                }
            }
            else if (data_type == PropertyDataType::JSON)
            {
                traced_assert(buffer->has<nlohmann::json>(property_name));
                null = buffer->get<nlohmann::json>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (use_presentation)
                    {
                        unsigned int row_ds_id = 0;
                        if (dbcont_manager_.metaCanGetVariable(
                                dbcontent_name, dbcontent_vars::meta_var_ds_id_))
                        {
                            auto& ds_var = dbcont_manager_.metaGetVariable(
                                dbcontent_name, dbcontent_vars::meta_var_ds_id_);
                            if (buffer->hasAnyPropertyNamed(ds_var.name()))
                            {
                                auto& ds_vec = buffer->get<unsigned int>(ds_var.name());
                                if (!ds_vec.isNull(buffer_index))
                                    row_ds_id = ds_vec.get(buffer_index);
                            }
                        }
                        value_str = var_it.second->getAsJSONRepresentationString(
                            buffer->get<nlohmann::json>(property_name).getRef(buffer_index),
                            &dbcont_manager_.compass().dbContextManager(),
                            row_ds_id);
                    }
                    else
                        value_str = buffer->get<nlohmann::json>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::TIMESTAMP)
            {
                traced_assert(buffer->has<boost::posix_time::ptime>(property_name));
                null = buffer->get<boost::posix_time::ptime>(property_name).isNull(buffer_index);
                if (!null)
                {
                    value_str = buffer->get<boost::posix_time::ptime>(property_name).getAsString(buffer_index);
                }
            }
            else
                throw std::domain_error("LabelGenerator: getFullTexts: unknown property data type");

            tmp.push_back(property_name);
            tmp.push_back(value_str);
            //tmp.push_back(var_it->description());
            tmp.push_back(var_it.second->dimensionUnitStr());
        }
    }

    traced_assert(tmp.size() % 3 == 0);

    return tmp;
}

bool LabelGenerator::autoLabel() const
{
    return config_.auto_label_;
}

void LabelGenerator::autoLabel(bool auto_label)
{
    if (config_.auto_label_ != auto_label)
        emit labelConfigChanged();

    config_.auto_label_ = auto_label;

    emit labelOptionsChangedSignal();
}

// adjusts the runtime auto lod only, the config (selected lod) stays untouched => no config change signal
void LabelGenerator::autoAdustCurrentLOD(unsigned int num_labels_on_screen)
{
    float old_lod = auto_lod_value_;

    if (num_labels_on_screen < 25)
        auto_lod_value_ = (2*old_lod + 3) / 3;
    else if (num_labels_on_screen < 50)
        auto_lod_value_ = (2*old_lod + 2) / 3;
    else if (num_labels_on_screen < 75)
        auto_lod_value_ = (2*old_lod + 1) / 3;
    else
        auto_lod_value_ = 1;

    // failsafe
    if (auto_lod_value_ < 1)
        auto_lod_value_ = 1;
    else if (auto_lod_value_ > 3)
        auto_lod_value_ = 3;

    logdbg << "num labels on screen "
           << num_labels_on_screen << " old " << (unsigned int) old_lod
           << " current " << round(auto_lod_value_) << " float " << auto_lod_value_;
}

// returns the auto adjusted lod if auto lod is active, the selected lod otherwise
unsigned int LabelGenerator::currentLOD() const
{
    return config_.auto_lod_ ? (unsigned int)std::round(auto_lod_value_) : config_.current_lod_;
}

void LabelGenerator::currentLOD(unsigned int current_lod)
{
    bool changed = config_.current_lod_ != current_lod;

    config_.current_lod_ = current_lod;

    if (changed)
        emit labelConfigChanged();

    emit labelOptionsChangedSignal();
}

// one combined change (auto lod off + selected lod) => listeners update once
void LabelGenerator::fixedLOD(unsigned int lod)
{
    bool changed = config_.auto_lod_ || config_.current_lod_ != lod;

    config_.auto_lod_    = false;
    config_.current_lod_ = lod;

    if (changed)
        emit labelConfigChanged();

    emit labelOptionsChangedSignal();
}

bool LabelGenerator::autoLOD() const
{
    return config_.auto_lod_;
}

void LabelGenerator::autoLOD(bool auto_lod)
{
    bool changed = config_.auto_lod_ != auto_lod;

    //start auto adjustment from the selected lod
    if (auto_lod && !config_.auto_lod_)
        auto_lod_value_ = (float)config_.current_lod_;

    config_.auto_lod_ = auto_lod;

    if (changed)
        emit labelConfigChanged();

    emit labelOptionsChangedSignal();
}

void LabelGenerator::toggleUseUTN()
{
    config_.use_utn_as_id_ = !config_.use_utn_as_id_;

    emit labelConfigChanged();
    emit labelContentsChangedSignal();
}

bool LabelGenerator::useUTN()
{
    return config_.use_utn_as_id_;
}

void LabelGenerator::addLabelDSID(unsigned int ds_id)
{
    config_.label_ds_ids_[to_string(ds_id)] = true;
    invalidateDSLabelSettings();

    emit labelConfigChanged();
    emit labelOptionsChangedSignal();
}

void LabelGenerator::removeLabelDSID(unsigned int ds_id)
{
    config_.label_ds_ids_[to_string(ds_id)] = false;
    invalidateDSLabelSettings();

    emit labelConfigChanged();
    emit labelOptionsChangedSignal();
}

void LabelGenerator::labelAllDSIDs()
{
    auto& ctx_man = dbcont_manager_.compass().dbContextManager();

    if (!ctx_man.hasActiveContext())
        return;

    for (const auto& [ds_id, ds] : ctx_man.activeContext().dataSources())
        config_.label_ds_ids_[to_string(ds.id())] = true;

    invalidateDSLabelSettings();

    emit labelConfigChanged();
    emit labelOptionsChangedSignal();
}

void LabelGenerator::labelNoDSIDs()
{
    config_.label_ds_ids_.clear();
    invalidateDSLabelSettings();

    emit labelConfigChanged();
    emit labelOptionsChangedSignal();
}

//const std::map<unsigned int, bool> LabelGenerator::labelDSIDs() const
//{
//    return label_ds_ids_.get<std::map<unsigned int, bool>>();
//}

bool LabelGenerator::anyDSIDLabelWanted()
{
    for (auto& ds_it : config_.label_ds_ids_.get<std::map<string, bool>>())
    {
        if (ds_it.second)
            return true;
    }

    return false;
}

bool LabelGenerator::labelWanted(unsigned int ds_id)
{
    auto& settings = ds_label_settings_[ds_id];

    if (!settings.wanted.has_value())
    {
        string key = to_string(ds_id);

        settings.wanted = config_.label_ds_ids_.contains(key) ? config_.label_ds_ids_.at(key).get<bool>() : false;
    }

    return settings.wanted.value();
}

// single row check, for many rows of a buffer use filterContext() once and labelWanted(context, index)
bool LabelGenerator::labelWanted(std::shared_ptr<Buffer> buffer, unsigned int index)
{
    return labelWanted(filterContext(buffer), index);
}

// resolves the columns needed by the label filters once per buffer (name based lookups are expensive per row)
LabelGenerator::FilterContext LabelGenerator::filterContext(const std::shared_ptr<Buffer>& buffer)
{
    FilterContext context;

    string dbcont_name = buffer->dbContentName();

    // line
    traced_assert(dbcont_manager_.metaCanGetVariable(dbcont_name, dbcontent_vars::meta_var_line_id_));
    dbContent::Variable& line_var = dbcont_manager_.metaGetVariable(
                dbcont_name, dbcontent_vars::meta_var_line_id_);
    traced_assert(dbcont_manager_.metaCanGetVariable(dbcont_name, dbcontent_vars::meta_var_ds_id_));
    dbContent::Variable& ds_id_var = dbcont_manager_.metaGetVariable(
                dbcont_name, dbcontent_vars::meta_var_ds_id_);

    traced_assert(buffer->has<unsigned int> (line_var.name()));
    traced_assert(buffer->has<unsigned int> (ds_id_var.name()));

    context.line_vec  = &buffer->get<unsigned int> (line_var.name());
    context.ds_id_vec = &buffer->get<unsigned int> (ds_id_var.name());

    const bool primary_only = config_.filter_primary_only_active_;

    // mode 3/a: needed by the mode 3/a filter, optional for primary only
    if (config_.filter_mode3a_active_ || primary_only)
    {
        if (dbcont_manager_.metaCanGetVariable(dbcont_name, dbcontent_vars::meta_var_m3a_))
        {
            dbContent::Variable& var = dbcont_manager_.metaGetVariable(dbcont_name, dbcontent_vars::meta_var_m3a_);
            traced_assert(buffer->has<unsigned int> (var.name()));
            context.m3a_vec = &buffer->get<unsigned int> (var.name());
        }
        else
        {
            context.m3a_missing = config_.filter_mode3a_active_;
        }
    }

    // mode c: needed by the mode c filter, optional for primary only
    const bool modec_filter = config_.filter_modec_min_active_ || config_.filter_modec_max_active_;

    if (modec_filter || primary_only)
    {
        if (dbcont_manager_.metaCanGetVariable(dbcont_name, dbcontent_vars::meta_var_mc_))
        {
            dbContent::Variable& var = dbcont_manager_.metaGetVariable(dbcont_name, dbcontent_vars::meta_var_mc_);
            traced_assert(buffer->has<float> (var.name()));
            context.mc_vec = &buffer->get<float> (var.name());
        }
        else
        {
            context.mc_missing = modec_filter;
        }
    }

    // acid: needed by the ti filter, optional for primary only
    if (config_.filter_ti_active_ || primary_only)
    {
        if (dbcont_manager_.metaCanGetVariable(dbcont_name, dbcontent_vars::meta_var_acid_))
        {
            dbContent::Variable& var = dbcont_manager_.metaGetVariable(dbcont_name, dbcontent_vars::meta_var_acid_);
            traced_assert(buffer->has<string> (var.name()));
            context.acid_vec = &buffer->get<string> (var.name());
        }
        else
        {
            context.acid_missing = config_.filter_ti_active_;
        }
    }

    // flight plan callsign: ti filter, only in cat062
    if (config_.filter_ti_active_ && !context.acid_missing && dbcont_name == "CAT062")
    {
        traced_assert(dbcont_manager_.canGetVariable(dbcont_name, dbcontent_vars::var_cat062_callsign_fpl_));

        dbContent::Variable& var = dbcont_manager_.getVariable(dbcont_name, dbcontent_vars::var_cat062_callsign_fpl_);

        traced_assert(buffer->has<string> (var.name()));
        context.cs_fpl_vec = &buffer->get<string> (var.name());
    }

    // acad: needed by the ta filter, optional for primary only
    if (config_.filter_ta_active_ || primary_only)
    {
        if (dbcont_manager_.metaCanGetVariable(dbcont_name, dbcontent_vars::meta_var_acad_))
        {
            dbContent::Variable& var = dbcont_manager_.metaGetVariable(dbcont_name, dbcontent_vars::meta_var_acad_);
            traced_assert(buffer->has<unsigned int> (var.name()));
            context.acad_vec = &buffer->get<unsigned int> (var.name());
        }
        else
        {
            context.acad_missing = config_.filter_ta_active_;
        }
    }

    // detection type: optional for primary only
    if (primary_only && dbcont_manager_.metaCanGetVariable(dbcont_name, dbcontent_vars::meta_var_detection_type_))
    {
        dbContent::Variable& var = dbcont_manager_.metaGetVariable(dbcont_name, dbcontent_vars::meta_var_detection_type_);
        traced_assert(buffer->has<unsigned char> (var.name()));
        context.type_vec = &buffer->get<unsigned char> (var.name());
    }

    return context;
}

// checks the label filters for a row of the buffer the context was created for
bool LabelGenerator::labelWanted(const FilterContext& context, unsigned int index)
{
    // check line
    {
        traced_assert(!context.line_vec->isNull(index));
        traced_assert(!context.ds_id_vec->isNull(index));

        unsigned int ds_id = context.ds_id_vec->get(index);

        if (!labelWanted(ds_id))
            return false;

        if (labelLine(ds_id) != context.line_vec->get(index))
            return false;
    }

    if (config_.filter_mode3a_active_)
    {
        if (context.m3a_missing)
            return false;

        const NullableVector<unsigned int>& data_vec = *context.m3a_vec;

        if (data_vec.isNull(index))
        {
            if (!filter_m3a_null_wanted_)
                return false; // null and not wanted
        }
        else if (!filter_m3a_values_set_.count(data_vec.get(index)))
            return false; // set and not in values
    }

    if (config_.filter_modec_min_active_ || config_.filter_modec_max_active_)
    {
        if (context.mc_missing)
            return false;

        const NullableVector<float>& data_vec = *context.mc_vec;

        if (data_vec.isNull(index))
        {
            if (!config_.filter_modec_null_wanted_)
                return false; // null and not wanted
        }
        else
        {
            if (config_.filter_modec_min_active_ && data_vec.get(index)/100.0 < config_.filter_modec_min_value_)
                return false;

            if (config_.filter_modec_max_active_ && data_vec.get(index)/100.0 > config_.filter_modec_max_value_)
                return false;
        }
    }

    if (config_.filter_ti_active_)
    {
        if (context.acid_missing)
            return false;

        const NullableVector<string>& acid_vec   = *context.acid_vec;
        const NullableVector<string>* cs_fpl_vec = context.cs_fpl_vec; // only set in cat062

        if (acid_vec.isNull(index)
                && (cs_fpl_vec != nullptr ? cs_fpl_vec->isNull(index) : true)) // null or not found
        {
            return filter_ti_null_wanted_;
        }
        else
        {
            bool found = false;

            if (!acid_vec.isNull(index))
            {
                for (auto& val_it : filter_ti_values_set_)
                {
                    if (acid_vec.get(index).find(val_it) != std::string::npos)
                    {
                        found = true;
                        break;
                    }
                }
            }

            if (cs_fpl_vec && !cs_fpl_vec->isNull(index))
            {
                for (auto& val_it : filter_ti_values_set_)
                {
                    if (cs_fpl_vec->get(index).find(val_it) != std::string::npos)
                    {
                        found = true;
                        break;
                    }
                }
            }

            if (!found)
                return false;
        }
    }

    if (config_.filter_ta_active_)
    {
        if (context.acad_missing)
            return false;

        const NullableVector<unsigned int>& data_vec = *context.acad_vec;

        if (data_vec.isNull(index))
        {
            if (!filter_ta_null_wanted_)
                return false; // null and not wanted
        }
        else if (!filter_ta_values_set_.count(data_vec.get(index)))
            return false; // set and not in values
    }

    if (config_.filter_primary_only_active_)
    {
        static const std::set<unsigned char> psr_detection {1,3,6,7};

        if (context.m3a_vec && !context.m3a_vec->isNull(index))
            return false;
        else if (context.mc_vec && !context.mc_vec->isNull(index))
            return false;
        else if (context.acad_vec && !context.acad_vec->isNull(index))
            return false;
        else if (context.acid_vec && !context.acid_vec->isNull(index))
            return false;
        else if (context.type_vec && !context.type_vec->isNull(index) && !psr_detection.count(context.type_vec->get(index)))
            return false;
    }

    return true;
}

bool LabelGenerator::filterMode3aActive() const
{
    return config_.filter_mode3a_active_;
}

void LabelGenerator::filterMode3aActive(bool filter_active)
{
    if (config_.filter_mode3a_active_ != filter_active)
        emit labelConfigChanged();

    config_.filter_mode3a_active_ = filter_active;

    emit labelOptionsChangedSignal();
}

std::string LabelGenerator::filterMode3aValues() const
{
    return config_.filter_mode3a_values_;
}

void LabelGenerator::filterMode3aValues(const std::string &filter_values)
{
    if (config_.filter_mode3a_values_ != filter_values)
        emit labelConfigChanged();

    config_.filter_mode3a_values_ = filter_values;
    updateM3AValuesFromStr(config_.filter_mode3a_values_);

    emit labelOptionsChangedSignal();
}

bool LabelGenerator::filterTIActive() const
{
    return config_.filter_ti_active_;
}

void LabelGenerator::filterTIActive(bool filter_active)
{
    if (config_.filter_ti_active_ != filter_active)
        emit labelConfigChanged();

    config_.filter_ti_active_ = filter_active;

    emit labelOptionsChangedSignal();
}

std::string LabelGenerator::filterTIValues() const
{
    return config_.filter_ti_values_;
}

void LabelGenerator::filterTIValues(const std::string &filter_values)
{
    if (config_.filter_ti_values_ != filter_values)
        emit labelConfigChanged();

    config_.filter_ti_values_ = filter_values;
    updateTIValuesFromStr(config_.filter_ti_values_);

    emit labelOptionsChangedSignal();
}

bool LabelGenerator::filterTAActive() const
{
    return config_.filter_ta_active_;
}

void LabelGenerator::filterTAActive(bool filter_active)
{
    if (config_.filter_ta_active_ != filter_active)
        emit labelConfigChanged();

    config_.filter_ta_active_ = filter_active;

    emit labelOptionsChangedSignal();
}

std::string LabelGenerator::filterTAValues() const
{
    return config_.filter_ta_values_;
}

void LabelGenerator::filterTAValues(const std::string &filter_values)
{
    if (config_.filter_ta_values_ != filter_values)
        emit labelConfigChanged();

    config_.filter_ta_values_ = filter_values;
    updateTAValuesFromStr(config_.filter_ta_values_);

    emit labelOptionsChangedSignal();
}

bool LabelGenerator::filterModecMinActive() const
{
    return config_.filter_modec_min_active_;
}

void LabelGenerator::filterModecMinActive(bool value)
{
    if (config_.filter_modec_min_active_ != value)
        emit labelConfigChanged();

    config_.filter_modec_min_active_ = value;

    emit labelOptionsChangedSignal();
}

float LabelGenerator::filterModecMinValue() const
{
    return config_.filter_modec_min_value_;
}

void LabelGenerator::filterModecMinValue(float value)
{
    if (config_.filter_modec_min_value_ != value)
        emit labelConfigChanged();

    config_.filter_modec_min_value_ = value;

    emit labelOptionsChangedSignal();
}

bool LabelGenerator::filterModecMaxActive() const
{
    return config_.filter_modec_max_active_;
}

void LabelGenerator::filterModecMaxActive(bool value)
{
    if (config_.filter_modec_max_active_ != value)
        emit labelConfigChanged();

    config_.filter_modec_max_active_ = value;

    emit labelOptionsChangedSignal();
}

float LabelGenerator::filterModecMaxValue() const
{
    return config_.filter_modec_max_value_;
}

void LabelGenerator::filterModecMaxValue(float value)
{
    if (config_.filter_modec_max_value_ != value)
        emit labelConfigChanged();

    config_.filter_modec_max_value_ = value;

    emit labelOptionsChangedSignal();
}

bool LabelGenerator::filterModecNullWanted() const
{
    return config_.filter_modec_null_wanted_;
}

void LabelGenerator::filterModecNullWanted(bool value)
{
    if (config_.filter_modec_null_wanted_ != value)
        emit labelConfigChanged();

    config_.filter_modec_null_wanted_ = value;

    emit labelOptionsChangedSignal();
}

void LabelGenerator::checkLabelConfig()
{
    string key;

    for (auto& dbcont_it : dbcont_manager_)
    {
        if (!config_.label_config_.contains(dbcont_it.first)) // create
        {
            config_.label_config_[dbcont_it.first] = json::object();

            json& dbcont_def = config_.label_config_.at(dbcont_it.first);

            for (unsigned int row=0; row < 3; row++)
            {
                for (unsigned int col=0; col < 3; col++)
                {
                    key = to_string(row*3 + col);

                    if (row == 0 && col == 0)
                        dbcont_def[key] = "Best available Identification";
                    else if (row == 0 && col == 1
                             && dbcont_manager_.metaCanGetVariable(dbcont_it.first, dbcontent_vars::meta_var_acid_))
                        dbcont_def[key] =
                                dbcont_manager_.metaGetVariable(dbcont_it.first, dbcontent_vars::meta_var_acid_).name();
                    else if (row == 0 && col == 2
                             && dbcont_manager_.metaCanGetVariable(dbcont_it.first, dbcontent_vars::meta_var_acad_))
                        dbcont_def[key] =
                                dbcont_manager_.metaGetVariable(dbcont_it.first, dbcontent_vars::meta_var_acad_).name();
                    else if (row == 1 && col == 0
                             && dbcont_manager_.metaCanGetVariable(dbcont_it.first, dbcontent_vars::meta_var_m3a_))
                        dbcont_def[key] =
                                dbcont_manager_.metaGetVariable(dbcont_it.first, dbcontent_vars::meta_var_m3a_).name();
                    else if (row == 1 && col == 1
                             && dbcont_manager_.metaCanGetVariable(dbcont_it.first, dbcontent_vars::meta_var_mc_))
                        dbcont_def[key] =
                                dbcont_manager_.metaGetVariable(dbcont_it.first, dbcontent_vars::meta_var_mc_).name();
                    else if (row == 1 && col == 2
                             && dbcont_manager_.metaCanGetVariable(dbcont_it.first, dbcontent_vars::meta_var_ds_id_))
                        dbcont_def[key] =
                                dbcont_manager_.metaGetVariable(dbcont_it.first, dbcontent_vars::meta_var_ds_id_).name();
                    else if (row == 2 && col == 0
                             && dbcont_manager_.metaCanGetVariable(dbcont_it.first, dbcontent_vars::meta_var_ground_speed_))
                        dbcont_def[key] =
                                dbcont_manager_.metaGetVariable(dbcont_it.first, dbcontent_vars::meta_var_ground_speed_).name();
                    else if (row == 2 && col == 2
                             && dbcont_manager_.metaCanGetVariable(dbcont_it.first, dbcontent_vars::meta_var_timestamp_))
                        dbcont_def[key] =
                                dbcont_manager_.metaGetVariable(dbcont_it.first, dbcontent_vars::meta_var_timestamp_).name();
                }
            }
        }
    }
}

LabelDirection LabelGenerator::labelDirection (unsigned int ds_id)
{
    auto& settings = ds_label_settings_[ds_id];

    if (settings.direction.has_value())
        return settings.direction.value();

    string key = to_string(ds_id);

    unsigned int direction;

    if (config_.label_directions_.contains(key))
    {
        direction = config_.label_directions_.at(key).get<unsigned int>();
        traced_assert(direction <= 3);
    }
    else
    {
        direction = Number::randomNumber(0, 3.99);
        traced_assert(direction <= 3);
        config_.label_directions_[key] = direction;
    }

    settings.direction = LabelDirection(direction);

    return settings.direction.value();
}

float LabelGenerator::labelDirectionAngle (unsigned int ds_id)
{
    LabelDirection direction = labelDirection(ds_id);

    return LabelGenerator::labelDirectionAngle(direction);
}

float LabelGenerator::labelDirectionAngle(LabelDirection direction)
{
    if (direction == LEFT_UP)
        return DEG2RAD * 135.0;
    else if (direction == RIGHT_UP)
        return DEG2RAD * 45.0;
    else if (direction == LEFT_DOWN)
        return DEG2RAD * 225.0;
    else // RIGHT_DOWN
        return DEG2RAD * 315.0;

    return 0.0f;
}

void LabelGenerator::labelDirection (unsigned int ds_id, LabelDirection direction)
{
    string key = to_string(ds_id);

    if (config_.label_directions_.contains(key) && config_.label_directions_.at(key) == direction)
        return;

    config_.label_directions_[key] = direction;
    invalidateDSLabelSettings();

    emit labelConfigChanged();
    emit labelOptionsChangedSignal(); // label positions depend on direction
}

void LabelGenerator::editLabelContents(const std::string& dbcontent_name)
{
    // created on demand and modal (no second one can be opened meanwhile), applied on "Done"
    LabelContentDialog dialog(dbcontent_name, *this);

    if (dialog.exec() != QDialog::Accepted)
        return;

    auto cfg_new = dialog.labelConfig();

    bool changed = config_.label_config_ != cfg_new;

    config_.label_config_ = cfg_new;

    if (changed)
    {
        emit labelConfigChanged();
        emit labelContentsChangedSignal(); // emit after assignment, receivers regenerate texts
    }
}

unsigned int LabelGenerator::labelLine (unsigned int ds_id) // returns 0...3
{
    auto& settings = ds_label_settings_[ds_id];

    if (settings.line.has_value())
        return settings.line.value();

    string key = to_string(ds_id);

    unsigned int line;

    if (config_.label_lines_.contains(key))
    {
        line = config_.label_lines_.at(key).get<unsigned int>();
        traced_assert(line <= 3);
    }
    else
    {
        auto& ctx_man = dbcont_manager_.compass().dbContextManager();
        traced_assert(ctx_man.hasDataSource(ds_id));

        line = 0;

        // find first line with loaded data
        auto lines_map = ctx_man.numInsertedLinesMap(ds_id);
        if (!lines_map.empty())
            line = lines_map.begin()->first;

        config_.label_lines_[key] = line;
    }

    settings.line = line;

    return line;
}

void LabelGenerator::labelLine (unsigned int ds_id, unsigned int line)
{
    if (!setLabelLine(ds_id, line))
        return;

    emit labelConfigChanged();
    emit labelOptionsChangedSignal(); // labeled target reports depend on line
}

// sets the label line without emitting any signals, returns if changed
bool LabelGenerator::setLabelLine (unsigned int ds_id, unsigned int line)
{
    traced_assert(line <= 3);
    string key = to_string(ds_id);

    if (config_.label_lines_.contains(key) && config_.label_lines_.at(key) == line)
        return false;

    config_.label_lines_[key] = line;
    invalidateDSLabelSettings();

    return true;
}

// clears the cached per data source settings, they are re-read from the config on next access
void LabelGenerator::invalidateDSLabelSettings()
{
    ds_label_settings_.clear();
    ++config_generation_;
}

// updates lines to be label according to available lines with loaded data
void LabelGenerator::updateAvailableLabelLines()
{
    logdbg;

    unsigned int ds_id;
    unsigned int line_id;

    auto& ctx_man = dbcont_manager_.compass().dbContextManager();

    bool something_changed {false};

    for (auto& line_it : config_.label_lines_.get<std::map<std::string, unsigned int>>())
    {
        ds_id = std::atoi(line_it.first.c_str());
        line_id = line_it.second;

        if (!ctx_man.hasDataSource(ds_id)) // check if existing in current db
            continue;

        auto lines_map = ctx_man.numInsertedLinesMap(ds_id);

        if (!lines_map.empty())
        {
            if (lines_map.count(line_id)) // check if current line id has data
                continue; // has data in current line
            else // set to first line with data
            {
                something_changed |= setLabelLine(ds_id, lines_map.begin()->first);
            }
        }
        else // set to first line as default
        {
            something_changed |= setLabelLine(ds_id, 0);
        }

        logdbg << "ds_id " << ds_id
               << " new line " << labelLine(ds_id);
    }

    if (something_changed)
    {
        logdbg << "emitting change";
        emit labelConfigChanged();
        emit labelLinesChangedSignal();
        emit labelOptionsChangedSignal(); // emit once for all changed lines
    }
}

nlohmann::json LabelGenerator::labelConfig() const
{
    return config_.label_config_;
}

dbContent::Variable* LabelGenerator::labelVariable(const std::string& dbcontent_name,
                                                   const std::string& var_name)
{
    if (!dbcont_manager_.existsDBContent(dbcontent_name))
        return nullptr;

    DBContent& db_content = dbcont_manager_.dbContent(dbcontent_name);

    if (db_content.hasVariable(var_name))
        return &db_content.variable(var_name);

    //a Report Variable is named "<report>: <display name>", unique across the reports, and
    //resolves for the host data contents of its Report Table only
    for (const auto& content : dbcont_manager_.reportContents())
    {
        if (!content->hasVariable(var_name))
            continue;

        auto& report_var = content->variable(var_name);

        if (report_var.existsIn(dbcontent_name))
            return &report_var.getFor(dbcontent_name);

        return nullptr;
    }

    return nullptr;
}

void LabelGenerator::addVariables (const std::string& dbcontent_name, dbContent::VariableSet& read_set)
{
    traced_assert(config_.label_config_.contains(dbcontent_name));

    json& dbcont_def = config_.label_config_.at(dbcontent_name);

    for (auto& var_it : dbcont_def.get<std::map<std::string, std::string>>())
    {
        if (var_it.second == "Best available Identification" || var_it.second == "")
            continue;

        Variable* var = labelVariable(dbcontent_name, var_it.second);

        if (!var)
        {
            //a Report Variable of another database is not available, the name stays
            logwrn << "unknown var '" << var_it.second << "' in " << dbcontent_name;
            continue;
        }

        if (!read_set.hasVariable(*var))
            read_set.add(*var);
    }

    DBContent& db_content = dbcont_manager_.dbContent(dbcontent_name);
    (void) db_content;

    // "Mode C Garbled"
    if (dbcont_manager_.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_g_))
    {
        Variable& var = dbcont_manager_.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_g_);
        if (!read_set.hasVariable(var))
            read_set.add(var);
    }
    // "Mode C Valid"
    if (dbcont_manager_.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_v_))
    {
        Variable& var = dbcont_manager_.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_v_);
        if (!read_set.hasVariable(var))
            read_set.add(var);
    }

    // "Mode 3/A Valid"
    if (dbcont_manager_.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_smoothed_))
    {
        Variable& var = dbcont_manager_.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_smoothed_);
        if (!read_set.hasVariable(var))
            read_set.add(var);
    }
    // "Mode 3/A Garbled"
    if (dbcont_manager_.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_g_))
    {
        Variable& var = dbcont_manager_.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_g_);
        if (!read_set.hasVariable(var))
            read_set.add(var);
    }

    // "Mode 3/A Smoothed"
    if (dbcont_manager_.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_v_))
    {
        Variable& var = dbcont_manager_.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_v_);
        if (!read_set.hasVariable(var))
            read_set.add(var);
    }

    // the CAT062 identification and Mode C are the selections "Aircraft Identification*" and
    // "Mode C Code", reached through the Meta Variables above
}

bool LabelGenerator::labelVariablesLoaded()
{
    // same buffers as used for label text generation
    std::map<std::string, std::shared_ptr<Buffer>> buffers = dbcont_manager_.compass().viewProvider().currentBuffers();

    for (auto& buf_it : buffers)
    {
        if (!dbcont_manager_.dbContent(buf_it.first).containsTargetReports()
                || !config_.label_config_.contains(buf_it.first))
            continue;

        VariableSet label_set;
        addVariables(buf_it.first, label_set);

        for (auto var : label_set.getSet())
        {
            if (!buf_it.second->hasAnyPropertyNamed(var->name()))
            {
                loginf << "label variable '" << var->name() << "' not loaded in " << buf_it.first;
                return false;
            }
        }
    }

    return true;
}

bool LabelGenerator::declutterLabels() const
{
    return config_.declutter_labels_;
}

void LabelGenerator::declutterLabels(bool declutter_labels)
{
    if (config_.declutter_labels_ != declutter_labels)
        emit labelConfigChanged();

    config_.declutter_labels_ = declutter_labels;

    // old workaround, also deleted manual labels. positions are reset by the options update below.
    //if (!config_.declutter_labels_)
    //    emit labelClearAllSignal(); // since since do not update otherwise - reason unknown

    emit labelOptionsChangedSignal(); // updates
}

/**
 * Adaptive decluttering of the constraint based method: labels in crowded areas stay at home, the rest is decluttered.
 */
bool LabelGenerator::declutterLabelsSmart() const
{
    return config_.declutter_labels_smart_;
}

/**
 */
void LabelGenerator::declutterLabelsSmart(bool declutter_labels_smart)
{
    if (config_.declutter_labels_smart_ == declutter_labels_smart)
        return;

    config_.declutter_labels_smart_ = declutter_labels_smart;

    emit labelConfigChanged();
    emit labelOptionsChangedSignal(); // updates
}

label_placement::Method LabelGenerator::labelPlacementMethod() const
{
    return label_placement_method_;
}

void LabelGenerator::labelPlacementMethod(label_placement::Method method)
{
    loginf << "value " << (int)method;

    if (label_placement_method_ == method)
        return;

    label_placement_method_ = method;

    emit labelConfigChanged();
    emit labelOptionsChangedSignal(); // updates
}

bool LabelGenerator::showDeclutteringInfoOnce() const
{
    return show_decluttering_info_once_;
}

void LabelGenerator::showDeclutteringInfoOnce(bool show_decluttering_info_once)
{
    show_decluttering_info_once_ = show_decluttering_info_once;
}

unsigned int LabelGenerator::maxDeclutterLabels() const
{
    // the constraint based method places more labels in the same time (measured: 500 labels ~13 ms on real data)
    return label_placement_method_ == label_placement::Method::ConstraintBased ? MaxDeclutterLabelsConstraintBased
                                                                               : MaxDeclutterLabels;
}

bool LabelGenerator::filterPrimaryOnlyActive() const
{
    return config_.filter_primary_only_active_;
}

void LabelGenerator::filterPrimaryOnlyActive(bool value)
{
    loginf << "value " << value;

    if (config_.filter_primary_only_active_ != value)
        emit labelConfigChanged();

    config_.filter_primary_only_active_ = value;

    emit labelOptionsChangedSignal();
}

float LabelGenerator::labelOpacity() const
{
    return config_.label_opacity_;
}

void LabelGenerator::labelOpacity(float label_opacity)
{
    if (config_.label_opacity_ == label_opacity)
        return;

    config_.label_opacity_ = label_opacity;

    emit labelConfigChanged();
    emit labelOpacityChangedSignal();
}

void LabelGenerator::updateFilterValuesFromStrings()
{
    updateM3AValuesFromStr(config_.filter_mode3a_values_);
    updateTIValuesFromStr(config_.filter_ti_values_);
    updateTAValuesFromStr(config_.filter_ta_values_);

    ++config_generation_; // config possibly set from outside (e.g. preset)
}

void LabelGenerator::setViewPointLabels(const std::map<unsigned long, unsigned int>& labels)
{
    loginf << labels.size() << " label(s)";

    view_point_labels_ = labels;
}

void LabelGenerator::clearViewPointLabels()
{
    if (view_point_labels_.empty())
        return;

    loginf << view_point_labels_.size() << " label(s)";

    view_point_labels_.clear();
}

float LabelGenerator::labelDistance() const
{
    return label_distance_;
}

void LabelGenerator::labelDistance(float label_distance)
{
    label_distance_ = label_distance;
}

bool LabelGenerator::updateM3AValuesFromStr(const std::string& values)
{
    set<unsigned int> values_tmp;
    vector<string> split_str = String::split(values, ',');
    
    bool ok = true;

    filter_m3a_null_wanted_ = false;

    for (auto& tmp_str : split_str)
    {
        if (String::trim(tmp_str) == "NULL" || String::trim(tmp_str) == "null")
        {
            filter_m3a_null_wanted_ = true;
            continue;
        }

        unsigned int utn_tmp = QString(tmp_str.c_str()).toInt(&ok, 8);

        if (!ok)
        {
            logerr << "value '" << tmp_str << "' not valid";
            break;
        }

        values_tmp.insert(utn_tmp);
    }

    if (!ok)
        return false;

    filter_m3a_values_set_ = values_tmp;

    return true;
}

bool LabelGenerator::updateTIValuesFromStr(const std::string& values)
{
    set<string> values_tmp;
    vector<string> split_str = String::split(values, ',');

    filter_ti_null_wanted_ = false;

    for (auto& tmp_str : split_str)
    {
        if (String::trim(tmp_str) == "NULL" || String::trim(tmp_str) == "null")
        {
            filter_ti_null_wanted_ = true;
            continue;
        }

        values_tmp.insert(boost::algorithm::to_upper_copy(tmp_str));
    }

    filter_ti_values_set_ = values_tmp;

    return true;
}

bool LabelGenerator::updateTAValuesFromStr(const std::string& values)
{
    set<unsigned int> values_tmp;
    vector<string> split_str = String::split(values, ',');

    bool ok = true;

    filter_ta_null_wanted_ = false;

    for (auto& tmp_str : split_str)
    {
        if (String::trim(tmp_str) == "NULL" || String::trim(tmp_str) == "null")
        {
            filter_ta_null_wanted_ = true;
            continue;
        }

        unsigned int utn_tmp = QString(tmp_str.c_str()).toInt(&ok, 16);

        if (!ok)
        {
            logerr << "value '" << tmp_str << "' not valid";
            break;
        }

        values_tmp.insert(utn_tmp);
    }

    if (!ok)
        return false;

    filter_ta_values_set_ = values_tmp;

    return true;
}

std::string LabelGenerator::getVariableName(const std::string& dbcontent_name, unsigned int key)
{
    traced_assert(key != 0);
    traced_assert(config_.label_config_.contains(dbcontent_name));

    json& dbcont_def = config_.label_config_.at(dbcontent_name);

    if (!dbcont_def.contains(to_string(key)))
        return "";

    string varname = dbcont_def.at(to_string(key));

    return varname;
}

std::string LabelGenerator::getVariableValue(const std::string& dbcontent_name, unsigned int key,
                                             const std::shared_ptr<Buffer>& buffer, unsigned int index, bool long_val)
{
    traced_assert(key != 0);
    traced_assert(config_.label_config_.contains(dbcontent_name));

    json& dbcont_def = config_.label_config_.at(dbcontent_name);

    if (!dbcont_def.contains(to_string(key)))
        return "";

    string varname = dbcont_def.at(to_string(key));

    Variable* var_ptr = labelVariable(dbcontent_name, varname);

    if (!var_ptr)
        return ""; // not available in this database

    Variable& var = *var_ptr;

    PropertyDataType data_type = var.dataType();
    string value;

    switch (data_type)
    {
    case PropertyDataType::BOOL:
    {
        if (!buffer->has<bool>(varname))
            return "";
        else
        {
            NullableVector<bool>& values = buffer->get<bool>(varname);

            if (values.isNull(index))
                return "";

            if (var.representation() == Variable::Representation::STANDARD)
                value = values.getAsString(index);
            else
                value = var.getAsSpecialRepresentationString(values.get(index));

            return value;
        }

        break;
    }
    case PropertyDataType::CHAR:
    {
        if (!buffer->has<char>(varname))
            return "";
        else
        {
            NullableVector<char>& values = buffer->get<char>(varname);

            if (values.isNull(index))
                return "";

            if (var.representation() == Variable::Representation::STANDARD)
                value = values.getAsString(index);
            else
                value = var.getAsSpecialRepresentationString(values.get(index));

            return value;
        }

        break;
    }
    case PropertyDataType::UCHAR:
    {
        if (!buffer->has<unsigned char>(varname))
            return "";
        else
        {
            NullableVector<unsigned char>& values = buffer->get<unsigned char>(varname);

            if (values.isNull(index))
                return "";

            if (var.representation() == Variable::Representation::STANDARD)
                value = values.getAsString(index);
            else
                value = var.getAsSpecialRepresentationString(values.get(index));

            return value;
        }

        break;
    }
    case PropertyDataType::INT:
    {
        if (!buffer->has<int>(varname))
            return "";
        else
        {
            NullableVector<int>& values = buffer->get<int>(varname);

            if (values.isNull(index))
                return "";

            if (var.representation() == Variable::Representation::STANDARD)
                value = values.getAsString(index);
            else
                value = var.getAsSpecialRepresentationString(values.get(index));

            return value;
        }

        break;
    }
    case PropertyDataType::UINT:
    {
        if (!buffer->has<unsigned int>(varname))
            return "";
        else
        {
            NullableVector<unsigned int>& values = buffer->get<unsigned int>(varname);

            if (values.isNull(index))
                return "";

            if (var.representation() == Variable::Representation::STANDARD)
                value = values.getAsString(index);
            else
                value = var.getAsSpecialRepresentationString(values.get(index));

            return value;
        }

        break;
    }
    case PropertyDataType::LONGINT:
    {
        if (!buffer->has<long int>(varname))
            return "";
        else
        {
            NullableVector<long int>& values = buffer->get<long int>(varname);

            if (values.isNull(index))
                return "";

            if (var.representation() == Variable::Representation::STANDARD)
                value = values.getAsString(index);
            else
                value = var.getAsSpecialRepresentationString(values.get(index));

            return value;
        }

        break;
    }
    case PropertyDataType::ULONGINT:
    {
        if (!buffer->has<unsigned long int>(varname))
            return "";
        else
        {
            NullableVector<unsigned long int>& values = buffer->get<unsigned long int>(varname);

            if (values.isNull(index))
                return "";

            if (var.representation() == Variable::Representation::STANDARD)
                value = values.getAsString(index);
            else
                value = var.getAsSpecialRepresentationString(values.get(index));

            return value;
        }

        break;
    }
    case PropertyDataType::FLOAT:
    {
        if (!buffer->has<float>(varname))
            return "";
        else
        {
            NullableVector<float>& values = buffer->get<float>(varname);

            if (values.isNull(index))
                return "";

            if (var.representation() == Variable::Representation::STANDARD)
                value = values.getAsString(index);
            else
                value = var.getAsSpecialRepresentationString(values.get(index));

            return value;
        }

        break;
    }
    case PropertyDataType::DOUBLE:
    {
        if (!buffer->has<double>(varname))
            return "";
        else
        {
            NullableVector<double>& values = buffer->get<double>(varname);

            if (values.isNull(index))
                return "";

            if (var.representation() == Variable::Representation::STANDARD)
                value = values.getAsString(index);
            else
                value = var.getAsSpecialRepresentationString(values.get(index));

            return value;
        }

        break;
    }
    case PropertyDataType::STRING:
    {
        if (!buffer->has<string>(varname))
            return "";
        else
        {
            NullableVector<string>& values = buffer->get<string>(varname);

            if (values.isNull(index))
                return "";

            // fixed length strings are padded with spaces (e.g. SID/STAR, aircraft type) => trimmed as elsewhere
            value = String::trim(values.getAsString(index));

            return value;
        }

        break;
    }
    case PropertyDataType::JSON:
    {
        if (!buffer->has<json>(varname))
            return "";
        else
        {
            NullableVector<json>& values = buffer->get<json>(varname);

            if (values.isNull(index))
                return "";

            if (var.representation() != Variable::Representation::STANDARD)
            {
                unsigned int row_ds_id = 0;
                if (dbcont_manager_.metaCanGetVariable(
                        dbcontent_name, dbcontent_vars::meta_var_ds_id_))
                {
                    auto& ds_var = dbcont_manager_.metaGetVariable(
                        dbcontent_name, dbcontent_vars::meta_var_ds_id_);
                    if (buffer->hasAnyPropertyNamed(ds_var.name()))
                    {
                        auto& ds_vec = buffer->get<unsigned int>(ds_var.name());
                        if (!ds_vec.isNull(index))
                            row_ds_id = ds_vec.get(index);
                    }
                }
                value = var.getAsJSONRepresentationString(
                    values.getRef(index),
                    &dbcont_manager_.compass().dbContextManager(),
                    row_ds_id);
            }
            else
                value = values.getAsString(index);

            return value;
        }

        break;
    }
    case PropertyDataType::TIMESTAMP:
    {
        if (!buffer->has<boost::posix_time::ptime>(varname))
            return "";
        else
        {
            NullableVector<boost::posix_time::ptime>& values = buffer->get<boost::posix_time::ptime>(varname);

            if (values.isNull(index))
                return "";

            traced_assert(var.representation() == Variable::Representation::STANDARD); // only 1 representation
            //value = values.getAsString(index);

            if (long_val)
                value = Time::toString(values.get(index)); // display with date
            else
                value = Time::toTimeString(values.get(index)); // only display as HH:SS:MM:ZZZ

            return value;
        }

        break;
    }
    default:
        logerr << "impossible for property type "
               << Property::asString(data_type);
        throw std::runtime_error(
                    "LabelGenerator: getVariableValue: impossible property type " +
                    Property::asString(data_type));
    }

}

std::string LabelGenerator::getVariableUnit(const std::string& dbcontent_name, unsigned int key)
{
    traced_assert(key != 0);
    traced_assert(config_.label_config_.contains(dbcontent_name));

    json& dbcont_def = config_.label_config_.at(dbcontent_name);

    if (!dbcont_def.contains(to_string(key)))
        return "";

    string varname = dbcont_def.at(to_string(key));

    Variable* var = labelVariable(dbcontent_name, varname);

    if (!var)
        return ""; // not available in this database

    return var->dimensionUnitStr();
}

std::string LabelGenerator::getMode3AText (const std::string& dbcontent_name,
                                           unsigned int buffer_index, const std::shared_ptr<Buffer>& buffer)
{
    string text;

    Variable& m3a_var = dbcont_manager_.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_);

    if (buffer->has<unsigned int>(m3a_var.name()) &&
            !buffer->get<unsigned int>(m3a_var.name()).isNull(buffer_index))
    {
        text = m3a_var.getAsSpecialRepresentationString(
                    buffer->get<unsigned int>(m3a_var.name()).get(buffer_index));

        bool valid=true, garbled=false, smoothed=false;

        // "Mode 3/A Valid"
        if (dbcont_manager_.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_v_))
        {
            Variable& var = dbcont_manager_.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_v_);

            if (buffer->has<bool>(var.name()) && !buffer->get<bool>(var.name()).isNull(buffer_index))
                valid = buffer->get<bool>(var.name()).get(buffer_index);
        }
        // "Mode 3/A Garbled"
        if (dbcont_manager_.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_g_))
        {
            Variable& var = dbcont_manager_.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_g_);

            if (buffer->has<bool>(var.name()) && !buffer->get<bool>(var.name()).isNull(buffer_index))
                garbled = buffer->get<bool>(var.name()).get(buffer_index);
        }

        // "Mode 3/A Smoothed"
        if (dbcont_manager_.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_smoothed_))
        {
            Variable& var = dbcont_manager_.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_smoothed_);

            if (buffer->has<bool>(var.name()) && !buffer->get<bool>(var.name()).isNull(buffer_index))
                smoothed = buffer->get<bool>(var.name()).get(buffer_index);
        }

        if (!valid || garbled || smoothed)
            text += " ";

        if (!valid)
            text += "I";

        if (garbled)
            text += "G";

        if (smoothed)
            text += "S";
    }

    return text;
}

std::string LabelGenerator::getModeCText (const std::string& dbcontent_name,
                                          unsigned int buffer_index, const std::shared_ptr<Buffer>& buffer)
{
    string text;

    // the Mode C Meta Variable, CAT062 is the selection "Mode C Code" over I062/340 when valid
    // and not garbled, I062/135 and I062/136, no own fallback chain here
    Variable& mc_var = dbcont_manager_.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_);

    if (buffer->has<float>(mc_var.name()) &&
            !buffer->get<float>(mc_var.name()).isNull(buffer_index))
    {
        text = String::doubleToStringPrecision(buffer->get<float>(mc_var.name()).get(buffer_index)/100.0,2);

        // a selection applied the flag conditions of its candidates, the flags of the decoded
        // item do not describe a value from another candidate
        if (mc_var.isSelection())
            return text;

        bool valid=true, garbled=false;

        // "Mode CValid"
        if (dbcont_manager_.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_v_))
        {
            Variable& var = dbcont_manager_.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_v_);

            if (buffer->has<bool>(var.name()) && !buffer->get<bool>(var.name()).isNull(buffer_index))
                valid = buffer->get<bool>(var.name()).get(buffer_index);
        }
        // "Mode C Garbled"
        if (dbcont_manager_.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_g_))
        {
            Variable& var = dbcont_manager_.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_g_);

            if (buffer->has<bool>(var.name()) && !buffer->get<bool>(var.name()).isNull(buffer_index))
                garbled = buffer->get<bool>(var.name()).get(buffer_index);
        }

        if (!valid || garbled)
            text += " ";

        if (!valid)
            text += "I";

        if (garbled)
            text += "G";
    }

    return text;
}

//void LabelGenerator::onConfigurationChanged(const std::vector<std::string>& changed_params)
//{
//    emit configChanged();
//}

}

