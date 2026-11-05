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

#include "dbcontent/variable/selectionfiller.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "unitmanager.h"
#include "dimension.h"
#include "buffer.h"
#include "logger.h"
#include "traced_assert.h"

#include <json.hpp>

#include <functional>
#include <type_traits>
#include <vector>

using namespace std;

namespace dbContent
{

namespace
{

// access to one candidate of one selection variable inside a buffer
struct CandidateAccess
{
    bool is_calculation {false};
    bool usable {false}; // false when the candidate column is missing or has another type

    std::function<bool(unsigned int)> is_null;
    std::function<void(unsigned int)> copy;

    std::vector<const NullableVector<bool>*> require_true;
    std::vector<const NullableVector<bool>*> require_false;

    bool conditionsHold(unsigned int index) const
    {
        // a missing or null flag does not block the candidate, the label generator rule

        for (const auto* flag_vec : require_true)
            if (!flag_vec->isNull(index) && !flag_vec->get(index))
                return false;

        for (const auto* flag_vec : require_false)
            if (!flag_vec->isNull(index) && flag_vec->get(index))
                return false;

        return true;
    }

    bool hasValue(unsigned int index) const
    {
        return usable && !is_null(index) && conditionsHold(index);
    }
};

struct Member
{
    Variable* variable {nullptr};
    std::vector<CandidateAccess> candidates;
};

// factor is the unit conversion from the candidate to the selection, 1 when none is needed
template <typename T>
void makeTypedAccess(Buffer& buffer, const std::string& candidate_name,
                     const std::string& selection_name,
                     double factor, CandidateAccess& access)
{
    NullableVector<T>& candidate_vec = buffer.get<T>(candidate_name);

    access.is_null = [&candidate_vec](unsigned int index) { return candidate_vec.isNull(index); };

    NullableVector<T>& selection_vec = buffer.get<T>(selection_name);

    if constexpr (std::is_arithmetic<T>::value && !std::is_same<T, bool>::value)
    {
        if (factor != 1.0) // the candidate has another unit than the selection
        {
            access.copy = [&candidate_vec, &selection_vec, factor](unsigned int index)
            { selection_vec.set(index, static_cast<T>(candidate_vec.get(index) * factor)); };
        }
    }

    if (!access.copy)
    {
        access.copy = [&candidate_vec, &selection_vec](unsigned int index)
        { selection_vec.set(index, candidate_vec.get(index)); };
    }

    access.usable = true;
}

void makeAccess(Buffer& buffer, PropertyDataType data_type, const std::string& candidate_name,
                const std::string& selection_name, double factor,
                CandidateAccess& access)
{
    switch (data_type)
    {
    case PropertyDataType::BOOL:
        makeTypedAccess<bool>(buffer, candidate_name, selection_name, factor, access);
        break;
    case PropertyDataType::CHAR:
        makeTypedAccess<char>(buffer, candidate_name, selection_name, factor, access);
        break;
    case PropertyDataType::UCHAR:
        makeTypedAccess<unsigned char>(buffer, candidate_name, selection_name, factor, access);
        break;
    case PropertyDataType::INT:
        makeTypedAccess<int>(buffer, candidate_name, selection_name, factor, access);
        break;
    case PropertyDataType::UINT:
        makeTypedAccess<unsigned int>(buffer, candidate_name, selection_name, factor, access);
        break;
    case PropertyDataType::LONGINT:
        makeTypedAccess<long>(buffer, candidate_name, selection_name, factor, access);
        break;
    case PropertyDataType::ULONGINT:
        makeTypedAccess<unsigned long>(buffer, candidate_name, selection_name, factor, access);
        break;
    case PropertyDataType::FLOAT:
        makeTypedAccess<float>(buffer, candidate_name, selection_name, factor, access);
        break;
    case PropertyDataType::DOUBLE:
        makeTypedAccess<double>(buffer, candidate_name, selection_name, factor, access);
        break;
    case PropertyDataType::STRING:
        makeTypedAccess<std::string>(buffer, candidate_name, selection_name, factor, access);
        break;
    case PropertyDataType::JSON:
        makeTypedAccess<nlohmann::json>(buffer, candidate_name, selection_name, factor, access);
        break;
    case PropertyDataType::TIMESTAMP:
        makeTypedAccess<boost::posix_time::ptime>(buffer, candidate_name, selection_name,
                                                  factor, access);
        break;
    default:
        logerr << "unknown data type for selection '" << selection_name << "'";
        break;
    }
}

void collectFlags(const DBContent& dbcontent, Buffer& buffer, const nlohmann::json& candidate,
                  const std::string& key, std::vector<const NullableVector<bool>*>& flags)
{
    if (!candidate.contains(key) || !candidate.at(key).is_array())
        return;

    for (const auto& flag_name_json : candidate.at(key))
    {
        if (!flag_name_json.is_string())
            continue;

        std::string flag_name = flag_name_json;

        if (!dbcontent.hasVariable(flag_name))
        {
            logerr << dbcontent.name() << " selection condition variable '" << flag_name << "' unknown";
            continue;
        }

        if (dbcontent.variable(flag_name).dataType() != PropertyDataType::BOOL)
        {
            logerr << dbcontent.name() << " selection condition variable '" << flag_name << "' not boolean";
            continue;
        }

        if (buffer.has<bool>(flag_name))
            flags.push_back(&buffer.get<bool>(flag_name));
    }
}

} // namespace

bool unitFactor(DBContentManager& dbcont_man, const Variable& candidate, const Variable& selection,
                double& factor)
{
    factor = 1.0;

    const std::string& cand_dim  = candidate.dimensionConst();
    const std::string& sel_dim   = selection.dimensionConst();
    const std::string& cand_unit = candidate.unitConst();
    const std::string& sel_unit  = selection.unitConst();

    if (cand_dim.empty() || sel_dim.empty() || cand_unit.empty() || sel_unit.empty())
        return true;

    if (cand_dim != sel_dim)
    {
        logerr << selection.dbContentName() << " '" << selection.name() << "' input '"
               << candidate.name() << "' has dimension '" << cand_dim << "' instead of '" << sel_dim << "'";
        return false;
    }

    if (cand_unit == sel_unit)
        return true;

    UnitManager& unit_man = dbcont_man.unitManager();

    if (!unit_man.hasDimension(sel_dim) || !unit_man.dimension(sel_dim).hasUnit(cand_unit)
        || !unit_man.dimension(sel_dim).hasUnit(sel_unit))
    {
        logerr << selection.dbContentName() << " '" << selection.name() << "' input '"
               << candidate.name() << "' unit '" << cand_unit << "' or '" << sel_unit
               << "' unknown in dimension '" << sel_dim << "'";
        return false;
    }

    factor = unit_man.dimension(sel_dim).getFactor(cand_unit, sel_unit);

    logdbg << selection.dbContentName() << " '" << selection.name() << "' input '"
           << candidate.name() << "' converted from " << cand_unit << " to " << sel_unit
           << " factor " << factor;

    return true;
}

void SelectionFiller::fill(DBContentManager& dbcont_man,
                           std::map<std::string, std::shared_ptr<Buffer>>& buffers)
{
    for (auto& buf_it : buffers)
    {
        if (!buf_it.second || !buf_it.second->size())
            continue;

        fill(dbcont_man, buf_it.first, *buf_it.second);
    }
}

void SelectionFiller::fill(DBContentManager& dbcont_man, const std::string& dbcontent_name, Buffer& buffer)
{
    if (!dbcont_man.existsDBContent(dbcontent_name))
        return;

    const DBContent& dbcontent = dbcont_man.dbContent(dbcontent_name);

    unsigned int buffer_size = buffer.size();

    // selection variables grouped by their source variable, chosen together

    std::map<std::string, std::vector<Variable*>> groups;

    for (const auto& var_it : dbcontent.variables())
    {
        Variable& var = *var_it.second;

        if (!var.isSelection())
            continue;

        if (var.sourceVariableName().empty())
        {
            logerr << dbcontent_name << " selection '" << var.name() << "' has no source variable";
            continue;
        }

        if (!dbcontent.hasVariable(var.sourceVariableName())
            || dbcontent.variable(var.sourceVariableName()).dataType() != PropertyDataType::UCHAR)
        {
            logerr << dbcontent_name << " selection '" << var.name() << "' source variable '"
                   << var.sourceVariableName() << "' missing or not UCHAR";
            continue;
        }

        groups[var.sourceVariableName()].push_back(&var);
    }

    if (groups.empty())
        return;

    // add the columns first, the accessors below keep references into the buffer

    for (const auto& group_it : groups)
    {
        if (!buffer.has<unsigned char>(group_it.first))
            buffer.addProperty(group_it.first, PropertyDataType::UCHAR);

        for (Variable* var : group_it.second)
        {
            if (!buffer.hasAnyPropertyNamed(var->name()))
                buffer.addProperty(var->name(), var->dataType());
        }
    }

    for (const auto& group_it : groups)
    {
        const std::string& source_var_name = group_it.first;

        std::vector<Member> members;
        unsigned int num_candidates = 0;
        bool group_ok = true;

        for (Variable* var : group_it.second)
        {
            if (members.empty())
                num_candidates = var->numSelectionCandidates();
            else if (var->numSelectionCandidates() != num_candidates)
            {
                logerr << dbcontent_name << " selections sharing '" << source_var_name
                       << "' have different candidate counts";
                group_ok = false;
                break;
            }

            Member member;
            member.variable = var;

            for (const auto& candidate : var->selection())
            {
                CandidateAccess access;

                if (!candidate.is_object())
                {
                    logerr << dbcontent_name << " selection '" << var->name() << "' has a malformed candidate";
                }
                else if (candidate.contains("calculation"))
                {
                    access.is_calculation = true;
                }
                else if (candidate.contains("variable") && candidate.at("variable").is_string())
                {
                    std::string candidate_name = candidate.at("variable");

                    if (!dbcontent.hasVariable(candidate_name))
                        logerr << dbcontent_name << " selection '" << var->name() << "' candidate '"
                               << candidate_name << "' unknown";
                    else if (dbcontent.variable(candidate_name).dataType() != var->dataType())
                        logerr << dbcontent_name << " selection '" << var->name() << "' candidate '"
                               << candidate_name << "' has another data type";
                    else if (buffer.hasAnyPropertyNamed(candidate_name)) // absent column: never a value
                    {
                        double factor = 1.0;

                        if (unitFactor(dbcont_man, dbcontent.variable(candidate_name), *var, factor))
                        {
                            makeAccess(buffer, var->dataType(), candidate_name, var->name(),
                                       factor, access);

                            collectFlags(dbcontent, buffer, candidate, "require_true", access.require_true);
                            collectFlags(dbcontent, buffer, candidate, "require_false", access.require_false);
                        }
                    }
                }
                else
                    logerr << dbcontent_name << " selection '" << var->name() << "' has a candidate without variable or calculation";

                member.candidates.push_back(std::move(access));
            }

            members.push_back(std::move(member));
        }

        if (!group_ok || members.empty() || !num_candidates)
            continue;

        NullableVector<unsigned char>& source_vec = buffer.get<unsigned char>(source_var_name);

        std::vector<unsigned int> chosen_counts(num_candidates, 0);
        unsigned int none_cnt = 0;

        for (unsigned int index = 0; index < buffer_size; ++index)
        {
            if (!source_vec.isNull(index)) // already chosen
                continue;

            bool chosen = false;

            for (unsigned int cand = 0; cand < num_candidates && !chosen; ++cand)
            {
                bool is_calculation = false;
                bool all_have_value = true;

                for (const Member& member : members)
                {
                    const CandidateAccess& access = member.candidates.at(cand);

                    if (access.is_calculation)
                    {
                        is_calculation = true;
                        break;
                    }

                    if (!access.hasValue(index))
                    {
                        all_have_value = false;
                        break;
                    }
                }

                if (is_calculation) // the calculation code fills the rows still null
                    break;

                if (!all_have_value)
                    continue;

                for (const Member& member : members)
                {
                    const CandidateAccess& access = member.candidates.at(cand);

                    if (access.copy)
                        access.copy(index);
                }

                source_vec.set(index, cand + 1);
                ++chosen_counts.at(cand);
                chosen = true;
            }

            if (!chosen)
                ++none_cnt;
        }

        if (buffer_size)
        {
            std::string counts;

            for (unsigned int cand = 0; cand < num_candidates; ++cand)
                counts += (cand ? " " : "") + std::to_string(chosen_counts.at(cand));

            logdbg << dbcontent_name << " '" << source_var_name << "' rows " << buffer_size
                   << " chosen per candidate " << counts << " none " << none_cnt;
        }
    }
}

}
