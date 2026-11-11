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

#include "dbcontent/variable/calculationplan.h"
#include "dbcontent/variable/calculationfunctions.h"
#include "dbcontent/variable/selectionfiller.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "buffer.h"
#include "number.h"
#include "logger.h"
#include "traced_assert.h"

#include <json.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <type_traits>

using namespace std;

namespace dbContent
{

const std::string CalculationPlan::OperatorFunction              = "function";
const std::string CalculationPlan::FunctionRadarProjection       = "radar_projection";
const std::string CalculationPlan::FunctionADSBSGVSpeedTrack     = "adsb_sgv_speed_track";
const std::string CalculationPlan::FunctionTimestamp             = "timestamp";
const std::string CalculationPlan::FunctionARTASTRIRecordNumbers = "artas_tri_record_numbers";

namespace
{

const double NaN = std::numeric_limits<double>::quiet_NaN();

using Function = void (*)(DBContentManager&, const std::string&, const std::shared_ptr<Buffer>&);

// the functions the plan runs, by name
const std::map<std::string, Function>& functions()
{
    static const std::map<std::string, Function> functions {
        {CalculationPlan::FunctionRadarProjection, &calculation_functions::radarProjection},
        {CalculationPlan::FunctionADSBSGVSpeedTrack, &calculation_functions::adsbSGVSpeedTrack}};

    return functions;
}

// the functions running elsewhere, their entries only document them
bool runsElsewhere(const std::string& function)
{
    return function == CalculationPlan::FunctionTimestamp
           || function == CalculationPlan::FunctionARTASTRIRecordNumbers;
}

bool isNumeric(PropertyDataType data_type)
{
    switch (data_type)
    {
    case PropertyDataType::CHAR:
    case PropertyDataType::UCHAR:
    case PropertyDataType::INT:
    case PropertyDataType::UINT:
    case PropertyDataType::LONGINT:
    case PropertyDataType::ULONGINT:
    case PropertyDataType::FLOAT:
    case PropertyDataType::DOUBLE:
        return true;
    default:
        return false;
    }
}

// one pass over the raw column, NaN for null
template <typename T>
bool readTyped(Buffer& buffer, const std::string& name, double factor, std::vector<double>& values)
{
    if (!buffer.has<T>(name))
        return false;

    NullableVector<T>& vec = buffer.get<T>(name);
    vec.ensureFullSize();

    const std::vector<T>& data     = vec.rawData();
    const std::vector<bool>& nulls = vec.rawNullFlags();

    size_t size = values.size();

    if (factor == 1.0)
    {
        for (size_t cnt = 0; cnt < size; ++cnt)
            values[cnt] = nulls[cnt] ? NaN : static_cast<double>(data[cnt]);
    }
    else
    {
        for (size_t cnt = 0; cnt < size; ++cnt)
            values[cnt] = nulls[cnt] ? NaN : static_cast<double>(data[cnt]) * factor;
    }

    return true;
}

// false when the column is missing, values are then all NaN
bool readColumn(Buffer& buffer, const std::string& name, PropertyDataType data_type, double factor,
                std::vector<double>& values)
{
    switch (data_type)
    {
    case PropertyDataType::CHAR:
        return readTyped<char>(buffer, name, factor, values);
    case PropertyDataType::UCHAR:
        return readTyped<unsigned char>(buffer, name, factor, values);
    case PropertyDataType::INT:
        return readTyped<int>(buffer, name, factor, values);
    case PropertyDataType::UINT:
        return readTyped<unsigned int>(buffer, name, factor, values);
    case PropertyDataType::LONGINT:
        return readTyped<std::int64_t>(buffer, name, factor, values);
    case PropertyDataType::ULONGINT:
        return readTyped<std::uint64_t>(buffer, name, factor, values);
    case PropertyDataType::FLOAT:
        return readTyped<float>(buffer, name, factor, values);
    case PropertyDataType::DOUBLE:
        return readTyped<double>(buffer, name, factor, values);
    default:
        return false;
    }
}

template <typename T>
void writeTyped(Buffer& buffer, const std::string& name, PropertyDataType data_type, bool fill_null,
                const std::vector<double>& results)
{
    if (!buffer.has<T>(name))
    {
        if (buffer.hasAnyPropertyNamed(name))
        {
            logerr << "column '" << name << "' has another data type";
            return;
        }

        buffer.addProperty(name, data_type);
    }

    NullableVector<T>& vec = buffer.get<T>(name);
    vec.ensureFullSize();

    std::vector<T>& data     = vec.rawData();
    std::vector<bool>& nulls = vec.rawNullFlags();

    size_t size = results.size();

    for (size_t cnt = 0; cnt < size; ++cnt)
    {
        if (fill_null && !nulls[cnt]) // keep the value already set
            continue;

        double value = results[cnt];

        if (std::isnan(value))
        {
            nulls[cnt] = true;
            continue;
        }

        if constexpr (std::is_integral<T>::value)
            data[cnt] = static_cast<T>(std::llround(value));
        else
            data[cnt] = static_cast<T>(value);

        nulls[cnt] = false;
    }
}

void writeColumn(Buffer& buffer, const CalculationPlan::Step& step, const std::vector<double>& results)
{
    switch (step.output_type)
    {
    case PropertyDataType::CHAR:
        writeTyped<char>(buffer, step.output, step.output_type, step.fill_null, results);
        break;
    case PropertyDataType::UCHAR:
        writeTyped<unsigned char>(buffer, step.output, step.output_type, step.fill_null, results);
        break;
    case PropertyDataType::INT:
        writeTyped<int>(buffer, step.output, step.output_type, step.fill_null, results);
        break;
    case PropertyDataType::UINT:
        writeTyped<unsigned int>(buffer, step.output, step.output_type, step.fill_null, results);
        break;
    case PropertyDataType::LONGINT:
        writeTyped<std::int64_t>(buffer, step.output, step.output_type, step.fill_null, results);
        break;
    case PropertyDataType::ULONGINT:
        writeTyped<std::uint64_t>(buffer, step.output, step.output_type, step.fill_null, results);
        break;
    case PropertyDataType::FLOAT:
        writeTyped<float>(buffer, step.output, step.output_type, step.fill_null, results);
        break;
    case PropertyDataType::DOUBLE:
        writeTyped<double>(buffer, step.output, step.output_type, step.fill_null, results);
        break;
    default:
        logerr << "output '" << step.output << "' not numeric";
        break;
    }
}

bool lookupConditionsHold(const CalculationPlan::LookupBranch& branch,
                          const std::vector<std::vector<double>>& values, size_t index)
{
    for (const auto& condition : branch.conditions)
    {
        double value = values[condition.input][index];

        if (std::isnan(value))
        {
            if (!condition.null_allowed)
                return false;

            continue;
        }

        if (std::find(condition.values.begin(), condition.values.end(), value) == condition.values.end())
            return false;
    }

    return true;
}

// false when the inputs needed are missing, then no output column is written
bool compute(const std::string& dbcontent_name, const CalculationPlan::Step& step, Buffer& buffer,
             std::vector<double>& results)
{
    using Operator = CalculationPlan::Operator;

    size_t size = results.size();

    if (step.op == Operator::ArrayLength)
    {
        const std::string& name = step.inputs.at(0);

        if (!buffer.has<nlohmann::json>(name))
            return false;

        NullableVector<nlohmann::json>& vec = buffer.get<nlohmann::json>(name);
        vec.ensureFullSize();

        const std::vector<nlohmann::json>& data = vec.rawData();
        const std::vector<bool>& nulls          = vec.rawNullFlags();

        for (size_t cnt = 0; cnt < size; ++cnt)
            if (!nulls[cnt] && data[cnt].is_array())
                results[cnt] = data[cnt].size();

        return true;
    }

    size_t num_inputs = step.inputs.size();

    std::vector<std::vector<double>> values(num_inputs, std::vector<double>(size, NaN));
    bool any_present = false;
    bool all_present = true;

    for (size_t input_cnt = 0; input_cnt < num_inputs; ++input_cnt)
    {
        if (readColumn(buffer, step.inputs[input_cnt], step.input_types[input_cnt],
                       step.input_factors[input_cnt], values[input_cnt]))
            any_present = true;
        else
            all_present = false;
    }

    switch (step.op)
    {
    case Operator::Max: // the inputs not null, as DuckDB greatest()
    {
        if (!any_present)
            return false;

        for (size_t cnt = 0; cnt < size; ++cnt)
        {
            double max = NaN;

            for (const auto& input : values)
                if (!std::isnan(input[cnt]) && (std::isnan(max) || input[cnt] > max))
                    max = input[cnt];

            results[cnt] = max;
        }

        return true;
    }
    case Operator::Add: // null when one input is null
    {
        if (!all_present)
            return false;

        for (size_t cnt = 0; cnt < size; ++cnt)
        {
            double sum = 0;

            for (const auto& input : values)
                sum += input[cnt];

            results[cnt] = sum;
        }

        return true;
    }
    case Operator::Hypot:
    {
        if (!all_present)
            return false;

        const std::vector<double>& x = values[0];
        const std::vector<double>& y = values[1];

        for (size_t cnt = 0; cnt < size; ++cnt)
            results[cnt] = std::sqrt(x[cnt] * x[cnt] + y[cnt] * y[cnt]);

        return true;
    }
    case Operator::BearingDeg: // north clockwise, 0 to 360
    {
        if (!all_present)
            return false;

        const std::vector<double>& x = values[0];
        const std::vector<double>& y = values[1];

        for (size_t cnt = 0; cnt < size; ++cnt)
        {
            if (std::isnan(x[cnt]) || std::isnan(y[cnt]))
                continue;

            double angle_deg = std::atan2(x[cnt], y[cnt]) * 180.0 / M_PI;

            if (angle_deg < 0)
                angle_deg += 360.0;

            results[cnt] = angle_deg;
        }

        return true;
    }
    case Operator::DsId: // a record without SAC or SIC gets 0/255
    {
        const std::vector<double>& sac = values[0];
        const std::vector<double>& sic = values[1];

        unsigned int num_fallback = 0;

        for (size_t cnt = 0; cnt < size; ++cnt)
        {
            if (std::isnan(sac[cnt]) || std::isnan(sic[cnt]))
            {
                results[cnt] = Utils::Number::dsIdFrom(0, 255);
                ++num_fallback;
                continue;
            }

            results[cnt] = Utils::Number::dsIdFrom(static_cast<unsigned int>(sac[cnt]),
                                                   static_cast<unsigned int>(sic[cnt]));
        }

        if (num_fallback)
            logwrn << dbcontent_name << " " << num_fallback << " records without SAC/SIC, setting 0/255";

        return true;
    }
    case Operator::Lookup: // the first branch with a key value and conditions holding decides
    {
        if (!any_present)
            return false;

        for (size_t cnt = 0; cnt < size; ++cnt)
        {
            for (const auto& branch : step.branches)
            {
                double key = values[branch.key_input][cnt];

                if (std::isnan(key) || !lookupConditionsHold(branch, values, cnt))
                    continue;

                long long key_index = std::llround(key);

                if (key_index >= 0 && key_index < (long long) branch.table.size())
                    results[cnt] = branch.table[key_index] * step.scale; // NaN when no entry

                break;
            }
        }

        return true;
    }
    default:
        return false;
    }
}

bool inputIndex(const CalculationPlan::Step& step, const std::string& name, unsigned int& index)
{
    auto it = std::find(step.inputs.begin(), step.inputs.end(), name);

    if (it == step.inputs.end())
        return false;

    index = it - step.inputs.begin();
    return true;
}

bool parseLookupBranch(const nlohmann::json& branch_json, CalculationPlan::Step& step,
                       CalculationPlan::LookupBranch& branch, std::string& error)
{
    if (!branch_json.is_object() || !branch_json.contains("key") || !branch_json.at("key").is_string())
    {
        error = "lookup branch without key";
        return false;
    }

    std::string key = branch_json.at("key");

    if (!inputIndex(step, key, branch.key_input))
    {
        error = "lookup key '" + key + "' not in inputs";
        return false;
    }

    if (!branch_json.contains("table") || !branch_json.at("table").is_object()
        || branch_json.at("table").empty())
    {
        error = "lookup branch '" + key + "' without table";
        return false;
    }

    std::map<unsigned int, double> entries;

    for (auto it = branch_json.at("table").begin(); it != branch_json.at("table").end(); ++it)
    {
        std::uint64_t key_value;

        try
        {
            size_t pos;
            key_value = std::stoull(it.key(), &pos);

            if (pos != it.key().size())
                throw std::invalid_argument(it.key());
        }
        catch (const std::exception&)
        {
            error = "lookup table key '" + it.key() + "' not an unsigned integer";
            return false;
        }

        if (key_value > 65535)
        {
            error = "lookup table key '" + it.key() + "' too large";
            return false;
        }

        if (!it.value().is_number())
        {
            error = "lookup table value of key '" + it.key() + "' not a number";
            return false;
        }

        entries[key_value] = it.value().get<double>();
    }

    branch.table.assign(entries.rbegin()->first + 1, NaN);

    for (const auto& entry : entries)
        branch.table[entry.first] = entry.second;

    if (branch_json.contains("when"))
    {
        const nlohmann::json& when = branch_json.at("when");

        if (!when.is_object())
        {
            error = "lookup branch '" + key + "' when not an object";
            return false;
        }

        for (auto it = when.begin(); it != when.end(); ++it)
        {
            CalculationPlan::LookupCondition condition;

            if (!inputIndex(step, it.key(), condition.input))
            {
                error = "lookup condition '" + it.key() + "' not in inputs";
                return false;
            }

            if (!it.value().is_array())
            {
                error = "lookup condition '" + it.key() + "' not a list";
                return false;
            }

            for (const auto& value : it.value())
            {
                if (value.is_null())
                    condition.null_allowed = true;
                else if (value.is_number())
                    condition.values.push_back(value.get<double>());
                else
                {
                    error = "lookup condition '" + it.key() + "' value not a number or null";
                    return false;
                }
            }

            branch.conditions.push_back(condition);
        }
    }

    return true;
}

} // namespace

bool CalculationPlan::operatorFromString(const std::string& op_str, Operator& op)
{
    static const std::map<std::string, Operator> operators {
        {"max", Operator::Max},
        {"add", Operator::Add},
        {"hypot", Operator::Hypot},
        {"bearing_deg", Operator::BearingDeg},
        {"array_length", Operator::ArrayLength},
        {"ds_id", Operator::DsId},
        {"lookup", Operator::Lookup},
        {OperatorFunction, Operator::Function}};

    auto it = operators.find(op_str);

    if (it == operators.end())
        return false;

    op = it->second;
    return true;
}

bool CalculationPlan::parseStep(const nlohmann::json& calculation, Step& step, std::string& error)
{
    if (!calculation.is_object() || !calculation.contains("op") || !calculation.at("op").is_string())
    {
        error = "no op";
        return false;
    }

    std::string op_str = calculation.at("op");

    if (!operatorFromString(op_str, step.op))
    {
        error = "unknown op '" + op_str + "'";
        return false;
    }

    if (calculation.contains("mode"))
    {
        if (calculation.at("mode") == "fill_null")
            step.fill_null = true;
        else if (calculation.at("mode") == "overwrite")
            step.fill_null = false;
        else
        {
            error = "unknown mode '" + calculation.at("mode").dump() + "'";
            return false;
        }
    }

    size_t num_inputs = step.inputs.size();

    switch (step.op)
    {
    case Operator::Max:
    case Operator::Add:
        if (num_inputs < 2)
        {
            error = op_str + " needs 2 or more inputs";
            return false;
        }
        break;
    case Operator::Hypot:
    case Operator::BearingDeg:
    case Operator::DsId:
        if (num_inputs != 2)
        {
            error = op_str + " needs 2 inputs";
            return false;
        }
        break;
    case Operator::ArrayLength:
        if (num_inputs != 1)
        {
            error = op_str + " needs 1 input";
            return false;
        }
        break;
    case Operator::Lookup:
    {
        if (!num_inputs)
        {
            error = "lookup needs 1 or more inputs";
            return false;
        }

        if (!calculation.contains("branches") || !calculation.at("branches").is_array()
            || calculation.at("branches").empty())
        {
            error = "lookup without branches";
            return false;
        }

        step.branches.clear();

        for (const auto& branch_json : calculation.at("branches"))
        {
            LookupBranch branch;

            if (!parseLookupBranch(branch_json, step, branch, error))
                return false;

            step.branches.push_back(branch);
        }

        if (calculation.contains("scale"))
        {
            if (!calculation.at("scale").is_number())
            {
                error = "lookup scale not a number";
                return false;
            }

            step.scale = calculation.at("scale");
        }
        break;
    }
    case Operator::Function:
        if (!calculation.contains("name") || !calculation.at("name").is_string()
            || calculation.at("name").get<std::string>().empty())
        {
            error = "function without name";
            return false;
        }

        step.function = calculation.at("name");
        break;
    }

    // inputs of a step built by hand default to factor 1
    if (step.input_factors.size() != num_inputs)
        step.input_factors.assign(num_inputs, 1.0);

    return true;
}

std::shared_ptr<const CalculationPlan> CalculationPlan::create(DBContentManager& dbcont_man)
{
    auto plan = std::make_shared<CalculationPlan>();
    plan->dbcont_man_ = &dbcont_man;

    for (auto& dbcont_it : dbcont_man)
    {
        const std::string& dbcontent_name = dbcont_it.first;
        const DBContent& dbcontent        = *dbcont_it.second;

        std::vector<Step> steps;

        for (const auto& var_it : dbcontent.variables())
        {
            const Variable& var = *var_it.second;

            if (!var.hasCalculation())
                continue;

            const nlohmann::json& calculation = var.calculation();

            Step step;
            step.output      = var.name();
            step.output_type = var.dataType();
            step.inputs      = var.inputs();

            std::string error;

            if (!parseStep(calculation, step, error))
            {
                logerr << dbcontent_name << " variable '" << var.name() << "' calculation: " << error;
                continue;
            }

            bool is_function = step.op == Operator::Function;

            if (is_function && runsElsewhere(step.function))
                continue;

            if (is_function && !functions().count(step.function))
            {
                logerr << dbcontent_name << " variable '" << var.name() << "' calculation: unknown function '"
                       << step.function << "'";
                continue;
            }

            if (!is_function && !isNumeric(step.output_type))
            {
                logerr << dbcontent_name << " variable '" << var.name() << "' calculation: output not numeric";
                continue;
            }

            bool ok = true;
            step.input_factors.clear();

            for (const auto& input_name : step.inputs)
            {
                if (!dbcontent.hasVariable(input_name))
                {
                    logerr << dbcontent_name << " variable '" << var.name() << "' calculation: input '"
                           << input_name << "' unknown";
                    ok = false;
                    break;
                }

                const Variable& input = dbcontent.variable(input_name);
                PropertyDataType input_type = input.dataType();

                bool type_ok = is_function || (step.op == Operator::ArrayLength ? input_type == PropertyDataType::JSON
                                                                                : isNumeric(input_type));
                if (!type_ok)
                {
                    logerr << dbcontent_name << " variable '" << var.name() << "' calculation: input '"
                           << input_name << "' has a wrong data type";
                    ok = false;
                    break;
                }

                // unit conversion only where input and output have the same dimension
                double factor = 1.0;

                if ((step.op == Operator::Max || step.op == Operator::Add || step.op == Operator::Hypot)
                    && !unitFactor(dbcont_man, input, var, factor))
                {
                    ok = false;
                    break;
                }

                step.input_types.push_back(input_type);
                step.input_factors.push_back(factor);
            }

            if (ok)
                steps.push_back(step);
        }

        // run order: a step after the steps writing its inputs

        std::vector<bool> done(steps.size(), false);
        unsigned int num_done = 0;
        bool progress = true;

        while (num_done < steps.size() && progress)
        {
            progress = false;

            for (size_t cnt = 0; cnt < steps.size(); ++cnt)
            {
                if (done[cnt])
                    continue;

                bool waiting = false;

                for (size_t other = 0; other < steps.size() && !waiting; ++other)
                    if (other != cnt && !done[other]
                        && std::count(steps[cnt].inputs.begin(), steps[cnt].inputs.end(), steps[other].output))
                        waiting = true;

                if (waiting)
                    continue;

                plan->addStep(dbcontent_name, steps[cnt]);
                done[cnt] = true;
                ++num_done;
                progress = true;
            }
        }

        for (size_t cnt = 0; cnt < steps.size(); ++cnt)
            if (!done[cnt])
                logerr << dbcontent_name << " variable '" << steps[cnt].output
                       << "' calculation: cyclic inputs, skipped";
    }

    return plan;
}

void CalculationPlan::addStep(const std::string& dbcontent_name, const Step& step)
{
    traced_assert(step.input_types.size() == step.inputs.size());
    traced_assert(step.input_factors.size() == step.inputs.size());

    steps_[dbcontent_name].push_back(step);
}

const std::vector<CalculationPlan::Step>& CalculationPlan::steps(const std::string& dbcontent_name) const
{
    static const std::vector<Step> no_steps;

    auto it = steps_.find(dbcontent_name);

    return it == steps_.end() ? no_steps : it->second;
}

void CalculationPlan::run(std::map<std::string, std::shared_ptr<Buffer>>& buffers) const
{
    for (auto& buf_it : buffers)
    {
        if (!buf_it.second || !buf_it.second->size())
            continue;

        run(buf_it.first, buf_it.second);
    }
}

void CalculationPlan::run(const std::string& dbcontent_name, const std::shared_ptr<Buffer>& buffer) const
{
    traced_assert(buffer);

    auto it = steps_.find(dbcontent_name);

    if (it == steps_.end())
        return;

    size_t size = buffer->size();

    if (!size)
        return;

    std::set<std::string> functions_run; // a function writing several variables runs once

    for (const auto& step : it->second)
    {
        if (step.op == Operator::Function)
        {
            if (functions_run.count(step.function))
                continue;

            functions_run.insert(step.function);

            traced_assert(dbcont_man_);
            functions().at(step.function)(*dbcont_man_, dbcontent_name, buffer);
            continue;
        }

        std::vector<double> results(size, NaN);

        if (!compute(dbcontent_name, step, *buffer, results))
            continue;

        writeColumn(*buffer, step, results);
    }
}

bool CalculationPlan::runsFunction(const std::string& dbcontent_name, const std::string& function) const
{
    for (const auto& step : steps(dbcontent_name))
        if (step.op == Operator::Function && step.function == function)
            return true;

    return false;
}

}
