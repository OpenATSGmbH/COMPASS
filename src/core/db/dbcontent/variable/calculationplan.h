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

#include "property.h"

#include <json_fwd.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

class Buffer;
class COMPASS;
class DBContentManager;

namespace dbContent
{

/**
 * The calculated variables declared with an operator in their "calculation" parameter, see
 * readme_calc_operators.md.
 *
 * create() reads the variable model, main thread only: it checks every calculation, resolves
 * the inputs, unit factors and lookup tables and sorts the calculations of a DBContent by their
 * inputs. A calculation with an error is logged and skipped. run() touches buffer columns only
 * and may run on any thread, one buffer per thread.
 *
 * Every operator works on whole columns: the inputs are read once into plain value vectors,
 * NaN for null, the result is written in one loop into the raw output column.
 *
 * The "function" operator runs a hardcoded calculation of calculation_functions on the whole
 * buffer, once per buffer even when it writes several variables. A function may read the
 * variable model and the managers. "timestamp" and "artas_tri_record_numbers" run elsewhere,
 * their entries only document them.
 */
class CalculationPlan
{
public:
    enum class Operator
    {
        Max,
        Add,
        Hypot,
        BearingDeg,
        ArrayLength,
        DsId,
        Lookup,
        Function
    };

    struct LookupCondition
    {
        unsigned int input {0};           // index into Step::inputs
        std::vector<double> values;       // allowed values
        bool null_allowed {false};        // a null value matches
    };

    struct LookupBranch
    {
        unsigned int key_input {0};       // index into Step::inputs
        std::vector<LookupCondition> conditions;
        std::vector<double> table;        // indexed by the key value, NaN when no entry
    };

    struct Step
    {
        Operator op {Operator::Max};
        std::string output;                         // variable name
        PropertyDataType output_type {PropertyDataType::DOUBLE};
        std::vector<std::string> inputs;            // variable names
        std::vector<PropertyDataType> input_types;
        std::vector<double> input_factors;          // unit factor input to output, 1 when none
        bool fill_null {false};                     // write rows with a null output only
        std::vector<LookupBranch> branches;         // lookup only
        double scale {1.0};                         // lookup only
        std::string function;                       // function only, the name
    };

    /// all calculations of all DBContents
    static std::shared_ptr<const CalculationPlan> create(DBContentManager& dbcont_man);

    /// parses op, mode, branches, scale and function name of a calculation into a step with
    /// inputs set, false with an error text when invalid. Does not check variables, unit factors
    /// or whether the function exists
    static bool parseStep(const nlohmann::json& calculation, Step& step, std::string& error);

    /// appends a step to the run order of a DBContent, for create() and the tests
    void addStep(const std::string& dbcontent_name, const Step& step);
    const std::vector<Step>& steps(const std::string& dbcontent_name) const;

    void run(std::map<std::string, std::shared_ptr<Buffer>>& buffers) const;
    void run(const std::string& dbcontent_name, const std::shared_ptr<Buffer>& buffer) const;

    /// true when the plan runs the named function for the DBContent
    bool runsFunction(const std::string& dbcontent_name, const std::string& function) const;

    static bool operatorFromString(const std::string& op_str, Operator& op);

    static const std::string OperatorFunction;
    static const std::string FunctionRadarProjection;
    static const std::string FunctionADSBSGVSpeedTrack;
    static const std::string FunctionTimestamp;
    static const std::string FunctionARTASTRIRecordNumbers;

private:
    std::map<std::string, std::vector<Step>> steps_;
    COMPASS* compass_ {nullptr}; // set by create(), the functions need it
};

}
