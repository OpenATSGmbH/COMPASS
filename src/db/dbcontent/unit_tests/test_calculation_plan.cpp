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

#include "catch.hpp"
#include "dbcontent/variable/calculationplan.h"
#include "buffer.h"
#include "propertylist.h"

#include "json.hpp"

#include <boost/optional.hpp>

#include <cmath>
#include <fstream>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <dirent.h>

using dbContent::CalculationPlan;
using Op = CalculationPlan::Operator;
using nlohmann::json;

namespace
{

const std::string Content = "CAT000";

using OptD = boost::optional<double>;

/// a step with all inputs DOUBLE and unit factor 1 unless given
CalculationPlan::Step makeStep(Op op, const std::string& output, PropertyDataType output_type,
                               const std::vector<std::string>& inputs,
                               std::vector<PropertyDataType> input_types = {},
                               std::vector<double> factors = {})
{
    CalculationPlan::Step step;
    step.op          = op;
    step.output      = output;
    step.output_type = output_type;
    step.inputs      = inputs;

    step.input_types = input_types.size() ? input_types
                                          : std::vector<PropertyDataType>(inputs.size(), PropertyDataType::DOUBLE);
    step.input_factors = factors.size() ? factors : std::vector<double>(inputs.size(), 1.0);

    return step;
}

/// a buffer with DOUBLE columns, nulls as boost::none
std::shared_ptr<Buffer> makeBuffer(const std::map<std::string, std::vector<OptD>>& columns)
{
    PropertyList pl;

    for (const auto& col_it : columns)
        pl.addProperty(col_it.first, PropertyDataType::DOUBLE);

    auto buffer = std::make_shared<Buffer>(pl, Content);

    for (const auto& col_it : columns)
    {
        NullableVector<double>& vec = buffer->get<double>(col_it.first);

        for (unsigned int cnt = 0; cnt < col_it.second.size(); ++cnt)
        {
            if (col_it.second[cnt])
                vec.set(cnt, *col_it.second[cnt]);
            else
                vec.setNull(cnt);
        }
    }

    return buffer;
}

std::vector<OptD> column(Buffer& buffer, const std::string& name)
{
    std::vector<OptD> values;
    NullableVector<double>& vec = buffer.get<double>(name);

    for (unsigned int cnt = 0; cnt < buffer.size(); ++cnt)
        values.push_back(vec.isNull(cnt) ? OptD() : OptD(vec.get(cnt)));

    return values;
}

void checkColumn(Buffer& buffer, const std::string& name, const std::vector<OptD>& expected)
{
    REQUIRE(buffer.has<double>(name));

    auto values = column(buffer, name);
    REQUIRE(values.size() == expected.size());

    for (unsigned int cnt = 0; cnt < expected.size(); ++cnt)
    {
        INFO(name << " row " << cnt);
        REQUIRE(bool(values[cnt]) == bool(expected[cnt]));

        if (expected[cnt])
            CHECK(*values[cnt] == Approx(*expected[cnt]));
    }
}

} // namespace

TEST_CASE("CalculationPlan: max ignores nulls", "[dbcontent][calculation]")
{
    CalculationPlan plan;
    plan.addStep(Content, makeStep(Op::Max, "XY Max", PropertyDataType::DOUBLE, {"X", "Y"}));

    auto buffer = makeBuffer({{"X", {1.0, 5.0, {}, {}}}, {"Y", {2.0, 3.0, 4.0, {}}}});
    plan.run(Content, buffer);

    checkColumn(*buffer, "XY Max", {2.0, 5.0, 4.0, {}});
}

TEST_CASE("CalculationPlan: max with one input column missing", "[dbcontent][calculation]")
{
    CalculationPlan plan;
    plan.addStep(Content, makeStep(Op::Max, "XY Max", PropertyDataType::DOUBLE, {"X", "Y"}));

    auto buffer = makeBuffer({{"X", {1.0, {}}}});
    plan.run(Content, buffer);

    checkColumn(*buffer, "XY Max", {1.0, {}});
}

TEST_CASE("CalculationPlan: missing inputs add no column", "[dbcontent][calculation]")
{
    CalculationPlan plan;
    plan.addStep(Content, makeStep(Op::Max, "XY Max", PropertyDataType::DOUBLE, {"X", "Y"}));
    plan.addStep(Content, makeStep(Op::Add, "Sum", PropertyDataType::DOUBLE, {"A", "B"}));

    auto buffer = makeBuffer({{"A", {1.0, 2.0}}, {"Other", {1.0, 2.0}}});
    plan.run(Content, buffer);

    CHECK_FALSE(buffer->hasAnyPropertyNamed("XY Max"));
    CHECK_FALSE(buffer->hasAnyPropertyNamed("Sum")); // B missing
}

TEST_CASE("CalculationPlan: add is null when one input is null", "[dbcontent][calculation]")
{
    CalculationPlan plan;
    plan.addStep(Content, makeStep(Op::Add, "Sum", PropertyDataType::UCHAR, {"A", "B"},
                                   {PropertyDataType::UCHAR, PropertyDataType::UCHAR}));

    PropertyList pl;
    pl.addProperty("A", PropertyDataType::UCHAR);
    pl.addProperty("B", PropertyDataType::UCHAR);
    auto buffer = std::make_shared<Buffer>(pl, Content);

    auto& a = buffer->get<unsigned char>("A");
    auto& b = buffer->get<unsigned char>("B");
    a.set(0, 2); b.set(0, 3);
    a.set(1, 4); b.setNull(1);
    a.set(2, 1); // B unset in row 2, shorter column

    plan.run(Content, buffer);

    REQUIRE(buffer->has<unsigned char>("Sum"));
    auto& sum = buffer->get<unsigned char>("Sum");
    REQUIRE(buffer->size() == 3);
    CHECK(sum.get(0) == 5);
    CHECK(sum.isNull(1));
    CHECK(sum.isNull(2));
}

TEST_CASE("CalculationPlan: hypot with unit factor", "[dbcontent][calculation]")
{
    const double ms2kt = 1.0 / 0.514444;

    CalculationPlan plan;
    plan.addStep(Content, makeStep(Op::Hypot, "Speed", PropertyDataType::DOUBLE, {"Vx", "Vy"}, {},
                                   {ms2kt, ms2kt}));

    auto buffer = makeBuffer({{"Vx", {3.0, {}}}, {"Vy", {4.0, 1.0}}});
    plan.run(Content, buffer);

    checkColumn(*buffer, "Speed", {5.0 * ms2kt, {}});
}

TEST_CASE("CalculationPlan: bearing north clockwise", "[dbcontent][calculation]")
{
    CalculationPlan plan;
    plan.addStep(Content, makeStep(Op::BearingDeg, "Angle", PropertyDataType::DOUBLE, {"Vx", "Vy"}));

    auto buffer = makeBuffer({{"Vx", {0.0, 1.0, 0.0, -1.0, {}}}, {"Vy", {1.0, 0.0, -1.0, 0.0, 1.0}}});
    plan.run(Content, buffer);

    checkColumn(*buffer, "Angle", {0.0, 90.0, 180.0, 270.0, {}});
}

TEST_CASE("CalculationPlan: fill_null keeps set values", "[dbcontent][calculation]")
{
    auto step = makeStep(Op::Hypot, "Speed", PropertyDataType::DOUBLE, {"Vx", "Vy"});
    step.fill_null = true;

    CalculationPlan plan;
    plan.addStep(Content, step);

    auto buffer = makeBuffer({{"Vx", {3.0, 3.0, {}}}, {"Vy", {4.0, 4.0, 1.0}}, {"Speed", {100.0, {}, 7.0}}});
    plan.run(Content, buffer);

    checkColumn(*buffer, "Speed", {100.0, 5.0, 7.0});
}

TEST_CASE("CalculationPlan: overwrite sets null where the result is null", "[dbcontent][calculation]")
{
    CalculationPlan plan;
    plan.addStep(Content, makeStep(Op::Hypot, "Speed", PropertyDataType::DOUBLE, {"Vx", "Vy"}));

    auto buffer = makeBuffer({{"Vx", {3.0, {}}}, {"Vy", {4.0, 1.0}}, {"Speed", {100.0, 7.0}}});
    plan.run(Content, buffer);

    checkColumn(*buffer, "Speed", {5.0, {}});
}

TEST_CASE("CalculationPlan: ds_id with SAC/SIC fallback", "[dbcontent][calculation]")
{
    CalculationPlan plan;
    plan.addStep(Content, makeStep(Op::DsId, "DS ID", PropertyDataType::UINT, {"SAC", "SIC"},
                                   {PropertyDataType::UCHAR, PropertyDataType::UCHAR}));

    PropertyList pl;
    pl.addProperty("SAC", PropertyDataType::UCHAR);
    pl.addProperty("SIC", PropertyDataType::UCHAR);
    auto buffer = std::make_shared<Buffer>(pl, Content);

    auto& sac = buffer->get<unsigned char>("SAC");
    auto& sic = buffer->get<unsigned char>("SIC");
    sac.set(0, 50); sic.set(0, 10);
    sac.setNull(1); sic.set(1, 10);

    plan.run(Content, buffer);

    auto& ds_id = buffer->get<unsigned int>("DS ID");
    CHECK(ds_id.get(0) == 50 * 256 + 10);
    CHECK(ds_id.get(1) == 0 * 256 + 255);
}

TEST_CASE("CalculationPlan: ds_id without SAC/SIC columns", "[dbcontent][calculation]")
{
    CalculationPlan plan;
    plan.addStep(Content, makeStep(Op::DsId, "DS ID", PropertyDataType::UINT, {"SAC", "SIC"},
                                   {PropertyDataType::UCHAR, PropertyDataType::UCHAR}));

    auto buffer = makeBuffer({{"Other", {1.0, 2.0}}});
    plan.run(Content, buffer);

    auto& ds_id = buffer->get<unsigned int>("DS ID");
    CHECK(ds_id.get(0) == 255);
    CHECK(ds_id.get(1) == 255);
}

TEST_CASE("CalculationPlan: array_length", "[dbcontent][calculation]")
{
    CalculationPlan plan;
    plan.addStep(Content, makeStep(Op::ArrayLength, "Num", PropertyDataType::UINT, {"RUs"},
                                   {PropertyDataType::JSON}));

    PropertyList pl;
    pl.addProperty("RUs", PropertyDataType::JSON);
    auto buffer = std::make_shared<Buffer>(pl, Content);

    auto& rus = buffer->get<json>("RUs");
    rus.set(0, json::array({1, 2, 3}));
    rus.setNull(1);
    rus.set(2, json::object());
    rus.set(3, json::array());

    plan.run(Content, buffer);

    auto& num = buffer->get<unsigned int>("Num");
    CHECK(num.get(0) == 3);
    CHECK(num.isNull(1));
    CHECK(num.isNull(2));
    CHECK(num.get(3) == 0);
}

TEST_CASE("CalculationPlan: lookup branches", "[dbcontent][calculation]")
{
    json calculation = json::parse(R"({
        "op": "lookup",
        "branches": [
            {"key": "NACp", "table": {"1": 100, "2": 50}},
            {"key": "NUCp", "when": {"MOPS": [null, 0]}, "table": {"1": 1000}},
            {"key": "NUCp", "table": {"1": 2000}}
        ],
        "scale": 0.5
    })");

    CalculationPlan::Step step;
    step.output = "Max";
    step.inputs = {"NACp", "NUCp", "MOPS"};
    step.input_types.assign(3, PropertyDataType::DOUBLE);

    std::string error;
    REQUIRE(CalculationPlan::parseStep(calculation, step, error));
    REQUIRE(step.branches.size() == 3);

    CalculationPlan plan;
    plan.addStep(Content, step);

    // NACp first, a NACp without entry ends the search, MOPS null or 0, MOPS other, nothing
    auto buffer = makeBuffer({{"NACp", {2.0, 0.0, {}, {}, {}, {}}},
                              {"NUCp", {1.0, 1.0, 1.0, 1.0, 1.0, {}}},
                              {"MOPS", {{}, {}, {}, 0.0, 2.0, {}}}});
    plan.run(Content, buffer);

    checkColumn(*buffer, "Max", {25.0, {}, 500.0, 500.0, 1000.0, {}});
}

TEST_CASE("CalculationPlan: parseStep errors", "[dbcontent][calculation]")
{
    auto parse = [](const std::string& text, const std::vector<std::string>& inputs)
    {
        CalculationPlan::Step step;
        step.inputs = inputs;
        std::string error;
        bool ok = CalculationPlan::parseStep(json::parse(text), step, error);
        INFO(text << " error '" << error << "'");
        return ok;
    };

    CHECK(parse(R"({"op": "max"})", {"A", "B"}));
    CHECK(parse(R"({"op": "max", "mode": "fill_null"})", {"A", "B"}));
    CHECK_FALSE(parse(R"({"op": "max"})", {"A"}));
    CHECK_FALSE(parse(R"({"op": "hypot"})", {"A", "B", "C"}));
    CHECK_FALSE(parse(R"({"op": "max", "mode": "sometimes"})", {"A", "B"}));
    CHECK_FALSE(parse(R"({"op": "unknown"})", {"A", "B"}));
    CHECK(parse(R"({"op": "function", "name": "timestamp"})", {"A"}));
    CHECK_FALSE(parse(R"({"op": "function"})", {"A"}));
    CHECK_FALSE(parse(R"({"op": "lookup"})", {"A"}));
    CHECK_FALSE(parse(R"({"op": "lookup", "branches": [{"key": "B", "table": {"1": 2}}]})", {"A"}));
    CHECK_FALSE(parse(R"({"op": "lookup", "branches": [{"key": "A", "table": {"x": 2}}]})", {"A"}));
    CHECK_FALSE(parse(R"({"op": "lookup", "branches": [{"key": "A", "table": {"1": 2},
                          "when": {"C": [1]}}]})", {"A"}));
}

/**
 * Every "calculation" in conf/default: op known, inputs are variables of the same DBContent
 * with fitting data types, the plan parses it and runs it on one row.
 */
TEST_CASE("CalculationPlan: conf/default calculations", "[dbcontent][calculation]")
{
    const std::string conf_dir = std::string(COMPASS_SOURCE_DIR) + "/conf/default/";

    std::vector<std::string> files;

    DIR* dir = opendir(conf_dir.c_str());
    REQUIRE(dir);

    while (dirent* entry = readdir(dir))
    {
        std::string name = entry->d_name;

        if (name.rfind("db_content_", 0) == 0 && name.size() > 5 && name.substr(name.size() - 5) == ".json")
            files.push_back(name);
    }

    closedir(dir);
    REQUIRE(files.size() == 15);

    const std::set<std::string> numeric {"CHAR", "UCHAR", "INT", "UINT", "LONGINT", "ULONGINT", "FLOAT", "DOUBLE"};
    const std::map<std::string, PropertyDataType> types {
        {"CHAR", PropertyDataType::CHAR}, {"UCHAR", PropertyDataType::UCHAR}, {"INT", PropertyDataType::INT},
        {"UINT", PropertyDataType::UINT}, {"LONGINT", PropertyDataType::LONGINT},
        {"ULONGINT", PropertyDataType::ULONGINT}, {"FLOAT", PropertyDataType::FLOAT},
        {"DOUBLE", PropertyDataType::DOUBLE}, {"JSON", PropertyDataType::JSON}};

    unsigned int num_run = 0;
    unsigned int num_function = 0;

    for (const auto& file : files)
    {
        std::ifstream stream(conf_dir + file);
        json config = json::parse(stream);

        // all variables of the file, name to parameters
        std::map<std::string, json> vars;

        std::function<void(const json&)> collect = [&](const json& j)
        {
            if (j.is_object())
            {
                if (j.contains("class_name") && j.at("class_name") == "Variable")
                    vars[j.at("parameters").at("name")] = j.at("parameters");

                for (const auto& v : j)
                    collect(v);
            }
            else if (j.is_array())
            {
                for (const auto& v : j)
                    collect(v);
            }
        };

        collect(config);

        for (const auto& var_it : vars)
        {
            const json& p = var_it.second;

            INFO(file << " variable '" << var_it.first << "'");

            CHECK_FALSE(p.contains("db_expression"));

            if (!p.contains("calculation") || !p.at("calculation").contains("op"))
                continue;

            const json& calculation = p.at("calculation");

            if (calculation.at("op") == CalculationPlan::OperatorFunction)
            {
                const std::set<std::string> known {
                    CalculationPlan::FunctionRadarProjection, CalculationPlan::FunctionADSBSGVSpeedTrack,
                    CalculationPlan::FunctionTimestamp, CalculationPlan::FunctionARTASTRIRecordNumbers};

                REQUIRE(calculation.contains("name"));
                CHECK(known.count(calculation.at("name").get<std::string>()));

                for (const auto& input : p.at("inputs"))
                {
                    INFO("input '" << input << "'");
                    CHECK(vars.count(input.get<std::string>()));
                }

                ++num_function;
                continue;
            }

            CalculationPlan::Step step;
            step.output = var_it.first;
            step.inputs = p.at("inputs").get<std::vector<std::string>>();

            std::string error;
            bool parsed = CalculationPlan::parseStep(calculation, step, error);
            INFO("error '" << error << "'");
            REQUIRE(parsed);

            REQUIRE(numeric.count(p.at("data_type_str")));
            step.output_type = types.at(p.at("data_type_str"));

            PropertyList pl;

            for (const auto& input : step.inputs)
            {
                INFO("input '" << input << "'");
                REQUIRE(vars.count(input));

                std::string type_str = vars.at(input).at("data_type_str");

                if (step.op == Op::ArrayLength)
                    CHECK(type_str == "JSON");
                else
                    CHECK(numeric.count(type_str));

                step.input_types.push_back(types.at(type_str));
                pl.addProperty(input, types.at(type_str));
            }

            // one row, all inputs null: runs without error
            auto buffer = std::make_shared<Buffer>(pl, Content);

            for (const auto& input : step.inputs)
                if (buffer->has<double>(input))
                    buffer->get<double>(input).setNull(0);
                else if (buffer->has<unsigned char>(input))
                    buffer->get<unsigned char>(input).setNull(0);

            CalculationPlan plan;
            plan.addStep(Content, step);
            plan.run(Content, buffer);

            ++num_run;
        }
    }

    // 4 x max, 1 x lookup, add, array_length, 14 x ds_id, 3 x hypot, 3 x bearing_deg
    CHECK(num_run == 27);
    CHECK(num_function > 0);
}
