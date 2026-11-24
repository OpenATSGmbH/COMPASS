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

#include "requirement/base/falsehelpers.h"

#include "json.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>

using namespace EvaluationRequirement;
using namespace EvaluationRequirement::FalseHelpers;

namespace
{

ValueComparisonResult cmpFromString(const std::string& s)
{
    if (s == "same")
        return ValueComparisonResult::Same;
    if (s == "different")
        return ValueComparisonResult::Different;
    if (s == "no_ref")
        return ValueComparisonResult::Unknown_NoRefData;
    if (s == "no_tst")
        return ValueComparisonResult::Unknown_NoTstData;

    throw std::runtime_error("unknown compare result '" + s + "'");
}

std::string casesDir()
{
    // the source path of the build, or COMPASS_SOURCE_DIR (e.g. tests of a cross build)
    const char* source_dir_env = std::getenv("COMPASS_SOURCE_DIR");
    return std::string(source_dir_env ? source_dir_env : COMPASS_SOURCE_DIR)
        + "/src/eval/requirement/base/unit_tests/false_cases/";
}

} // namespace

TEST_CASE("FalseHelpers::classify of one value", "[eval][false_helpers]")
{
    CHECK(classify(ValueComparisonResult::Unknown_NoRefData) == ReportClass::NoRefValue);
    CHECK(classify(ValueComparisonResult::Unknown_NoTstData) == ReportClass::Unknown);
    CHECK(classify(ValueComparisonResult::Same)              == ReportClass::Correct);
    CHECK(classify(ValueComparisonResult::Different)         == ReportClass::False);

    // one used value gives the same class as the single value classification
    for (auto cmp : { ValueComparisonResult::Unknown_NoRefData, ValueComparisonResult::Unknown_NoTstData,
                      ValueComparisonResult::Same, ValueComparisonResult::Different })
    {
        CHECK(classify({ cmp }, false) == classify(cmp));
        CHECK(classify({ cmp }, true)  == classify(cmp));
    }
}

TEST_CASE("FalseHelpers::Counts::probabilityFalse", "[eval][false_helpers]")
{
    Counts c;
    CHECK(!c.probabilityFalse().has_value());

    c.add(ReportClass::NoRefValue);
    c.add(ReportClass::Unknown);
    CHECK(!c.probabilityFalse().has_value()); // not counted

    c.add(ReportClass::Correct);
    c.add(ReportClass::Correct);
    c.add(ReportClass::Correct);
    c.add(ReportClass::False);
    REQUIRE(c.probabilityFalse().has_value());
    CHECK(c.probabilityFalse().value() == Approx(0.25));
}

TEST_CASE("FalseHelpers cases from JSON", "[eval][false_helpers]")
{
    std::vector<std::filesystem::path> files;
    for (const auto& entry : std::filesystem::directory_iterator(casesDir()))
        if (entry.path().extension() == ".json")
            files.push_back(entry.path());

    std::sort(files.begin(), files.end());

    REQUIRE(!files.empty());

    for (const auto& file : files)
    {
        std::ifstream in(file);
        nlohmann::json j = nlohmann::json::parse(in);

        INFO("case " << file.filename().string() << ": " << j.value("description", ""));

        bool require_all_false = j.value("require_all_false", false);

        Counts counts;

        for (const auto& report : j.at("reports"))
        {
            unsigned int repeat = report.value("repeat", 1u);

            std::vector<ValueComparisonResult> cmp_results;
            for (const auto& cmp : report.at("cmp"))
                cmp_results.push_back(cmpFromString(cmp.get<std::string>()));

            auto report_class = classify(cmp_results, require_all_false);

            for (unsigned int i = 0; i < repeat; ++i)
                counts.add(report_class);
        }

        const auto& e = j.at("expected");

        CHECK(counts.num_no_ref_val == e.at("no_ref_val").get<unsigned int>());
        CHECK(counts.num_unknown    == e.at("unknown").get<unsigned int>());
        CHECK(counts.num_correct    == e.at("correct").get<unsigned int>());
        CHECK(counts.num_false      == e.at("false").get<unsigned int>());

        auto prob = counts.probabilityFalse();

        if (e.at("probability_false").is_null())
        {
            CHECK(!prob.has_value());
        }
        else
        {
            REQUIRE(prob.has_value());
            CHECK(prob.value() == Approx(e.at("probability_false").get<double>()));
        }
    }
}
