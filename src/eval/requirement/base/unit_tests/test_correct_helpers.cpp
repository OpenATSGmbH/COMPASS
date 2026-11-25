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

#include "requirement/base/correcthelpers.h"
#include "results/base/detailcode.h"

#include "json.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>

using namespace EvaluationRequirement;
using namespace EvaluationRequirement::CorrectHelpers;

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
        + "/src/eval/requirement/base/unit_tests/correct_cases/";
}

} // namespace

TEST_CASE("CorrectHelpers::classify of one value", "[eval][correct_helpers]")
{
    for (bool require_all_correct : { false, true })
    {
        CHECK(classify({ ValueComparisonResult::Unknown_NoRefData }, require_all_correct) == ReportClass::NoRefValue);
        CHECK(classify({ ValueComparisonResult::Unknown_NoTstData }, require_all_correct) == ReportClass::NotCorrect);
        CHECK(classify({ ValueComparisonResult::Same },              require_all_correct) == ReportClass::Correct);
        CHECK(classify({ ValueComparisonResult::Different },         require_all_correct) == ReportClass::NotCorrect);
    }

    CHECK(classify({}, false) == ReportClass::NoRefValue);
    CHECK(classify({}, true)  == ReportClass::NoRefValue);
}

TEST_CASE("CorrectHelpers cases from JSON", "[eval][correct_helpers]")
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

        bool require_all_correct = j.value("require_all_correct", false);

        unsigned int num_no_ref_val {0}, num_correct {0}, num_not_correct {0};

        for (const auto& report : j.at("reports"))
        {
            unsigned int repeat = report.value("repeat", 1u);

            std::vector<ValueComparisonResult> cmp_results;
            for (const auto& cmp : report.at("cmp"))
                cmp_results.push_back(cmpFromString(cmp.get<std::string>()));

            auto report_class = classify(cmp_results, require_all_correct);

            if (report.contains("class"))
            {
                INFO("report " << report.dump());
                std::string expected_class = report.at("class").get<std::string>();
                CHECK((report_class == ReportClass::NoRefValue ? "no_ref_val"
                       : report_class == ReportClass::Correct  ? "correct" : "not_correct") == expected_class);
            }

            for (unsigned int i = 0; i < repeat; ++i)
            {
                if (report_class == ReportClass::NoRefValue)
                    ++num_no_ref_val;
                else if (report_class == ReportClass::Correct)
                    ++num_correct;
                else
                    ++num_not_correct;
            }
        }

        const auto& e = j.at("expected");

        CHECK(num_no_ref_val  == e.at("no_ref_val").get<unsigned int>());
        CHECK(num_correct     == e.at("correct").get<unsigned int>());
        CHECK(num_not_correct == e.at("not_correct").get<unsigned int>());

        // PID as in SingleCorrectBase: correct / (correct + not correct)
        unsigned int num_total = num_correct + num_not_correct;

        if (e.at("probability").is_null())
        {
            CHECK(num_total == 0);
        }
        else
        {
            REQUIRE(num_total > 0);
            CHECK((double)num_correct / (double)num_total == Approx(e.at("probability").get<double>()));
        }
    }
}

TEST_CASE("comparisonCode names the failed value", "[eval][correct_helpers]")
{
    namespace Value = EvaluationRequirementResult::DetailCode::Value;

    CHECK(comparisonCode(ValueComparisonResult::Different, Value::AcidDiffers, Value::AcidMissing) == Value::AcidDiffers);
    CHECK(comparisonCode(ValueComparisonResult::Unknown_NoTstData, Value::AcidDiffers, Value::AcidMissing) == Value::AcidMissing);
    CHECK(comparisonCode(ValueComparisonResult::Same, Value::AcidDiffers, Value::AcidMissing) == 0);
    CHECK(comparisonCode(ValueComparisonResult::Unknown_NoRefData, Value::AcidDiffers, Value::AcidMissing) == 0);

    // a correct report can name a failed value, if one judged value suffices
    std::vector<ValueComparisonResult> cmp { ValueComparisonResult::Same, ValueComparisonResult::Unknown_NoTstData };
    CHECK(classify(cmp, false) == ReportClass::Correct);
    CHECK((comparisonCode(cmp[ 0 ], Value::AcidDiffers, Value::AcidMissing) |
           comparisonCode(cmp[ 1 ], Value::M3ADiffers, Value::M3AMissing)) == Value::M3AMissing);
}
