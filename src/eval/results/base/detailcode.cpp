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

#include "detailcode.h"
#include "reporttable.h"
#include "traced_assert.h"

namespace EvaluationRequirementResult
{
namespace DetailCode
{

/**
 */
void addCommonTexts(ReportTableDefinition& def)
{
    def.detail_code_texts[ 0 ] = "No reference data";
    def.detail_code_texts[ 1 ] = "No reference position";
    def.detail_code_texts[ 2 ] = "No reference speed";
    def.detail_code_texts[ 4 ] = "Inaccurate reference position";
    def.detail_code_texts[ 5 ] = "Value not calculable";
}

/**
 */
void addTypeText(ReportTableDefinition& def, std::uint32_t bit, const std::string& text)
{
    traced_assert(bit != 0 && (bit & (bit - 1)) == 0); // exactly one bit

    unsigned int index = 0;
    while (!(bit & (1u << index)))
        ++index;

    traced_assert(index >= TypeBit);

    def.detail_code_texts[ index ] = text;
}

namespace Value
{

/**
 */
void addTexts(ReportTableDefinition& def)
{
    addTypeText(def, NoRefValue, "No reference value");
    addTypeText(def, NoTstValue, "Value missing in the test report");
    addTypeText(def, Differs, "Value differs");

    addValueTexts(def);
}

/**
 */
void addValueTexts(ReportTableDefinition& def)
{
    addTypeText(def, AcidDiffers, "ACID differs");
    addTypeText(def, AcidMissing, "ACID missing in the test report");
    addTypeText(def, AcadDiffers, "ACAD differs");
    addTypeText(def, AcadMissing, "ACAD missing in the test report");
    addTypeText(def, M3ADiffers, "M3A differs");
    addTypeText(def, M3AMissing, "M3A missing in the test report");
    addTypeText(def, ModeCDiffers, "Mode C differs more than the tolerance");
    addTypeText(def, ModeCMissing, "Mode C missing in the test report");
}

} // namespace Value

namespace Gap
{

/**
 */
void addTexts(ReportTableDefinition& def)
{
    addTypeText(def, MissAtPeriodBegin, "Miss at the start of the reference period");
    addTypeText(def, MissInPeriod, "Miss inside the reference period");
    addTypeText(def, MissAtPeriodEnd, "Miss at the end of the reference period");
    addTypeText(def, PeriodWithoutValidUpdate, "No valid test report in the reference period");
    addTypeText(def, MissInStatusCycle, "No test report in the reported update cycles");
    addTypeText(def, NoRefValue, "No reference value at the valid test report of the gap");
}

} // namespace Gap

} // namespace DetailCode
} // namespace EvaluationRequirementResult
