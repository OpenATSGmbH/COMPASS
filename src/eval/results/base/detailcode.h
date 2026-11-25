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

#include <cstdint>
#include <string>

struct ReportTableDefinition;

namespace EvaluationRequirementResult
{

/**
 * Reason bits of an evaluation detail, stored in the report table column "detail_code" and turned
 * into the comment text of the per-target sections (experimental_src/evaluation/readme_detail_codes.md). Bits 0 to 7 are the
 * common reasons below, bits from TypeBit are defined by each result type.
 */
namespace DetailCode
{

enum : std::uint32_t
{
    NoRefData       = 1u << 0,
    NoRefPosition   = 1u << 1,
    NoRefSpeed      = 1u << 2,
    RefInaccurate   = 1u << 4,
    ValueInvalid    = 1u << 5,

    NoRef           = NoRefData | NoRefPosition | NoRefSpeed
};

/// first bit index of the bits of a result type
const unsigned int TypeBit = 8;

/// bit of a result type, index counted from TypeBit
constexpr std::uint32_t typeBit(unsigned int index) { return 1u << (TypeBit + index); }

/// adds the texts of the common bits to the table definition
void addCommonTexts(ReportTableDefinition& def);

/// adds the text of a bit of a result type to the table definition
void addTypeText(ReportTableDefinition& def, std::uint32_t bit, const std::string& text);

/**
 * Type bits of the result types that compare a test value with the reference value
 * (Identification, Mode A and Mode C Correct, False and Present, Generic, Correct Periods). "Missing"
 * means the test report has no value, "differs" that the values differ.
 */
namespace Value
{

enum : std::uint32_t
{
    NoRefValue   = typeBit(0),
    NoTstValue   = typeBit(1),
    Differs      = typeBit(2), // value of a Generic requirement

    AcidDiffers  = typeBit(8),
    AcidMissing  = typeBit(9),
    AcadDiffers  = typeBit(10),
    AcadMissing  = typeBit(11),
    M3ADiffers   = typeBit(12),
    M3AMissing   = typeBit(13),
    ModeCDiffers = typeBit(14),
    ModeCMissing = typeBit(15)
};

/// adds the texts of all bits above
void addTexts(ReportTableDefinition& def);
/// adds the texts of the bits of the single values (ACID to Mode C)
void addValueTexts(ReportTableDefinition& def);

} // namespace Value

/**
 * Type bits of the gap rows (Detection, Correct Periods). The value bits of the invalid test
 * reports inside a gap (Value::AcidDiffers to Value::ModeCMissing) follow from type bit 8.
 */
namespace Gap
{

enum : std::uint32_t
{
    MissAtPeriodBegin        = typeBit(0), // from the period begin to the first valid test report
    MissInPeriod             = typeBit(1), // between two valid test reports
    MissAtPeriodEnd          = typeBit(2), // from the last valid test report to the period end
    PeriodWithoutValidUpdate = typeBit(3), // the whole period
    MissInStatusCycle        = typeBit(4), // expected period of the status message method
    NoRefValue               = typeBit(5)  // the valid test report at the gap had no reference value
};

/// adds the texts of the bits above
void addTexts(ReportTableDefinition& def);

} // namespace Gap

} // namespace DetailCode

} // namespace EvaluationRequirementResult
