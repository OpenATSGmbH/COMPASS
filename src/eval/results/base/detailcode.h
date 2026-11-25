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
 * into the comment text of the per-target sections (readme_detail_codes.md). Bits 0 to 7 are the
 * common reasons below, bits from TypeBit are defined by each result type.
 */
namespace DetailCode
{

enum : std::uint32_t
{
    NoRefData       = 1u << 0,
    NoRefPosition   = 1u << 1,
    NoRefSpeed      = 1u << 2,
    OutsideSector   = 1u << 3,
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

} // namespace DetailCode

} // namespace EvaluationRequirementResult
