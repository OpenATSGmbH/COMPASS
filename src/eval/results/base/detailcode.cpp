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
    def.detail_code_texts[ 3 ] = "Outside sector";
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

} // namespace DetailCode
} // namespace EvaluationRequirementResult
