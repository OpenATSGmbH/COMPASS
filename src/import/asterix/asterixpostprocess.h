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

#include "json_fwd.hpp"

class ASTERIXImportTask;

class ASTERIXPostProcess
{
  public:
    ASTERIXPostProcess();

    // flat format: post-process flat category data in-place, adding category/ds_id/line_id arrays
    void postProcessFlat(unsigned int category, unsigned int line_id,
                         nlohmann::json& flat_cat_data, size_t num_records);

  protected:
    friend class ASTERIXImportTask;  // uses the members for config

    // jASTERIX now handles CAT001 truncated time fix and writes "140.Time-of-Day" directly
    // std::map<std::pair<unsigned int, unsigned int>, double> cat002_last_tod_period_;
    // std::map<std::pair<unsigned int, unsigned int>, double> cat002_last_tod_;

    // void postProcessFlatCAT001(nlohmann::json& d, size_t num_records);
    // void postProcessFlatCAT002(nlohmann::json& d, size_t num_records);
    void postProcessFlatCAT020(nlohmann::json& d, size_t num_records);
    void postProcessFlatCAT062(nlohmann::json& d, size_t num_records);
};
