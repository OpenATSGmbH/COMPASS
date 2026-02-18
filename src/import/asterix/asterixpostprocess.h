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

    void postProcess(unsigned int category, nlohmann::json& record);

  protected:
    friend class ASTERIXImportTask;  // uses the members for config

    std::map<std::pair<unsigned int, unsigned int>, double> cat002_last_tod_period_;
    std::map<std::pair<unsigned int, unsigned int>, double> cat002_last_tod_;

    void postProcessCAT001(int sac, int sic, nlohmann::json& record);
    void postProcessCAT002(int sac, int sic, nlohmann::json& record);
    void postProcessCAT010(int sac, int sic, nlohmann::json& record);
    void postProcessCAT020(int sac, int sic, nlohmann::json& record);
    void postProcessCAT021(int sac, int sic, nlohmann::json& record);
    void postProcessCAT048(int sac, int sic, nlohmann::json& record);
    void postProcessCAT062(int sac, int sic, nlohmann::json& record);
};
