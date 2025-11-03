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

#include <memory>

#include "job.h"
#include "json_fwd.hpp"

class ASTERIXPostProcess;

class JSONParseJob : public Job
{
  public:
    JSONParseJob(std::vector<std::string> objects, const std::string& current_schema,
                 ASTERIXPostProcess& post_process);  // is moved from objects
    virtual ~JSONParseJob();

    std::unique_ptr<nlohmann::json> jsonObjects();  // for move operation

    size_t objectsParsed() const;
    size_t parseErrors() const;

  protected:
    void run_impl() override;

  private:
    std::vector<std::string> objects_;
    std::string current_schema_;
    std::unique_ptr<nlohmann::json> json_objects_;

    ASTERIXPostProcess& post_process_;

    size_t objects_parsed_{0};
    size_t parse_errors_{0};

    void checkCAT001SacSics(nlohmann::json& data_block);
};
