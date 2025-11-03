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
#include <vector>

#include "job.h"
#include "json_fwd.hpp"

class JSONObjectParser;
class ASTERIXJSONParser;
class Buffer;

class JSONMappingJob : public Job
{
  public:
    JSONMappingJob(std::unique_ptr<nlohmann::json> data,
                   const std::vector<std::string>& data_record_keys,
                   unsigned int line_id,
                   const std::map<std::string, std::unique_ptr<JSONObjectParser>>& parsers); // TODO ugly

    JSONMappingJob(std::unique_ptr<nlohmann::json> data,
                   const std::vector<std::string>& data_record_keys,
                   unsigned int line_id,
                   const std::map<unsigned int, std::unique_ptr<ASTERIXJSONParser>>& parsers); // TODO ugly
    // json obj moved, mappings referenced
    virtual ~JSONMappingJob();

    size_t numMapped() const;
    size_t numNotMapped() const;
    size_t numErrors() const;
    size_t numCreated() const;

    std::map<std::string, std::shared_ptr<Buffer>> buffers() { return std::move(buffers_); }

    std::map<unsigned int, std::pair<size_t, size_t>> categoryMappedCounts() const;

protected:
    void run_impl() override;

private:
    std::map<unsigned int, std::pair<size_t, size_t>>
        category_mapped_counts_;  // mapped, not mapped
    size_t num_mapped_{0};        // number of parsed where a parse was successful
    size_t num_not_mapped_{0};    // number of parsed where no parse was successful
    size_t num_errors_{0};        // number of failed parses
    size_t num_created_{0};       // number of created objects from parsing

    std::unique_ptr<nlohmann::json> data_;
    const std::vector<std::string> data_record_keys_;
    unsigned int line_id_ {0};

    const std::map<std::string, std::unique_ptr<JSONObjectParser>>* json_parsers_;
    const std::map<unsigned int, std::unique_ptr<ASTERIXJSONParser>>* asterix_parsers_;

    std::map<std::string, std::shared_ptr<Buffer>> buffers_;

    void parseJSON();
    void parseASTERIX();
};
