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

#include "data_source.h"
#include "fft.h"
#include "asterix_decoding_config.h"

#include "json_fwd.hpp"

#include <memory>
#include <string>
#include <vector>

class Sector;

namespace context
{

/**
 * Groups sensor definitions, FFTs, ASTERIX decoding configs, and sectors
 * into a single named configuration context.
 *
 * Does NOT inherit from Configurable — uses its own file-based persistence
 * at ~/.compass/data_contexts/<name>/.
 */
class DBContext
{
public:
    DBContext();
    explicit DBContext(const std::string& name);

    // metadata
    const std::string& name() const { return name_; }
    void name(const std::string& name) { name_ = name; }

    const std::string& description() const { return description_; }
    void description(const std::string& desc) { description_ = desc; }

    const std::string& created() const { return created_; }
    void created(const std::string& ts) { created_ = ts; }

    const std::string& modified() const { return modified_; }
    void modified(const std::string& ts) { modified_ = ts; }

    // data sections
    std::vector<DataSource>& dataSources() { return data_sources_; }
    const std::vector<DataSource>& dataSources() const { return data_sources_; }
    void addOrReplaceDataSource(DataSource ds);

    std::vector<FFT>& ffts() { return ffts_; }
    const std::vector<FFT>& ffts() const { return ffts_; }

    std::vector<ASTERIXDecodingConfig>& asterixDecoding() { return asterix_decoding_; }
    const std::vector<ASTERIXDecodingConfig>& asterixDecoding() const { return asterix_decoding_; }

    std::vector<std::shared_ptr<Sector>>& sectors() { return sectors_; }
    const std::vector<std::shared_ptr<Sector>>& sectors() const { return sectors_; }

    // serialization (full context as one json object)
    nlohmann::json toJSON() const;
    static DBContext fromJSON(const nlohmann::json& j);

    bool operator==(const DBContext& other) const;
    bool operator!=(const DBContext& other) const { return !(*this == other); }

    // timestamp helpers
    static std::string currentTimestamp();

private:
    std::string name_;
    std::string description_;
    std::string created_;
    std::string modified_;

    std::vector<DataSource> data_sources_;
    std::vector<FFT> ffts_;
    std::vector<ASTERIXDecodingConfig> asterix_decoding_;
    std::vector<std::shared_ptr<Sector>> sectors_;
};

} // namespace context
