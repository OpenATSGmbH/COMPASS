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

#include <QColor>

#include <map>
#include <memory>
#include <string>
#include <vector>

class Sector;

namespace context
{

/**
 * User-editable color configuration attached to a DBContext.
 * Preference drives the band used when auto-generating new DS base colors.
 */
struct ContextColors
{
    enum class Preference { Light, Dark };

    Preference preference {Preference::Light};
    std::map<std::string, QColor> ds_type_colors;    // populated with defaults at context creation
    std::map<std::string, QColor> dbcontent_colors;  // populated with defaults at context creation

    /// returns a ContextColors with the default DSType and DBContent palettes.
    static ContextColors withDefaults();

    nlohmann::json toJSON() const;
    static ContextColors fromJSON(const nlohmann::json& j);

    bool operator==(const ContextColors& other) const;
    bool operator!=(const ContextColors& other) const { return !(*this == other); }
};

/**
 * Groups sensor definitions, FFTs, ASTERIX decoding configs, and sectors
 * into a single named configuration context.
 *
 * Does NOT inherit from Configurable - uses its own file-based persistence
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
    // Keyed by ds_id. Callers should not cache DataSource* across event-loop
    // turns: erase/clear/wholesale-replace can happen via edit/merge dialogs
    // and command handlers, and the previous cached-pointer scheme dangled.
    // Look up by ds_id every time (O(log N), ~200 ns).
    std::map<unsigned int, DataSource>& dataSources() { return data_sources_; }
    const std::map<unsigned int, DataSource>& dataSources() const { return data_sources_; }
    void addOrReplaceDataSource(DataSource ds);

    std::vector<FFT>& ffts() { return ffts_; }
    const std::vector<FFT>& ffts() const { return ffts_; }

    std::vector<ASTERIXDecodingConfig>& asterixDecoding() { return asterix_decoding_; }
    const std::vector<ASTERIXDecodingConfig>& asterixDecoding() const { return asterix_decoding_; }

    std::vector<std::shared_ptr<Sector>>& sectors() { return sectors_; }
    const std::vector<std::shared_ptr<Sector>>& sectors() const { return sectors_; }

    // colors
    ContextColors& colors() { return colors_; }
    const ContextColors& colors() const { return colors_; }
    void colors(const ContextColors& colors) { colors_ = colors; }

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

    std::map<unsigned int, DataSource> data_sources_;
    std::vector<FFT> ffts_;
    std::vector<ASTERIXDecodingConfig> asterix_decoding_;
    std::vector<std::shared_ptr<Sector>> sectors_;

    ContextColors colors_ {ContextColors::withDefaults()};
};

} // namespace context
