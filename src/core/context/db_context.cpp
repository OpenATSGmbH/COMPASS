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

#include "db_context.h"

#include "sector.h"
#include "traced_assert.h"

#include <json.hpp>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

using namespace std;
using namespace nlohmann;

namespace context
{

DBContext::DBContext() = default;

DBContext::DBContext(const string& name)
    : name_(name)
    , created_(currentTimestamp())
    , modified_(created_)
{
}

json DBContext::toJSON() const
{
    json j;

    // metadata
    j["name"] = name_;
    j["description"] = description_;
    j["created"] = created_;
    j["modified"] = modified_;

    // data sources
    json ds_arr = json::array();
    for (const auto& ds : data_sources_)
        ds_arr.push_back(ds.toJSON());
    j["data_sources"] = ds_arr;

    // ffts
    json fft_arr = json::array();
    for (const auto& fft : ffts_)
        fft_arr.push_back(fft.toJSON());
    j["ffts"] = fft_arr;

    // asterix decoding
    json ast_arr = json::array();
    for (const auto& cfg : asterix_decoding_)
        ast_arr.push_back(cfg.toJSON());
    j["asterix_decoding"] = ast_arr;

    // sectors
    json sec_arr = json::array();
    for (const auto& sec : sectors_)
    {
        traced_assert(sec);
        sec_arr.push_back(sec->jsonData());
    }
    j["sectors"] = sec_arr;

    return j;
}

DBContext DBContext::fromJSON(const json& j)
{
    DBContext ctx;

    traced_assert(j.contains("name"));
    ctx.name_ = j.at("name");

    if (j.contains("description"))
        ctx.description_ = j.at("description");

    if (j.contains("created"))
        ctx.created_ = j.at("created");

    if (j.contains("modified"))
        ctx.modified_ = j.at("modified");

    // data sources
    if (j.contains("data_sources"))
    {
        for (const auto& ds_j : j.at("data_sources"))
            ctx.data_sources_.push_back(DataSource::fromJSON(ds_j));
    }

    // ffts
    if (j.contains("ffts"))
    {
        for (const auto& fft_j : j.at("ffts"))
            ctx.ffts_.push_back(FFT::fromJSON(fft_j));
    }

    // asterix decoding
    if (j.contains("asterix_decoding"))
    {
        for (const auto& cfg_j : j.at("asterix_decoding"))
            ctx.asterix_decoding_.push_back(ASTERIXDecodingConfig::fromJSON(cfg_j));
    }

    // sectors
    if (j.contains("sectors"))
    {
        for (const auto& sec_j : j.at("sectors"))
        {
            unsigned int id = sec_j.at("id");
            string name = sec_j.at("name");
            string layer_name = sec_j.at("layer_name");

            auto sector = make_shared<Sector>(id, name, layer_name, false);
            sector->readJSON(sec_j);
            ctx.sectors_.push_back(sector);
        }
    }

    return ctx;
}

bool DBContext::operator==(const DBContext& other) const
{
    if (name_ != other.name_ || description_ != other.description_)
        return false;

    if (data_sources_ != other.data_sources_)
        return false;

    if (ffts_ != other.ffts_)
        return false;

    if (asterix_decoding_ != other.asterix_decoding_)
        return false;

    if (sectors_.size() != other.sectors_.size())
        return false;

    for (size_t i = 0; i < sectors_.size(); ++i)
    {
        if (sectors_[i]->jsonData() != other.sectors_[i]->jsonData())
            return false;
    }

    return true;
}

string DBContext::currentTimestamp()
{
    auto now = chrono::system_clock::now();
    auto time_t_now = chrono::system_clock::to_time_t(now);
    struct tm tm_buf;
    gmtime_r(&time_t_now, &tm_buf);

    ostringstream oss;
    oss << put_time(&tm_buf, "%Y-%m-%dT%H:%M:%SZ");
    return oss.str();
}

} // namespace context
