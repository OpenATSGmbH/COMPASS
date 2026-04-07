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

#include "asterix_decoding_config.h"

#include "traced_assert.h"

#include <json.hpp>

using namespace std;
using namespace nlohmann;

namespace context
{

ASTERIXDecodingConfig::ASTERIXDecodingConfig() = default;

ASTERIXDecodingConfig::ASTERIXDecodingConfig(unsigned int category, bool decode,
                                             const string& edition,
                                             const string& ref,
                                             const string& spf)
    : category_(category)
    , decode_(decode)
    , edition_(edition)
    , ref_(ref)
    , spf_(spf)
{
}

json ASTERIXDecodingConfig::toJSON() const
{
    json j;

    j["category"] = category_;
    j["decode"] = decode_;
    j["edition"] = edition_;

    if (!ref_.empty())
        j["ref"] = ref_;

    if (!spf_.empty())
        j["spf"] = spf_;

    return j;
}

ASTERIXDecodingConfig ASTERIXDecodingConfig::fromJSON(const json& j)
{
    ASTERIXDecodingConfig cfg;

    traced_assert(j.contains("category"));
    cfg.category_ = j.at("category");

    traced_assert(j.contains("decode"));
    cfg.decode_ = j.at("decode");

    traced_assert(j.contains("edition"));
    cfg.edition_ = j.at("edition");

    if (j.contains("ref"))
        cfg.ref_ = j.at("ref");

    if (j.contains("spf"))
        cfg.spf_ = j.at("spf");

    return cfg;
}

bool ASTERIXDecodingConfig::operator==(const ASTERIXDecodingConfig& other) const
{
    return category_ == other.category_
        && decode_ == other.decode_
        && edition_ == other.edition_
        && ref_ == other.ref_
        && spf_ == other.spf_;
}

} // namespace context
