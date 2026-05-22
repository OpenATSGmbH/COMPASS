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

#include <string>

namespace context
{

/**
 * ASTERIX decoding configuration for a single category.
 * Controls whether a category is decoded and which edition/REF/SPF to use.
 */
class ASTERIXDecodingConfig
{
public:
    ASTERIXDecodingConfig();
    ASTERIXDecodingConfig(unsigned int category,
                          const std::string& edition,
                          const std::string& ref = "",
                          const std::string& spf = "");

    unsigned int category() const { return category_; }
    void category(unsigned int cat) { category_ = cat; }

    std::string edition() const { return edition_; }
    void edition(const std::string& edition) { edition_ = edition; }

    std::string ref() const { return ref_; }
    void ref(const std::string& ref) { ref_ = ref; }

    std::string spf() const { return spf_; }
    void spf(const std::string& spf) { spf_ = spf; }

    nlohmann::json toJSON() const;
    static ASTERIXDecodingConfig fromJSON(const nlohmann::json& j);

    bool operator==(const ASTERIXDecodingConfig& other) const;
    bool operator!=(const ASTERIXDecodingConfig& other) const { return !(*this == other); }

private:
    unsigned int category_{0};
    std::string edition_;
    std::string ref_;
    std::string spf_;
};

} // namespace context
