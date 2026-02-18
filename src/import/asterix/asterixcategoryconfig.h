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

#include <string>

#include "configurable.h"

/**
*/
class ASTERIXCategoryConfig : public Configurable
{
public:
    struct Config
    {
        unsigned int category = 0;
        bool         decode   = false;
        std::string  edition  = "";
        std::string  ref      = "";
        std::string  spf      = ""; 
    };

    ASTERIXCategoryConfig(unsigned int category, 
                          const std::string& class_id,
                          const std::string& instance_id, 
                          Configurable* parent)
    :   Configurable(class_id, instance_id, parent)
    {
        config_.category = category;

        registerParameter("category", &config_.category, Config().category);
        registerParameter("decode"  , &config_.decode  , Config().decode  );
        registerParameter("edition" , &config_.edition , Config().edition );
        registerParameter("ref"     , &config_.ref     , Config().ref     );
        registerParameter("spf"     , &config_.spf     , Config().spf     );
    }

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id)
    {
    }

    unsigned int category() const { return config_.category; }

    bool decode() const { return config_.decode; }
    void decode(bool decode) { config_.decode = decode; }

    std::string edition() const { return config_.edition; }

    void edition(const std::string& edition) { config_.edition = edition; }

    std::string ref() const { return config_.ref; }

    void ref(const std::string& ref) { config_.ref = ref; }

    std::string spf() const { return config_.spf; }

    void spf(const std::string& spf) { config_.spf = spf; }

protected:
    virtual void checkSubConfigurables() {}

    void onConfigurationChanged(const std::vector<std::string>& changed_params) override {}

private:
    Config config_;
};
