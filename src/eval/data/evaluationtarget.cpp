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

#include "evaluationtarget.h"

#include "logger.h"

const std::string EvaluationTarget::KEY_INTEREST_FACTORS = "interest_factors";
const std::string EvaluationTarget::KEY_DBCONTENT_REF    = "dbcontent_ref";
const std::string EvaluationTarget::KEY_DBCONTENT_TEST   = "dbcontent_test";

const std::string EvaluationTarget::FieldInterestID      = "req_sum_id";
const std::string EvaluationTarget::FieldInterestFactor  = "factor";

/**
 */
EvaluationTarget::EvaluationTarget(unsigned int utn, 
                                   const nlohmann::json& info)
:   dbContent::Target(utn, info)
{
    if (info.contains(KEY_INTEREST_FACTORS))
    {
        bool ok = readInterestFactors(interest_factors_, info.at(KEY_INTEREST_FACTORS));
        if (!ok)
            logerr << "could not read interest factors from target info";
    }
}

/**
 */
EvaluationTarget::~EvaluationTarget() = default;

/**
 */
bool EvaluationTarget::readInterestFactors(InterestMap& ifactors, const nlohmann::json& j)
{
    ifactors.clear();

    if (!j.is_array())
        return false;

    for (const auto& j_if : j)
    {
        if (!j_if.is_object()               ||
            !j_if.contains(FieldInterestID) ||
            !j_if.contains(FieldInterestFactor))
            return false;

        Evaluation::RequirementSumResultID id;
        if (!id.fromJSON(j_if[ FieldInterestID ]))
            return false;

        double factor = j_if[ FieldInterestFactor ];

        ifactors[ id ] = factor;
    }

    return true;
}

/**
 */
void EvaluationTarget::writeInterestFactors(const InterestMap& ifactors, nlohmann::json& j)
{
    j = nlohmann::json::array();

    for (const auto& ifactor : ifactors)
    {
        nlohmann::json j_if;
        j_if[ FieldInterestID     ] = ifactor.first.toJSON();
        j_if[ FieldInterestFactor ] = ifactor.second;

        j.push_back(j_if);
    }
}

/**
 */
void EvaluationTarget::interestFactors(const InterestMap& interests)
{
    interest_factors_ = interests;

    writeInterestFactors(interest_factors_, info_[ KEY_INTEREST_FACTORS ]);
}

/**
 */
const EvaluationTarget::InterestMap& EvaluationTarget::interestFactors() const
{
    return interest_factors_;
}

/**
 */
double EvaluationTarget::totalInterest(const InterestEnabledFunc& enabled_func,
                                       size_t* num_contributors) const
{
    double sum = 0.0;

    if (num_contributors)
        *num_contributors = 0;

    for (const auto& ifactor : interest_factors_)
    {
        if (!enabled_func || enabled_func(ifactor.first))
        {
            sum += ifactor.second;

            if (num_contributors)
                *num_contributors += 1;
        }
    }

    return sum;
}

/**
 */
void EvaluationTarget::dbContentRef(const std::string& db_content)
{
    info_[ KEY_DBCONTENT_REF ] = db_content;
}

/**
 */
std::string EvaluationTarget::dbContentRef() const
{
    return info_[ KEY_DBCONTENT_REF ];
}

/**
 */
void EvaluationTarget::dbContentTest(const std::string& db_content)
{
    info_[ KEY_DBCONTENT_TEST ] = db_content;
}

/**
 */
std::string EvaluationTarget::dbContentTest() const
{
    return info_[ KEY_DBCONTENT_TEST ];
}

/**
 */
unsigned int EvaluationTarget::refCount() const
{
    auto dbc = dbContentRef();
    return dbc.empty() ? 0 : dbContentCount(dbc);
}

/**
 */
unsigned int EvaluationTarget::testCount() const
{
    auto dbc = dbContentTest();
    return dbc.empty() ? 0 : dbContentCount(dbc);
}
