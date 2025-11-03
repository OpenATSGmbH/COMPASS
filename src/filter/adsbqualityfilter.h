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

#include "dbfilter.h"

class ADSBQualityFilter : public DBFilter
{
public:
    ADSBQualityFilter(const std::string& class_id, const std::string& instance_id,
                      Configurable* parent);
    virtual ~ADSBQualityFilter();

    virtual std::string getConditionString(const std::string& dbcontent_name, bool& first) override;

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id) override;

    virtual bool filters(const std::string& dbcont_name) override;
    virtual void reset() override;

    virtual void saveViewPointConditions (nlohmann::json& filters) override;
    virtual void loadViewPointConditions (const nlohmann::json& filters) override;

    bool useV0() const;
    void useV0(bool value);

    bool useV1() const;
    void useV1(bool value);

    bool useV2() const;
    void useV2(bool value);

    // nucp
    bool useMinNUCP() const;
    void useMinNUCP(bool value);

    unsigned int minNUCP() const;
    void minNUCP(unsigned int value);

    bool useMaxNUCP() const;
    void useMaxNUCP(bool value);

    unsigned int maxNUCP() const;
    void maxNUCP(unsigned int value);

    // nic
    bool useMinNIC() const;
    void useMinNIC(bool value);

    unsigned int minNIC() const;
    void minNIC(unsigned int value);

    bool useMaxNIC() const;
    void useMaxNIC(bool value);

    unsigned int maxNIC() const;
    void maxNIC(unsigned int value);

    // nacp
    bool useMinNACp() const;
    void useMinNACp(bool value);

    unsigned int minNACp() const;
    void minNACp(unsigned int value);

    bool useMaxNACp() const;
    void useMaxNACp(bool value);

    unsigned int maxNACp() const;
    void maxNACp(unsigned int value);

    // sil v1
    bool useMinSILv1() const;
    void useMinSILv1(bool value);

    unsigned int minSILv1() const;
    void minSILv1(unsigned int value);

    bool useMaxSILv1() const;
    void useMaxSILv1(bool value);

    unsigned int maxSILv1() const;
    void maxSILv1(unsigned int value);

    // sil v2
    bool useMinSILv2() const;
    void useMinSILv2(bool value);

    unsigned int minSILv2() const;
    void minSILv2(unsigned int value);

    bool useMaxSILv2() const;
    void useMaxSILv2(bool value);

    unsigned int maxSILv2() const;
    void maxSILv2(unsigned int value);

protected:
    bool use_v0_ {false};
    bool use_v1_ {false};
    bool use_v2_ {false};

    // nucp
    bool use_min_nucp_ {false};
    unsigned int min_nucp_ {0};
    bool use_max_nucp_ {false};
    unsigned int max_nucp_ {0};

    // nic
    bool use_min_nic_ {false};
    unsigned int min_nic_ {0};
    bool use_max_nic_ {false};
    unsigned int max_nic_ {0};

    // nacp
    bool use_min_nacp_ {false};
    unsigned int min_nacp_ {0};
    bool use_max_nacp_ {false};
    unsigned int max_nacp_ {0};

    // sil v1
    bool use_min_sil_v1_ {false};
    unsigned int min_sil_v1_ {0};
    bool use_max_sil_v1_ {false};
    unsigned int max_sil_v1_ {0};

    // sil v2
    bool use_min_sil_v2_ {false};
    unsigned int min_sil_v2_ {0};
    bool use_max_sil_v2_ {false};
    unsigned int max_sil_v2_ {0};

    virtual void checkSubConfigurables();
    virtual DBFilterWidget* createWidget() override;
};
