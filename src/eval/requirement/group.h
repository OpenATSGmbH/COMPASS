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

#include "configurable.h"
#include "evaluationstandardtreeitem.h"

#include <QObject>
#include <QMenu>

#include <memory>

class EvaluationStandard;
class EvaluationCalculator;

namespace EvaluationRequirement {
class BaseConfig;
}

class Group : public QObject, public Configurable, public EvaluationStandardTreeItem
{
    Q_OBJECT

signals:
     void configsChangedSignal();
//     void selectionChanged();

// public slots:
//     void deleteGroupSlot();
//     void addRequirementSlot();
//     void deleteRequirementSlot();

public:
    Group(nlohmann::json& config,
          EvaluationStandard* parent);

    EvaluationStandard* parentConfigurable() const;
    virtual ~Group();

    void generateSubConfigurable(nlohmann::json& child_json) override;

    void use(bool ok) override;
    bool used() const override;
    bool checkable() const override;

    std::string name() const;

    bool hasRequirementConfig (const std::string& name);
    void addRequirementConfig (const std::string& class_id, const std::string& name, const std::string& short_name);
    EvaluationRequirement::BaseConfig& requirementConfig (const std::string& name);
    void removeRequirementConfig (const std::string& name);

    using EvaluationRequirementConfigIterator =
    typename std::vector<std::unique_ptr<EvaluationRequirement::BaseConfig>>::iterator;

    EvaluationRequirementConfigIterator begin() { return configs_.begin(); }
    EvaluationRequirementConfigIterator end() { return configs_.end(); }
    unsigned int size () { return configs_.size(); };

    void useAll();
    void useNone();

    unsigned int numUsedRequirements() const;

    virtual EvaluationStandardTreeItem *child(int row) override;
    virtual int childCount() const override;
    virtual int columnCount() const override;
    virtual QVariant data(int column) const override;
    virtual int row() const override;

    static const std::map<std::string, std::string> requirement_type_mapping_;

    const std::vector<std::unique_ptr<EvaluationRequirement::BaseConfig>>& configs() const;

protected:
    EvaluationStandard& standard_;
    EvaluationCalculator& calculator_;

    bool        use_ = true;
    std::string name_;

    std::vector<std::unique_ptr<EvaluationRequirement::BaseConfig>> configs_;

    virtual void checkSubConfigurables() override;

    void sortConfigs();
};
