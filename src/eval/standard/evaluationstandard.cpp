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

#include "evaluationstandard.h"
#include "evaluationmanager.h"
#include "eval/requirement/group.h"
#include "evaluationstandardwidget.h"
#include "evaluationstandardtreemodel.h"
#include "logger.h"

#include "task/result/report/report.h"
#include "task/result/report/section.h"
#include "task/result/report/sectioncontenttext.h"
#include "task/result/report/sectioncontenttable.h"

#include "eval/requirement/group.h"
#include "eval/requirement/base/baseconfig.h"

#include <QTreeView>

using namespace std;
using namespace EvaluationResultsReport;

EvaluationStandard::EvaluationStandard(const std::string& class_id, 
                                       const std::string& instance_id,
                                       EvaluationCalculator& calculator)
    : Configurable(class_id, instance_id, &calculator), 
      EvaluationStandardTreeItem(&root_item_), 
      calculator_(calculator),
      root_item_(*this)
{
    registerParameter("name", &name_, std::string());

    registerParameter("reference_max_time_diff", &reference_max_time_diff_, reference_max_time_diff_);
    registerParameter("reference_min_accuracy", &reference_min_accuracy_, reference_min_accuracy_);

    traced_assert(name_.size());

    createSubConfigurables();
}

EvaluationStandard::~EvaluationStandard()
{
}

void EvaluationStandard::generateSubConfigurable(const std::string& class_id,
                                                 const std::string& instance_id)
{
    if (class_id == "EvaluationRequirementGroup")
    {
        Group* group = new Group(class_id, instance_id, *this, calculator_);

        logdbg << "adding group " << group->name();

        traced_assert(!hasGroup(group->name()));

        groups_.emplace_back(group);

        connect (group, &Group::configsChangedSignal, this, &EvaluationStandard::groupsChangedSlot);
        // connect (group, &Group::selectionChanged, this, &EvaluationStandard::selectionChanged);
    }
    else
        throw std::runtime_error("EvaluationStandard: generateSubConfigurable: unknown class_id " +
                                 class_id);
}

std::string EvaluationStandard::name() const
{
    return name_;
}

bool EvaluationStandard::hasGroup (const std::string& name)
{
    auto iter = std::find_if(groups_.begin(), groups_.end(),
                             [&name](const unique_ptr<Group>& x) { return x->name() == name;});

    return iter != groups_.end();
}

void EvaluationStandard::addGroup (const std::string& name)
{
    traced_assert(!hasGroup(name));

    // if (widget_)
    //     beginModelReset();

    std::string instance = "EvaluationRequirementGroup" + name + "0";

    auto config = Configuration::create("EvaluationRequirementGroup", instance);
    config->addParameter<std::string>("name", name);

    generateSubConfigurableFromConfig(std::move(config));

    traced_assert(hasGroup(name));

    // if (widget_)
    //     endModelReset();
}

Group& EvaluationStandard::group (const std::string& name)
{
    traced_assert(hasGroup(name));

    auto iter = std::find_if(groups_.begin(), groups_.end(),
                             [&name](const unique_ptr<Group>& x) { return x->name() == name;});

    traced_assert(iter != groups_.end());

    return *iter->get();
}

void EvaluationStandard::removeGroup (const std::string& name)
{
    traced_assert(hasGroup(name));

    auto iter = std::find_if(groups_.begin(), groups_.end(),
                             [&name](const unique_ptr<Group>& x) { return x->name() == name;});

    traced_assert(iter != groups_.end());

    groups_.erase(iter);

    // if (widget_)
    //     endModelReset();
}

EvaluationStandardWidget* EvaluationStandard::widget()
{
    return new EvaluationStandardWidget(*this);
}

void EvaluationStandard::checkSubConfigurables()
{
}

EvaluationStandardTreeItem* EvaluationStandard::child(int row)
{
    if (row < 0 || row >= (int)groups_.size())
        return nullptr;

    auto group_it = groups_.begin();

    std::advance(group_it, row);

    traced_assert(group_it != groups_.end());

    return group_it->get();
}

int EvaluationStandard::childCount() const
{
    return groups_.size();
}

int EvaluationStandard::columnCount() const
{
    return 1;
}

QVariant EvaluationStandard::data(int column) const
{
    traced_assert(column == 0);

    return name_.c_str();
}

int EvaluationStandard::row() const
{
    return 0;
}

EvaluationStandardRootItem& EvaluationStandard::rootItem()
{
    return root_item_;
}


// void EvaluationStandard::beginModelReset()
// {
//     widget()->model().beginReset();
// }

// void EvaluationStandard::endModelReset()
// {
//     widget()->model().endReset();
//     widget()->expandAll();
// }

void EvaluationStandard::name(const std::string &name)
{
    name_ = name;
}


void EvaluationStandard::groupsChangedSlot()
{
    loginf;

    emit configChangedSignal();
}

void EvaluationStandard::addToReport (std::shared_ptr<ResultReport::Report> report)
{
    auto& section = report->getSection("Overview:Standard");

    // reqs overview

    section.addTable("req_overview_table", 4, {"Short Name", "Name", "Group", "Type", }, false);

    auto& req_table = section.getTable("req_overview_table");

    for (auto& std_it : groups_)
    {
        if (!std_it->used())
            continue;

        for (auto& req_it : *std_it)
        {
            if (!req_it->used())
                continue;

            req_table.addRow({req_it->shortName(), req_it->name(), std_it->name(),
                              Group::requirement_type_mapping_.at(req_it->classId())});

            req_it->addToReport(report);
        }
    }
}

std::set<std::string> EvaluationStandard::getAllRequirementNames() const
{
    std::set<std::string> names;

    for (auto& std_it : groups_)
    {
        for (auto& req_it : *std_it)
        {
            if (!names.count(req_it->name()))
                names.insert(req_it->name());
        }
    }

    return names;
}
