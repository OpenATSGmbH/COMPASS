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

#include "eval/requirement/base/baseconfig.h"
#include "eval/requirement/group.h"
#include "eval/requirement/base/comparisontype.h"
#include "logger.h"

#include "task/result/report/report.h"
#include "task/result/report/section.h"
#include "task/result/report/sectioncontenttable.h"

#include <QFormLayout>
#include <QLineEdit>

using namespace std;

namespace EvaluationRequirement
{

std::string comparisonTypeString(COMPARISON_TYPE type)
{
    if (type == LESS_THAN)
        return "<";
    else if (type == LESS_THAN_OR_EQUAL)
        return "<=";
    else if (type == GREATER_THAN)
        return ">";
    else if (type == GREATER_THAN_OR_EQUAL)
        return ">=";
    else
        throw std::runtime_error("EvaluationRequirement: comparisonTypeString: unknown type "
                                 + std::to_string((unsigned int) type));
}

std::string comparisonTypeLongString(COMPARISON_TYPE type)
{
    if (type == LESS_THAN)
        return "Less Than (<)";
    else if (type == LESS_THAN_OR_EQUAL)
        return "Less Than or Equal (<=)";
    else if (type == GREATER_THAN)
        return "Greater Than (>)";
    else if (type == GREATER_THAN_OR_EQUAL)
        return "Greater Than or Equal (>=)";
    else
        throw std::runtime_error("EvaluationRequirement: comparisonTypeString: unknown type "
                                 + std::to_string((unsigned int) type));
}

BaseConfig::BaseConfig(
        const std::string& class_id, const std::string& instance_id,
        Group& group, EvaluationStandard& standard, EvaluationCalculator& calculator)
    : Configurable(class_id, instance_id, &group), EvaluationStandardTreeItem(&group),
      group_(group), standard_(standard), calculator_(calculator)
{
    registerParameter("use", &use_, true);
    registerParameter("name", &name_, std::string());
    registerParameter("short_name", &short_name_, std::string());
    registerParameter("comment", &comment_, std::string());

    traced_assert(name_.size());
    traced_assert(short_name_.size());

    createSubConfigurables();
}

BaseConfig::~BaseConfig()
{
}

void BaseConfig::use(bool ok)
{
    use_ = ok;
}

bool BaseConfig::used() const
{
    return use_;
}

bool BaseConfig::checkable() const
{
    return true;
}

void BaseConfig::generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id)
{
    traced_assert(false);
}

std::string BaseConfig::name() const
{
    return name_;
}

void BaseConfig::checkSubConfigurables()
{
}

BaseConfigWidget* BaseConfig::widget()
{
    return createWidget();
}

std::string BaseConfig::comment() const
{
    return comment_;
}

void BaseConfig::comment(const std::string &comment)
{
    comment_ = comment;
}

BaseConfigWidget* BaseConfig::createWidget()
{
    return new BaseConfigWidget(*this);
}

EvaluationStandardTreeItem* BaseConfig::child(int row)
{
    return nullptr;
}

int BaseConfig::childCount() const
{
    return 0;
}

int BaseConfig::columnCount() const
{
    return 1;
}

QVariant BaseConfig::data(int column) const
{
    traced_assert(column == 0);

    return name_.c_str();
}

int BaseConfig::row() const
{
    return 0;
}

void BaseConfig::name(const std::string& name)
{
    loginf << "value '" << name << "'";

    name_ = name;

    emit group_.configsChangedSignal();
}

void BaseConfig::shortName(const std::string& short_name)
{
    loginf << "value '" << short_name << "'";

    short_name_ = short_name;
}

std::string BaseConfig::shortName() const
{
    return short_name_;
}

void BaseConfig::addToReport (std::shared_ptr<ResultReport::Report> report)
{
    auto& section = report->getSection("Appendix:Requirements:"+group_.name()+":"+name_);

    section.addTable("req_table", 3, {"Name", "Comment", "Value"}, false);

    auto& table = section.getTable("req_table");

    table.addRow({"Name", "Requirement name", name_});
    table.addRow({"Short Name", "Requirement short name", short_name_});
    table.addRow({"Comment", "", comment_});
    table.addRow({"Group", "Group name", group_.name()});

    // prob & check type added in subclass
}

}
