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

#include <QObject>
#include "traced_assert.h"

#include "configurable.h"

class QWidget;
class QLineEdit;
class QLabel;

class DBFilter;

namespace dbContent
{
class Variable;
class MetaVariable;
}


class DBFilterCondition : public QObject, public Configurable
{
    Q_OBJECT

private slots:
    void valueChanged();

signals:
    void possibleFilterChange();

public:
    DBFilterCondition(const std::string& class_id, const std::string& instance_id,
                      DBFilter* filter_parent);
    virtual ~DBFilterCondition();

    void invert();
    bool filters(const std::string& dbcontent_name);
    std::string getConditionString(const std::string& dbcontent_name, bool& first);

    QLabel* getLabel()
    {
        traced_assert(label_);
        return label_;
    }
    QLineEdit* getEdit()
    {
        traced_assert(edit_);
        return edit_;
    }

    void update();

    bool getChanged() { return changed_; }
    void setChanged(bool changed) { changed_ = changed; }

    std::string getVariableName() const;
    void setVariableName(const std::string& variable_name);

    bool hasVariable (const std::string& dbcontent_name);
    dbContent::Variable& variable (const std::string& dbcontent_name);

    bool getAbsoluteValue() { return absolute_value_; }
    void setAbsoluteValue(bool abs) { absolute_value_ = abs; }

    std::string getOperator() { return operator_; }
    void setOperator(std::string operator_val) { operator_ = operator_val; }

    std::string getValue() { return value_; }
    void setValue(const std::string& value);

    std::string getResetValue() { return reset_value_; }
    void setResetValue(std::string reset_value) { reset_value_ = reset_value; }

    void reset();

    bool valueInvalid() const { return value_invalid_; }
    bool usable() const { return usable_; }

    bool getDisplayInstanceId() const;

private:
    DBFilter* filter_parent_{nullptr};
    std::string operator_;
    bool op_and_{true};
    bool absolute_value_{false};
    std::string value_;
    std::string reset_value_;
    std::string variable_dbcontent_name_;
    std::string variable_name_;
    bool display_instance_id_ {false};

    bool usable_{true};
    bool changed_{true};
    bool value_invalid_{false};

    QLineEdit* edit_{nullptr};
    QLabel* label_{nullptr};

    // transformed val, null contained
    std::pair<std::string, bool> getTransformedValue(const std::string& untransformed_value,
                                                     dbContent::Variable* variable);
    bool checkValueInvalid(const std::string& new_value);
};
