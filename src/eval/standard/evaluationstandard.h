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
#include "evaluationstandardrootitem.h"
#include "eval/results/report/rootitem.h"

#include <QObject>

#include <memory>

class EvaluationCalculator;
class Group;
class EvaluationStandardWidget;
class EvaluationStandardTreeModel;

namespace ResultReport
{
    class Report;
}

class EvaluationStandard : public QObject, public Configurable, public EvaluationStandardTreeItem
{
    Q_OBJECT

 signals:
    void configChangedSignal();
//     void selectionChanged();

 public slots:
     void groupsChangedSlot();

public:
    EvaluationStandard(const std::string& class_id, const std::string& instance_id, EvaluationCalculator& calculator);
    virtual ~EvaluationStandard();

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id) override;

    std::string name() const;
    void name(const std::string &name);

    bool hasGroup (const std::string& name);
    void addGroup (const std::string& name);
    Group& group (const std::string& name);
    void removeGroup (const std::string& name);

    using EvaluationRequirementGroupIterator =
    typename std::vector<std::unique_ptr<Group>>::iterator;
    using EvaluationRequirementGroupConstIterator =
    typename std::vector<std::unique_ptr<Group>>::const_iterator;

    EvaluationRequirementGroupIterator begin() { return groups_.begin(); }
    EvaluationRequirementGroupIterator end() { return groups_.end(); }
    EvaluationRequirementGroupConstIterator begin() const { return groups_.begin(); }
    EvaluationRequirementGroupConstIterator end() const { return groups_.end(); }
    unsigned int size () { return groups_.size(); };

    EvaluationStandardWidget* widget();

    virtual EvaluationStandardTreeItem *child(int row) override;
    virtual int childCount() const override;
    virtual int columnCount() const override;
    virtual QVariant data(int column) const override;
    virtual int row() const override;

    EvaluationStandardRootItem& rootItem();

    void addToReport (std::shared_ptr<ResultReport::Report> report);

    std::set<std::string> getAllRequirementNames() const;

    float referenceMaxTimeDiff() const { return reference_max_time_diff_; }
    void referenceMaxTimeDiff(float value) { reference_max_time_diff_= value; }
    float referenceMinAccuracy() const { return reference_min_accuracy_; }
    void referenceMinAccuracy(float value) { reference_min_accuracy_= value; }
    
protected:
    EvaluationCalculator& calculator_;
    std::string name_;

    EvaluationStandardRootItem root_item_;

    float reference_max_time_diff_ {5};
    float reference_min_accuracy_ {10};

    std::vector<std::unique_ptr<Group>> groups_;

    virtual void checkSubConfigurables() override;
};

