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

#include "evaluationstandardwidget.h"
#include "evaluationstandard.h"
#include "eval/requirement/group.h"
#include "eval/requirement/base/baseconfig.h"
#include "eval/requirement/base/baseconfigwidget.h"
#include "logger.h"

#include <QVBoxLayout>
#include <QLabel>
#include <QHBoxLayout>
#include <QStackedWidget>
#include <QScrollArea>
#include <QSplitter>
#include <QSettings>
#include <QInputDialog>
#include <QMessageBox>
#include <QFormLayout>

using namespace std;

EvaluationStandardWidget::EvaluationStandardWidget(EvaluationStandard& standard)
    : QWidget(), standard_(standard), standard_model_(standard)
{
    QVBoxLayout* main_layout = new QVBoxLayout();

    splitter_ = new QSplitter();
    splitter_->setOrientation(Qt::Horizontal);

    connect (&standard_, &EvaluationStandard::configChangedSignal,
            this, &EvaluationStandardWidget::standardConfigChangedSlot);

    tree_view_.reset(new QTreeView());
    tree_view_->setModel(&standard_model_);
    tree_view_->setRootIsDecorated(false);
    tree_view_->expandAll();

    tree_view_->setContextMenuPolicy(Qt::ContextMenuPolicy::CustomContextMenu);

    connect (tree_view_.get(), &QTreeView::customContextMenuRequested,
            this, &EvaluationStandardWidget::showContextMenu);
    connect (tree_view_.get(), &QTreeView::clicked,
            this, &EvaluationStandardWidget::itemClickedSlot);
    // connect (&standard_, &EvaluationStandard::selectionChanged,
    //         &standard_model_, &EvaluationStandardTreeModel::updateCheckStates);

    splitter_->addWidget(tree_view_.get());
    //req_layout->addWidget(tree_view_.get());

    // requirements stack
    QScrollArea* scroll_area = new QScrollArea();
    scroll_area->setWidgetResizable(true);

    requirements_widget_ = new QStackedWidget();

    scroll_area->setWidget(requirements_widget_);

    splitter_->addWidget(scroll_area);
    //req_layout->addWidget(scroll_area, 1);

    //qt hack: very big screen size x stretch 
    splitter_->setSizes({ 10000, 30000 });

    QSettings settings("COMPASS", ("EvalStandardWidget"+standard_.name()).c_str());
    if (settings.value("splitterSizes").isValid())
        splitter_->restoreState(settings.value("splitterSizes").toByteArray());

    //main_layout->addLayout(req_layout);
    main_layout->addWidget(splitter_);

    setContentsMargins(0, 0, 0, 0);
    setLayout(main_layout);

    // menu creation
    {
        QAction* add_action = menu_.addAction("Add Group");
        connect(add_action, &QAction::triggered, this, &EvaluationStandardWidget::addGroupSlot);
    }
}

EvaluationStandardWidget::~EvaluationStandardWidget()
{
    traced_assert(splitter_);

    QSettings settings("COMPASS", ("EvalStandardWidget"+standard_.name()).c_str());
    settings.setValue("splitterSizes", splitter_->saveState());
}

EvaluationStandardTreeModel& EvaluationStandardWidget::model()
{
    return standard_model_;
}

void EvaluationStandardWidget::showContextMenu(const QPoint& pos)
{
    traced_assert(tree_view_);

    QModelIndex index = tree_view_->indexAt(pos);
    if (!index.isValid())
        return;

    EvaluationStandardTreeItem* item = static_cast<EvaluationStandardTreeItem*>(index.internalPointer());
    traced_assert(item);

    if (dynamic_cast<EvaluationStandard*>(item))
    {
        loginf << "got standard";

        //EvaluationStandard* std = dynamic_cast<EvaluationStandard*>(item);
        showMenu();

    }
    else if (dynamic_cast<Group*>(item))
    {
        loginf << "got group";

        Group* group = dynamic_cast<Group*>(item);
        traced_assert(group);
        showGroupMenu(*group);
    }
}

void EvaluationStandardWidget::standardConfigChangedSlot()
{
    model().beginReset();
    model().endReset();
    expandAll();
}

void EvaluationStandardWidget::itemClickedSlot(const QModelIndex& index)
{
    EvaluationStandardTreeItem* item = static_cast<EvaluationStandardTreeItem*>(index.internalPointer());
    traced_assert(item);

    if (dynamic_cast<EvaluationStandard*>(item))
    {
        loginf << "got standard";

        showRequirementWidget(createMainWidget());
    }
    else if (dynamic_cast<Group*>(item))
    {
        loginf << "got group";

        showRequirementWidget(nullptr);
    }
    else if (dynamic_cast<EvaluationRequirement::BaseConfig*>(item))
    {
        loginf << "got config";

        EvaluationRequirement::BaseConfig* config =
                dynamic_cast<EvaluationRequirement::BaseConfig*>(item);

        showRequirementWidget(config->widget());
    }
    else
        traced_assert(false);
}

void EvaluationStandardWidget::addGroupSlot()
{
    loginf << standard_.name();

    bool ok;
    QString text =
        QInputDialog::getText(nullptr, tr("Group Name"),
                              tr("Specify a (unique) group name:"), QLineEdit::Normal, "", &ok);

    if (ok && !text.isEmpty())
    {
        std::string name = text.toStdString();

        if (!name.size())
        {
            QMessageBox m_warning(QMessageBox::Warning, "Adding Group Failed",
                                  "Group has to have a non-empty name.", QMessageBox::Ok);
            m_warning.exec();
            return;
        }

        if (standard_.hasGroup(name))
        {
            QMessageBox m_warning(QMessageBox::Warning, "Adding Group Failed",
                                  "Group with this name already exists.", QMessageBox::Ok);
            m_warning.exec();
            return;
        }

        model().beginReset();

        standard_.addGroup(name);

        model().endReset();
        expandAll();

        loginf << standard_.name() << ": added " << name;
    }
}

void EvaluationStandardWidget::deleteGroupSlot(Group& group)
{
    loginf << standard_.name();

    standard_.removeGroup (group.name());

    // if (widget_)
    //     endModelReset();
}

void EvaluationStandardWidget::addRequirementSlot(Group& group, const std::string& class_id)
{
    loginf << standard_.name() << ": class_id " << class_id;

    bool ok;
    QString text =
        QInputDialog::getText(nullptr, tr("Requirement Name"),
                              tr("Specify a (unique) requirement name:"), QLineEdit::Normal,
                              "", &ok);

    if (!ok)
        return;

    std::string req_name;

    if (!text.isEmpty())
    {
        req_name = text.toStdString();
        if (!req_name.size())
        {
            QMessageBox m_warning(QMessageBox::Warning, "Adding Requirement Failed",
                                  "Requirement has to have a non-empty name.", QMessageBox::Ok);
            m_warning.exec();
            return;
        }

        if (group.hasRequirementConfig(req_name))
        {
            QMessageBox m_warning(QMessageBox::Warning, "Adding Requirement Failed",
                                  "Requirement with this name already exists.", QMessageBox::Ok);
            m_warning.exec();
            return;
        }
    }

    std::string req_short_name;

    text =  QInputDialog::getText(nullptr, tr("Requirement Short Name"),
                                 tr("Specify a requirement short name:"), QLineEdit::Normal,
                                 "", &ok);

    if (!ok)
        return;

    if (!text.isEmpty())
        req_short_name = text.toStdString();

    loginf << group.name() << ": class_id " << class_id
           << " req_name '" << req_name << "' req_short_name '" << req_short_name << "'";

    if (req_name.size() && req_short_name.size())
    {
        model().beginReset();

        group.addRequirementConfig(class_id, req_name, req_short_name);

        model().endReset();
        expandAll();
    }
    else
    {
        QMessageBox m_warning(QMessageBox::Warning, "Adding Requirement Failed",
                              "Requirement has to have a non-empty name and short name.", QMessageBox::Ok);
        m_warning.exec();
        return;
    }
}

void EvaluationStandardWidget::deleteRequirementSlot(Group& group, EvaluationRequirement::BaseConfig& req)
{
    loginf << group.name();

    // QAction* action = dynamic_cast<QAction*>(QObject::sender());
    // traced_assert(action);

    //QVariant data = action->data();
    //traced_assert(data.isValid());

    //string name = data.toString().toStdString();
    string name = req.name();

    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(nullptr, "Delete Requirement", ("Confirm to delete requirement '"+name+"'").c_str(),
                                  QMessageBox::Yes|QMessageBox::No);
    if (reply == QMessageBox::Yes)
    {
        model().beginReset();

        group.removeRequirementConfig(name);

        model().endReset();
        expandAll();
    }
}

void EvaluationStandardWidget::expandAll()
{
    tree_view_->expandAll();
}

void EvaluationStandardWidget::showRequirementWidget(QWidget* widget)
{
    traced_assert(requirements_widget_);

    if (!widget)
    {
        while (requirements_widget_->count() > 0)  // remove all widgets
        {
            auto* widget = requirements_widget_->widget(0);
            requirements_widget_->removeWidget(widget);
            delete widget;
        }
        return;
    }

    if (requirements_widget_->indexOf(widget) < 0)
        requirements_widget_->addWidget(widget);

    requirements_widget_->setCurrentWidget(widget);
}

void EvaluationStandardWidget::showMenu ()
{
    menu_.exec(QCursor::pos());
}

void EvaluationStandardWidget::showGroupMenu (Group& group)
{
    QMenu menu;

    // menu creation
    {
        QAction* del_action = menu.addAction("Delete Group");
        connect(del_action, &QAction::triggered, [this,&group]() {this->deleteGroupSlot(group);});

        // requirements
        QMenu* req_menu = menu.addMenu("Add Requirement");

        for (auto& req_it : Group::requirement_type_mapping_)
        {
            std::string class_id = req_it.first;
            QAction* action = req_menu->addAction(class_id.c_str());
            connect(action, &QAction::triggered, [this,&group,class_id]() {this->addRequirementSlot(group, class_id);});
        }

        {
            QMenu* del_menu = menu.addMenu("Delete Requirement");

            for (auto& cfg_it : group.configs())
            {
                QAction* action = del_menu->addAction(cfg_it->name().c_str());
                connect(action, &QAction::triggered, [this,&cfg_it,&group]() {
                    this->deleteRequirementSlot(group, *cfg_it.get());});
            }
        }

        menu.addSeparator();

        {
            QAction* sel_action = menu.addAction("Select All");
            connect(sel_action, &QAction::triggered, [this,&group]() {
                group.useAll();
                this->model().updateCheckStates();
            });

            QAction* unsel_action = menu.addAction("Deselect All");
            connect(unsel_action, &QAction::triggered, [this,&group]() {
                group.useNone();
                this->model().updateCheckStates();
            });
        }
    }

    menu.exec(QCursor::pos());
}


QWidget* EvaluationStandardWidget::createMainWidget()
{
    QWidget* widget = new QWidget();

    QFormLayout* layout = new QFormLayout();

    // max ref time diff
    QLineEdit* ref_max_time_diff_edit =
        new QLineEdit(QString::number(standard_.referenceMaxTimeDiff()));
    ref_max_time_diff_edit->setValidator(new QDoubleValidator(0.1, 30.0, 2, this));
    connect(ref_max_time_diff_edit, &QLineEdit::textEdited, 
        this, &EvaluationStandardWidget::refMaxTimeDiffEditSlot);

    layout->addRow("Reference Maximum Time Difference [s]", ref_max_time_diff_edit);

    QLineEdit* ref_min_acc_edit = new QLineEdit(QString::number(standard_.referenceMinAccuracy()));
    ref_min_acc_edit->setValidator(new QDoubleValidator(0.1, 1000.0, 1, this));
    connect(ref_min_acc_edit, &QLineEdit::textEdited, 
        this, &EvaluationStandardWidget::refMinAccEditSlot);

    layout->addRow("Reference Minimum Accuracy [m]", ref_min_acc_edit);

    widget->setLayout(layout);

    return widget;
}

void EvaluationStandardWidget::refMaxTimeDiffEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        standard_.referenceMaxTimeDiff(val);
    else
        loginf << "invalid value";
}

void EvaluationStandardWidget::refMinAccEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        standard_.referenceMinAccuracy(val);
    else
        loginf << "invalid value";
}