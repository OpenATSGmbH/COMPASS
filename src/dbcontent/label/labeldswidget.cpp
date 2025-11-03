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

#include "dbcontent/label/labeldswidget.h"
#include "compass.h"
#include "datasourcemanager.h"
#include "dbcontentmanager.h"
#include "dbcontent/label/labelgenerator.h"
#include "logger.h"
#include "files.h"
#include "stringconv.h"

#include <QHBoxLayout>
#include <QMenu>
#include <QPushButton>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QCheckBox>
#include <QLabel>
#include <QVariant>

using namespace std;
using namespace Utils;

namespace dbContent
{


LabelDSWidget::LabelDSWidget(LabelGenerator& label_generator, QWidget* parent, Qt::WindowFlags f)
    : QWidget(parent, f), label_generator_(label_generator)
{
    arrow_lu_ = Files::IconProvider::getIcon("arrow_lu.png");
    arrow_ru_ = Files::IconProvider::getIcon("arrow_ru.png");
    arrow_ld_ = Files::IconProvider::getIcon("arrow_ld.png");
    arrow_rd_ = Files::IconProvider::getIcon("arrow_rd.png");

    QVBoxLayout* main_layout = new QVBoxLayout();

    ds_grid_ = new QGridLayout();
    ds_grid_->setContentsMargins(2,2,2,2);
    ds_grid_->setSpacing(1);
    updateListSlot();
    main_layout->addLayout(ds_grid_);

    setLayout(main_layout);

    connect(&COMPASS::instance().dataSourceManager(), &DataSourceManager::dataSourcesChangedSignal,
            this, &LabelDSWidget::updateListSlot); // update if data sources changed

    connect(&label_generator, &LabelGenerator::labelLinesChangedSignal,
            this, &LabelDSWidget::updateListSlot); // update if lines changed
}

LabelDSWidget::~LabelDSWidget() = default;

void LabelDSWidget::forceUpdateList()
{
    old_sources_.clear();

    updateListSlot();
}

void LabelDSWidget::updateListSlot()
{
    logdbg;

    DataSourceManager& ds_man = COMPASS::instance().dataSourceManager();

    std::map<std::string, std::string> current_sources;
    for (const auto& ds_it : ds_man.dbDataSources())
        current_sources[ds_it->name()] = String::lineStrFrom(label_generator_.labelLine(ds_it->id()))
                + to_string((unsigned int)label_generator_.labelDirection(ds_it->id()));

    if (old_sources_ == current_sources)
        return;

    traced_assert(ds_grid_);

    QLayoutItem* child;
    while (!ds_grid_->isEmpty() && (child = ds_grid_->takeAt(0)) != nullptr)
    {
        if (child->widget())
            delete child->widget();
        delete child;
    }

    direction_buttons_.clear();
    line_buttons_.clear();

    unsigned int row=0;

    QFont font_bold;
    font_bold.setBold(true);

    QLabel* src_label = new QLabel("Data Source");
    src_label->setFont(font_bold);
    ds_grid_->addWidget(src_label, row, 0);

    QLabel* line_label = new QLabel("Line");
    line_label->setFont(font_bold);
    ds_grid_->addWidget(line_label, row, 1);

    QLabel* dir_label = new QLabel("Direction");
    dir_label->setFont(font_bold);
    ds_grid_->addWidget(dir_label, row, 2);

    for (const auto& ds_it : ds_man.dbDataSources())
    {
        ++row;

        QCheckBox* box = new QCheckBox(ds_it->name().c_str());
        box->setProperty("ds_id", ds_it->id());
        box->setChecked(label_generator_.labelWanted(ds_it->id()));
        connect(box, &QCheckBox::clicked, this, &LabelDSWidget::sourceClickedSlot);
        ds_grid_->addWidget(box, row, 0);

        QPushButton* line = new QPushButton(String::lineStrFrom(label_generator_.labelLine(ds_it->id())).c_str());
        line->setProperty("ds_id", ds_it->id());
        line->setFixedWidth(2*UI_ICON_SIZE.width());
        //direction->setFixedSize(UI_ICON_SIZE);
        line->setFlat(UI_ICON_BUTTON_FLAT);
        connect(line, &QPushButton::clicked, this, &LabelDSWidget::changeLineSlot);
        ds_grid_->addWidget(line, row, 1);

        line_buttons_[ds_it->id()] = line;

        QPushButton* direction = new QPushButton();
        direction->setProperty("ds_id", ds_it->id());
        direction->setIcon(iconForDirection(label_generator_.labelDirection(ds_it->id())));
        direction->setFixedWidth(2*UI_ICON_SIZE.width());
        //direction->setFixedSize(UI_ICON_SIZE);
        direction->setFlat(UI_ICON_BUTTON_FLAT);
        connect(direction, &QPushButton::clicked, this, &LabelDSWidget::changeDirectionSlot);
        ds_grid_->addWidget(direction, row, 2);

        direction_buttons_[ds_it->id()] = direction;
    }

    old_sources_ = current_sources;
}

void LabelDSWidget::sourceClickedSlot()
{
    QCheckBox* widget = static_cast<QCheckBox*>(sender());
    traced_assert(widget);

    QVariant ds_id_var = widget->property("ds_id");
    unsigned int ds_id = ds_id_var.value<unsigned int>();

    loginf << "ds_id " << ds_id;

    if (label_generator_.labelWanted(ds_id))
        label_generator_.removeLabelDSID(ds_id);
    else
        label_generator_.addLabelDSID(ds_id);
}

void LabelDSWidget::changeLineSlot()
{
    QPushButton* widget = static_cast<QPushButton*>(sender());
    traced_assert(widget);

    QVariant ds_id_var = widget->property("ds_id");
    unsigned int ds_id = ds_id_var.value<unsigned int>();

    loginf << "ds_id " << ds_id;

    DataSourceManager& ds_man = COMPASS::instance().dataSourceManager();
    traced_assert(ds_man.hasDBDataSource(ds_id));

    dbContent::DBDataSource& ds = ds_man.dbDataSource(ds_id);

    QMenu menu;

    for (unsigned int line_cnt=0; line_cnt < 4; ++line_cnt)
    {
        if (!ds.hasNumLoaded(line_cnt))
            continue;

        QAction* action = menu.addAction(String::lineStrFrom(line_cnt).c_str());
        action->setProperty("ds_id", ds_id);
        action->setProperty("line", line_cnt);
        connect(action, &QAction::triggered, this, &LabelDSWidget::selectLineSlot);
    }

    menu.exec(QCursor::pos());
}

void LabelDSWidget::changeDirectionSlot()
{
    QPushButton* widget = static_cast<QPushButton*>(sender());
    traced_assert(widget);

    QVariant ds_id_var = widget->property("ds_id");
    unsigned int ds_id = ds_id_var.value<unsigned int>();

    loginf << "ds_id " << ds_id;

    QMenu menu;

    QAction* lu_action = menu.addAction("Left Up");
    lu_action->setProperty("ds_id", ds_id);
    lu_action->setProperty("direction", 0);
    lu_action->setIcon(arrow_lu_);
    connect(lu_action, &QAction::triggered, this, &LabelDSWidget::selectDirectionSlot);

    QAction* ru_action = menu.addAction("Right Up");
    ru_action->setProperty("ds_id", ds_id);
    ru_action->setProperty("direction", 1);
    ru_action->setIcon(arrow_ru_);
    connect(ru_action, &QAction::triggered, this, &LabelDSWidget::selectDirectionSlot);

    QAction* ld_action = menu.addAction("Left Down");
    ld_action->setProperty("ds_id", ds_id);
    ld_action->setProperty("direction", 2);
    ld_action->setIcon(arrow_ld_);
    connect(ld_action, &QAction::triggered, this, &LabelDSWidget::selectDirectionSlot);

    QAction* rd_action = menu.addAction("Right Down");
    rd_action->setProperty("ds_id", ds_id);
    rd_action->setProperty("direction", 3);
    rd_action->setIcon(arrow_rd_);
    connect(rd_action, &QAction::triggered, this, &LabelDSWidget::selectDirectionSlot);

    menu.exec(QCursor::pos());
}

QIcon& LabelDSWidget::iconForDirection(LabelDirection direction)
{
    if (direction == LabelDirection::LEFT_UP)
        return arrow_lu_;
    else if (direction == LabelDirection::RIGHT_UP)
        return arrow_ru_;
    else if (direction == LabelDirection::LEFT_DOWN)
        return arrow_ld_;
    else
        return arrow_rd_;
}

void LabelDSWidget::selectDirectionSlot()
{
    QVariant ds_id_var = sender()->property("ds_id");
    unsigned int ds_id = ds_id_var.value<unsigned int>();

    QVariant dir_var = sender()->property("direction");
    unsigned int dir = dir_var.value<unsigned int>();
    traced_assert(dir <= 3);

    loginf << "ds_id " << ds_id << " dir " << dir;

    LabelDirection direction = LabelDirection(dir);
    label_generator_.labelDirection(ds_id, direction);

    traced_assert(direction_buttons_.count(ds_id));
    direction_buttons_.at(ds_id)->setIcon(iconForDirection(direction));
}

void LabelDSWidget::selectLineSlot()
{
    QVariant ds_id_var = sender()->property("ds_id");
    unsigned int ds_id = ds_id_var.value<unsigned int>();

    QVariant line_var = sender()->property("line");
    unsigned int line = line_var.value<unsigned int>();
    traced_assert(line <= 3);

    loginf << "ds_id " << ds_id << " line " << line;

    label_generator_.labelLine(ds_id, line);

    traced_assert(line_buttons_.count(ds_id));
    line_buttons_.at(ds_id)->setText(String::lineStrFrom(line).c_str());
}

}
