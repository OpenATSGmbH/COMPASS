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

#include "dbfilterwidget.h"
//#include "compass.h"
#include "dbfilter.h"
#include "dbfiltercondition.h"
#include "files.h"
//#include "filtermanager.h"
#include "global.h"
#include "logger.h"

#include <QAction>
#include <QCheckBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <QLabel>
#include <QLineEdit>

using namespace Utils;

DBFilterWidget::DBFilterWidget(DBFilter& filter)
    : QFrame(), filter_(filter)
{
    logdbg;

    QVBoxLayout* main_layout = new QVBoxLayout();
    main_layout->setContentsMargins(1, 1, 1, 1);
    main_layout->setSpacing(1);

    setFrameStyle(QFrame::Panel | QFrame::Sunken);

    QHBoxLayout* config_layout = new QHBoxLayout();
    config_layout->setContentsMargins(1, 1, 1, 1);
    config_layout->setSpacing(1);

    active_checkbox_ = new QCheckBox();
    connect(active_checkbox_, SIGNAL(clicked()), this, SLOT(toggleActive()));
    config_layout->addWidget(active_checkbox_);

    visible_checkbox_ = new QCheckBox(tr(filter_.getName().c_str()));
    connect(visible_checkbox_, SIGNAL(clicked()), this, SLOT(toggleVisible()));

    std::string style_str =
        " QCheckBox::indicator {  width: 12px; height: 12px; }  "
        "QCheckBox::indicator:checked   {     image: url(" +
        Files::getIconFilepath("collapse.png") +
        ");   }"
        "QCheckBox::indicator:unchecked   {     image: url(" +
        Files::getIconFilepath("expand.png") + "); }";
    visible_checkbox_->setStyleSheet(style_str.c_str());

    config_layout->addWidget(visible_checkbox_);
    config_layout->addStretch();

    manage_button_ = new QPushButton();
    manage_button_->setVisible(false); // TODO_ASSERT
    manage_button_->setIcon(Files::IconProvider::getIcon("edit.png"));
    manage_button_->setFixedSize(UI_ICON_SIZE);
    manage_button_->setFlat(UI_ICON_BUTTON_FLAT);
    manage_button_->setToolTip(tr("Manage filter"));
    connect(manage_button_, SIGNAL(clicked()), this, SLOT(showMenuSlot()));
    config_layout->addWidget(manage_button_);
    main_layout->addLayout(config_layout);

    child_ = new QWidget();
    child_->setVisible(filter_.widgetVisible());

    child_layout_ = new QGridLayout();
    child_layout_->setContentsMargins(5, 1, 1, 1);
    child_layout_->setSpacing(1);

    updateChildWidget();

    child_->setLayout(child_layout_);

    main_layout->addWidget(child_);

    main_layout->addStretch();
    setLayout(main_layout);

//    connect(this, SIGNAL(deleteFilterSignal(DBFilter*)), &COMPASS::instance().filterManager(),
//            SLOT(deleteFilterSlot(DBFilter*)), Qt::QueuedConnection);

    createMenu();

    update();
}

DBFilterWidget::~DBFilterWidget() {}

void DBFilterWidget::createMenu()
{
    if (!filter_.isGeneric())
        return;

    QAction* reset_action = menu_.addAction(tr("Reset"));
    connect(reset_action, SIGNAL(triggered()), this, SLOT(reset()));

    QAction* edit_action = menu_.addAction(tr("Edit"));
    connect(edit_action, SIGNAL(triggered()), this, SLOT(filterEditSlot()));

    QAction* delete_action = menu_.addAction(tr("Delete"));
    connect(delete_action, SIGNAL(triggered()), this, SLOT(deleteFilter()));
}

void DBFilterWidget::addChildWidget(QWidget* widget)
{
    traced_assert(widget);
    child_layout_->addWidget(widget);
}

void DBFilterWidget::updateChildWidget()
{
    deleteChildrenFromLayout();

    std::vector<DBFilterCondition*>& conditions = filter_.getConditions();
    for (unsigned int cnt = 0; cnt < conditions.size(); cnt++)
    {
        auto label = conditions.at(cnt)->getLabel();
        auto edit  = conditions.at(cnt)->getEdit();

        traced_assert(label);
        traced_assert(edit);

        int row = child_layout_->rowCount();
        child_layout_->addWidget(label, row, 0);
        child_layout_->addWidget(edit , row, 1);

        connect(conditions.at(cnt), SIGNAL(possibleFilterChange()), this,
                SLOT(possibleSubFilterChange()), Qt::UniqueConnection);
    }
}

void DBFilterWidget::toggleVisible()
{
    logdbg;
    filter_.widgetVisible(!filter_.widgetVisible());
    child_->setVisible(filter_.widgetVisible());
}

void DBFilterWidget::toggleAnd()
{
    logdbg;

    // checked is or is false
    // unchecked is and is true
    /*
  filter_.setAnd(and_checkbox_->checkState() == Qt::Unchecked);
     */
    emit possibleFilterChange();
}

void DBFilterWidget::toggleActive()
{
    logdbg;
    filter_.setActive(active_checkbox_->checkState() == Qt::Checked);

    emit possibleFilterChange();
}

void DBFilterWidget::update(void)
{
    logdbg;

    visible_checkbox_->setText(filter_.getName().c_str());

    active_checkbox_->setChecked(filter_.getActive());
    visible_checkbox_->setChecked(filter_.widgetVisible());

    //  if (!filter_.getAnd())
    //    and_checkbox_->setChecked(Qt::Checked);
    //  else
    //    and_checkbox_->setChecked(Qt::Unchecked);

    //  std::vector <DBFilterCondition *> &conditions = filter_.getConditions();
    //  for (unsigned int cnt=0; cnt < conditions.size(); cnt++)
    //  {
    //    conditions.at(cnt)->update();
    //  }
}

void DBFilterWidget::collapse()
{
    filter_.widgetVisible(false);
    child_->setVisible(false);
}

void DBFilterWidget::expand()
{
    filter_.widgetVisible(true);
    child_->setVisible(true);
}

void DBFilterWidget::possibleSubFilterChange()
{
    logdbg;
    emit possibleFilterChange();
}

void DBFilterWidget::reset()
{
    filter_.reset();
    updateChildWidget();
    emit possibleFilterChange();
}

void DBFilterWidget::showMenuSlot() { menu_.exec(QCursor::pos()); }

void DBFilterWidget::deleteFilter() { emit deleteFilterSignal(&filter_); }

void DBFilterWidget::filterEditSlot()
{
    logdbg;
    emit filterEdit(&filter_);
}

namespace
{
    void clearLayoutItems(QLayout* layout)
    {
        while (QLayoutItem* item = layout->takeAt(0)) 
        {
            if (QWidget* widget = item->widget()) 
            {
                widget->setParent(nullptr);
                delete widget;
            } 
            else if (QLayout* childLayout = item->layout()) 
            {
                clearLayoutItems(childLayout);
                delete childLayout;
            }
            delete item;
        }
    };
}

void DBFilterWidget::deleteChildrenFromLayout()
{
    clearLayoutItems(child_layout_);
}

void DBFilterWidget::addNameValuePair(const std::string& label, QWidget* widget, int row, int col)
{
    int insert_row = row >= 0 ? row : child_layout_->rowCount();

    auto lwidget = new QLabel(label.c_str());
    lwidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);

    widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    child_layout_->addWidget(lwidget, insert_row, col    );
    child_layout_->addWidget(widget , insert_row, col + 1);
}

void DBFilterWidget::addNameValuePair(const std::string& label, const std::string& label2, int row, int col)
{
    addNameValuePair(label, new QLabel(label2.c_str()), row, col);
}

int DBFilterWidget::columnWidth(int layout_column) const
{
    if (layout_column < 0 || layout_column >= child_layout_->columnCount())
        return 0;

    int max_width = 0;
    for (int r = 0; r < child_layout_->rowCount(); ++r)
    {
        auto litem = child_layout_->itemAtPosition(r, layout_column);

        if (litem && litem->widget() && litem->widget()->sizeHint().width() > max_width)
        {
            max_width = litem->widget()->sizeHint().width();
            //if (dynamic_cast<QLabel*>(litem->widget()))
            //    loginf << "scanned filter row '" << dynamic_cast<QLabel*>(litem->widget())->text().toStdString() << "'";
        }
    }

    return max_width;
}

void DBFilterWidget::setFixedColumnWidth(int layout_column, int width)
{
    if (layout_column < 0 || layout_column >= child_layout_->columnCount())
        return;
    
    for (int r = 0; r < child_layout_->rowCount(); ++r)
    {
        auto litem = child_layout_->itemAtPosition(r, layout_column);
        if (litem && litem->widget())
            litem->widget()->setFixedWidth(width);
    }
}

void DBFilterWidget::setMaximumColumnWidth(int layout_column, int width)
{
    if (layout_column < 0 || layout_column >= child_layout_->columnCount())
        return;
    
    for (int r = 0; r < child_layout_->rowCount(); ++r)
    {
        auto litem = child_layout_->itemAtPosition(r, layout_column);
        if (litem && litem->widget())
            litem->widget()->setMaximumWidth(width);
    }
}
