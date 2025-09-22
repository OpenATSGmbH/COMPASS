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

#include "datasourcestoolwidget.h"
#include "compass.h"
#include "datasourcemanager.h"
#include "datasourceswidget.h"
#include "dbcontentmanager.h"

#include "stringconv.h"
#include "number.h"
#include "files.h"
#include "timeconv.h"

#include <QLabel>
#include <QMenu>
#include <QAction>
#include <QMessageBox>
#include <QVBoxLayout>

DataSourcesToolWidget::DataSourcesToolWidget(DataSourceManager& ds_man)
: ds_man_(ds_man)
{
    createUI();
}
DataSourcesToolWidget::~DataSourcesToolWidget()
{

}

void DataSourcesToolWidget::createUI()
{
    QFont font_bold;
    font_bold.setBold(true);

    QVBoxLayout* main_layout = new QVBoxLayout();
    setLayout(main_layout);

    ds_widget_ = new DataSourcesWidget(ds_man_);
    ds_widget_->setContentsMargins(0, 0, 0, 0);

    main_layout->addWidget(ds_widget_);

    QHBoxLayout* assoc_layout = new QHBoxLayout();

    // time
    QLabel* ts_label = new QLabel("Timestamps");
    ts_label->setFont(font_bold);
    assoc_layout->addWidget(ts_label);

    assoc_layout->addWidget(new QLabel("Min"));

    ts_min_label_ = new QLabel("None");
    assoc_layout->addWidget(ts_min_label_);

    assoc_layout->addWidget(new QLabel("Max"));

    ts_max_label_ = new QLabel("None");
    assoc_layout->addWidget(ts_max_label_);

    assoc_layout->addStretch();

    // assoc
    QLabel* assoc_label = new QLabel("Associations");
    assoc_label->setFont(font_bold);
    assoc_layout->addWidget(assoc_label);

    associations_label_ = new QLabel("None");
    assoc_layout->addWidget(associations_label_);

    assoc_layout->addStretch();
    main_layout->addLayout(assoc_layout);
}

/**
 */
QIcon DataSourcesToolWidget::toolIcon() const 
{
    return QIcon(Utils::Files::getIconFilepath("data_sources.png").c_str());
}

/**
 */
std::string DataSourcesToolWidget::toolName() const 
{
    return "Data Sources";
}

/**
 */
std::string DataSourcesToolWidget::toolInfo() const 
{
    return "Data Sources";
}

/**
 */
std::vector<std::string> DataSourcesToolWidget::toolLabels() const 
{
    return { "Data", "Sources" };
}

/**
 */
toolbox::ScreenRatio DataSourcesToolWidget::defaultScreenRatio() const 
{
    return ToolBoxWidget::defaultScreenRatio();
}

/**
 */
void DataSourcesToolWidget::addToConfigMenu(QMenu* menu) 
{
    ds_widget_->addActionsToConfigMenu(menu);
}

/**
 */
void DataSourcesToolWidget::addToToolBar(QToolBar* tool_bar)
{
}

/**
 */
void DataSourcesToolWidget::loadingStarted()
{
    ds_widget_->setEnabled(false);
}

/**
 */
void DataSourcesToolWidget::loadingDone()
{
    ds_widget_->setEnabled(true);
}

void DataSourcesToolWidget::updateContent(bool recreate_required)
{
    ds_widget_->updateContent(recreate_required);

    updateAdditionalInfo();
}

/**
 */
void DataSourcesToolWidget::updateAdditionalInfo()
{
    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    traced_assert(ts_min_label_);
    traced_assert(ts_max_label_);

    if (dbcont_man.hasMinMaxTimestamp())
    {
        ts_min_label_->setText(Utils::Time::toString(std::get<0>(dbcont_man.minMaxTimestamp()), 0).c_str());
        ts_max_label_->setText(Utils::Time::toString(std::get<1>(dbcont_man.minMaxTimestamp()), 0).c_str());
    }
    else
    {
        ts_min_label_->setText("None");
        ts_max_label_->setText("None");
    }

    traced_assert(associations_label_);
    if (dbcont_man.hasAssociations())
    {
        std::string tmp = "From " + dbcont_man.associationsID();
        associations_label_->setText(tmp.c_str());
    }
    else
    {
        associations_label_->setText("None");
    }
}

