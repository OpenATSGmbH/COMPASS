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
#include "db_context_manager.h"
#include "datasourceswidget.h"
#include "dbcontentmanager.h"

#include "logger.h"
#include "stringconv.h"
#include "number.h"
#include "files.h"
#include "timeconv.h"

#include <QComboBox>
#include <QLabel>
#include <QMenu>
#include <QAction>
#include <QMessageBox>
#include <QVBoxLayout>

DataSourcesToolWidget::DataSourcesToolWidget(context::DBContextManager& ctx_man)
: ctx_man_(ctx_man)
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

    ds_widget_ = new DataSourcesWidget(true, ctx_man_);
    ds_widget_->setContentsMargins(0, 0, 0, 0);

    connect(&ctx_man_, &context::DBContextManager::activeContextChangedSignal,
            this, [this] { logdbg << "activeContextChangedSignal received"; updateContent(true); });

    connect(&ctx_man_, &context::DBContextManager::countsChangedSignal,
            this, [this] { logdbg << "countsChangedSignal received"; updateContent(false); });

    auto& dbcont_man = ctx_man_.compass().dbContentManager();

    connect(&dbcont_man, &DBContentManager::associationStatusChangedSignal,
            this, [this] { logdbg << "associationStatusChangedSignal received"; updateAdditionalInfo(); });

    connect(&dbcont_man, &DBContentManager::dbContentStatusChanged,
            this, [this] { logdbg << "dbContentStatusChanged received"; updateAdditionalInfo(); });

    main_layout->addWidget(ds_widget_);

    // Color Mode row (persisted per-user on the COMPASS instance)
    {
        QHBoxLayout* mode_layout = new QHBoxLayout();

        QLabel* mode_label = new QLabel("Color Mode:");
        mode_label->setFont(font_bold);
        mode_layout->addWidget(mode_label);

        color_mode_combo_ = new QComboBox();
        color_mode_combo_->addItem("DSType");
        color_mode_combo_->addItem("DBContent");
        color_mode_combo_->addItem("Data Source");
        color_mode_combo_->addItem("Data Source + Line");

        unsigned int current = ctx_man_.compass().colorMode();
        if (current >= (unsigned int)color_mode_combo_->count())
            current = 0;
        color_mode_combo_->setCurrentIndex((int)current);

        connect(color_mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, &DataSourcesToolWidget::colorModeChangedSlot);

        mode_layout->addWidget(color_mode_combo_);
        mode_layout->addStretch();

        main_layout->addLayout(mode_layout);
    }

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

/**
 */
void DataSourcesToolWidget::updateContent(bool recreate_required)
{
    ds_widget_->updateContent(recreate_required);

    updateAdditionalInfo();
}

/**
 */
void DataSourcesToolWidget::updateAdditionalInfo()
{
    DBContentManager& dbcont_man = ctx_man_.compass().dbContentManager();

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

void DataSourcesToolWidget::colorModeChangedSlot(int index)
{
    if (index < 0)
        return;

    loginf << "color mode " << index;

    ctx_man_.compass().colorMode((unsigned int)index);
    if (ds_widget_)
        ds_widget_->updateContent();
}


