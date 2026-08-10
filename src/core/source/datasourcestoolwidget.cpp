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
#include "dbcontentdataengine.h"

#include "logger.h"
#include "stringconv.h"
#include "number.h"
#include "files.h"
#include "timeconv.h"
#include "ui_test_common.h"

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
    // Flight-Deck panel doesn't use row selection - silence the blue
    // highlight and current-item marker.
    ds_widget_->disableSelection();

    connect(&ctx_man_, &context::DBContextManager::activeContextChangedSignal,
            this, [this] { logdbg << "activeContextChangedSignal received"; updateContent(true); });
    connect(&ctx_man_, &context::DBContextManager::dataSourcesChangedSignal,
            this, [this] { logdbg << "dataSourcesChangedSignal received"; updateContent(true); });

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
        UI_TEST_OBJ_NAME(color_mode_combo_, "Color Mode")
        // Items carry the ColorProvider::Mode enum value as user data so the
        // UI order can be changed independently of the persisted numeric mode.
        color_mode_combo_->addItem("DSType",              (unsigned int)0); // DSType
        color_mode_combo_->addItem("Data Source",         (unsigned int)2); // DataSource
        color_mode_combo_->addItem("Data Source + Line",  (unsigned int)3); // DataSourceLine
        color_mode_combo_->addItem("DBContent",           (unsigned int)1); // DBContent

        unsigned int current = ctx_man_.compass().colorMode();
        int current_idx = color_mode_combo_->findData(current);
        if (current_idx < 0) current_idx = 0;
        color_mode_combo_->setCurrentIndex(current_idx);

        connect(color_mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, &DataSourcesToolWidget::colorModeChangedSlot);

        mode_layout->addWidget(color_mode_combo_);
        mode_layout->addStretch();

        // Reference-trajectory / UTN-association status on the right end of
        // the Color Mode row.
        associations_label_ = new QLabel("No References Calculated");
        mode_layout->addWidget(associations_label_);

        main_layout->addLayout(mode_layout);
    }

    QHBoxLayout* ts_layout = new QHBoxLayout();

    QLabel* ts_label = new QLabel("Timestamps");
    ts_label->setFont(font_bold);
    ts_layout->addWidget(ts_label);

    ts_layout->addWidget(new QLabel("Min"));

    ts_min_label_ = new QLabel("None");
    ts_layout->addWidget(ts_min_label_);

    ts_layout->addStretch();

    ts_layout->addWidget(new QLabel("Max"));

    ts_max_label_ = new QLabel("None");
    ts_layout->addWidget(ts_max_label_);

    main_layout->addLayout(ts_layout);
}

/**
 */
QIcon DataSourcesToolWidget::toolIcon() const 
{
    return Utils::Files::IconProvider::getIcon("data_sources.png");
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
    // The offline load progress dialog (Qt::ApplicationModal) blocks input
    // application-wide. In LiveRunning the started→done envelope wraps the
    // entire live session, so disabling here would lock the data sources
    // panel for the duration of live mode. Either way: nothing to disable.
}

/**
 */
void DataSourcesToolWidget::loadingDone()
{
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

    if (dbcont_man.dataEngine().hasMinMaxTimestamp())
    {
        ts_min_label_->setText(Utils::Time::toString(std::get<0>(dbcont_man.dataEngine().minMaxTimestamp()), 0).c_str());
        ts_max_label_->setText(Utils::Time::toString(std::get<1>(dbcont_man.dataEngine().minMaxTimestamp()), 0).c_str());
    }
    else
    {
        ts_min_label_->setText("None");
        ts_max_label_->setText("None");
    }

    traced_assert(associations_label_);
    if (dbcont_man.hasAssociations())
        associations_label_->setText("References Calculated");
    else
        associations_label_->setText("No References Calculated");
}

void DataSourcesToolWidget::colorModeChangedSlot(int index)
{
    if (index < 0)
        return;

    // combo index != mode value - the UI order diverges from the enum order
    const unsigned int mode = color_mode_combo_->itemData(index).toUInt();
    loginf << "color mode " << mode;

    ctx_man_.compass().colorMode(mode);
    if (ds_widget_)
        ds_widget_->updateContent();
}


