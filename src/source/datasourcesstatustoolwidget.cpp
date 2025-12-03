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

#include "datasourcesstatustoolwidget.h"
#include "compass.h"
#include "datasourcemanager.h"
#include "datasourcesstatuswidget.h"
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
#include <QComboBox>
#include <QTextEdit>
#include <QScrollBar>

const QColor DataSourcesStatusToolWidget::ColorWarning = QColor(255, 165, 0);
const QColor DataSourcesStatusToolWidget::ColorError   = Qt::red;

/**
 */
DataSourcesStatusToolWidget::DataSourcesStatusToolWidget(DataSourceManager& ds_man)
: ds_man_(ds_man)
{
    createUI();
    updateTrackerSelection();
}

/**
 */
DataSourcesStatusToolWidget::~DataSourcesStatusToolWidget() = default;

/**
 */
void DataSourcesStatusToolWidget::createUI()
{
    QFont font_bold;
    font_bold.setBold(true);

    QVBoxLayout* main_layout = new QVBoxLayout();
    setLayout(main_layout);

    QHBoxLayout* selection_layout = new QHBoxLayout;

    tracker_combo_ = new QComboBox;
    tracker_combo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    line_combo_ = new QComboBox;
    line_combo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    auto tracker_label = new QLabel("Tracker");
    tracker_label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

    auto line_label = new QLabel("Line");
    line_label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

    selection_layout->addWidget(tracker_label);
    selection_layout->addWidget(tracker_combo_);
    selection_layout->addWidget(line_label);
    selection_layout->addWidget(line_combo_);

    auto& dbc_man = COMPASS::instance().dbContentManager();

    ds_widget_ = new DataSourcesStatusWidget(ds_man_, dbc_man);
    ds_widget_->setContentsMargins(0, 0, 0, 0);
    ds_widget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    event_box_ = new QTextEdit;
    event_box_->setReadOnly(true);
    event_box_->setAcceptRichText(true);
    event_box_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    event_box_->setPlaceholderText("No Events");

    main_layout->addLayout(selection_layout);
    main_layout->addWidget(ds_widget_);
    main_layout->addWidget(event_box_);

    connect(tracker_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &DataSourcesStatusToolWidget::updateActiveTracker);
    connect(line_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &DataSourcesStatusToolWidget::updateActiveTracker);
    connect(ds_widget_, &DataSourcesStatusWidget::eventAdded,
            this, &DataSourcesStatusToolWidget::updateEventBox);
}

/**
 */
QIcon DataSourcesStatusToolWidget::toolIcon() const 
{
    return QIcon(Utils::Files::getIconFilepath("data_sources.png").c_str());
}

/**
 */
std::string DataSourcesStatusToolWidget::toolName() const 
{
    return "Sensor Status";
}

/**
 */
std::string DataSourcesStatusToolWidget::toolInfo() const 
{
    return "Sensor Status";
}

/**
 */
std::vector<std::string> DataSourcesStatusToolWidget::toolLabels() const 
{
    return { "Sensor", "Status" };
}

/**
 */
toolbox::ScreenRatio DataSourcesStatusToolWidget::defaultScreenRatio() const 
{
    return ToolBoxWidget::defaultScreenRatio();
}

/**
 */
void DataSourcesStatusToolWidget::addToConfigMenu(QMenu* menu) 
{
    ds_widget_->addActionsToConfigMenu(menu);
}

/**
 */
void DataSourcesStatusToolWidget::addToToolBar(QToolBar* tool_bar)
{
}

/**
 */
void DataSourcesStatusToolWidget::loadingStarted()
{
    ds_widget_->setEnabled(false);
}

/**
 */
void DataSourcesStatusToolWidget::loadingDone()
{
    ds_widget_->setEnabled(true);
}

/**
 */
void DataSourcesStatusToolWidget::updateContent(bool recreate_required)
{
    updateTrackerSelection();

    ds_widget_->updateContent(recreate_required);
}

/**
 */
void DataSourcesStatusToolWidget::resetStatus()
{
    ds_widget_->reset();

    resetEvents();
}

/**
 */
void DataSourcesStatusToolWidget::resetEvents()
{
    event_box_->clear();
}

/**
 */
void DataSourcesStatusToolWidget::updateTrackerSelection()
{
    const std::string DBCName = "CAT063";

    auto active_tracker = ds_widget_->activeTracker();

    tracker_combo_->blockSignals(true);
    tracker_combo_->clear();

    for (const auto& db_ds : ds_man_.dbDataSources())
    {
        if (dbContent::DataSourceBase::dsTypeFromString(db_ds->dsType()) != dbContent::DataSourceType::Tracker)
            continue;

        //no cat063 content?
        if (!db_ds->hasNumInserted(DBCName))
            continue;

        tracker_combo_->addItem(QString::fromStdString(db_ds->name()), QVariant(db_ds->id()));
    }

    int idx = active_tracker.has_value() ? tracker_combo_->findData(QVariant(active_tracker->first)) : -1;
    
    if (tracker_combo_->count() > 0)
        tracker_combo_->setCurrentIndex(idx >= 0 ? idx : 0);

    tracker_combo_->blockSignals(false);

    updateLineSelection();
}

/**
 */
void DataSourcesStatusToolWidget::updateLineSelection()
{
    const std::string DBCName = "CAT063";

    auto active_tracker = ds_widget_->activeTracker();

    line_combo_->blockSignals(true);
    line_combo_->clear();

    if (tracker_combo_->count() > 0 && tracker_combo_->currentIndex() >= 0)
    {
        auto ds_id = tracker_combo_->currentData().toUInt();
        bool ds_id_is_current = active_tracker.has_value() && ds_id == active_tracker->first;

        traced_assert(ds_man_.hasDBDataSource(ds_id));
        const auto& db_ds = ds_man_.dbDataSource(ds_id);

        for (unsigned int line_id = 0; line_id < 4; ++line_id)
        {
            if (!db_ds.hasNumInserted(DBCName, line_id))
                continue;
            
            line_combo_->addItem("L" + QString::number(line_id + 1), QVariant(line_id));
        }

        int idx = ds_id_is_current ? line_combo_->findData(QVariant(active_tracker->second)) : -1;

        if (line_combo_->count() > 0)
            line_combo_->setCurrentIndex(idx >= 0 ? idx : 0);
    }

    line_combo_->blockSignals(false);

    updateActiveTracker();
}

/**
 */
void DataSourcesStatusToolWidget::updateActiveTracker()
{
    if (tracker_combo_->currentData().isValid() &&
        line_combo_->currentData().isValid())
    {
        unsigned int ds_id   = tracker_combo_->currentData().toUInt();
        unsigned int line_id = line_combo_->currentData().toUInt();

        //set to selected tracker + line
        ds_widget_->setActiveTracker(ds_id, line_id);
    }
    else
    {
        //no valid selection => unset tracker
        ds_widget_->unsetActiveTracker();
    }
}

/**
 */
void DataSourcesStatusToolWidget::updateEventBox()
{
    const auto& events = ds_widget_->events();
    if (events.empty())
        return;

    const auto& evt = events.back();

    auto txt = evt.displayInfo();
    
    if (evt.severity == DataSourcesStatusWidget::Event::Severity::Info)
    {
        event_box_->append(QString::fromStdString(txt));
    }
    else
    {
        QColor color = evt.severity == DataSourcesStatusWidget::Event::Severity::Warning ? ColorWarning : ColorError;
        event_box_->append("<span style='color:" + color.name() + ";'>" + QString::fromStdString(txt) + "</span>");
    }

    event_box_->verticalScrollBar()->setValue(event_box_->verticalScrollBar()->maximum());
}
