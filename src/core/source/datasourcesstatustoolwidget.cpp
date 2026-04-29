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
#include "db_context_manager.h"
#include "data_source.h"
#include "datasourcebase.h"
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
#include <QCheckBox>
#include <QSplitter>

const int DataSourcesStatusToolWidget::DefaultStretchDataSources = 3;
const int DataSourcesStatusToolWidget::DefaultStretchEvents      = 1;

/**
 */
DataSourcesStatusToolWidget::DataSourcesStatusToolWidget(context::DBContextManager& ctx_man)
: ctx_man_(ctx_man)
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

    //create tracker selection
    QHBoxLayout* selection_layout = new QHBoxLayout;
    selection_layout->setContentsMargins(0, 0, 0, 0);

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

    //create splitter
    main_splitter_ = new QSplitter;
    main_splitter_->setOrientation(Qt::Vertical);

    //create status widget
    auto ds_status_widget = new QWidget;

    QVBoxLayout* status_layout = new QVBoxLayout();
    status_layout->setContentsMargins(0, 0, 0, 0);
    status_layout->setSpacing(1);
    ds_status_widget->setLayout(status_layout);

    auto& dbc_man = ctx_man_.compass().dbContentManager();

    ds_widget_ = new DataSourcesStatusWidget(ctx_man_, dbc_man);
    ds_widget_->setContentsMargins(0, 0, 0, 0);
    ds_widget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    info_received_label_ = new QLabel("-");
    auto info_received_txt = new QLabel("Last Received: ");
    info_received_txt->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);

    info_refreshed_label_ = new QLabel("-");
    auto info_refreshed_txt = new QLabel("Last Update: ");
    info_refreshed_txt->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);

    auto info_layout = new QHBoxLayout;
    info_layout->setContentsMargins(0, 0, 0, 0);

    info_layout->addWidget(info_received_txt);
    info_layout->addWidget(info_received_label_);
    info_layout->addWidget(info_refreshed_txt);
    info_layout->addWidget(info_refreshed_label_);
    
    status_layout->addWidget(ds_widget_);
    status_layout->addLayout(info_layout);

    //create event widget
    auto event_widget = new QWidget;

    auto event_layout = new QVBoxLayout;
    event_layout->setContentsMargins(0, 0, 0, 0);
    event_layout->setSpacing(1);
    event_widget->setLayout(event_layout);

    event_box_ = new QTextEdit;
    event_box_->setReadOnly(true);
    event_box_->setAcceptRichText(true);
    event_box_->document()->setMaximumBlockCount(ctx_man_.sensorConfig().sensor_status_max_event_buf_size);
    event_box_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    event_box_->setPlaceholderText("No Events");

    event_layout->addWidget(event_box_);

    //add to splitter
    main_splitter_->addWidget(ds_status_widget);
    main_splitter_->addWidget(event_widget);

    main_splitter_->setStretchFactor(0, DefaultStretchDataSources);
    main_splitter_->setStretchFactor(1, DefaultStretchEvents     );

    //add to main layout
    main_layout->addLayout(selection_layout);
    main_layout->addWidget(main_splitter_);

    connect(tracker_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &DataSourcesStatusToolWidget::updateActiveTracker);
    connect(line_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &DataSourcesStatusToolWidget::updateActiveTracker);
    
    connect(ds_widget_, &DataSourcesStatusWidget::activeTrackerChanged,
            this, &DataSourcesStatusToolWidget::onActiveTrackerChanged);
    connect(ds_widget_, &DataSourcesStatusWidget::eventsAdded,
            this, &DataSourcesStatusToolWidget::addNewestEvents);
    connect(ds_widget_, &DataSourcesStatusWidget::refreshed,
            this, &DataSourcesStatusToolWidget::updateInfos);
    
    connect(&ctx_man_, &context::DBContextManager::activeContextChangedSignal,
            this, &DataSourcesStatusToolWidget::onDataSourcesChanged);
    connect(&ctx_man_, &context::DBContextManager::dataSourcesChangedSignal,
            this, &DataSourcesStatusToolWidget::onDataSourcesChanged);
}

/**
 */
QIcon DataSourcesStatusToolWidget::toolIcon() const 
{
    return QIcon(Utils::Files::getIconFilepath("sensor_status.png").c_str());
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
    auto update_age_menu = menu->addMenu("Maximum Status Age");

    const auto& max_status_age_options = ctx_man_.sensorConfig().sensor_status_max_status_age_options;
    traced_assert(max_status_age_options.is_array());

    auto max_status_age_index = ctx_man_.sensorConfig().sensor_status_max_status_age_index;

    // Create an exclusive action group
    QActionGroup* group = new QActionGroup(this);
    group->setExclusive(true);

    auto addStatusAgeAction = [ max_status_age_index, update_age_menu, this, &max_status_age_options, group ] (unsigned int index)
    {
        unsigned int value = max_status_age_options.at(index);

        auto action = update_age_menu->addAction(QString::number(value) + "s");
        action->setCheckable(true);

        action->setChecked(index == max_status_age_index);

        group->addAction(action);

        connect(action, &QAction::triggered, 
            [ this, index ] () 
            { 
                this->ctx_man_.sensorConfig().sensor_status_max_status_age_index = index; 
                this->resetStatus();
            });
    };

    for (size_t i = 0; i < max_status_age_options.size(); ++i)
        addStatusAgeAction((unsigned int)i);

    auto last_update_action = menu->addAction("Show Last Update ToD");
    last_update_action->setCheckable(true);
    last_update_action->setChecked(ctx_man_.sensorConfig().sensor_status_show_last_updates);

    connect(last_update_action, &QAction::toggled, [ this ] (bool ok) { this->ds_widget_->showLastUpdates(ok); });
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
    //ds_widget_->setEnabled(false);
}

/**
 */
void DataSourcesStatusToolWidget::loadingDone()
{
    //ds_widget_->setEnabled(true);
}

/**
 */
void DataSourcesStatusToolWidget::updateContent(bool recreate_required)
{
    //needed because new tracker data could have been added during load
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
DataSourcesStatusToolWidget::ActiveTrackerLines DataSourcesStatusToolWidget::activeTrackerLines() const
{
    const std::string DBCName = "CAT063";

    ActiveTrackerLines tracker_lines;

    if (ctx_man_.hasActiveContext())
    {
        for (const auto& db_ds : ctx_man_.activeContext().dataSources())
        {
            if (dbContent::DataSourceBase::dsTypeFromString(db_ds.dsType()) != dbContent::DataSourceType::Tracker)
                continue;

            //no cat063 content?
            if (ctx_man_.numInserted(db_ds.id(), DBCName) == 0)
                continue;

            auto& lines = tracker_lines[ db_ds.id() ];

            for (unsigned int line_id = 0; line_id < 4; ++line_id)
            {
                //add if line has inserted data
                auto per_line = ctx_man_.numInsertedPerLine(db_ds.id(), DBCName);
                if (!per_line.count(line_id) || per_line.at(line_id) == 0)
                    continue;

                lines.push_back(line_id);
            }
        }
    }

    return tracker_lines;
}

/**
 */
void DataSourcesStatusToolWidget::updateTrackerSelection()
{
    //determine active tracker lines
    //caching: skip if tracker lines to be added did not change since the last update
    auto tracker_lines = activeTrackerLines();
    if (tracker_lines == current_tracker_lines_)
        return;

    auto active_tracker = ds_widget_->activeTracker();

    tracker_combo_->blockSignals(true);
    tracker_combo_->clear();

    for (const auto& tl : tracker_lines)
    {
        traced_assert(ctx_man_.hasDataSource(tl.first));
        const auto& db_ds = *ctx_man_.dataSource(tl.first);

        tracker_combo_->addItem(QString::fromStdString(db_ds.name()), QVariant(db_ds.id()));
    }

    int idx = active_tracker.has_value() ? tracker_combo_->findData(QVariant(active_tracker->first)) : -1;
    
    if (tracker_combo_->count() > 0)
        tracker_combo_->setCurrentIndex(idx >= 0 ? idx : 0);

    tracker_combo_->blockSignals(false);

    updateLineSelection(tracker_lines);

    //cache currently added tracker lines
    current_tracker_lines_ = tracker_lines;
}

/**
 */
void DataSourcesStatusToolWidget::updateLineSelection(const ActiveTrackerLines& tracker_lines)
{
    auto active_tracker = ds_widget_->activeTracker();

    line_combo_->blockSignals(true);
    line_combo_->clear();

    if (tracker_combo_->count() > 0 && tracker_combo_->currentIndex() >= 0)
    {
        auto ds_id = tracker_combo_->currentData().toUInt();
        bool ds_id_is_current = active_tracker.has_value() && ds_id == active_tracker->first;

        traced_assert(tracker_lines.count(ds_id));

        for (unsigned int line_id : tracker_lines.at(ds_id))
            line_combo_->addItem("L" + QString::number(line_id + 1), QVariant(line_id));

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

        //set to selected tracker + line => might trigger content update
        ds_widget_->setActiveTracker(ds_id, line_id);
    }
    else
    {
        //unset tracker => might trigger content update
        ds_widget_->unsetActiveTracker();
    }
}

/**
 * Reacts on changing the active tracker in the data sources status widget.
 */
void DataSourcesStatusToolWidget::onActiveTrackerChanged()
{
    //update dependent ui elements
    updateEventBox();
    updateInfos();
}

/**
 * Reacts on changing data sources.
 */
void DataSourcesStatusToolWidget::onDataSourcesChanged()
{
    //might trigger further ui changes
    updateTrackerSelection();
}

namespace
{
    /**
     */
    void addEventToTextEdit(QTextEdit* edit,
                            const sensor_status::Event& evt,
                            context::DBContextManager& ctx_man)
    {
        auto txt  = evt.toString(ctx_man, false, true);
        auto col  = DataSourcesStatusWidget::colorFromEvent(evt);
        auto font = DataSourcesStatusWidget::fontFromEvent(evt);

        auto qt_txt = (font.bold() ? "<b>" : "") + QString::fromStdString(txt) + (font.bold() ? "</b>" : "");
    
        if (!col.isValid())
        {
            //add in default color
            edit->append(qt_txt);
        }
        else
        {
            //colorize event text
            edit->append("<span style='color:" + col.name() + ";'>" + qt_txt + "</span>");
        }
    }
}

/**
 */
void DataSourcesStatusToolWidget::updateEventBox()
{
    event_box_->clear();

    for (const auto& evt : ds_widget_->currentEventQueue().queue())
        addEventToTextEdit(event_box_, evt, ctx_man_);
}

/**
 */
void DataSourcesStatusToolWidget::addNewestEvents()
{
    for (const auto& evt : ds_widget_->currentEventQueue().consumeNewEvents())
        addEventToTextEdit(event_box_, *evt, ctx_man_);
}

/**
 */
void DataSourcesStatusToolWidget::updateInfos()
{
    auto refresh_ts = ds_widget_->lastRefresh();
    std::string refresh_txt = refresh_ts.is_not_a_date_time() ? "-" : Utils::Time::toTimeString(refresh_ts, false);

    info_refreshed_label_->setText(QString::fromStdString(refresh_txt));

    std::string update_recv_txt = "-";
    if (ds_widget_->hasActiveTrackerStates())
    {
        const auto& states = ds_widget_->activeTrackerStates();
        update_recv_txt = states.last_update_ts.is_not_a_date_time() ? "-" : Utils::Time::toTimeString(states.last_update_ts, false);
    }

    info_received_label_->setText(QString::fromStdString(update_recv_txt));
}

