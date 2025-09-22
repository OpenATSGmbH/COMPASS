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

#include "datasourceswidget.h"
#include "datasourcemanager.h"

#include "compass.h"
#include "dbcontent/dbcontentmanager.h"

#include "stringconv.h"
#include "number.h"
#include "files.h"
#include "timeconv.h"

#include <QLabel>
#include <QCheckBox>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QHeaderView>

/**************************************************************************************************
 * DataSourcesWidgetItem
 **************************************************************************************************/

/**
 */
DataSourcesWidgetItem::DataSourcesWidgetItem(DataSourcesWidget* widget,
                                             DataSourcesWidgetItem* parent,
                                             Type type)
:   widget_(widget)
,   parent_(parent) 
,   type_  (type  )
{
    traced_assert(widget_);
}

/**
 */
void DataSourcesWidgetItem::setItemWidget(int column, QWidget* w)
{
    traced_assert(widget_);
    traced_assert(widget_->tree_widget_);

    widget_->tree_widget_->setItemWidget(this, column, w);
}

/**************************************************************************************************
 * DataSourceTypeItem
 **************************************************************************************************/

/**
 */
DataSourceTypeItem::DataSourceTypeItem(DataSourcesWidget* widget,
                                       DataSourcesWidgetItem* parent)
:   DataSourcesWidgetItem(widget, parent, Type::DataSourceType)
{
}

/**
 */
bool DataSourceTypeItem::init(const std::string& ds_type)
{
    bool changes = false;
    if (!is_init_ || ds_type != ds_type_)
    {
        ds_type_ = ds_type;

        auto font_bold = font(0);
        font_bold.setBold(true);

        setCheckState(0, Qt::Checked);
        setText(0, QString::fromStdString(ds_type_));
        setFont(0, font_bold);

        is_init_ = true;
        changes  = true;
    }

    updateContent();

    return changes;
}

/**
 */
void DataSourceTypeItem::updateContent()
{
    traced_assert(is_init_);
    traced_assert(!ds_type_.empty());

    bool has_sources = false;

    auto& ds_man = widget_->dsManager();
    for (const auto& ds_it : ds_man.dbDataSources())
        if (ds_it->dsType() == ds_type_)
            has_sources = true;

    if (has_sources)
    {
        setCheckState(0, widget_->getUseDSType(ds_type_) ? Qt::Checked : Qt::Unchecked);
        setFlags(flags() | Qt::ItemFlag::ItemIsEnabled);
    }
    else
    {
        setCheckState(0, Qt::Unchecked);
        setFlags(flags() & ~Qt::ItemFlag::ItemIsEnabled);
    }
}

/**************************************************************************************************
 * DataSourceItem
 **************************************************************************************************/

/**
 */
DataSourceItem::DataSourceItem(DataSourcesWidget* widget,
                               DataSourcesWidgetItem* parent)
:   DataSourcesWidgetItem(widget, parent, Type::DataSource)
{
}

/**
 */
bool DataSourceItem::init(unsigned int ds_id)
{
    bool changes = false;
    if (!is_init_ || ds_id != ds_id_)
    {
        ds_id_ = ds_id;

        auto& ds_man = widget_->dsManager();
        traced_assert(ds_man.hasDBDataSource(ds_id));

        const auto& data_source = ds_man.dbDataSource(ds_id);
        ds_ = &data_source;

        std::string ds_name = data_source.name();

        setCheckState(0, Qt::Checked);
        setText(0, QString::fromStdString(ds_name));

        if (!has_widget_)
        {
            auto w = createLinesWidget();
            setItemWidget(1, w);

            has_widget_ = true;

            for (auto lb : line_buttons_)
                lb->init(ds_id_);
        }

        is_init_ = true;
        changes  = true;
    }

    updateContent();

    return changes;
}

/**
 */
void DataSourceItem::updateContent()
{
    traced_assert(is_init_);
    traced_assert(has_widget_);

    setCheckState(0, widget_->getUseDS(ds_id_) ? Qt::Checked : Qt::Unchecked);

    for (auto lb : line_buttons_)
        lb->updateContent();
}

/**
 */
QWidget* DataSourceItem::createLinesWidget()
{
    QWidget* widget = new QWidget();
    widget->setContentsMargins(0, 0, 0, 0);

    QHBoxLayout* button_layout = new QHBoxLayout();
    button_layout->setContentsMargins(0, 1, 0, 1);

    std::string line_str;

    for (unsigned int cnt=0; cnt < 4; ++cnt)
    {
        auto w = new DataSourceLineButton(widget_, cnt, DataSourcesWidget::LineButtonSize);
        button_layout->addWidget(w);

        line_buttons_.push_back(w);
    }

    widget->setLayout(button_layout);

    return widget;
}

/**************************************************************************************************
 * DataSourceCountItem
 **************************************************************************************************/

/**
 */
DataSourceCountItem::DataSourceCountItem(DataSourcesWidget* widget,
                                         DataSourcesWidgetItem* parent)
:   DataSourcesWidgetItem(widget, parent, Type::DataSourceCount)
{
}

/**
*/
bool DataSourceCountItem::init(unsigned int ds_id,
                               const std::string& dbc_name)
{
    bool changes = false;
    if (!is_init_ || ds_id != ds_id_ || dbc_name != dbc_name_)
    {
        ds_id_    = ds_id;
        dbc_name_ = dbc_name;

        auto& ds_man = widget_->dsManager();
        traced_assert(ds_man.hasDBDataSource(ds_id));

        const auto& data_source = ds_man.dbDataSource(ds_id);
        ds_ = &data_source;

        setText(0, QString::fromStdString(dbc_name));

        is_init_ = true;
        changes  = true;
    }

    updateContent();

    return changes;
}

/**
*/
void DataSourceCountItem::updateContent()
{
    traced_assert(is_init_);
    traced_assert(!dbc_name_.empty());

    auto num_inserted = ds_->numInsertedSummedLinesMap();
    auto it = num_inserted.find(dbc_name_);
    traced_assert(it != num_inserted.end());

    setText(2, QString::number(ds_->numLoaded(dbc_name_)));
    setText(3, QString::number(it->second));
}

/**************************************************************************************************
 * DataSourceLineButton
 **************************************************************************************************/

/**
 */
DataSourceLineButton::DataSourceLineButton(DataSourcesWidget* widget, 
                                           unsigned int line_id,
                                           unsigned int button_size_px)
:   widget_ (widget )
,   line_id_(line_id)
{
    traced_assert(widget_);

    line_str_ = "L" + std::to_string(line_id_ + 1);

    setText(QString::fromStdString(line_str_));

    setFixedSize(button_size_px, button_size_px);
    setCheckable(true);

    bool dark_mode = COMPASS::instance().darkMode();

    if (dark_mode)
    {
        setStyleSheet(" QPushButton:pressed { border: 3px outset white; } " \
                      " QPushButton:checked { border: 3px outset white; }");
        }
    else
    {
        setStyleSheet(" QPushButton:pressed { border: 3px outset; } " \
                      " QPushButton:checked { border: 3px outset; }");
    }

    setProperty("Line ID", line_id_);

    connect(this, &QPushButton::toggled, [ = ] (bool ok) { widget_->lineChanged(ds_id_, line_id_, ok); });

    QSizePolicy sp_retain = widget->sizePolicy();
    sp_retain.setRetainSizeWhenHidden(true);
    setSizePolicy(sp_retain);
}

/**
 */
bool DataSourceLineButton::init(unsigned int ds_id)
{
    bool changes = false;
    if (!is_init_ || ds_id != ds_id_)
    {
        ds_id_ = ds_id;

        auto& ds_man = widget_->dsManager();
        traced_assert(ds_man.hasDBDataSource(ds_id_));

        auto& ds_src = ds_man.dbDataSource(ds_id_);
        ds_ = &ds_src;

        is_init_ = true;
        changes  = true;
    }

    updateContent();

    return changes;
}

/**
 */
void DataSourceLineButton::updateContent()
{
    traced_assert(is_init_);

    AppMode app_mode = COMPASS::instance().appMode();

    bool live_mode = app_mode == AppMode::LivePaused || app_mode == AppMode::LiveRunning;
    bool dark_mode = COMPASS::instance().darkMode();

    auto& ds_man = widget_->dsManager();

    if (live_mode)
    {
        std::map<unsigned int, std::map<std::string, std::shared_ptr<DataSourceLineInfo>>> net_lines = ds_man.getNetworkLines();

        bool hidden = !net_lines.count(ds_id_) || !net_lines.at(ds_id_).count(line_str_); // hide if not defined

        setHidden(hidden);

        if (!hidden)
        {
            bool disabled = app_mode == AppMode::LivePaused ? !ds_man.dbDataSource(ds_id_).hasNumInserted(line_id_) : false;
            setDisabled(disabled);

            if (disabled)
            {
                setChecked(false);

                QPalette pal = palette();
                if (dark_mode)
                    pal.setColor(QPalette::Button, QColor(Qt::darkGray));
                else
                    pal.setColor(QPalette::Button, QColor(Qt::lightGray));

                setAutoFillBackground(true);
                setPalette(pal);
                update();
            }
            else
            {
                setChecked(widget_->getUseDSLine(ds_id_, line_id_));

                boost::posix_time::ptime current_time = Utils::Time::currentUTCTime();

                if (ds_->hasLiveData(line_id_, current_time))
                {
                    QPalette pal = palette();

                    if (dark_mode)
                        pal.setColor(QPalette::Button, QColor(Qt::darkGreen));
                    else
                        pal.setColor(QPalette::Button, QColor(Qt::green));

                    setAutoFillBackground(true);
                    setPalette(pal);
                    update();
                }
                else
                {
                    QPalette pal = palette();

                    if (dark_mode)
                        pal.setColor(QPalette::Button, QColor(Qt::darkGray));
                    else
                        pal.setColor(QPalette::Button, QColor(Qt::lightGray));

                    setAutoFillBackground(true);
                    setPalette(pal);
                    update();
                }
            }
        }
    }
    else
    {
        std::map<unsigned int, unsigned int> inserted_lines = ds_->numInsertedLinesMap();

        setChecked(widget_->getUseDSLine(ds_id_, line_id_));
        setHidden(!inserted_lines.count(line_id_)); // hide if no data
    }
}

/**************************************************************************************************
 * DataSourcesWidget
 **************************************************************************************************/

 const int DataSourcesWidget::LineButtonSize = 25;

/**
 */
DataSourcesWidget::DataSourcesWidget(bool can_show_counts, DataSourceManager& ds_man)
:   can_show_counts_(can_show_counts), ds_man_(ds_man)
{
    createUI();
}

/**
 */
DataSourcesWidget::~DataSourcesWidget() = default;


/**
 */
void DataSourcesWidget::createUI()
{
    QFont font_bold;
    font_bold.setBold(true);

    QVBoxLayout* main_layout = new QVBoxLayout();
    setLayout(main_layout);

    // buttons
    QHBoxLayout* button_layout = new QHBoxLayout();

    main_layout->addLayout(button_layout);

    // tree widget
    tree_widget_ = new QTreeWidget;

    QStringList header_labels;
    header_labels << "Name";
    header_labels << "Lines";

    if (can_show_counts_)
    {
        header_labels << "Loaded";
        header_labels << "Count";
    }

    tree_widget_->setColumnCount(header_labels.size());
    tree_widget_->setHeaderLabels(header_labels);
    tree_widget_->header()->setSectionResizeMode(QHeaderView::ResizeMode::ResizeToContents);

    connect(tree_widget_, &QTreeWidget::itemChanged, this, &DataSourcesWidget::itemChanged);

    main_layout->addWidget(tree_widget_);

    // update
    updateContent(true);
}

void DataSourcesWidget::addActionsToConfigMenu(QMenu* menu)
{
    QAction* sel_dstyp_action = menu->addAction("Select All DSTypes");
    connect(sel_dstyp_action, &QAction::triggered, this, &DataSourcesWidget::selectAllDSTypes);

    QAction* desel_dstyp_action = menu->addAction("Deselect All DSTypes");
    connect(desel_dstyp_action, &QAction::triggered, this, &DataSourcesWidget::deselectAllDSTypes);

    menu->addSeparator();

    QMenu* select_ds = menu->addMenu("Select Data Sources");

    QAction* sel_ds_action = select_ds->addAction("All");
    connect(sel_ds_action, &QAction::triggered, this, &DataSourcesWidget::selectAllDataSources);

    for (const auto& ds_type_it : ds_man_.data_source_types_)
    {
        QAction* action = select_ds->addAction(("From "+ds_type_it).c_str());
        action->setProperty("ds_type", ds_type_it.c_str());
        connect(action, &QAction::triggered, this, &DataSourcesWidget::selectDSTypeSpecificDataSources);
    }

    QMenu* deselect_ds = menu->addMenu("Deselect Data Sources");

    QAction* desel_ds_action = deselect_ds->addAction("All");
    connect(desel_ds_action, &QAction::triggered, this, &DataSourcesWidget::deselectAllDataSources);

    for (const auto& ds_type_it : ds_man_.data_source_types_)
    {
        QAction* action = deselect_ds->addAction(("From "+ds_type_it).c_str());
        action->setProperty("ds_type", ds_type_it.c_str());
        connect(action, &QAction::triggered, this, &DataSourcesWidget::deselectDSTypeSpecificDataSources);
    }

    menu->addSeparator();

    QMenu* set_lines = menu->addMenu("Set Line");

    QAction* desel_line_action = set_lines->addAction("Deselect All");
    connect(desel_line_action, &QAction::triggered, this, &DataSourcesWidget::deselectAllLines);

    for (unsigned int cnt=0; cnt < 4; ++cnt)
    {
        QAction* desel_line_action = set_lines->addAction(("Select " + Utils::String::lineStrFrom(cnt)).c_str());
        desel_line_action->setProperty("line_id", cnt);
        connect(desel_line_action, &QAction::triggered, this, &DataSourcesWidget::selectSpecificLines);
    }

    menu->addSeparator();

    QAction* show_cnt_action = menu->addAction("Toggle Show Counts");
    connect(show_cnt_action, &QAction::triggered, this, &DataSourcesWidget::toogleShowCounts);
}

/**
 */
void DataSourcesWidget::clear()
{
    tree_widget_->clear();
}

/**
 */
int DataSourcesWidget::generateContent(bool force_rebuild)
{
    logdbg;

    tree_widget_->blockSignals(true);

    //clear everything to force a complete rebuild
    if (force_rebuild)
        clear();

    const auto& data_source_types = DataSourceManager::data_source_types_;

    //create needed items
    int n    = (int)data_source_types.size();
    int ncur = tree_widget_->topLevelItemCount();

    if (n > ncur)
    {
        //add more items if needed
        for (int i = ncur; i < n; ++i)
        {
            auto dstype_item = new DataSourceTypeItem(this, nullptr);
            tree_widget_->addTopLevelItem(dstype_item);
        }
    }
    else if (n < ncur)
    {
        //remove unneeded items
        while (tree_widget_->topLevelItemCount() > n)
        {
            auto item = tree_widget_->takeTopLevelItem(0);
            delete item;
        }
    }

    //configure data source type items
    unsigned int cnt = 0;
    int changes = 0;
    for (const auto& ds_type_name : data_source_types)
    {
        logdbg << "type " << ds_type_name << " cnt " << cnt;

        auto item = dynamic_cast<DataSourceTypeItem*>(tree_widget_->topLevelItem(cnt));
        traced_assert(item);

        changes += generateDataSourceType(item, ds_type_name);

        ++cnt;
    }

    tree_widget_->blockSignals(false);
    tree_widget_->expandAll();

    //updateAdditionalInfo();

    return changes;
}

/**
 */
int DataSourcesWidget::generateDataSourceType(DataSourceTypeItem* item,
                                              const std::string& ds_type_name)
{
    //init item
    int changes = item->init(ds_type_name) ? 1 : 0;

    const auto& db_data_sources = ds_man_.dbDataSources();

    //create needed items
    int ncur = item->childCount();
    int n    = 0;
    for (const auto& ds_it : db_data_sources)
        if (ds_it->dsType() == ds_type_name)
            ++n;

    if (n > ncur)
    {
        //add more items if needed
        for (int i = ncur; i < n; ++i)
        {
            auto ds_item = new DataSourceItem(this, item);
            item->addChild(ds_item);
        }
    }
    else if (n < ncur)
    {
        //remove unneeded items
        while (item->childCount() > n)
        {
            auto child = item->child(0);
            item->removeChild(child);
            delete child;
        }
    }

    //configure data source items
    int cnt = 0;
    for (const auto& ds_it : db_data_sources)
    {
        if (ds_it->dsType() == ds_type_name)
        {
            auto ds_item = dynamic_cast<DataSourceItem*>(item->child(cnt));
            traced_assert(ds_item);

            changes += generateDataSource(ds_item, item, *ds_it);

            ++cnt;
        }
    }

    return changes;
}

/**
 */
int DataSourcesWidget::generateDataSource(DataSourceItem* item,
                                          DataSourcesWidgetItem* parent_item, 
                                          const dbContent::DBDataSource& data_source)
{
    unsigned int ds_id   = Utils::Number::dsIdFrom(data_source.sac(), data_source.sic());
    std::string  ds_name = data_source.name();

    logdbg << "create '" << data_source.dsType() << "' '" << ds_name << "'";

    //init item
    int changes = item->init(ds_id) ? 1 : 0;

    //handle count items
    bool show_counts = can_show_counts_ && getShowCounts();
    if (!show_counts)
    {
        //no counts shown => remove any existing children
        while (item->childCount() > 0)
        {
            auto child = item->child(0);
            item->removeChild(child);
            delete child;
        }
    }
    else
    {
        // counts shown => create needed items
        auto count_map = data_source.numInsertedSummedLinesMap();
        int n    = (int)count_map.size();
        int ncur = item->childCount();

        if (n > ncur)
        {
            //add more items if needed
            for (int i = ncur; i < n; ++i)
            {
                auto ds_cnt_item = new DataSourceCountItem(this, item);
                item->addChild(ds_cnt_item);
            }
        }
        else if (n < ncur)
        {
            //remove unneeded items
            while (item->childCount() > n)
            {
                auto child = item->child(0);
                item->removeChild(child);
                delete child;
            }
        }

        //configure count items
        int cnt = 0;
        for (auto& cnt_it : count_map)
        {
            auto ds_cnt_item = dynamic_cast<DataSourceCountItem*>(item->child(cnt));
            traced_assert(ds_cnt_item);

            changes += generateDataSourceCount(ds_cnt_item, item, data_source, cnt_it.first);

            ++cnt;
        }
    }

    return changes;
}

/**
 */
int DataSourcesWidget::generateDataSourceCount(DataSourceCountItem* item,
                                               DataSourcesWidgetItem* parent_item,
                                               const dbContent::DBDataSource& data_source,
                                               const std::string& dbc_name)
{
    //init item
    int changes = item->init(data_source.id(), dbc_name) ? 1 : 0;

    return changes;
}

/**
 */
void DataSourcesWidget::updateContent(bool recreate_required)
{
    logdbg << "recreate_required " << recreate_required
           << " num data sources " << ds_man_.dbDataSources().size();

    int changes = generateContent(recreate_required);

    logdbg << "update generated " << changes << " change(s)";
}

/**
 */
void DataSourcesWidget::itemChanged(QTreeWidgetItem *item, int column)
{
    auto w_item = dynamic_cast<DataSourcesWidgetItem*>(item);
    if (!w_item || !w_item->isInit())
        return;

    //react on item changes
    if (w_item->type() == DataSourcesWidgetItem::Type::DataSourceType)
    {   
        auto ds_type_item = dynamic_cast<DataSourceTypeItem*>(w_item);
        traced_assert(ds_type_item);

        if (column == 0)
        {
            bool load = ds_type_item->checkState(0) == Qt::Checked;
            setUseDSType(ds_type_item->dsType(), load);

            loginf << "ds_type " << ds_type_item->dsType() << " load " << load;
        }
    }
    else if (w_item->type() == DataSourcesWidgetItem::Type::DataSource)
    {   
        auto ds_item = dynamic_cast<DataSourceItem*>(w_item);
        traced_assert(ds_item);

        if (column == 0)
        {
            bool load = ds_item->checkState(0) == Qt::Checked;
            setUseDS(ds_item->dsID(), load);

            loginf << "ds_id " << ds_item->dsID() << " load " << load;
        }
    }
    else if (w_item->type() == DataSourcesWidgetItem::Type::DataSourceCount)
    {   
        auto ds_cnt_item = dynamic_cast<DataSourceCountItem*>(w_item);
        traced_assert(ds_cnt_item);

        //nothing to do yet
    }
}

/**
 */
void DataSourcesWidget::lineChanged(unsigned int ds_id, unsigned int ds_line, bool use)
{
    logdbg << "ds_id " << ds_id << " line " << ds_line << " use " << use;

    setUseDSLine(ds_id, ds_line, use);
}

namespace
{
    void updateContentRecursive(QTreeWidgetItem* item)
    {
        auto w_item = dynamic_cast<DataSourcesWidgetItem*>(item);
        if (w_item)
            w_item->updateContent();

        for (int i = 0; i < item->childCount(); ++i)
            updateContentRecursive(item->child(i));
    }
}

/**
 * Only updates all contents.
 */
void DataSourcesWidget::updateAllContent()
{
    for (int i = 0; i < tree_widget_->topLevelItemCount(); ++i)
        updateContentRecursive(tree_widget_->topLevelItem(i));

}


/**
 */
void DataSourcesWidget::setUseDSType(const std::string& ds_type_name, bool use)
{
    ds_man_.dsTypeLoadingWanted(ds_type_name, use);
}

/**
 */
bool DataSourcesWidget::getUseDSType(const std::string& ds_type_name) const
{
    return ds_man_.dsTypeLoadingWanted(ds_type_name);
}

/**
 */
void DataSourcesWidget::setUseDS(unsigned int ds_id, bool use)
{
    ds_man_.dbDataSource(ds_id).loadingWanted(use);
}

/**
 */
bool DataSourcesWidget::getUseDS(unsigned int ds_id) const
{
    return ds_man_.dbDataSource(ds_id).loadingWanted();
}

/**
 */
void DataSourcesWidget::setUseDSLine(unsigned int ds_id, unsigned int ds_line, bool use)
{
    ds_man_.dbDataSource(ds_id).lineLoadingWanted(ds_line, use);
}

/**
 */
bool DataSourcesWidget::getUseDSLine(unsigned int ds_id, unsigned int ds_line) const
{
    return ds_man_.dbDataSource(ds_id).lineLoadingWanted(ds_line);
}

/**
 */
void DataSourcesWidget::setShowCounts(bool show) const
{
    ds_man_.config().load_widget_show_counts_ = show;
}

/**
 */
bool DataSourcesWidget::getShowCounts() const
{
    return ds_man_.config().load_widget_show_counts_;
}

/**
 */
void DataSourcesWidget::selectAllDSTypes()
{
    loginf;

    for (auto& ds_type_name : DataSourceManager::data_source_types_)
        setUseDSType(ds_type_name, true);

    updateContent();
}

/**
 */
void DataSourcesWidget::deselectAllDSTypes()
{
    loginf;

    for (auto& ds_type_name : DataSourceManager::data_source_types_)
        setUseDSType(ds_type_name, false);

    updateContent();
}

/**
 */
void DataSourcesWidget::selectAllDataSources()
{
    loginf;

    for (const auto& ds_it : ds_man_.dbDataSources())
        setUseDS(ds_it->id(), true);

    updateContent();
}

/**
 */
void DataSourcesWidget::deselectAllDataSources()
{
    loginf;

    for (const auto& ds_it : ds_man_.dbDataSources())
        setUseDS(ds_it->id(), false);

    updateContent();
}

/**
 */
void DataSourcesWidget::selectDSTypeSpecificDataSources()
{
    QAction* action = dynamic_cast<QAction*>(sender());
    traced_assert(action);

    std::string ds_type = action->property("ds_type").toString().toStdString();

    loginf << "ds_type '" << ds_type << "'";

    for (const auto& ds_it : ds_man_.dbDataSources())
        if (ds_it->dsType() == ds_type)
            setUseDS(ds_it->id(), true);

    updateContent();
}

/**
 */
void DataSourcesWidget::deselectDSTypeSpecificDataSources()
{
    QAction* action = dynamic_cast<QAction*>(sender());
    traced_assert(action);

    std::string ds_type = action->property("ds_type").toString().toStdString();

    loginf << "ds_type '" << ds_type << "'";

    for (const auto& ds_it : ds_man_.dbDataSources())
        if (ds_it->dsType() == ds_type)
            setUseDS(ds_it->id(), false);

    updateContent();
}

/**
 */
void DataSourcesWidget::deselectAllLines()
{
    loginf;

    for (const auto& ds_it : ds_man_.dbDataSources())
        for (int line = 0; line < 4; ++line)
            setUseDSLine(ds_it->id(), line, false);

    updateContent();
}

/**
 */
void DataSourcesWidget::selectSpecificLines()
{
    QAction* action = dynamic_cast<QAction*>(sender());
    traced_assert(action);

    unsigned int line_id = action->property("line_id").toUInt();

    loginf << "line_id " << line_id;

    for (const auto& ds_it : ds_man_.dbDataSources())
        setUseDSLine(ds_it->id(), line_id, true);

    updateContent();
}

/**
 */
void DataSourcesWidget::toogleShowCounts()
{
    loginf;

    setShowCounts(!getShowCounts());

    updateContent();
}