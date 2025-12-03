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

#include "datasourceswidgetbase.h"
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
 * DataSourcesWidgetItemBase
 **************************************************************************************************/

/**
 */
DataSourcesWidgetItemBase::DataSourcesWidgetItemBase(DataSourcesWidgetBase* widget,
                                                     DataSourcesWidgetItemBase* parent,
                                                     Type type)
:   widget_(widget)
,   parent_(parent) 
,   type_  (type  )
{
    traced_assert(widget_);
}

/**
 */
void DataSourcesWidgetItemBase::setItemWidget(int column, QWidget* w)
{
    traced_assert(widget_);
    traced_assert(widget_->tree_widget_);

    widget_->tree_widget_->setItemWidget(this, column, w);
}

/**
 */
void DataSourcesWidgetItemBase::notifyChanges()
{
    has_changes_ = true;
}

/**
 */
bool DataSourcesWidgetItemBase::updateContent()
{
    bool changes = false;

    //needs init?
    if (!is_init_)
    {
        //init now
        init_impl();
        is_init_ = true;
        changes = true;
    }

    //needs changed content update?
    if (has_changes_)
    {
        //update to content changes
        updateContentChanges_impl();
        has_changes_ = false;
        changes = true;
    }

    //run default content update
    updateContent_impl();

    return changes;
}

/**************************************************************************************************
 * DataSourceTypeItemBase
 **************************************************************************************************/

/**
 */
DataSourceTypeItemBase::DataSourceTypeItemBase(DataSourcesWidgetBase* widget,
                                               DataSourcesWidgetItemBase* parent)
:   DataSourcesWidgetItemBase(widget, parent, Type::DataSourceType)
{
}

/**
 */
void DataSourceTypeItemBase::setDSType(const std::string& ds_type)
{
    if (ds_type_ == ds_type)
        return;

    ds_type_ = ds_type;

    notifyChanges();
}

/**
 */
void DataSourceTypeItemBase::init_impl()
{
    auto font_bold = font(0);
    font_bold.setBold(true);

    setFont(0, font_bold);
}

/**
 */
void DataSourceTypeItemBase::updateContentChanges_impl()
{
    traced_assert(is_init_);
    traced_assert(!ds_type_.empty());

    setText(0, QString::fromStdString(ds_type_));
}

/**
 */
void DataSourceTypeItemBase::updateContent_impl()
{
    traced_assert(is_init_);
    traced_assert(!ds_type_.empty());

    bool has_sources = false;

    //enable or disable the item based on availability in data sources
    auto& ds_man = widget_->dsManager();
    for (const auto& ds_it : ds_man.dbDataSources())
        if (ds_it->dsType() == ds_type_)
            has_sources = true;

    if (has_sources)
    {
        setFlags(flags() | Qt::ItemFlag::ItemIsEnabled);
    }
    else
    {
        setFlags(flags() & ~Qt::ItemFlag::ItemIsEnabled);
    }
}

/**************************************************************************************************
 * DataSourceItemBase
 **************************************************************************************************/

/**
 */
DataSourceItemBase::DataSourceItemBase(DataSourcesWidgetBase* widget,
                                       DataSourcesWidgetItemBase* parent)
:   DataSourcesWidgetItemBase(widget, parent, Type::DataSource)
{
}

/**
 */
void DataSourceItemBase::setDSID(unsigned int ds_id)
{
    if (ds_id_ == ds_id)
        return;

    ds_id_ = ds_id;

    notifyChanges();
}

/**
 */
void DataSourceItemBase::init_impl()
{
    //nothing to do yet
}

/**
 */
void DataSourceItemBase::updateContentChanges_impl()
{
    traced_assert(is_init_);

    auto& ds_man = widget_->dsManager();
    traced_assert(ds_man.hasDBDataSource(ds_id_));

    const auto& data_source = ds_man.dbDataSource(ds_id_);
    ds_ = &data_source;

    std::string ds_name = data_source.name();
    setText(0, QString::fromStdString(ds_name));
}

/**
 */
void DataSourceItemBase::updateContent_impl()
{
    traced_assert(is_init_);

    //nothing to do yet
}

/**************************************************************************************************
 * DataSourceCountItemBase
 **************************************************************************************************/

/**
 */
DataSourceCountItemBase::DataSourceCountItemBase(DataSourcesWidgetBase* widget,
                                                 DataSourcesWidgetItemBase* parent)
:   DataSourcesWidgetItemBase(widget, parent, Type::DataSourceCount)
{
}

/**
*/
void DataSourceCountItemBase::setup(unsigned int ds_id,
                                    const std::string& dbc_name)
{
    if (ds_id_ == ds_id &&
        dbc_name_ == dbc_name)
        return;

    ds_id_    = ds_id;
    dbc_name_ = dbc_name;

    notifyChanges();
}

/**
*/
void DataSourceCountItemBase::init_impl()
{
    //nothing to do yet
}

/**
*/
void DataSourceCountItemBase::updateContentChanges_impl()
{
    traced_assert(is_init_);
    traced_assert(!dbc_name_.empty());

    auto& ds_man = widget_->dsManager();
    traced_assert(ds_man.hasDBDataSource(ds_id_));

    const auto& data_source = ds_man.dbDataSource(ds_id_);
    ds_ = &data_source;

    setText(0, QString::fromStdString(dbc_name_));
}

/**
*/
void DataSourceCountItemBase::updateContent_impl()
{
    traced_assert(is_init_);
    traced_assert(!dbc_name_.empty());

    //set current counts
    auto num_inserted = ds_->numInsertedSummedLinesMap();
    auto it = num_inserted.find(dbc_name_);
    traced_assert(it != num_inserted.end());

    setText(2, QString::number(ds_->numLoaded(dbc_name_)));
    setText(3, QString::number(it->second));
}

/**************************************************************************************************
 * DataSourcesWidgetBase
 **************************************************************************************************/

/**
 */
DataSourcesWidgetBase::DataSourcesWidgetBase(DataSourceManager& ds_man,
                                             bool can_show_counts,
                                             bool init_ui)
:   ds_man_(ds_man), can_show_counts_(can_show_counts)
{
    if (init_ui)
        init();
}

/**
 */
DataSourcesWidgetBase::~DataSourcesWidgetBase() = default;

/**
 */
void DataSourcesWidgetBase::init()
{
    if (init_)
        return;

    createUI();

    init_ = true;
}

/**
 */
void DataSourcesWidgetBase::createUI()
{
    QVBoxLayout* main_layout = new QVBoxLayout();
    main_layout->setContentsMargins(0, 0, 0, 0);
    setLayout(main_layout);

    // tree widget
    top_layout_ = new QHBoxLayout();
    top_layout_->setContentsMargins(0, 0, 0, 0);

    tree_widget_ = new QTreeWidget;

    QStringList header_labels = getColumnHeaders();

    tree_widget_->setColumnCount(header_labels.size());
    tree_widget_->setHeaderLabels(header_labels);
    tree_widget_->header()->setSectionResizeMode(QHeaderView::ResizeMode::ResizeToContents);

    connect(tree_widget_, &QTreeWidget::itemChanged, this, &DataSourcesWidgetBase::itemChanged);
    connect(tree_widget_, &QTreeWidget::itemSelectionChanged, 
            this, &DataSourcesWidgetBase::onItemSelectionChanged);

    top_layout_->addWidget(tree_widget_);
    main_layout->addLayout(top_layout_);

    // update
    updateContent(true);
}

/**
 */
QStringList DataSourcesWidgetBase::getColumnHeaders() const
{
    QStringList headers;
    headers << "Name";
    
    if (can_show_counts_)
    {
        headers << "Loaded";
        headers << "Count";
    }

    headers += getCustomColumnHeaders();

    return headers;
}

/**
 */
bool DataSourcesWidgetBase::showsCounts() const
{
    return can_show_counts_;
}

/**
 */
DataSourceTypeItemBase* DataSourcesWidgetBase::createDSTypeItem(DataSourcesWidgetItemBase* parent)
{
    return new DataSourceTypeItemBase(this, parent);
}

/**
 */
DataSourceItemBase* DataSourcesWidgetBase::createDSItem(DataSourcesWidgetItemBase* parent)
{
    return new DataSourceItemBase(this, parent);
}

/**
 */
DataSourceCountItemBase* DataSourcesWidgetBase::createDSCountItem(DataSourcesWidgetItemBase* parent)
{
    return new DataSourceCountItemBase(this, parent);
}

/**
 */
void DataSourcesWidgetBase::clear()
{
    if (!tree_widget_)
        return;
    
    tree_widget_->clear();
}

/**
 */
int DataSourcesWidgetBase::generateContent(bool force_rebuild)
{
    logdbg;

    if (!tree_widget_)
        return 0;

    tree_widget_->blockSignals(true);

    //clear everything to force a complete rebuild
    if (force_rebuild)
        clear();

    const auto& data_source_types = DataSourceManager::data_source_types_;

    std::vector<std::string> filtered_types;
    for (const auto& ds_type : data_source_types)
        if (showDSType(ds_type))
            filtered_types.push_back(ds_type);

    //create needed items
    int n    = (int)filtered_types.size();
    int ncur = tree_widget_->topLevelItemCount();

    if (n > ncur)
    {
        //add more items if needed
        for (int i = ncur; i < n; ++i)
        {
            auto dstype_item = createDSTypeItem(nullptr);
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
    for (const auto& ds_type_name : filtered_types)
    {
        logdbg << "type " << ds_type_name << " cnt " << cnt;

        auto item = dynamic_cast<DataSourceTypeItemBase*>(tree_widget_->topLevelItem(cnt));
        traced_assert(item);

        changes += generateDataSourceType(item, ds_type_name);

        ++cnt;
    }

    tree_widget_->blockSignals(false);
    tree_widget_->expandAll();

    return changes;
}

/**
 */
int DataSourcesWidgetBase::generateDataSourceType(DataSourceTypeItemBase* item,
                                                  const std::string& ds_type_name)
{
    //init item
    item->setDSType(ds_type_name);
    int changes = item->updateContent() ? 1 : 0;

    const auto& db_data_sources = ds_man_.dbDataSources();

    //create needed items
    int ncur = item->childCount();
    int n    = 0;
    for (const auto& ds_it : db_data_sources)
        if (ds_it->dsType() == ds_type_name && showDS(ds_it->id()))
            ++n;

    if (n > ncur)
    {
        //add more items if needed
        for (int i = ncur; i < n; ++i)
        {
            auto ds_item = createDSItem(item);
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
        if (ds_it->dsType() == ds_type_name && showDS(ds_it->id()))
        {
            auto ds_item = dynamic_cast<DataSourceItemBase*>(item->child(cnt));
            traced_assert(ds_item);

            changes += generateDataSource(ds_item, item, *ds_it);

            ++cnt;
        }
    }

    return changes;
}

/**
 */
int DataSourcesWidgetBase::generateDataSource(DataSourceItemBase* item,
                                              DataSourcesWidgetItemBase* parent_item, 
                                              const dbContent::DBDataSource& data_source)
{
    unsigned int ds_id   = Utils::Number::dsIdFrom(data_source.sac(), data_source.sic());
    std::string  ds_name = data_source.name();

    logdbg << "create '" << data_source.dsType() << "' '" << ds_name << "'";

    //init item
    item->setDSID(ds_id);
    int changes = item->updateContent() ? 1 : 0;

    //handle count items
    bool show_counts = showsCounts();
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
                auto ds_cnt_item = createDSCountItem(item);
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
            auto ds_cnt_item = dynamic_cast<DataSourceCountItemBase*>(item->child(cnt));
            traced_assert(ds_cnt_item);

            changes += generateDataSourceCount(ds_cnt_item, item, data_source, cnt_it.first);

            ++cnt;
        }
    }

    return changes;
}

/**
 */
int DataSourcesWidgetBase::generateDataSourceCount(DataSourceCountItemBase* item,
                                                   DataSourcesWidgetItemBase* parent_item,
                                                   const dbContent::DBDataSource& data_source,
                                                   const std::string& dbc_name)
{
    //init item
    item->setup(data_source.id(), dbc_name);
    int changes = item->updateContent() ? 1 : 0;

    return changes;
}

/**
 */
void DataSourcesWidgetBase::updateContent(bool recreate_required)
{
    logdbg << "recreate_required " << recreate_required
           << " num data sources " << ds_man_.dbDataSources().size();

    int changes = generateContent(recreate_required);

    logdbg << "update generated " << changes << " change(s)";
}

/**
 * Handle tree widget selection changes
 */
void DataSourcesWidgetBase::onItemSelectionChanged()
{
    QList<QTreeWidgetItem*> selected_items = tree_widget_->selectedItems();
    if (selected_items.isEmpty())
        return;
    
    // Get the first selected item
    QTreeWidgetItem* item = selected_items.first();
    auto ds_item = dynamic_cast<DataSourcesWidgetItemBase*>(item);
    
    if (!ds_item || !ds_item->isInit())
        return;
    
    if (ds_item->type() == DataSourcesWidgetItemBase::Type::DataSource)
    {
        auto data_source_item = dynamic_cast<DataSourceItemBase*>(ds_item);
        loginf << "selected data source " << data_source_item->dsID();
        // React to data source selection

        emit dataSourceSelectedSignal(data_source_item->dsID());
    }
}

/**
 */
void DataSourcesWidgetBase::itemChanged(QTreeWidgetItem *item, int column)
{
    auto w_item = dynamic_cast<DataSourcesWidgetItemBase*>(item);
    if (!w_item || !w_item->isInit())
        return;

    //react on item changes
    if (w_item->type() == DataSourcesWidgetItemBase::Type::DataSourceType)
    {   
        auto ds_type_item = dynamic_cast<DataSourceTypeItemBase*>(w_item);
        traced_assert(ds_type_item);

        dsTypeItemChanged(ds_type_item, column);
    }
    else if (w_item->type() == DataSourcesWidgetItemBase::Type::DataSource)
    {   
        auto ds_item = dynamic_cast<DataSourceItemBase*>(w_item);
        traced_assert(ds_item);

        dsItemChanged(ds_item, column);
    }
    else if (w_item->type() == DataSourcesWidgetItemBase::Type::DataSourceCount)
    {   
        auto ds_cnt_item = dynamic_cast<DataSourceCountItemBase*>(w_item);
        traced_assert(ds_cnt_item);

        dsCountItemChanged(ds_cnt_item, column);
    }
}

namespace
{
    void updateContentRecursive(QTreeWidgetItem* item)
    {
        auto w_item = dynamic_cast<DataSourcesWidgetItemBase*>(item);
        if (w_item)
            w_item->updateContent();

        for (int i = 0; i < item->childCount(); ++i)
            updateContentRecursive(item->child(i));
    }
}

/**
 * Only updates all contents.
 */
void DataSourcesWidgetBase::updateAllContent()
{
    if (!tree_widget_)
        return;

    for (int i = 0; i < tree_widget_->topLevelItemCount(); ++i)
        updateContentRecursive(tree_widget_->topLevelItem(i));
}
