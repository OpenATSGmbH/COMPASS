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

#include "tableviewdatawidget.h"
#include "tableviewwidget.h"
#include "tableview.h"
#include "allbuffertablewidget.h"
#include "allbuffertablemodel.h"
#include "compass.h"
#include "buffer.h"
#include "data_source.h"
#include "db_context_manager.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/metavariable.h"
#include "color_provider.h"
#include "dbcontentlayer.h"
#include "layertreemodel.h"
#include "tableleafpayload.h"
#include "logger.h"
#include "number.h"
#include "stringconv.h"

#include "boost/date_time/posix_time/posix_time.hpp"

#include <QApplication>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QTableView>
#include <QHeaderView>

TableViewDataWidget::TableViewDataWidget(TableViewWidget* view_widget,
                                             QWidget* parent,
                                             Qt::WindowFlags f)
:   ViewDataWidget(view_widget, parent, f)
{
    view_ = view_widget->getView();
    traced_assert(view_);

    data_source_ = view_->getDataSource();
    traced_assert(data_source_);

    QHBoxLayout* layout = new QHBoxLayout();
    layout->setMargin(0);

    all_buffer_table_widget_ = new AllBufferTableWidget(*view_, *data_source_);
    layout->addWidget(all_buffer_table_widget_);
    connect(all_buffer_table_widget_, &AllBufferTableWidget::exportDoneSignal, this,
            &TableViewDataWidget::exportDoneSlot);

    setLayout(layout);
}

TableViewDataWidget::~TableViewDataWidget() = default;

void TableViewDataWidget::attachLayerPanel(DBContentRootItem* root, LayerTreeModel* layer_model)
{
    db_content_root_ = root;
    layer_model_     = layer_model;

    // Single fire at end of any visibility change (group toggles cascade
    // through multiple leaves but emit hiddenChangedSignal once).
    connect(layer_model_, &LayerTreeModel::hiddenChangedSignal,
            this, [this]() {
                pushAllowedSliceKeysToModel();
                redrawData(true);
            });

    rebuildLayerTree();
}

void TableViewDataWidget::clearData_impl()
{
    logdbg << "begin";

    if (all_buffer_table_widget_)
        all_buffer_table_widget_->clear();

    logdbg << "end";
}

void TableViewDataWidget::clearIntermediateRedrawData_impl()
{
    //nothing to do here
}

void TableViewDataWidget::loadingStarted_impl()
{
    loginf;
    //nothing to do yet
}

void TableViewDataWidget::updateData_impl(bool requires_reset)
{
    logdbg << "begin";

    //nothing to do yet

    logdbg << "end";
}

void TableViewDataWidget::loadingDone_impl()
{
    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();

    unsigned int num_records = 0;
    for (auto& buf_it : viewData())
        num_records += buf_it.second->size();

    loginf << "begin with " << num_records << " records";

    // Rebuild the layer tree first so the panel reflects current data; the
    // base redraw below will then consult the allowed-slice-keys set we push
    // into the model from rebuildLayerTree.
    rebuildLayerTree();

    //default behavior
    ViewDataWidget::loadingDone_impl();

    boost::posix_time::ptime stop_time = boost::posix_time::microsec_clock::local_time();
    double elapsed_s = (stop_time - start_time).total_milliseconds() / 1000.0;

    loginf << "done with " << num_records << " records in "
           << Utils::String::timeStringFromDouble(elapsed_s, true);
}

ViewDataWidget::DrawState TableViewDataWidget::redrawData_impl(bool recompute)
{
    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();

    unsigned int num_records = 0;
    for (auto& buf_it : viewData())
        num_records += buf_it.second->size();

    loginf << "start - recompute " << recompute << " records " << num_records;

    traced_assert(all_buffer_table_widget_);

    setUpdatesEnabled(false);

    all_buffer_table_widget_->show(viewData());

    selectFirstSelectedRow();

    setUpdatesEnabled(true);

    boost::posix_time::ptime stop_time = boost::posix_time::microsec_clock::local_time();
    double elapsed_s = (stop_time - start_time).total_milliseconds() / 1000.0;

    loginf << "done with " << num_records << " records in "
           << Utils::String::timeStringFromDouble(elapsed_s, true);

    return (all_buffer_table_widget_->rowCount() > 0 ? DrawState::DrawnContent : DrawState::Drawn);
}

void TableViewDataWidget::liveReload_impl()
{
    //implement live reload behavior here
}

void TableViewDataWidget::exportDataSlot()
{
    logdbg;

    if (!all_buffer_table_widget_ || all_buffer_table_widget_->rowCount() == 0)
    {
        QMessageBox msgBox;
        msgBox.setText("Export can not be used without loaded data.");
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.exec();

        emit exportDoneSignal(true);
        return;
    }

    all_buffer_table_widget_->exportSlot();
}

void TableViewDataWidget::exportDoneSlot(bool cancelled)
{
    emit exportDoneSignal(cancelled);
}

void TableViewDataWidget::updateToSettingsChange()
{
    loginf;

    if (all_buffer_table_widget_)
        all_buffer_table_widget_->updateToSettingsChange();
}

void TableViewDataWidget::resetModels()
{
    if (all_buffer_table_widget_)
        all_buffer_table_widget_->resetModel();
}

void TableViewDataWidget::updateToSelection()
{
    if (all_buffer_table_widget_)
        all_buffer_table_widget_->updateToSelection();
}

void TableViewDataWidget::selectFirstSelectedRow()
{
    if (all_buffer_table_widget_)
        all_buffer_table_widget_->selectSelectedRows();
}

AllBufferTableWidget* TableViewDataWidget::getAllBufferTableWidget ()
{
    traced_assert(all_buffer_table_widget_);
    return all_buffer_table_widget_;
}

void TableViewDataWidget::toolChanged_impl(int mode)
{
    //nothing to do here
}

void TableViewDataWidget::viewInfoJSON_impl(nlohmann::json& info) const
{
    nlohmann::json table_infos = nlohmann::json::array();

    auto addTable = [ & ] (const std::string& db_content,
                           const QTableView* table,
                           bool show_only_selected,
                           bool use_presentation, bool ignore_non_target_reports)
    {
        nlohmann::json table_info;

        table_info[ "content"            ] = db_content;
        table_info[ "show_only_selected" ] = show_only_selected;
        table_info[ "use_presentation"   ] = use_presentation;
        table_info[ "ignore_non_target_reports"   ] = ignore_non_target_reports;
        table_info[ "count"              ] = table->model()->rowCount();

        //get properties
        std::vector<std::string> properties;
        for(int i = 0; i < table->model()->columnCount(); i++)
            properties.push_back(table->model()->headerData(i, Qt::Horizontal).toString().toStdString());

        table_info[ "properties" ] = properties;

        //get line zero
        std::vector<std::string> line0;
        if (table->model()->rowCount() > 0)
        {
            for(int i = 0; i < table->model()->columnCount(); i++)
            {
                auto index = table->model()->index(0, i);
                line0.push_back(table->model()->data(index, Qt::DisplayRole).toString().toStdString());
            }
        }

        table_info[ "line0" ] = line0;

        table_infos.push_back(table_info);
    };

    addTable("All",
             all_buffer_table_widget_->table(),
             view_->showOnlySelected(),
             view_->usePresentation(), view_->ignoreNonTargetReports());

    info[ "tables" ] = table_infos;
}

namespace
{
    /// Aggregate a single loaded buffer into per-slice row counts, keyed by
    /// "<ds_type>:<ds_name>:L<n>:<dbcontent>". Mirrors the grouping logic
    /// used by VariableViewStashDataWidget and AllBufferTableModel's filter.
    struct SliceAgg
    {
        std::string ds_type;
        std::string ds_name;
        std::string line;
        std::string dbcontent;
        unsigned int count = 0;
        int          line_index = 0;   // 0..3, for color resolution
    };
}

void TableViewDataWidget::rebuildLayerTree()
{
    if (!db_content_root_ || !layer_model_)
        return;

    auto& compass = view_->compass();
    auto& dbcont_man = compass.dbContentManager();
    auto& ctx_mgr = compass.dbContextManager();

    // Count records per (ds_type, ds_name, line, dbcontent) slice by scanning
    // ds_id + line_id columns on each loaded buffer. Lookups cached.
    std::map<std::string, SliceAgg> agg;  // full_key -> aggregate

    for (auto& buf_it : viewData())
    {
        const std::string& dbcontent_name = buf_it.first;
        Buffer&            buffer         = *buf_it.second;
        if (buffer.size() == 0)
            continue;

        if (!dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_ds_id_) ||
            !dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_line_id_))
            continue;

        const std::string ds_id_name = dbcont_man.metaGetVariable(dbcontent_name,
            dbcontent_vars::meta_var_ds_id_).name();
        const std::string line_id_name = dbcont_man.metaGetVariable(dbcontent_name,
            dbcontent_vars::meta_var_line_id_).name();

        if (!buffer.has<unsigned int>(ds_id_name) ||
            !buffer.has<unsigned int>(line_id_name))
            continue;

        const auto& ds_ids  = buffer.get<unsigned int>(ds_id_name);
        const auto& line_ids = buffer.get<unsigned int>(line_id_name);

        const NullableVector<unsigned char>* sensor_sacs = nullptr;
        const NullableVector<unsigned char>* sensor_sics = nullptr;
        if (dbcontent_name == "CAT063")
        {
            const std::string sac_name = dbcont_man.getVariable(
                dbcontent_name, dbcontent_vars::var_cat063_sensor_sac_).name();
            const std::string sic_name = dbcont_man.getVariable(
                dbcontent_name, dbcontent_vars::var_cat063_sensor_sic_).name();
            if (buffer.has<unsigned char>(sac_name) &&
                buffer.has<unsigned char>(sic_name))
            {
                sensor_sacs = &buffer.get<unsigned char>(sac_name);
                sensor_sics = &buffer.get<unsigned char>(sic_name);
            }
        }

        const unsigned int n = buffer.size();
        for (unsigned int i = 0; i < n; ++i)
        {
            if (ds_ids.isNull(i) || line_ids.isNull(i))
                continue;

            unsigned int ds_id = ds_ids.get(i);
            const unsigned int line_id = line_ids.get(i);

            if (sensor_sacs && sensor_sics)
            {
                if (sensor_sacs->isNull(i) || sensor_sics->isNull(i))
                    continue;
                ds_id = Utils::Number::dsIdFrom(sensor_sacs->get(i), sensor_sics->get(i));
            }

            std::string ds_type, ds_name;
            if (ctx_mgr.hasDataSource(ds_id))
            {
                const auto* ds = ctx_mgr.dataSource(ds_id);
                ds_type = ds->dsType();
                ds_name = ds->name();
            }
            else
            {
                ds_type = "Other";
                ds_name = std::to_string(Utils::Number::sacFromDsId(ds_id))
                        + "/" + std::to_string(Utils::Number::sicFromDsId(ds_id));
            }

            const std::string line_str = Utils::String::lineStrFrom(line_id);
            const std::string full_key = ds_type + ":" + ds_name + ":"
                                       + line_str + ":" + dbcontent_name;

            auto it = agg.find(full_key);
            if (it == agg.end())
            {
                SliceAgg a;
                a.ds_type    = ds_type;
                a.ds_name    = ds_name;
                a.line       = line_str;
                a.dbcontent  = dbcontent_name;
                a.count      = 1;
                a.line_index = (line_str.size() >= 2 && line_str[0] == 'L')
                             ? std::max(0, std::atoi(line_str.c_str() + 1) - 1)
                             : 0;
                agg.emplace(full_key, std::move(a));
            }
            else
            {
                it->second.count++;
            }
        }
    }

    // Build payloads + LeafEntry list.
    std::vector<std::unique_ptr<TableLeafPayload>> new_payloads;
    std::vector<DBContentRootItem::LeafEntry>      entries;
    new_payloads.reserve(agg.size());
    entries.reserve(agg.size());

    for (const auto& kv : agg)
    {
        const std::string& full_key = kv.first;
        const SliceAgg&    a        = kv.second;

        QColor color = context::resolveSeriesColor(
            a.ds_type, a.ds_name, a.line_index, a.dbcontent, compass);

        new_payloads.emplace_back(std::make_unique<TableLeafPayload>(
            full_key, a.count, color));

        entries.push_back({a.ds_type, a.ds_name, a.line, a.dbcontent,
                           new_payloads.back().get()});
    }

    // Scoped subtree refresh — keeps the header widths untouched.
    layer_model_->refreshSubtree(db_content_root_, [&]() {
        payloads_ = std::move(new_payloads);
        return db_content_root_->buildChildrenFrom(entries);
    });
    db_content_root_->recomputeColorsRecursive();

    if (!hidden_slice_keys_.empty())
        layer_model_->applyPersistedHiddenIds(hidden_slice_keys_);

    // Push initial allowed set to the model so the first redraw filters correctly.
    pushAllowedSliceKeysToModel();

    emit layerTreeRebuiltSignal();
}

void TableViewDataWidget::pushAllowedSliceKeysToModel()
{
    if (!all_buffer_table_widget_)
        return;

    // Capture hidden keys snapshot for viewpoint round-trip across reloads.
    if (layer_model_)
        hidden_slice_keys_ = layer_model_->persistedHiddenIds();

    auto* model = all_buffer_table_widget_->allBufferTableModel();
    if (!model)
        return;

    if (payloads_.empty())
    {
        model->setAllowedSliceKeys(std::nullopt);
        model->setSliceColors({});
        return;
    }

    std::set<std::string> allowed;
    std::map<std::string, QColor> slice_colors;

    auto& dbcont_man = view_->compass().dbContentManager();
    for (const auto& payload : payloads_)
    {
        if (payload->visible())
            allowed.insert(payload->persistenceId());

        // Only target-report DBContents get a colored icon. Matches scatter's
        // leafColorFor rule: non-TR DBContents render as blank space.
        const std::string dbcontent = [&]() {
            // persistenceId is "<ds_type>:<ds_name>:L<n>:<dbcontent>"
            const std::string& k = payload->persistenceId();
            auto last = k.rfind(':');
            return last == std::string::npos ? std::string{} : k.substr(last + 1);
        }();

        QColor color;
        if (dbcont_man.existsDBContent(dbcontent) &&
            dbcont_man.dbContent(dbcontent).containsTargetReports())
            color = payload->color();

        slice_colors.emplace(payload->persistenceId(), color);
    }

    model->setAllowedSliceKeys(std::move(allowed));
    model->setSliceColors(std::move(slice_colors));
}
