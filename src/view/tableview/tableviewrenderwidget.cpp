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

#include "tableviewrenderwidget.h"
#include "tableviewmodel.h"
#include "appstate.h"
#include "viewmanager.h"
#include "tableview.h"
#include "allbuffertablewidget.h"
#include "allbuffertablemodel.h"
#include "buffer.h"
#include "data_source.h"
#include "db_context_manager.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/metavariable.h"
#include "color_provider.h"
#include "dbcontentlayer.h"
#include "viewlayertreemodel.h"
#include "viewlayersnapshot.h"
#include "logger.h"
#include "number.h"
#include "stringconv.h"
#include "viewasyncprocessor.h"

#include "json.hpp"

#include "boost/date_time/posix_time/posix_time.hpp"

#include <QApplication>
#include <QBoxLayout>
#include <QMessageBox>
#include <QPainter>
#include <QTableView>
#include <QHeaderView>

TableViewRenderWidget::TableViewRenderWidget(TableView& view, TableViewModel& model, QWidget* parent)
:   ViewRenderWidget(parent)
,   view_           (view)
,   model_          (model)
{
    data_source_ = view_.getDataSource();
    traced_assert(data_source_);

    all_buffer_table_widget_ = new AllBufferTableWidget(view_, *data_source_);
    contentLayout()->addWidget(all_buffer_table_widget_);
    connect(all_buffer_table_widget_, &AllBufferTableWidget::exportDoneSignal, this,
            &TableViewRenderWidget::exportDoneSlot);
}

/**
 * The asynchronous tasks of the model commit into this widget (commitLoadedData()):
 * their commits must not run after it is destroyed.
 */
TableViewRenderWidget::~TableViewRenderWidget()
{
    model_.shutdownAsyncProcessor();
}

/**
 */
void TableViewRenderWidget::layerStateChangedSlot()
{
    if (!all_buffer_table_widget_)
        return;

    auto* model = all_buffer_table_widget_->allBufferTableModel();
    if (!model)
        return;

    const auto& layers = model_.layerSnapshots();

    if (layers.empty())
    {
        model->setAllowedLayerIds(std::nullopt);
        model->setLayerColors({});
        return;
    }

    std::set<std::string> allowed;
    std::map<std::string, QColor> layer_colors;

    auto& dbcont_man = view_.viewManager().dbContentManager();
    for (const auto& layer : layers)
    {
        if (layer->visible())
            allowed.insert(layer->key());

        // Only target-report DBContents get a colored icon. Matches scatter's
        // leafColorFor rule: non-TR DBContents render as blank space.
        const std::string& dbcontent = layer->id().dbcontent;

        QColor color;
        if (dbcont_man.existsDBContent(dbcontent) &&
            dbcont_man.dbContent(dbcontent).containsTargetReports())
            color = layer->color();

        layer_colors.emplace(layer->key(), color);
    }

    model->setAllowedLayerIds(std::move(allowed));
    model->setLayerColors(std::move(layer_colors));
}

/**
 */
void TableViewRenderWidget::clearDataSlot()
{
    logdbg << "begin";

    if (all_buffer_table_widget_)
        all_buffer_table_widget_->clear();

    logdbg << "end";
}

/**
 * The model cleared before, see ViewWidget::clearData().
 */
void TableViewRenderWidget::clear()
{
    setDrawState(RenderState::NotDrawn);
}

// Offline the heavy per-redraw work - the layer scan, the row index build and the
// sort - runs on a worker; commitLoadedData applies the results on completion, and
// the base defers dataLoaded / viewRefreshed until then. Used by both the load-done
// path and interactive recompute redraws (style, color mode, filter changes). Live
// stays synchronous: the feed mutates the same buffers every tick. Returns false
// when the synchronous path should run instead.
bool TableViewRenderWidget::launchAsyncPrepare()
{
    const auto& view_data = model_.viewData();

    unsigned int num_records = 0;
    for (auto& buf_it : view_data)
        num_records += buf_it.second->size();

    bool async = view_.viewManager().appState().appMode() != AppMode::LiveRunning && num_records > 0;

    if (!async)
        return false;

    traced_assert(all_buffer_table_widget_);

    auto* model = all_buffer_table_widget_->allBufferTableModel();
    traced_assert(model);

    auto scan_input = std::make_shared<view_layer_scan::ScanInput>(
        view_layer_scan::makeScanInput(view_data,
                                       view_.viewManager().dbContentManager(),
                                       view_.viewManager().dbContextManager()));
    auto prep_input = std::make_shared<AllBufferTableModel::PrepareInput>(
        model->makePrepareInput(view_data, /*for_load*/ true));

    auto agg      = std::make_shared<std::map<std::string, view_layer_scan::LayerAgg>>();
    auto prepared = std::make_shared<AllBufferTableModel::PreparedData>();

    model_.asyncProcessor().launch(
        "table view row data with " + std::to_string(num_records) + " records",
        [ scan_input, prep_input, agg, prepared ] ()
        {
            *agg      = view_layer_scan::aggregateLayers(*scan_input);
            *prepared = AllBufferTableModel::prepareData(*prep_input);
        },
        [ this, agg, prepared ] ()
        {
            commitLoadedData(*agg, std::move(*prepared));
        });

    return true;
}

void TableViewRenderWidget::loadingDoneSlot()
{
    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();

    unsigned int num_records = 0;
    for (auto& buf_it : model_.viewData())
        num_records += buf_it.second->size();

    loginf << "begin with " << num_records << " records";

    if (launchAsyncPrepare())
        return;

    // Rebuild the layer tree first so the panel reflects current data; the
    // recompute below will then consult the allowed-layer-ids set we push
    // into the table model from rebuildLayerTree.
    model_.rebuildLayerTree();

    //default behavior
    model_.recompute();

    boost::posix_time::ptime stop_time = boost::posix_time::microsec_clock::local_time();
    double elapsed_s = (stop_time - start_time).total_milliseconds() / 1000.0;

    loginf << "done with " << num_records << " records in "
           << Utils::String::timeStringFromDouble(elapsed_s, true);
}

/**
 * Main-thread commit of the asynchronously prepared load data.
 */
void TableViewRenderWidget::commitLoadedData(const std::map<std::string, view_layer_scan::LayerAgg>& agg,
                                           AllBufferTableModel::PreparedData&& prepared)
{
    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();

    traced_assert(all_buffer_table_widget_);

    setUpdatesEnabled(false);

    //the layer tree from the worker-computed aggregation
    model_.applyLayerAggregates(agg);

    auto selected_ranges = std::move(prepared.selected_ranges);

    all_buffer_table_widget_->showPrepared(std::move(prepared));

    if (model_.anyLayerHidden())
    {
        // the prepared row data is unfiltered; re-filter against the freshly pushed
        // allowed set (rare - only when the stored panel state hides layers)
        loginf << "hidden layers stored, re-filtering row data";

        all_buffer_table_widget_->allBufferTableModel()->rebuild();
        all_buffer_table_widget_->selectSelectedRows();
    }
    else
    {
        all_buffer_table_widget_->selectSelectedRows(selected_ranges);
    }

    setUpdatesEnabled(true);

    setDrawState(all_buffer_table_widget_->rowCount() > 0 ? RenderState::DrawnContent
                                                          : RenderState::Drawn);
    emit displayChangedSignal();

    boost::posix_time::ptime stop_time = boost::posix_time::microsec_clock::local_time();
    double elapsed_s = (stop_time - start_time).total_milliseconds() / 1000.0;

    loginf << "done with " << all_buffer_table_widget_->rowCount() << " rows in "
           << Utils::String::timeStringFromDouble(elapsed_s, true);
}

/**
 */
void TableViewRenderWidget::redraw()
{
    setDrawState(redrawData(false));
}

/**
 * Computes the rows again, the model has reset its counts.
 */
void TableViewRenderWidget::recomputeSlot()
{
    setDrawState(redrawData(true));

    emit displayChangedSignal();
}

/**
 */
TableViewRenderWidget::RenderState TableViewRenderWidget::redrawData(bool recompute)
{
    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();

    unsigned int num_records = 0;
    for (auto& buf_it : model_.viewData())
        num_records += buf_it.second->size();

    loginf << "start - recompute " << recompute << " records " << num_records;

    traced_assert(all_buffer_table_widget_);

    //offline, the row data prepare runs on a worker and commitLoadedData applies it;
    //the view keeps showing the previous rows until the commit, and viewRefreshed is
    //held back by the pending processing guard
    if (launchAsyncPrepare())
        return (all_buffer_table_widget_->rowCount() > 0 ? RenderState::DrawnContent : RenderState::Drawn);

    setUpdatesEnabled(false);

    all_buffer_table_widget_->show(model_.viewData());

    selectFirstSelectedRow();

    setUpdatesEnabled(true);

    boost::posix_time::ptime stop_time = boost::posix_time::microsec_clock::local_time();
    double elapsed_s = (stop_time - start_time).total_milliseconds() / 1000.0;

    loginf << "done with " << num_records << " records in "
           << Utils::String::timeStringFromDouble(elapsed_s, true);

    return (all_buffer_table_widget_->rowCount() > 0 ? RenderState::DrawnContent : RenderState::Drawn);
}

void TableViewRenderWidget::exportDataSlot()
{
    logdbg;

    if (!all_buffer_table_widget_ || all_buffer_table_widget_->rowCount() == 0)
    {
        QMessageBox msgBox(this);
        msgBox.setText("Export can not be used without loaded data.");
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.exec();

        emit exportDoneSignal(true);
        return;
    }

    all_buffer_table_widget_->exportSlot();
}

void TableViewRenderWidget::exportDoneSlot(bool canceled)
{
    emit exportDoneSignal(canceled);
}

void TableViewRenderWidget::updateToSettingsChange()
{
    loginf;

    if (all_buffer_table_widget_)
        all_buffer_table_widget_->updateToSettingsChange();
}

void TableViewRenderWidget::resetModels()
{
    if (all_buffer_table_widget_)
        all_buffer_table_widget_->resetModel();
}

void TableViewRenderWidget::updateToSelection()
{
    if (all_buffer_table_widget_)
        all_buffer_table_widget_->updateToSelection();
}

void TableViewRenderWidget::selectFirstSelectedRow()
{
    if (all_buffer_table_widget_)
        all_buffer_table_widget_->selectSelectedRows();
}

AllBufferTableWidget* TableViewRenderWidget::getAllBufferTableWidget ()
{
    traced_assert(all_buffer_table_widget_);
    return all_buffer_table_widget_;
}

/**
 * Today's screenshot of the widget.
 */
bool TableViewRenderWidget::render(QImage& image, const QSize& /*size*/, double /*scale*/)
{
    image = QImage(this->size(), QImage::Format_ARGB32);

    QPainter painter(&image);
    QWidget::render(&painter);

    return !image.isNull();
}

void TableViewRenderWidget::viewInfoJSON(nlohmann::json& info) const
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
            properties.push_back(table->model()->headerData(i, Qt::Horizontal, BaseBufferTableModel::VariableNameRole).toString().toStdString());

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

        // non-null cells per column, parallel to properties, a prefix column counts every row
        std::vector<unsigned int> data_counts = all_buffer_table_widget_->allBufferTableModel()->validCounts();
        int num_prefix = table->model()->columnCount() - (int)data_counts.size();

        std::vector<unsigned int> column_valid_counts(num_prefix > 0 ? num_prefix : 0,
                                                      (unsigned int)table->model()->rowCount());
        column_valid_counts.insert(column_valid_counts.end(), data_counts.begin(), data_counts.end());

        table_info[ "column_valid_counts" ] = column_valid_counts;

        table_infos.push_back(table_info);
    };

    addTable("All",
             all_buffer_table_widget_->table(),
             view_.showOnlySelected(),
             view_.usePresentation(), view_.ignoreNonTargetReports());

    info[ "tables" ] = table_infos;
}
