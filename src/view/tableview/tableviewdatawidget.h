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

#pragma once

#include "viewdatawidget.h"
#include "viewlayerscan.h"
#include "allbuffertablemodel.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

class TableView;
class TableViewWidget;
class TableViewDataSource;
class AllBufferTableWidget;
class Buffer;
class DBContent;

class TableViewDataWidget : public ViewDataWidget
{
    Q_OBJECT

signals:
    void exportDoneSignal(bool canceled);

public slots:
    void exportDataSlot();
    void exportDoneSlot(bool canceled);

public:
    TableViewDataWidget(TableViewWidget* view_widget,
                          QWidget* parent = nullptr,
                          Qt::WindowFlags f = Qt::WindowFlags());
    virtual ~TableViewDataWidget();

    void resetModels();
    void updateToSelection();

    void selectFirstSelectedRow();

    AllBufferTableWidget* getAllBufferTableWidget ();

    void updateToSettingsChange();

protected:
    virtual void toolChanged_impl(int mode) override;
    virtual void loadingStarted_impl() override;
    virtual void loadingDone_impl() override;
    virtual void updateFromSource_impl(const DBContentDataSet& source,
                                       const std::vector<std::string>& names, bool reset, bool last) override;
    virtual void clearData_impl() override;
    virtual void clearIntermediateRedrawData_impl() override;
    virtual DrawState redrawData_impl(bool recompute) override;
    virtual void liveReload_impl() override;
    virtual bool hasAnnotations_impl() const override { return false; }

    void viewInfoJSON_impl(nlohmann::json& info) const override;

    /// layer snapshots from the (cached) layer aggregation of the loaded buffers
    virtual void rebuildLayerTree() override final;
    /// the allowed layers => model filter + redraw
    virtual void layerVisibilityChanged() override final;

    TableView*           view_{nullptr};
    TableViewDataSource* data_source_{nullptr};

    AllBufferTableWidget*  all_buffer_table_widget_{nullptr};

private:
    /// layer snapshots + data subtree from layer aggregates (the scan may run on a worker),
    /// pushes the allowed layers to the model
    void applyLayerTree(const LayerAggregates& agg);

    /// Compute the allowed-layer-ids set + per-layer icon colors from current
    /// panel state and push them onto the AllBufferTableModel. Empty layer
    /// panel (no payloads) = no filter (std::nullopt); otherwise the set of
    /// currently-visible layer ids.
    void pushLayerStateToModel();

    /// true when the current panel state hides any layer (the asynchronously prepared
    /// row data is unfiltered and needs a re-filtering rebuild then)
    bool anyLayerHidden() const;

    /// main-thread commit of the asynchronously prepared load data: layer tree, model
    /// row data, selection, draw state
    bool launchAsyncPrepare();
    void commitLoadedData(const LayerAggregates& agg,
                          AllBufferTableModel::PreparedData&& prepared);
};
