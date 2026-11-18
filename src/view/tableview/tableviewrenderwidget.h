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

#include "compass_view_export.h"

#include "viewrenderwidget.h"
#include "viewlayerscan.h"
#include "allbuffertablemodel.h"

#include <map>
#include <string>

class TableView;
class TableViewModel;
class TableViewDataSource;
class AllBufferTableWidget;

/**
 * Render widget of the Table View.
 *
 * Exceptions of the Table View (readme_view_render_widgets.md, section 7.6): it holds a pointer to its model,
 * reads the buffers of the model and builds the table rows, and reads the view settings. There is no display
 * data and no render options. render() gives the screenshot of the widget.
 * The slots below are the flows of the model, connected by the view widget.
 */
class COMPASS_VIEW_API TableViewRenderWidget : public ViewRenderWidget
{
    Q_OBJECT

signals:
    void exportDoneSignal(bool canceled);
    /// the display changed outside a redraw, e.g. by the commit of the asynchronously prepared rows
    void displayChangedSignal();

public slots:
    void exportDataSlot();
    void exportDoneSlot(bool canceled);

    void clearDataSlot();
    void loadingDoneSlot();
    void recomputeSlot();
    /// pushes the allowed layers and the layer colors to the table model
    void layerStateChangedSlot();

public:
    TableViewRenderWidget(TableView& view, TableViewModel& model, QWidget* parent = nullptr);
    virtual ~TableViewRenderWidget();

    void redraw() override;
    void clear() override;
    /// the screenshot of the widget, size and scale are not used
    bool render(QImage& image, const QSize& size, double scale = 1.0) override;

    void viewInfoJSON(nlohmann::json& info) const override;

    void resetModels();
    void updateToSelection();

    void selectFirstSelectedRow();

    AllBufferTableWidget* getAllBufferTableWidget ();

    void updateToSettingsChange();

private:
    /// shows the rows of the buffers, returns the new draw state
    DrawState redrawData(bool recompute);

    /// main-thread commit of the asynchronously prepared load data: layer tree, model
    /// row data, selection, draw state
    bool launchAsyncPrepare();
    void commitLoadedData(const std::map<std::string, view_layer_scan::LayerAgg>& agg,
                          AllBufferTableModel::PreparedData&& prepared);

    TableView&           view_;
    TableViewModel&      model_;
    TableViewDataSource* data_source_{nullptr};

    AllBufferTableWidget*  all_buffer_table_widget_{nullptr};
};
