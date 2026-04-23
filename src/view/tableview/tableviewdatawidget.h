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

#include <memory>
#include <set>
#include <string>
#include <vector>

class TableView;
class TableViewWidget;
class TableViewDataSource;
class AllBufferTableWidget;
class Buffer;
class DBContent;
class DBContentRootItem;
class LayerTreeModel;
class TableLeafPayload;

class TableViewDataWidget : public ViewDataWidget
{
    Q_OBJECT

signals:
    void exportDoneSignal(bool cancelled);

    /// Emitted after rebuildLayerTree() has replaced the DBContent subtree.
    /// The config widget uses this to re-apply default expansion.
    void layerTreeRebuiltSignal();

public slots:
    void exportDataSlot();
    void exportDoneSlot(bool cancelled);

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

    /// Called by TableViewConfigWidget once the LayerPanelWidget is built.
    /// Provides the DBContent root item (owned by the panel's model) and the
    /// layer tree model used for hidden-state round-tripping.
    void attachLayerPanel(DBContentRootItem* root, LayerTreeModel* layer_model);

protected:
    virtual void toolChanged_impl(int mode) override;
    virtual void loadingStarted_impl() override;
    virtual void loadingDone_impl() override;
    virtual void updateData_impl(bool requires_reset) override;
    virtual void clearData_impl() override;
    virtual void clearIntermediateRedrawData_impl() override;
    virtual DrawState redrawData_impl(bool recompute) override;
    virtual void liveReload_impl() override;
    virtual bool hasAnnotations_impl() const override { return false; }

    void viewInfoJSON_impl(nlohmann::json& info) const override;

    TableView*           view_{nullptr};
    TableViewDataSource* data_source_{nullptr};

    AllBufferTableWidget*  all_buffer_table_widget_{nullptr};

    DBContentRootItem* db_content_root_{nullptr};   // owned by layer panel model
    LayerTreeModel*    layer_model_    {nullptr};   // owned by LayerPanelWidget

    std::vector<std::unique_ptr<TableLeafPayload>> payloads_;

    std::set<std::string> hidden_slice_keys_;  // transient: remember unchecked slices across reloads

private:
    /// Rebuild payloads_ from the loaded buffers and repopulate the DBContent
    /// subtree. Emits layerTreeRebuiltSignal.
    void rebuildLayerTree();

    /// Compute the allowed-slice-keys set from current panel state and push it
    /// onto the AllBufferTableModel. Empty layer panel (no payloads) = no
    /// filter (std::nullopt); otherwise the set of currently-visible keys.
    void pushAllowedSliceKeysToModel();
};
