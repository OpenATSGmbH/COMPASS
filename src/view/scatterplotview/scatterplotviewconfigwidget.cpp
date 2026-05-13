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

#include "scatterplotviewconfigwidget.h"
#include "compass.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variableselectionwidget.h"
#include "scatterplotviewwidget.h"
#include "scatterplotview.h"
#include "scatterplotviewdatawidget.h"
#include "logger.h"
#include "variable.h"
#include "metavariable.h"
#include "ui_test_common.h"

#include "dbcontentlayer.h"
#include "viewlayerpanelwidget.h"
#include "viewlayertreemodel.h"
#include "annotationsrootitem.h"

#include <QCheckBox>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTreeView>
#include <QVBoxLayout>

using namespace Utils;
using namespace dbContent;

namespace
{
    /// Sum aggregator for integer-valued custom columns - ignores invalid
    /// entries and returns an invalid QVariant if no valid value was found.
    QVariant sumULongLong(const std::vector<QVariant>& vals)
    {
        unsigned long long sum = 0;
        bool any = false;
        for (const auto& v : vals)
        {
            bool ok = false;
            unsigned long long n = v.toULongLong(&ok);
            if (ok) { sum += n; any = true; }
        }
        return any ? QVariant((qulonglong)sum) : QVariant();
    }
}

/**
*/
ScatterPlotViewConfigWidget::ScatterPlotViewConfigWidget(ScatterPlotViewWidget* view_widget,
                                                         QWidget* parent)
:   VariableViewConfigWidget(view_widget, view_widget->getView(), parent)
{
    view_ = view_widget->getView();
    traced_assert(view_);

    auto layout = configLayout();

    {
        color_mode_label_ = new QLabel(this);
        color_mode_label_->setText("Color Mode: " + colorModeText(view_->compass().colorMode()));
        layout->addWidget(color_mode_label_);

        connect(&view_->compass(), &COMPASS::colorModeChangedSignal,
                this, &ScatterPlotViewConfigWidget::colorModeChangedSlot);
    }

    {
        // Register the scatter-specific "# NULL" column (custom col 0).
        LayerColumnSpec null_col;
        null_col.header           = "# NULL";
        null_col.default_width    = 70;
        null_col.resize_mode      = QHeaderView::Interactive;
        null_col.alignment        = Qt::AlignRight | Qt::AlignVCenter;
        null_col.group_aggregator = &sumULongLong;

        layer_panel_     = new ViewLayerPanelWidget({ null_col }, view_->canShowAnnotations(), this);
        db_content_root_ = layer_panel_->model()->dbContentRootItem();

        // Hand the root + model to the data widget so it can populate and
        // round-trip hidden state.
        auto* data_widget = view_widget->getViewDataWidget();
        data_widget->attachLayerPanel(db_content_root_, layer_panel_->model());

        // Visibility change -> redraw chart.
        connect(layer_panel_->model(), &LayerTreeModel::hiddenChangedSignal,
                data_widget, &ScatterPlotViewDataWidget::updateChartSlot);

        // Data widget rebuilt the tree -> re-apply default expansion.
        connect(data_widget, &ScatterPlotViewDataWidget::layerTreeRebuiltSignal,
                this, &ScatterPlotViewConfigWidget::applyDefaultExpansionSlot);

        layout->addWidget(layer_panel_);
    }

    use_connection_lines_ = new QCheckBox("Use Connection Lines");
    use_connection_lines_->setChecked(view_->useConnectionLines());
    UI_TEST_OBJ_NAME(use_connection_lines_, use_connection_lines_->text())

    connect(use_connection_lines_, &QCheckBox::clicked,
            this, &ScatterPlotViewConfigWidget::useConnectionLinesSlot);

    layout->addWidget(use_connection_lines_);
}

/**
*/
ScatterPlotViewConfigWidget::~ScatterPlotViewConfigWidget() = default;

/**
*/
void ScatterPlotViewConfigWidget::useConnectionLinesSlot()
{
    loginf;

    traced_assert(use_connection_lines_);
    view_->useConnectionLines(use_connection_lines_->checkState() == Qt::Checked);
}

/**
*/
void ScatterPlotViewConfigWidget::applyDefaultExpansionSlot()
{
    if (!db_content_root_ || !layer_panel_)
        return;
    db_content_root_->applyDefaultExpansionForColorMode(
        layer_panel_->treeView(), view_->compass().colorMode());
}

/**
*/
void ScatterPlotViewConfigWidget::colorModeChangedSlot(unsigned int mode)
{
    traced_assert(color_mode_label_);
    color_mode_label_->setText("Color Mode: " + colorModeText(mode));

    // Re-expand so the level that differentiates colors stays visible; the
    // data widget will re-run resolveSeriesColor on the next redraw triggered
    // by COMPASS::colorModeChangedSignal elsewhere.
    applyDefaultExpansionSlot();
}

/**
*/
QString ScatterPlotViewConfigWidget::colorModeText(unsigned int mode)
{
    switch (mode)
    {
        case 0: return "DSType";
        case 1: return "DBContent";
        case 2: return "Data Source";
        case 3: return "Data Source + Line";
        default: return "Unknown";
    }
}

/**
*/
void ScatterPlotViewConfigWidget::onDisplayChange_impl()
{
    traced_assert(use_connection_lines_);
    use_connection_lines_->setChecked(view_->useConnectionLines());
}

/**
*/
void ScatterPlotViewConfigWidget::viewInfoJSON_impl(nlohmann::json& info) const
{
    //!call base!
    VariableViewConfigWidget::viewInfoJSON_impl(info);

    info[ "use_connection_lines" ] = use_connection_lines_->isChecked();
}

/**
*/
void ScatterPlotViewConfigWidget::configChanged_impl()
{
    use_connection_lines_->blockSignals(true);
    use_connection_lines_->setChecked(view_->useConnectionLines());
    use_connection_lines_->blockSignals(false);
}
