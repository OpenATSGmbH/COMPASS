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
#include "colormodelabel.h"
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

/**
*/
ScatterPlotViewConfigWidget::ScatterPlotViewConfigWidget(ScatterPlotViewWidget* view_widget,
                                                         QWidget* parent)
:   VariableViewConfigWidget(view_widget, view_widget->getView(), parent)
{
    view_ = view_widget->getView();
    traced_assert(view_);

    auto layout = configLayout();

    layout->addWidget(new ColorModeLabel(view_->compass(), this));

    layer_panel_ = createLayerPanel({ ViewLayerPanelWidget::nullCountColumn() }, view_->canShowAnnotations());
    layout->addWidget(layer_panel_);

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
