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

#include "variableviewconfigwidget.h"
#include "grid2d_defs.h"
#include "property.h"

class GridViewWidget;
class GridView;

class ColorScaleSelection;
class PropertyValueEdit;

class QCheckBox;
class QLineEdit;
class QComboBox;
class QSpinBox;
class QPushButton;
class QToolButton;
class QLabel;
class QDialog;

/**
 * @brief Widget with configuration elements for a ScatterPlotView
 *
 */
class GridViewConfigWidget : public VariableViewConfigWidget
{
public:
    GridViewConfigWidget(GridViewWidget* view_widget, 
                         QWidget* parent = nullptr);
    virtual ~GridViewConfigWidget();

    virtual void redrawDone() override;
    virtual void loadingDone() override;

    static const int DecimalsDefault;

protected:
    virtual void viewInfoJSON_impl(nlohmann::json& info) const override;
    virtual void configChanged_impl() override;

    virtual void variableChangedEvent(int idx) override;
    virtual void dataSourceChangedEvent() override;

    void attachExportMenu();

    void valueTypeChanged();
    void gridResolutionChanged();
    void colorScaleChanged();
    void colorStepsChanged();
    void minValueChanged();
    void maxValueChanged();
    
    void updateConfig();
    void updateDistributedVariable();
    void updateVariableDataType();
    void updateExport();
    void updateUIFromSource();
    void checkRanges();

    std::string exportName() const;
    void exportToGeographicView();
    void exportToGeoTiff();

    GridView* view_ = nullptr;

    QComboBox*           value_type_combo_             = nullptr;
    QLabel*              value_type_placeh_label_      = nullptr;
    QSpinBox*            grid_resolution_box_          = nullptr;
    QLabel*              grid_resolution_placeh_label_ = nullptr;
    ColorScaleSelection* color_selection_              = nullptr;
    QSpinBox*            color_steps_box_              = nullptr;
    PropertyValueEdit*   color_value_min_box_          = nullptr;
    PropertyValueEdit*   color_value_max_box_          = nullptr;
    QPushButton*         reset_min_button_             = nullptr;
    QPushButton*         reset_max_button_             = nullptr;
    QPushButton*         export_button_                = nullptr;
    QLabel*              range_info_label_             = nullptr;
};
