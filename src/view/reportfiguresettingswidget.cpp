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

#include "reportfiguresettingswidget.h"

#include "ui_test_common.h"
#include "traced_assert.h"

#include <QCheckBox>
#include <QFormLayout>
#include <QSpinBox>
#include <QVBoxLayout>

const int ReportFigureSettingsWidget::MinHistogramBars = 2;
const int ReportFigureSettingsWidget::MaxHistogramBars = 200;

/**
 */
ReportFigureSettingsWidget::ReportFigureSettingsWidget(
        const context::DBContextManager& ctx_man,
        const ReportFigureBackgroundWidget::DefaultHiddenLayersFunc& default_hidden_layers,
        bool show_histogram_bars,
        QWidget* parent)
:   QGroupBox("Report Figures", parent)
{
    UI_TEST_OBJ_NAME(this, "report_figures");

    auto layout = new QVBoxLayout(this);
    auto form   = new QFormLayout;

    if (show_histogram_bars)
    {
        histogram_bars_box_ = new QSpinBox;
        histogram_bars_box_->setRange(MinHistogramBars, MaxHistogramBars);
        histogram_bars_box_->setSingleStep(1);
        histogram_bars_box_->setToolTip("Number of bars of the histograms of value distributions");
        UI_TEST_OBJ_NAME(histogram_bars_box_, "histogram_bars");
        connect(histogram_bars_box_, QOverload<int>::of(&QSpinBox::valueChanged), this, [ this ] (int)
        {
            if (!updating_)
                emit histogramBarsChangedSignal();
        });
        form->addRow("Histogram Bars", histogram_bars_box_);
    }

    log_scale_check_ = new QCheckBox;
    log_scale_check_->setToolTip("Draw the count axis of the histograms with one data series in logarithmic scale. "
                                 "Histograms with stacked data series always have a linear count axis.");
    UI_TEST_OBJ_NAME(log_scale_check_, "histogram_log_scale");
    connect(log_scale_check_, &QCheckBox::toggled, this, [ this ] (bool on)
    {
        if (updating_)
            return;
        options_.histogram_log_scale = on;
        emit optionsChangedSignal();
    });
    form->addRow("Histogram Log Scale", log_scale_check_);

    const QString size_tooltip = QString(" of the layout of an exported figure image. The image has %1 times this "
                                         "number of pixels. A smaller layout gives larger texts, for example for "
                                         "two figures side by side.").arg(report_figure::ExportScale);

    auto createSizeBox = [ & ] (const QString& name, const QString& tooltip)
    {
        auto box = new QSpinBox;
        box->setRange(ReportFigureOptions::MinExportLayoutSize, ReportFigureOptions::MaxExportLayoutSize);
        box->setSingleStep(100);
        box->setToolTip(tooltip);
        UI_TEST_OBJ_NAME(box, name);
        connect(box, QOverload<int>::of(&QSpinBox::valueChanged), this, [ this ] (int)
        {
            if (updating_)
                return;
            options_.export_layout_size = QSize(export_width_box_->value(), export_height_box_->value());
            emit optionsChangedSignal();
        });
        return box;
    };

    export_width_box_  = createSizeBox("export_image_width" , "Width"  + size_tooltip);
    export_height_box_ = createSizeBox("export_image_height", "Height" + size_tooltip);

    form->addRow("Export Image Width", export_width_box_);
    form->addRow("Export Image Height", export_height_box_);

    layout->addLayout(form);

    background_widget_ = new ReportFigureBackgroundWidget(ctx_man, default_hidden_layers);
    connect(background_widget_, &ReportFigureBackgroundWidget::backgroundChangedSignal,
            this, &ReportFigureSettingsWidget::backgroundChangedSignal);
    layout->addWidget(background_widget_);

    updateFromOptions();
}

/**
 */
ReportFigureSettingsWidget::~ReportFigureSettingsWidget() = default;

/**
 */
void ReportFigureSettingsWidget::setHistogramBars(unsigned int num_bars)
{
    traced_assert(histogram_bars_box_);

    updating_ = true;
    histogram_bars_box_->setValue(static_cast<int>(num_bars));
    updating_ = false;
}

/**
 */
unsigned int ReportFigureSettingsWidget::histogramBars() const
{
    traced_assert(histogram_bars_box_);

    return static_cast<unsigned int>(histogram_bars_box_->value());
}

/**
 */
void ReportFigureSettingsWidget::setOptions(const ReportFigureOptions& options)
{
    options_ = options;

    updateFromOptions();
}

/**
 */
void ReportFigureSettingsWidget::setBackground(const ReportFigureBackground& background)
{
    background_widget_->setBackground(background);
}

/**
 */
const ReportFigureBackground& ReportFigureSettingsWidget::background() const
{
    return background_widget_->background();
}

/**
 */
void ReportFigureSettingsWidget::updateFromOptions()
{
    updating_ = true;

    log_scale_check_->setChecked(options_.histogram_log_scale);
    export_width_box_->setValue(options_.export_layout_size.width());
    export_height_box_->setValue(options_.export_layout_size.height());

    updating_ = false;
}
