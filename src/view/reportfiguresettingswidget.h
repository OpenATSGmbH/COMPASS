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

#include "reportfigureoptions.h"
#include "reportfigurebackgroundwidget.h"

#include <QGroupBox>

class QCheckBox;
class QSpinBox;

/**
 * Group "Report Figures" of the report generation tasks (evaluation, data source analysis): the histograms, the
 * export image size and the background of the report figures.
 */
class COMPASS_VIEW_API ReportFigureSettingsWidget : public QGroupBox
{
    Q_OBJECT

signals:
    void histogramBarsChangedSignal();
    void optionsChangedSignal();
    void backgroundChangedSignal();

public:
    /// show_histogram_bars: the group edits the number of bars of the histograms. The analysis has no such row,
    /// its inspectors have their own control.
    ReportFigureSettingsWidget(const context::DBContextManager& ctx_man,
                               const ReportFigureBackgroundWidget::DefaultHiddenLayersFunc& default_hidden_layers,
                               bool show_histogram_bars,
                               QWidget* parent = nullptr);
    ~ReportFigureSettingsWidget() override;

    void setHistogramBars(unsigned int num_bars);
    unsigned int histogramBars() const;

    void setOptions(const ReportFigureOptions& options);
    const ReportFigureOptions& options() const { return options_; }

    void setBackground(const ReportFigureBackground& background);
    const ReportFigureBackground& background() const;

    static const int MinHistogramBars;
    static const int MaxHistogramBars;

private:
    void updateFromOptions();

    ReportFigureOptions options_;

    QSpinBox*                     histogram_bars_box_  {nullptr};
    QCheckBox*                    log_scale_check_     {nullptr};
    QSpinBox*                     export_width_box_    {nullptr};
    QSpinBox*                     export_height_box_   {nullptr};
    ReportFigureBackgroundWidget* background_widget_   {nullptr};

    bool updating_ = false;
};
