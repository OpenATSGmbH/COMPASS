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

#include "evaluationmaintabwidget.h"
#include "evaluationmanagerwidget.h"
#include "evaluationdatasourcewidget.h"
#include "evaluationcalculator.h"
#include "evaluationdialog.h"
#include "evaluationstandardcombobox.h"
#include "evaluationsectorwidget.h"

#include "logger.h"
#include "sectorlayer.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QFormLayout>
#include <QCheckBox>
#include <QLineEdit>
#include <QComboBox>

/**
 */
EvaluationMainTabWidget::EvaluationMainTabWidget(EvaluationCalculator& calculator, 
                                                 EvaluationDialog& dialog)
:   QWidget    (nullptr)
,   calculator_(calculator)
,   dialog_    (dialog)
{
    QVBoxLayout* main_layout = new QVBoxLayout();

    QFont font_bold;
    font_bold.setBold(true);

    // data sources
    QLabel* main_label = new QLabel("Data Selection");
    main_label->setFont(font_bold);
    main_layout->addWidget(main_label);

    QHBoxLayout* data_sources_layout = new QHBoxLayout();

    const auto& settings = calculator_.settings();

    // titles define which data sources to display
    data_source_ref_widget_.reset(
                new EvaluationDataSourceWidget(calculator_, "Reference Data", calculator_.dbContentNameRef(), settings.line_id_ref_));
    connect (data_source_ref_widget_.get(), &EvaluationDataSourceWidget::dbContentNameChangedSignal,
             this, &EvaluationMainTabWidget::dbContentRefNameChangedSlot);
    connect (data_source_ref_widget_.get(), &EvaluationDataSourceWidget::usedDataSourceChangedSignal,
            this, &EvaluationMainTabWidget::usedDataSourcesChangedSlot);
    connect (data_source_ref_widget_.get(), &EvaluationDataSourceWidget::lineChangedSignal,
             this, &EvaluationMainTabWidget::lineRefChangedSlot);
    data_sources_layout->addWidget(data_source_ref_widget_.get());

    data_source_tst_widget_.reset(
                new EvaluationDataSourceWidget(calculator_, "Test Data", calculator_.dbContentNameTst(), settings.line_id_tst_));
    connect (data_source_tst_widget_.get(), &EvaluationDataSourceWidget::dbContentNameChangedSignal,
             this, &EvaluationMainTabWidget::dbContentTstNameChangedSlot);
    connect (data_source_tst_widget_.get(), &EvaluationDataSourceWidget::usedDataSourceChangedSignal,
            this, &EvaluationMainTabWidget::usedDataSourcesChangedSlot);
    connect (data_source_tst_widget_.get(), &EvaluationDataSourceWidget::lineChangedSignal,
             this, &EvaluationMainTabWidget::lineTstChangedSlot);
    data_sources_layout->addWidget(data_source_tst_widget_.get());

    main_layout->addLayout(data_sources_layout);

    // standard
    QHBoxLayout* std_layout = new QHBoxLayout();

    QLabel* standard_label = new QLabel("Standard");
    standard_label->setFont(font_bold);
    std_layout->addWidget(standard_label);

    standard_box_.reset(new EvaluationStandardComboBox(calculator_));
    std_layout->addWidget(standard_box_.get());

    main_layout->addLayout(std_layout);

    // minimum height filter
    QHBoxLayout* height_filter_layout = new QHBoxLayout();

    QLabel* height_filter_label = new QLabel("Sector Layers: Minimum Height Filter");
    height_filter_label->setFont(font_bold);
    height_filter_layout->addWidget(height_filter_label);

    min_height_filter_combo_ = new QComboBox;
    height_filter_layout->addWidget(min_height_filter_combo_);

    connect(min_height_filter_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &EvaluationMainTabWidget::minHeightFilterChangedSlot);

    main_layout->addLayout(height_filter_layout);

    // sector requirement mapping
    QVBoxLayout* sec_layout = new QVBoxLayout();

    QLabel* sec_label = new QLabel("Sector Layers: Requirement Groups Usage");
    sec_label->setFont(font_bold);
    sec_layout->addWidget(sec_label);

    sector_widget_.reset(new EvaluationSectorWidget(calculator_, dialog_));
    sector_widget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    sec_layout->addWidget(sector_widget_.get());

    main_layout->addLayout(sec_layout);

    // connections
    connect (&calculator_, &EvaluationCalculator::standardsChanged,
             this, &EvaluationMainTabWidget::changedStandardsSlot);
    connect (&calculator_, &EvaluationCalculator::currentStandardChanged,
             this, &EvaluationMainTabWidget::changedCurrentStandardSlot);

    setContentsMargins(0, 0, 0, 0);
    setLayout(main_layout);
}

/**
 */
void EvaluationMainTabWidget::updateDataSources()
{
    if (data_source_ref_widget_)
        data_source_ref_widget_->updateDataSourcesSlot();

    if (data_source_tst_widget_)
        data_source_tst_widget_->updateDataSourcesSlot();
}

/**
 */
void EvaluationMainTabWidget::updateSectors()
{
    traced_assert(sector_widget_);
    traced_assert(min_height_filter_combo_);

    sector_widget_->update();

    updateMinHeightFilterCombo();
}

/**
 */
void EvaluationMainTabWidget::updateMinHeightFilterCombo()
{
    traced_assert(min_height_filter_combo_);

    min_height_filter_combo_->blockSignals(true);

    min_height_filter_combo_->clear();
    min_height_filter_combo_->addItem("None");

    if (calculator_.sectorsLoaded())
    {
        for (const auto& sec_lay_it : calculator_.sectorLayers())
            min_height_filter_combo_->addItem(QString::fromStdString(sec_lay_it->name()));
    }

    min_height_filter_combo_->setCurrentIndex(0);

    if (calculator_.filterMinimumHeight())
    {
        const auto& layer_name = calculator_.minHeightFilterLayerName();
        int idx = min_height_filter_combo_->findText(QString::fromStdString(layer_name));

        //assert(idx >= 0);

        if (idx >= 0)
            min_height_filter_combo_->setCurrentIndex(idx);
    }

    min_height_filter_combo_->blockSignals(false);
}

/**
 */
void EvaluationMainTabWidget::minHeightFilterChangedSlot(int idx)
{
    if (idx <= 0)
    {
        calculator_.minHeightFilterLayerName("");
        return;
    }

    calculator_.minHeightFilterLayerName(min_height_filter_combo_->itemText(idx).toStdString());
}

/**
 */
void EvaluationMainTabWidget::dbContentRefNameChangedSlot(const std::string& dbcontent_name)
{
    loginf << "name " << dbcontent_name;

    calculator_.dbContentNameRef(dbcontent_name);

    dialog_.updateButtons();
}

/**
 */
void EvaluationMainTabWidget::lineRefChangedSlot(unsigned int line_id)
{
    loginf << "value " << line_id;

    calculator_.settings().line_id_ref_ = line_id;
}

/**
 */
void EvaluationMainTabWidget::dbContentTstNameChangedSlot(const std::string& dbcontent_name)
{
    loginf << "name " << dbcontent_name;

    calculator_.dbContentNameTst(dbcontent_name);

    dialog_.updateButtons();
}

/**
 */
void EvaluationMainTabWidget::lineTstChangedSlot(unsigned int line_id)
{
    loginf << "value " << line_id;

    calculator_.settings().line_id_tst_ = line_id;
}

/**
 */
void EvaluationMainTabWidget::usedDataSourcesChangedSlot()
{
    dialog_.updateButtons();
}

/**
 */
void EvaluationMainTabWidget::changedStandardsSlot()
{
    loginf;

    traced_assert(standard_box_);
    standard_box_->updateStandards();
}

/**
 */
void EvaluationMainTabWidget::changedCurrentStandardSlot()
{
    loginf;

    traced_assert(standard_box_);
    standard_box_->setStandardName(calculator_.currentStandardName());

    logdbg << "sectors";

    traced_assert(sector_widget_);
    sector_widget_->update();

    dialog_.updateButtons();

    logdbg << "done";
}
