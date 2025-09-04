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


#include "reconstructormainwidget.h"
#include "reconstructorbase.h"
#include "reconstructortask.h"

#include "logger.h"
#include "propertyvalueedit.h"

#include <QLabel>
#include <QFormLayout>
#include <QComboBox>
#include <QSpinBox>
#include <QLineEdit>
#include <QCheckBox>

/**
*/
ReconstructorMainWidget::ReconstructorMainWidget(ReconstructorBase& reconstructor, 
                                                 QWidget* parent)
:   QWidget       (parent       )
,   reconstructor_(reconstructor)
{
    //reset chosen timeframe
    reconstructor_.resetTimeframeSettings();

    QFormLayout* layout = new QFormLayout;
    layout->setFormAlignment(Qt::AlignRight | Qt::AlignTop);

    setLayout(layout);

    auto addSection = [ & ] (const std::string& name)
    {
        QLabel* label = new QLabel(QString::fromStdString(name));
        auto f = label->font();
        f.setBold(true);
        label->setFont(f);

        layout->addRow(label);
    };

    addSection("Data Source");

    ds_name_edit_ = new QLineEdit;
    ds_name_edit_->setReadOnly(true);
    ds_name_edit_->setFrame(false);

    layout->addRow("Name", ds_name_edit_);

    ds_sac_box_ = new QSpinBox;
    ds_sac_box_->setReadOnly(true);
    ds_sac_box_->setButtonSymbols(QAbstractSpinBox::ButtonSymbols::NoButtons);
    ds_sac_box_->setFrame(false);
    ds_sac_box_->setMinimum(0);
    ds_sac_box_->setMaximum(std::numeric_limits<int>::max());

    layout->addRow("SAC", ds_sac_box_);

    ds_sic_box_ = new QSpinBox;
    ds_sic_box_->setReadOnly(true);
    ds_sic_box_->setButtonSymbols(QAbstractSpinBox::ButtonSymbols::NoButtons);
    ds_sic_box_->setFrame(false);
    ds_sic_box_->setMinimum(0);
    ds_sic_box_->setMaximum(std::numeric_limits<int>::max());

    layout->addRow("SIC", ds_sic_box_);

    ds_line_combo_ = new QComboBox;
    ds_line_combo_->addItem("1", QVariant(0));
    ds_line_combo_->addItem("2", QVariant(1));
    ds_line_combo_->addItem("3", QVariant(2));
    ds_line_combo_->addItem("4", QVariant(3));

    connect(ds_line_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
        [ = ] (int idx) { this->reconstructor_.settings().ds_line = ds_line_combo_->itemData(idx).toInt(); });

    layout->addRow("Line ID", ds_line_combo_);

    addSection("Data Timeframe");

    ds_time_begin_box_ = new PropertyValueEdit(PropertyDataType::TIMESTAMP);
    ds_time_end_box_   = new PropertyValueEdit(PropertyDataType::TIMESTAMP);

    connect(ds_time_begin_box_, &PropertyValueEdit::valueChanged, 
        [ = ] () 
        { 
            auto ts = ds_time_begin_box_->valueAs<boost::posix_time::ptime>();
            this->reconstructor_.settings().data_timestamp_min = ts.has_value() ? ts.value() : boost::posix_time::not_a_date_time;
            this->reconstructor_.informConfigChanged();
        });
    connect(ds_time_end_box_, &PropertyValueEdit::valueChanged,
        [ = ] () 
        { 
            auto ts = ds_time_end_box_->valueAs<boost::posix_time::ptime>();
            this->reconstructor_.settings().data_timestamp_max = ts.has_value() ? ts.value() : boost::posix_time::not_a_date_time;
            this->reconstructor_.informConfigChanged();
        });

    //PropertyValueEdit::connectRange(ds_time_begin_box_, ds_time_end_box_);

    layout->addRow("Begin", ds_time_begin_box_);
    layout->addRow("End", ds_time_end_box_);
    
    addSection("Data Slicing");

    slice_length_box_ = new QSpinBox;
    slice_length_box_->setSuffix(" min");
    slice_length_box_->setMinimum(10);
    slice_length_box_->setMaximum(120);

    connect(slice_length_box_, QOverload<int>::of(&QSpinBox::valueChanged), 
        [ & ] (int v) { this->reconstructor_.settings().slice_duration_in_minutes = v; });

    layout->addRow("Slice Length", slice_length_box_);

    slice_overlap_box_ = new QSpinBox;
    slice_overlap_box_->setSuffix(" min");
    slice_overlap_box_->setMinimum(2);
    slice_overlap_box_->setMaximum(20);

    connect(slice_overlap_box_, QOverload<int>::of(&QSpinBox::valueChanged), 
        [ & ] (int v) { this->reconstructor_.settings().outdated_duration_in_minutes = v; });

    layout->addRow("Slice Overlap Length", slice_overlap_box_);

    addSection("");

    delete_refs_box_ = new QCheckBox("Delete All Calculated Reference Trajectories");

    connect(delete_refs_box_, &QCheckBox::toggled, 
        [ = ] (bool ok) { this->reconstructor_.settings().delete_all_calc_reftraj = ok; });

    layout->addRow("", delete_refs_box_);

    // skip ref writing

    skip_reference_data_writing_box_ = new QCheckBox("Skip Writing of Reference Trajectories");
    skip_reference_data_writing_box_->setToolTip("Targets are created but no calculated reference"
                                                 " target reports or associations are written"
                                                 " into the database");

    connect(skip_reference_data_writing_box_, &QCheckBox::toggled,
            [ = ] (bool ok) { this->reconstructor_.task().skipReferenceDataWriting(ok); });

    layout->addRow("", skip_reference_data_writing_box_);

    updateValues();
}

/**
*/
ReconstructorMainWidget::~ReconstructorMainWidget() = default;

/**
*/
void ReconstructorMainWidget::updateValues()
{
    loginf;

    const auto& settings = reconstructor_.settings();

    traced_assert(ds_name_edit_);
    ds_name_edit_->setText(QString::fromStdString(settings.ds_name));

    traced_assert(ds_sac_box_);
    ds_sac_box_->setValue(settings.ds_sac);

    traced_assert(ds_sic_box_);
    ds_sic_box_->setValue(settings.ds_sic);

    traced_assert(ds_line_combo_);
    int line_idx = ds_line_combo_->findData(QVariant(settings.ds_line));
    traced_assert(line_idx >= 0);
    ds_line_combo_->setCurrentIndex(line_idx);

    traced_assert(ds_time_begin_box_);
    ds_time_begin_box_->setValue<boost::posix_time::ptime>(settings.data_timestamp_min);

    traced_assert(ds_time_end_box_);
    ds_time_end_box_->setValue<boost::posix_time::ptime>(settings.data_timestamp_max);

    traced_assert(slice_length_box_);
    slice_length_box_->setValue(settings.slice_duration_in_minutes);

    traced_assert(slice_overlap_box_);
    slice_overlap_box_->setValue(settings.outdated_duration_in_minutes);

    traced_assert(delete_refs_box_);
    delete_refs_box_->setChecked(settings.delete_all_calc_reftraj);

    traced_assert(skip_reference_data_writing_box_);
    skip_reference_data_writing_box_->setChecked(reconstructor_.task().skipReferenceDataWriting());
}
