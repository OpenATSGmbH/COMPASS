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

#include "createartasassociationstaskwidget.h"
#include "createartasassociationstask.h"
#include "dbdatasourceselectioncombobox.h"
#include "compass.h"
#include "dbcontent/dbcontentmanager.h"
//#include "dbcontent/dbcontent.h"
//#include "dbcontent/variable/variable.h"
//#include "dbcontent/variable/variableselectionwidget.h"
//#include "logger.h"
//#include "dbcontent/variable/metavariable.h"
//#include "taskmanager.h"

#include <QDoubleValidator>
#include <QCheckBox>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QComboBox>

using namespace dbContent;

CreateARTASAssociationsTaskWidget::CreateARTASAssociationsTaskWidget(
        CreateARTASAssociationsTask& task, QWidget* parent, Qt::WindowFlags f)
    : QWidget(parent, f), task_(task)
{
    QFont font_bold;
    font_bold.setBold(true);

    QVBoxLayout* main_layout = new QVBoxLayout();

    {
        QGridLayout* grid = new QGridLayout();
        int row_cnt = 0;

        // tracker data source
        grid->addWidget(new QLabel("CAT062 Data Source"), row_cnt, 0);

        traced_assert(COMPASS::instance().dbContentManager().existsDBContent("CAT062"));

        ds_combo_ = new DBDataSourceComboBox();
        ds_combo_->showDBContentOnly("CAT062");
        connect(ds_combo_, &DBDataSourceComboBox::changedSource, this,
                &CreateARTASAssociationsTaskWidget::currentDataSourceChangedSlot);

        grid->addWidget(ds_combo_, row_cnt, 1);

        // line
        row_cnt++;
        grid->addWidget(new QLabel("Line ID"), row_cnt, 0);
        ds_line_combo_ = new QComboBox;
        ds_line_combo_->addItem("1", QVariant(0u));
        ds_line_combo_->addItem("2", QVariant(1u));
        ds_line_combo_->addItem("3", QVariant(2u));
        ds_line_combo_->addItem("4", QVariant(3u));

        ds_line_combo_->setCurrentIndex(ds_line_combo_->findData(QVariant(task.currentDataSourceLineID())));

        connect(ds_line_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
                &CreateARTASAssociationsTaskWidget::currentDataSourceLineChangedSlot);

        grid->addWidget(ds_line_combo_, row_cnt, 1);

        // time stuff
        row_cnt++;
        grid->addWidget(new QLabel("End Track Time (s)"), row_cnt, 0);
        end_track_time_edit_ = new QLineEdit(QString::number(task.endTrackTime()));
        end_track_time_edit_->setValidator(new QDoubleValidator(10, 60 * 3600, 2, this));
        connect(end_track_time_edit_, &QLineEdit::textEdited, this,
                &CreateARTASAssociationsTaskWidget::endTrackTimeEditSlot);
        grid->addWidget(end_track_time_edit_, row_cnt, 1);

        row_cnt++;
        grid->addWidget(new QLabel("Association Time Past (s)"), row_cnt, 0);
        association_time_past_edit_ = new QLineEdit(QString::number(task.associationTimePast()));
        association_time_past_edit_->setValidator(new QDoubleValidator(0, 600, 2, this));
        connect(association_time_past_edit_, &QLineEdit::textEdited, this,
                &CreateARTASAssociationsTaskWidget::associationTimePastEditSlot);
        grid->addWidget(association_time_past_edit_, row_cnt, 1);

        row_cnt++;
        grid->addWidget(new QLabel("Association Time Future (s)"), row_cnt, 0);
        association_time_future_edit_ =
                new QLineEdit(QString::number(task.associationTimeFuture()));
        association_time_future_edit_->setValidator(new QDoubleValidator(0, 600, 2, this));
        connect(association_time_future_edit_, &QLineEdit::textEdited, this,
                &CreateARTASAssociationsTaskWidget::associationTimeFutureEditSlot);
        grid->addWidget(association_time_future_edit_, row_cnt, 1);

        row_cnt++;
        grid->addWidget(new QLabel("Acceptable Time for Misses (s)"), row_cnt, 0);
        misses_acceptable_time_edit_ = new QLineEdit(QString::number(task.missesAcceptableTime()));
        misses_acceptable_time_edit_->setValidator(new QDoubleValidator(0, 600, 2, this));
        connect(misses_acceptable_time_edit_, &QLineEdit::textEdited, this,
                &CreateARTASAssociationsTaskWidget::missesAcceptableTimeEditSlot);
        grid->addWidget(misses_acceptable_time_edit_, row_cnt, 1);

        row_cnt++;
        grid->addWidget(new QLabel("Dubious Association Distant Time (s)"), row_cnt, 0);
        associations_dubious_distant_time_edit_ =
                new QLineEdit(QString::number(task.associationsDubiousDistantTime()));
        associations_dubious_distant_time_edit_->setValidator(
                    new QDoubleValidator(0, 600, 2, this));
        connect(associations_dubious_distant_time_edit_, &QLineEdit::textEdited, this,
                &CreateARTASAssociationsTaskWidget::associationsDubiousDistantTimeEditSlot);
        grid->addWidget(associations_dubious_distant_time_edit_, row_cnt, 1);

        row_cnt++;
        grid->addWidget(new QLabel("Dubious Association Close Time Past (s)"), row_cnt, 0);
        association_dubious_close_time_past_edit_ =
                new QLineEdit(QString::number(task.associationDubiousCloseTimePast()));
        association_dubious_close_time_past_edit_->setValidator(
                    new QDoubleValidator(0, 600, 2, this));
        connect(association_dubious_close_time_past_edit_, &QLineEdit::textEdited, this,
                &CreateARTASAssociationsTaskWidget::associationDubiousCloseTimePastEditSlot);
        grid->addWidget(association_dubious_close_time_past_edit_, row_cnt, 1);

        row_cnt++;
        grid->addWidget(new QLabel("Dubious Association Close Time Future (s)"), row_cnt, 0);
        association_dubious_close_time_future_edit_ =
                new QLineEdit(QString::number(task.associationDubiousCloseTimeFuture()));
        association_dubious_close_time_future_edit_->setValidator(
                    new QDoubleValidator(0, 600, 2, this));
        connect(association_dubious_close_time_future_edit_, &QLineEdit::textEdited, this,
                &CreateARTASAssociationsTaskWidget::associationDubiousCloseTimeFutureEditSlot);
        grid->addWidget(association_dubious_close_time_future_edit_, row_cnt, 1);

        main_layout->addLayout(grid);
    }

    // track flag stuff
    ignore_track_end_associations_check_ = new QCheckBox("Ignore Track End Associations");
    ignore_track_end_associations_check_->setChecked(task_.ignoreTrackEndAssociations());
    connect(ignore_track_end_associations_check_, &QCheckBox::clicked, this,
            &CreateARTASAssociationsTaskWidget::anyTrackFlagChangedSlot);
    main_layout->addWidget(ignore_track_end_associations_check_);

    mark_track_end_associations_dubious_check_ =
            new QCheckBox("Mark Track End Associations Dubious");
    mark_track_end_associations_dubious_check_->setChecked(task_.markTrackEndAssociationsDubious());
    connect(mark_track_end_associations_dubious_check_, &QCheckBox::clicked, this,
            &CreateARTASAssociationsTaskWidget::anyTrackFlagChangedSlot);
    main_layout->addWidget(mark_track_end_associations_dubious_check_);

    ignore_track_coasting_associations_check_ = new QCheckBox("Ignore Track Coasting Associations");
    ignore_track_coasting_associations_check_->setChecked(task_.ignoreTrackCoastingAssociations());
    connect(ignore_track_coasting_associations_check_, &QCheckBox::clicked, this,
            &CreateARTASAssociationsTaskWidget::anyTrackFlagChangedSlot);
    main_layout->addWidget(ignore_track_coasting_associations_check_);

    mark_track_coasting_associations_dubious_check_ =
            new QCheckBox("Mark Track Costing Associations Dubious");
    mark_track_coasting_associations_dubious_check_->setChecked(
                task_.markTrackCoastingAssociationsDubious());
    connect(mark_track_coasting_associations_dubious_check_, &QCheckBox::clicked, this,
            &CreateARTASAssociationsTaskWidget::anyTrackFlagChangedSlot);
    main_layout->addWidget(mark_track_coasting_associations_dubious_check_);

    main_layout->addStretch();

    update();

    setLayout(main_layout);
}

CreateARTASAssociationsTaskWidget::~CreateARTASAssociationsTaskWidget() {}

void CreateARTASAssociationsTaskWidget::currentDataSourceChangedSlot()
{
    traced_assert(ds_combo_);
    task_.currentDataSourceName(ds_combo_->getDSName());
}

void CreateARTASAssociationsTaskWidget::currentDataSourceLineChangedSlot()
{
    traced_assert(ds_line_combo_);
    task_.currentDataSourceLineID(ds_line_combo_->currentData().toUInt());
}

void CreateARTASAssociationsTaskWidget::update()
{
    if (task_.currentDataSourceName().size() &&
            ds_combo_->hasDSName(task_.currentDataSourceName()))
        ds_combo_->setDSName(task_.currentDataSourceName());

    if (ds_combo_->getDSName() != task_.currentDataSourceName())
        task_.currentDataSourceName(ds_combo_->getDSName());
}

void CreateARTASAssociationsTaskWidget::endTrackTimeEditSlot(QString value)
{
    bool ok;
    float val = value.toFloat(&ok);
    traced_assert(ok);
    task_.endTrackTime(val);
}

void CreateARTASAssociationsTaskWidget::associationTimePastEditSlot(QString value)
{
    bool ok;
    float val = value.toFloat(&ok);
    traced_assert(ok);
    task_.associationTimePast(val);
}

void CreateARTASAssociationsTaskWidget::associationTimeFutureEditSlot(QString value)
{
    bool ok;
    float val = value.toFloat(&ok);
    traced_assert(ok);
    task_.associationTimeFuture(val);
}

void CreateARTASAssociationsTaskWidget::missesAcceptableTimeEditSlot(QString value)
{
    bool ok;
    float val = value.toFloat(&ok);
    traced_assert(ok);
    task_.missesAcceptableTime(val);
}

void CreateARTASAssociationsTaskWidget::associationsDubiousDistantTimeEditSlot(QString value)
{
    bool ok;
    float val = value.toFloat(&ok);
    traced_assert(ok);
    task_.associationsDubiousDistantTime(val);
}

void CreateARTASAssociationsTaskWidget::associationDubiousCloseTimePastEditSlot(QString value)
{
    bool ok;
    float val = value.toFloat(&ok);
    traced_assert(ok);
    task_.associationDubiousCloseTimePast(val);
}

void CreateARTASAssociationsTaskWidget::associationDubiousCloseTimeFutureEditSlot(QString value)
{
    bool ok;
    float val = value.toFloat(&ok);
    traced_assert(ok);
    task_.associationDubiousCloseTimeFuture(val);
}

void CreateARTASAssociationsTaskWidget::anyTrackFlagChangedSlot()
{
    traced_assert(ignore_track_end_associations_check_);
    if ((ignore_track_end_associations_check_->checkState() == Qt::Checked) !=
            task_.ignoreTrackEndAssociations())
        task_.ignoreTrackEndAssociations(ignore_track_end_associations_check_->checkState() ==
                                         Qt::Checked);

    traced_assert(mark_track_end_associations_dubious_check_);
    if ((mark_track_end_associations_dubious_check_->checkState() == Qt::Checked) !=
            task_.markTrackEndAssociationsDubious())
        task_.markTrackEndAssociationsDubious(
                    mark_track_end_associations_dubious_check_->checkState() == Qt::Checked);

    traced_assert(ignore_track_coasting_associations_check_);
    if ((ignore_track_coasting_associations_check_->checkState() == Qt::Checked) !=
            task_.ignoreTrackCoastingAssociations())
        task_.ignoreTrackCoastingAssociations(
                    ignore_track_coasting_associations_check_->checkState() == Qt::Checked);

    traced_assert(mark_track_coasting_associations_dubious_check_);
    if ((mark_track_coasting_associations_dubious_check_->checkState() == Qt::Checked) !=
            task_.markTrackCoastingAssociationsDubious())
        task_.markTrackCoastingAssociationsDubious(
                    mark_track_coasting_associations_dubious_check_->checkState() == Qt::Checked);
}
