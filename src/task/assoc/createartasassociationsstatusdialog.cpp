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

#include "createartasassociationsstatusdialog.h"

#include "compass.h"
#include "createartasassociationstask.h"
//#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "logger.h"
#include "stringconv.h"
#include "dbcontent.h"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include <iomanip>

using namespace std;
using namespace Utils;

CreateARTASAssociationsStatusDialog::CreateARTASAssociationsStatusDialog(
    CreateARTASAssociationsTask& task, QWidget* parent, Qt::WindowFlags f)
    : QDialog(parent, f), task_(task)
{
    setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint);

    setWindowTitle("Calculate ARTAS Target Report Usage Status");

    setModal(true);

    setMinimumSize(QSize(800, 600));

    QFont font_bold;
    font_bold.setBold(true);

    QFont font_big;
    font_big.setPointSize(16);

    QVBoxLayout* main_layout = new QVBoxLayout();

    int row = 0;
    {
        QGridLayout* general_grid = new QGridLayout();

        ++row;
        general_grid->addWidget(new QLabel("Elapsed Time"), row, 0);
        time_label_ = new QLabel();
        time_label_->setAlignment(Qt::AlignRight);
        general_grid->addWidget(time_label_, row, 1);

        main_layout->addLayout(general_grid);
    }

    main_layout->addStretch();

    // associations
    QLabel* map_label = new QLabel("TRI Associations");
    map_label->setFont(font_big);
    main_layout->addWidget(map_label);

    row = 0;
    {
        QGridLayout* association_grid = new QGridLayout();

        association_grid->addWidget(new QLabel("Status"), row, 0);
        association_status_label_ = new QLabel();
        association_status_label_->setAlignment(Qt::AlignRight);
        association_grid->addWidget(association_status_label_, row, 1);

        ++row;
        association_grid->addWidget(
            new QLabel("Acceptable Missing Hashes (in Beginning/End " +
                       QString::number(task_.missesAcceptableTime()) + "s)"),
            row, 0);
        missing_hashes_at_beginning_label_ = new QLabel();
        missing_hashes_at_beginning_label_->setAlignment(Qt::AlignRight);
        association_grid->addWidget(missing_hashes_at_beginning_label_, row, 1);

        ++row;
        association_grid->addWidget(new QLabel("Missing Hashes"), row, 0);
        missing_hashes_label_ = new QLabel();
        missing_hashes_label_->setAlignment(Qt::AlignRight);
        association_grid->addWidget(missing_hashes_label_, row, 1);

        ++row;
        association_grid->addWidget(new QLabel("Found Hashes"), row, 0);
        found_hashes_label_ = new QLabel();
        found_hashes_label_->setAlignment(Qt::AlignRight);
        association_grid->addWidget(found_hashes_label_, row, 1);

        ++row;
        association_grid->addWidget(new QLabel("Dubious Associations"), row, 0);
        dubious_label_ = new QLabel();
        dubious_label_->setAlignment(Qt::AlignRight);
        association_grid->addWidget(dubious_label_, row, 1);

        ++row;
        association_grid->addWidget(new QLabel("Found Duplicate Hashes"), row, 0);
        found_duplicates_label_ = new QLabel();
        found_duplicates_label_->setAlignment(Qt::AlignRight);
        association_grid->addWidget(found_duplicates_label_, row, 1);

        main_layout->addLayout(association_grid);
    }

    main_layout->addStretch();

    // per dbcont associations
    //    QLabel* dbcont_associated_label = new QLabel("DBContent Associations");
    //    dbcont_associated_label->setFont(font_big);
    //    main_layout->addWidget(dbcont_associated_label);

    dbcont_associated_grid_ = new QGridLayout();
    updateDBContentAssociatedGrid();
    main_layout->addLayout(dbcont_associated_grid_);

    main_layout->addStretch();

    QHBoxLayout* button_layout = new QHBoxLayout();

    ok_button_ = new QPushButton("OK");
    ok_button_->setVisible(false);
    connect(ok_button_, &QPushButton::clicked, this,
            &CreateARTASAssociationsStatusDialog::okClickedSlot);
    button_layout->addWidget(ok_button_);

    main_layout->addLayout(button_layout);

    setLayout(main_layout);
}

void CreateARTASAssociationsStatusDialog::okClickedSlot() { emit closeSignal(); }

void CreateARTASAssociationsStatusDialog::markStartTime()
{
    start_time_ = boost::posix_time::microsec_clock::local_time();
}

void CreateARTASAssociationsStatusDialog::setDone()
{
    traced_assert(ok_button_);

    updateTime();
    updateDBContentAssociatedGrid();

    loginf << "done after " << elapsed_time_str_;

    ok_button_->setVisible(true);
}

void CreateARTASAssociationsStatusDialog::setAssociationStatus(const std::string& status)
{
    status_ = status;

    traced_assert(association_status_label_);
    association_status_label_->setText(status_.c_str());

    updateTime();
}

void CreateARTASAssociationsStatusDialog::setAssociationCounts(
        std::map<std::string, std::pair<unsigned int,unsigned int>> association_counts)
{
    association_counts_ = association_counts;

    updateDBContentAssociatedGrid();
}

void CreateARTASAssociationsStatusDialog::setDubiousAssociations(const size_t& dubious_associations)
{
    dubious_associations_ = dubious_associations;

    traced_assert(dubious_label_);
    dubious_label_->setText(QString::number(dubious_associations_));
    ;
}

void CreateARTASAssociationsStatusDialog::setMissingHashesAtBeginning(
    const size_t& missing_hashes_at_beginning)
{
    missing_hashes_at_beginning_ = missing_hashes_at_beginning;

    traced_assert(missing_hashes_at_beginning_label_);
    missing_hashes_at_beginning_label_->setText(QString::number(missing_hashes_at_beginning_));
    ;
}

void CreateARTASAssociationsStatusDialog::setMissingHashes(const size_t& missing_hashes)
{
    missing_hashes_ = missing_hashes;

    traced_assert(missing_hashes_label_);
    missing_hashes_label_->setText(QString::number(missing_hashes_));
}

void CreateARTASAssociationsStatusDialog::setFoundHashes(const size_t& found_hashes)
{
    found_hashes_ = found_hashes;

    traced_assert(found_hashes_label_);
    found_hashes_label_->setText(QString::number(found_hashes_));
}

void CreateARTASAssociationsStatusDialog::setFoundDuplicates(const size_t& found_duplicates)
{
    found_duplicates_ = found_duplicates;

    traced_assert(found_duplicates_label_);
    found_duplicates_label_->setText(QString::number(found_duplicates_));
}

void CreateARTASAssociationsStatusDialog::updateTime()
{
    traced_assert(time_label_);

    stop_time_ = boost::posix_time::microsec_clock::local_time();
    time_diff_ = stop_time_ - start_time_;
    elapsed_time_str_ =
        String::timeStringFromDouble(time_diff_.total_milliseconds() / 1000.0, false);

    time_label_->setText(elapsed_time_str_.c_str());
}

void CreateARTASAssociationsStatusDialog::updateDBContentAssociatedGrid()
{
    traced_assert(dbcont_associated_grid_);

    // loginf << "rowcount " <<
    // cat_counters_grid_->rowCount();

    int row = 1;
    if (dbcont_associated_grid_->rowCount() == 1)
    {
        // loginf << "adding first row";

        QFont font_bold;
        font_bold.setBold(true);

        QLabel* dbcont_label = new QLabel("DBContent");
        dbcont_label->setFont(font_bold);
        dbcont_associated_grid_->addWidget(dbcont_label, row, 0);

        QLabel* count_label = new QLabel("Count");
        count_label->setFont(font_bold);
        count_label->setAlignment(Qt::AlignRight);
        dbcont_associated_grid_->addWidget(count_label, row, 1);

        QLabel* associated_label = new QLabel("Associated");
        associated_label->setFont(font_bold);
        associated_label->setAlignment(Qt::AlignRight);
        dbcont_associated_grid_->addWidget(associated_label, row, 2);

        QLabel* percent_label = new QLabel("Percent");
        percent_label->setFont(font_bold);
        percent_label->setAlignment(Qt::AlignRight);
        dbcont_associated_grid_->addWidget(percent_label, row, 3);
    }

    for (auto& dbcont_it : COMPASS::instance().dbContentManager())
    {
        if (!dbcont_it.second->containsTargetReports() ||
            dbcont_it.second->isReferenceContent())
            continue;

        ++row;

        unsigned int total_cnt = 0;
        unsigned int assoc_cnt = 0;
        float assoc_perc = 0;

        if (association_counts_.count(dbcont_it.first))
        {
            total_cnt = get<0>(association_counts_.at(dbcont_it.first));
            assoc_cnt = get<1>(association_counts_.at(dbcont_it.first));

            if (total_cnt)
                assoc_perc = 100.0 * (float) assoc_cnt / (float) total_cnt;
        }

        if (dbcont_associated_grid_->rowCount() < row + 1)
        {
            // loginf << "adding row " <<
            // row;

            dbcont_associated_grid_->addWidget(new QLabel(), row, 0);

            QLabel* count_label = new QLabel();
            count_label->setAlignment(Qt::AlignRight);
            dbcont_associated_grid_->addWidget(count_label, row, 1);

            QLabel* associated_label = new QLabel();
            associated_label->setAlignment(Qt::AlignRight);
            dbcont_associated_grid_->addWidget(associated_label, row, 2);

            QLabel* percent_label = new QLabel();
            percent_label->setAlignment(Qt::AlignRight);
            dbcont_associated_grid_->addWidget(percent_label, row, 3);
        }

        // loginf << "setting row " << row;

        QLabel* dbcont_label =
            dynamic_cast<QLabel*>(dbcont_associated_grid_->itemAtPosition(row, 0)->widget());
        traced_assert(dbcont_label);
        dbcont_label->setText(dbcont_it.first.c_str());

        QLabel* count_label =
            dynamic_cast<QLabel*>(dbcont_associated_grid_->itemAtPosition(row, 1)->widget());
        traced_assert(count_label);
        count_label->setText(QString::number(total_cnt));

        QLabel* associated_label =
            dynamic_cast<QLabel*>(dbcont_associated_grid_->itemAtPosition(row, 2)->widget());
        traced_assert(associated_label);
        associated_label->setText(QString::number(assoc_cnt));

        QLabel* percent_label =
            dynamic_cast<QLabel*>(dbcont_associated_grid_->itemAtPosition(row, 3)->widget());
        traced_assert(percent_label);

        if (total_cnt)
            percent_label->setText((String::percentToString(assoc_perc) + "%").c_str());
        else
            percent_label->setText("0");
    }
}
