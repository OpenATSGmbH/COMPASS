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

#include <QDialog>

#include "boost/date_time/posix_time/posix_time.hpp"

class QLabel;
class QPushButton;
class QGridLayout;
class CreateARTASAssociationsTask;

class CreateARTASAssociationsStatusDialog : public QDialog
{
    Q_OBJECT

signals:
    void closeSignal();

public slots:
    void okClickedSlot();

public:
    CreateARTASAssociationsStatusDialog(CreateARTASAssociationsTask& task,
                                        QWidget* parent=nullptr, Qt::WindowFlags f = Qt::WindowFlags());

    void markStartTime();

    void setAssociationStatus(const std::string& status);
    void setAssociationCounts(std::map<std::string, std::pair<unsigned int,unsigned int>> association_counts);

    void setDone();


    void setMissingHashesAtBeginning(const size_t& missing_hashes_at_beginning);
    void setMissingHashes(const size_t& missing_hashes);
    void setFoundHashes(const size_t& found_hashes);
    void setDubiousAssociations(const size_t& dubious_associations);
    void setFoundDuplicates(const size_t& found_duplicates);

private:
    CreateARTASAssociationsTask& task_;

    boost::posix_time::ptime start_time_;
    boost::posix_time::ptime stop_time_;
    boost::posix_time::time_duration time_diff_;
    std::string elapsed_time_str_;

    std::string status_{"Waiting"};

    std::map<std::string, std::pair<unsigned int,unsigned int>> association_counts_; // dbcontent -> total, assoc cnt

    size_t missing_hashes_at_beginning_{0};
    size_t missing_hashes_{0};
    size_t found_hashes_{0};  // dbcont name -> cnt
    size_t dubious_associations_{0};
    size_t found_duplicates_{0};

    QLabel* time_label_{nullptr};

    QLabel* association_status_label_{nullptr};
    QLabel* missing_hashes_at_beginning_label_{nullptr};
    QLabel* missing_hashes_label_{nullptr};
    QLabel* found_hashes_label_{nullptr};
    QLabel* dubious_label_{nullptr};
    QLabel* found_duplicates_label_{nullptr};

    QGridLayout* dbcont_associated_grid_{nullptr};

    QPushButton* ok_button_{nullptr};

    void updateTime();
    void updateDBContentAssociatedGrid();
};
