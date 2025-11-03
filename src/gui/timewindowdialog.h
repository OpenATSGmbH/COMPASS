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
#include <QDateTimeEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <boost/date_time/posix_time/posix_time.hpp>

class TimeWindowDialog : public QDialog
{
    Q_OBJECT

public:
    explicit TimeWindowDialog(QWidget* parent = nullptr,
                              const boost::posix_time::ptime& begin = boost::posix_time::not_a_date_time,
                              const boost::posix_time::ptime& end = boost::posix_time::not_a_date_time);

    boost::posix_time::ptime begin() const;
    boost::posix_time::ptime end() const;

private:
    QDateTimeEdit* begin_edit_;
    QDateTimeEdit* end_edit_;
};
