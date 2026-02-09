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

#include "timewindowdialog.h"
#include "timeconv.h"
#include "logger.h"

#include <QDialogButtonBox>
#include <QFormLayout>

// const std::string time_format{"yyyy-MM-dd HH:mm:sss"};
// const std::string time_format_long{"yyyy-MM-dd HH:mm:sss.zzz"};

using namespace Utils;

TimeWindowDialog::TimeWindowDialog(QWidget* parent, 
                                   const boost::posix_time::ptime& begin, 
                                   const boost::posix_time::ptime& end,
                                   const std::string& comment)
    : QDialog(parent)
{
    loginf << "begin " << Time::toString(begin) << " end " << Time::toString(end);

    begin_edit_ = new QDateTimeEdit(this);
    end_edit_ = new QDateTimeEdit(this);

    begin_edit_->setDisplayFormat(Time::QT_DATETIME_FORMAT_SHORT.c_str());
    end_edit_->setDisplayFormat(Time::QT_DATETIME_FORMAT_SHORT.c_str());

    if (!begin.is_not_a_date_time())
        begin_edit_->setDateTime(Time::qtFrom(begin, false));
    if (!end.is_not_a_date_time())
        end_edit_->setDateTime(Time::qtFrom(end, false));

    comment_edit_ = new QLineEdit;
    comment_edit_->setText(QString::fromStdString(comment));

    auto* layout = new QVBoxLayout(this);

    auto* layout_content = new QFormLayout;
    layout_content->addRow("Begin", begin_edit_);
    layout_content->addRow("End", end_edit_);
    layout_content->addRow("Comment", comment_edit_);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);

    layout->addLayout(layout_content);
    layout->addStretch();
    layout->addWidget(buttons);

    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
}

boost::posix_time::ptime TimeWindowDialog::begin() const
{
    return boost::posix_time::time_from_string(begin_edit_->dateTime().toString(Time::QT_DATETIME_FORMAT.c_str()).toStdString());
}

boost::posix_time::ptime TimeWindowDialog::end() const
{
    return boost::posix_time::time_from_string(end_edit_->dateTime().toString(Time::QT_DATETIME_FORMAT.c_str()).toStdString());
}

std::string TimeWindowDialog::comment() const
{
    return comment_edit_->text().toStdString();
}
