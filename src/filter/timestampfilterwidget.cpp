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

#include "timestampfilterwidget.h"
#include "timeconv.h"
#include "logger.h"

#include <QFormLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QCheckBox>
#include <QDateTimeEdit>
#include <QPushButton>
#include <QWidget>

using namespace std;
using namespace Utils;

TimestampFilterWidget::TimestampFilterWidget(TimestampFilter& filter)
    : DBFilterWidget(filter), filter_(filter)
{
    //QFormLayout* layout = new QFormLayout();

    min_edit_ = new QDateTimeEdit(QDateTime::currentDateTime());
    min_edit_->setObjectName("filter_min_edit");
    min_edit_->setDisplayFormat(Time::QT_DATETIME_FORMAT.c_str());
    connect(min_edit_, &QDateTimeEdit::dateTimeChanged, this, &TimestampFilterWidget::minDateTimeChanged);

    addNameValuePair("Timestamp >=", min_edit_);

    max_edit_ = new QDateTimeEdit(QDateTime::currentDateTime());
    max_edit_->setObjectName("filter_max_edit");
    max_edit_->setDisplayFormat(Time::QT_DATETIME_FORMAT.c_str());
    connect(max_edit_, &QDateTimeEdit::dateTimeChanged, this, &TimestampFilterWidget::maxDateTimeChanged);

    addNameValuePair("Timestamp <=", max_edit_);

    QWidget* step_widget = new QWidget();
    QHBoxLayout* step_layout = new QHBoxLayout();
    step_layout->setContentsMargins(0, 0, 0, 0);
    step_layout->setSpacing(2);

    const int step_minutes[] = {-60, -45, -30, -15, 15, 30, 45, 60};
    int idx = 0;
    for (int delta : step_minutes)
    {
        if (delta > 0 && step_minutes[idx - 1] < 0)
            step_layout->addStretch();

        QString text = delta > 0 ? QString("+%1m").arg(delta) : QString("%1m").arg(delta);
        QPushButton* button = new QPushButton(text);
        button->setObjectName(QString("filter_step_%1_button").arg(delta));
        button->setIcon(QIcon());
        button->setToolTip(QString("Shift window by %1 minutes").arg(delta));
        button->setFixedWidth(40);
        connect(button, &QPushButton::clicked, this, [this, delta]() { filter_.shiftWindow(delta); });
        step_layout->addWidget(button);
        step_buttons_.push_back({delta, button});
        ++idx;
    }

    step_widget->setLayout(step_layout);
    addNameValuePair("Step", step_widget);

    update();
}

TimestampFilterWidget::~TimestampFilterWidget()
{
}

void TimestampFilterWidget::update()
{
    loginf;

    update_active_ = true;

    DBFilterWidget::update();

    traced_assert(min_edit_);
    traced_assert(max_edit_);

    min_edit_->setDateTime(QDateTime::fromString(Time::toString(filter_.minValue()).c_str(),
                                                 Time::QT_DATETIME_FORMAT.c_str()));
    max_edit_->setDateTime(QDateTime::fromString(Time::toString(filter_.maxValue()).c_str(),
                                                 Time::QT_DATETIME_FORMAT.c_str()));

    updateStepButtons();

    update_active_ = false;
}

void TimestampFilterWidget::updateStepButtons()
{
    for (auto& entry : step_buttons_)
        entry.second->setEnabled(filter_.canShiftWindow(entry.first));
}


void TimestampFilterWidget::minDateTimeChanged(const QDateTime& datetime)
{
    if (update_active_)
        return;

    loginf << "value "
           << datetime.toString(Time::QT_DATETIME_FORMAT.c_str()).toStdString();

    filter_.minValue(Time::fromString(datetime.toString(Time::QT_DATETIME_FORMAT.c_str()).toStdString()), false);
    updateStepButtons();
}

void TimestampFilterWidget::maxDateTimeChanged(const QDateTime& datetime)
{
    if (update_active_)
        return;

    loginf << "value "
           << datetime.toString(Time::QT_DATETIME_FORMAT.c_str()).toStdString();

    filter_.maxValue(Time::fromString(datetime.toString(Time::QT_DATETIME_FORMAT.c_str()).toStdString()), false);
    updateStepButtons();
}

