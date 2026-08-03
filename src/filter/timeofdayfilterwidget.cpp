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

#include "timeofdayfilterwidget.h"
#include "dbfilter.h"
#include "dbfiltercondition.h"
#include "filtermanager.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/dbcontentdataengine.h"
#include "logger.h"

#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTime>
#include <QWidget>

#include <algorithm>

namespace
{
    constexpr const char* TIME_FORMAT = "HH:mm:ss.zzz";
    constexpr int SECONDS_PER_DAY = 24 * 60 * 60;

    std::string formatSeconds(int secs, int msec)
    {
        int h = secs / 3600;
        int m = (secs % 3600) / 60;
        int s = secs % 60;

        QTime t(h, m, s, msec);
        return t.toString(TIME_FORMAT).toStdString();
    }
}

TimeOfDayFilterWidget::TimeOfDayFilterWidget(DBFilter& filter)
    : DBFilterWidget(filter)
{
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
        button->setIcon(QIcon());
        button->setToolTip(QString("Shift window by %1 minutes").arg(delta));
        button->setFixedWidth(40);
        connect(button, &QPushButton::clicked, this, [this, delta]() { shiftWindow(delta); });
        step_layout->addWidget(button);
        step_buttons_.push_back({delta, button});
        ++idx;
    }

    step_widget->setLayout(step_layout);
    addNameValuePair("Step", step_widget);

    updateButtons();

    for (const auto& cond : filter_.getConditions())
        connect(cond->getEdit(), &QLineEdit::textChanged, this, [this](const QString&) { updateButtons(); });
}

TimeOfDayFilterWidget::~TimeOfDayFilterWidget() = default;

void TimeOfDayFilterWidget::update()
{
    DBFilterWidget::update();
    updateButtons();
}

std::pair<int, int> TimeOfDayFilterWidget::effectiveBoundsSecs() const
{
    auto& dbcont_man = filter_.filterManager().dbContentManager();

    if (!dbcont_man.dataEngine().hasMinMaxTimestamp())
        return {0, SECONDS_PER_DAY - 1};

    auto minmax = dbcont_man.dataEngine().minMaxTimestamp();
    long span_secs = (minmax.second - minmax.first).total_seconds();

    if (span_secs >= SECONDS_PER_DAY)
        return {0, SECONDS_PER_DAY - 1};

    auto tod_secs = [](const boost::posix_time::ptime& t) {
        return static_cast<int>(t.time_of_day().total_seconds());
    };

    int db_min_tod = tod_secs(minmax.first);
    int db_max_tod = tod_secs(minmax.second);

    if (db_min_tod > db_max_tod)
        return {0, SECONDS_PER_DAY - 1};

    return {db_min_tod, db_max_tod};
}

void TimeOfDayFilterWidget::updateButtons()
{
    int min_secs = SECONDS_PER_DAY;
    int max_secs = -1;

    for (const auto& cond : filter_.getConditions())
    {
        QTime t = QTime::fromString(QString::fromStdString(cond->getValue()), TIME_FORMAT);
        if (!t.isValid())
            continue;

        int secs = t.hour() * 3600 + t.minute() * 60 + t.second();
        min_secs = std::min(min_secs, secs);
        max_secs = std::max(max_secs, secs);
    }

    auto bounds = effectiveBoundsSecs();

    for (auto& entry : step_buttons_)
    {
        bool can = entry.first > 0
            ? max_secs < bounds.second
            : min_secs > bounds.first;
        entry.second->setEnabled(can);
    }
}

void TimeOfDayFilterWidget::shiftWindow(int minutes)
{
    int delta_secs = minutes * 60;
    auto bounds = effectiveBoundsSecs();

    for (const auto& cond : filter_.getConditions())
    {
        QTime t = QTime::fromString(QString::fromStdString(cond->getValue()), TIME_FORMAT);
        if (!t.isValid())
        {
            logwrn << "skipping condition with unparseable value '" << cond->getValue() << "'";
            continue;
        }

        int secs = t.hour() * 3600 + t.minute() * 60 + t.second();
        int msec = t.msec();
        int new_secs = secs + delta_secs;

        if (new_secs < bounds.first)
            new_secs = bounds.first;
        if (new_secs > bounds.second)
            new_secs = bounds.second;

        cond->setValue(formatSeconds(new_secs, msec));
    }

    updateButtons();
}
