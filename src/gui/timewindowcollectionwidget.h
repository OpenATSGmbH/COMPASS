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

#include "util/timewindow.h"

#include <QWidget>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>

namespace Utils {
class TimeWindowCollection;
}

class TimeWindowCollectionWidget : public QWidget
{
    Q_OBJECT

private slots:
    void addTimeWindow();
    void editTimeWindow(QListWidgetItem* item);

public:
    explicit TimeWindowCollectionWidget(Utils::TimeWindowCollection& collection, QWidget* parent = nullptr);

    void refreshList();

    bool somethingChangedFlag() const;

private:
    QString timeWindowToString(const Utils::TimeWindow& tw) const;

    Utils::TimeWindowCollection& collection_;
    QListWidget* list_widget_;
    QPushButton* add_button_;

    bool something_changed_flag_ {false};
};
