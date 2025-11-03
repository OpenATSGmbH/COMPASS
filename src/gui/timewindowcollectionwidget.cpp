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

#include "timewindowcollectionwidget.h"
#include "timewindowdialog.h"
#include "util/files.h"
#include "util/timeconv.h"
#include "compass.h"
#include "dbcontentmanager.h"

#include <QHBoxLayout>
#include <QMessageBox>

using namespace Utils;

TimeWindowCollectionWidget::TimeWindowCollectionWidget(TimeWindowCollection& collection, QWidget* parent)
    : QWidget(parent), collection_(collection)
{
    //list_widget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);


    QVBoxLayout* main_layout = new QVBoxLayout();

    list_widget_ = new QListWidget();
    main_layout->addWidget(list_widget_);

    QHBoxLayout* button_layout = new QHBoxLayout();
    button_layout->addStretch();

    add_button_ = new QPushButton("Add");
    button_layout->addWidget(add_button_);

    main_layout->addLayout(button_layout);

    main_layout->setContentsMargins(0,0,0,0);

    connect(add_button_, &QPushButton::clicked, this, &TimeWindowCollectionWidget::addTimeWindow);
    connect(list_widget_, &QListWidget::itemDoubleClicked, this, &TimeWindowCollectionWidget::editTimeWindow);

    refreshList();

    setContentsMargins(0,0,0,0);
    //setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    setLayout(main_layout);
}

void TimeWindowCollectionWidget::refreshList()
{
    list_widget_->clear();

    QIcon del_icon(Files::IconProvider::getIcon("delete.png"));

    for (unsigned int i = 0; i < collection_.size(); ++i)
    {
        const TimeWindow& tw = collection_.get(i);
        auto* item = new QListWidgetItem(timeWindowToString(tw), list_widget_);
        item->setData(Qt::UserRole, QVariant::fromValue(i));

        // Add context menu actions for edit/delete
        QWidget* item_widget = new QWidget();

        QPushButton* delete_btn = new QPushButton();
        delete_btn->setIcon(del_icon);
        //delete_btn->setIconSize(UI_ICON_SIZE);
        //delete_btn->setMaximumWidth(UI_ICON_BUTTON_MAX_WIDTH);
        delete_btn->setFlat(UI_ICON_BUTTON_FLAT);

        QHBoxLayout* layout = new QHBoxLayout(item_widget);
        layout->addStretch();
        layout->addWidget(delete_btn);
        layout->setContentsMargins(0, 0, 0, 0);
        item_widget->setLayout(layout);

        list_widget_->setItemWidget(item, item_widget);

        connect(delete_btn, &QPushButton::clicked, [this, i]() {
            collection_.erase(i);
            refreshList();
        });
    }
}

QString TimeWindowCollectionWidget::timeWindowToString(const TimeWindow& tw) const
{
    return
        Time::qtFrom(tw.begin()).toString(Time::QT_DATETIME_FORMAT_SHORT.c_str()) + " - " +
        Time::qtFrom(tw.end()).toString(Time::QT_DATETIME_FORMAT_SHORT.c_str());
}

void TimeWindowCollectionWidget::addTimeWindow()
{
    std::unique_ptr<TimeWindowDialog> dialog;

    auto& dbcont_man = COMPASS::instance().dbContentManager();

    if (dbcont_man.hasMinMaxTimestamp())
    {
        auto time_stamps = dbcont_man.minMaxTimestamp();
        dialog.reset(new TimeWindowDialog(this, std::get<0>(time_stamps), std::get<1>(time_stamps)));
    }
    else
        dialog.reset(new TimeWindowDialog(this));

    if (dialog->exec() == QDialog::Accepted)
    {
        TimeWindow new_tw(dialog->begin(), dialog->end());
        collection_.add(new_tw);
        refreshList();

        something_changed_flag_ = true;
    }
}

void TimeWindowCollectionWidget::editTimeWindow(QListWidgetItem* item)
{
    int index = item->data(Qt::UserRole).toInt();
    TimeWindow& tw = const_cast<TimeWindow&>(collection_.get(index));

    TimeWindowDialog dialog(this, tw.begin(), tw.end());
    if (dialog.exec() == QDialog::Accepted) {
        tw = TimeWindow(dialog.begin(), dialog.end());
        refreshList();

        something_changed_flag_ = true;
    }
}

bool TimeWindowCollectionWidget::somethingChangedFlag() const
{
    return something_changed_flag_;
}
