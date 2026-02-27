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
#include "dbcontentmanager.h"

#include <QHBoxLayout>
#include <QMessageBox>
#include <QHeaderView>

using namespace Utils;

TimeWindowCollectionWidget::TimeWindowCollectionWidget(DBContentManager& dbcont_man,
                                                       TimeWindowCollection& collection,
                                                       QWidget* parent)
    : QWidget(parent), dbcont_man_(dbcont_man), collection_(collection)
{
    //list_widget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    QVBoxLayout* main_layout = new QVBoxLayout();

    QStringList headers;
    headers << "Begin";
    headers << "End";
    headers << "Comment";
    headers << ""; // remove button

    tree_widget_ = new QTreeWidget();
    tree_widget_->setHeaderLabels(headers);

    tree_widget_->header()->setStretchLastSection(false);

    tree_widget_->header()->setSectionResizeMode(0, QHeaderView::ResizeMode::ResizeToContents);
    tree_widget_->header()->setSectionResizeMode(1, QHeaderView::ResizeMode::ResizeToContents);
    tree_widget_->header()->setSectionResizeMode(2, QHeaderView::ResizeMode::Stretch);
    tree_widget_->header()->setSectionResizeMode(3, QHeaderView::ResizeMode::Fixed);

    tree_widget_->header()->resizeSection(3, 30);

    main_layout->addWidget(tree_widget_);

    QHBoxLayout* button_layout = new QHBoxLayout();
    button_layout->addStretch();

    add_button_ = new QPushButton("Add");
    button_layout->addWidget(add_button_);

    main_layout->addLayout(button_layout);

    main_layout->setContentsMargins(0,0,0,0);

    connect(add_button_, &QPushButton::clicked, this, &TimeWindowCollectionWidget::addTimeWindow);
    connect(tree_widget_, &QTreeWidget::itemDoubleClicked, this, &TimeWindowCollectionWidget::editTimeWindow);

    refreshList();

    setContentsMargins(0,0,0,0);
    //setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    setLayout(main_layout);
}

void TimeWindowCollectionWidget::refreshList()
{
    tree_widget_->clear();

    QIcon del_icon(Files::IconProvider::getIcon("delete.png"));

    collection_.sort();

    for (unsigned int i = 0; i < collection_.size(); ++i)
    {
        const TimeWindow& tw = collection_.get(i);

        auto* item = new QTreeWidgetItem;
        item->setText(0, timeWindowBeginToString(tw));
        item->setText(1, timeWindowEndToString(tw));
        item->setText(2, QString::fromStdString(tw.comment()));

        item->setData(0, Qt::UserRole, QVariant::fromValue(i));

        tree_widget_->addTopLevelItem(item);

        // Add context menu actions for edit/delete
        QWidget* item_widget = new QWidget();

        QPushButton* delete_btn = new QPushButton();
        delete_btn->setIcon(del_icon);
        //delete_btn->setIconSize(UI_ICON_SIZE);
        //delete_btn->setMaximumWidth(UI_ICON_BUTTON_MAX_WIDTH);
        delete_btn->setFlat(UI_ICON_BUTTON_FLAT);
        delete_btn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

        QHBoxLayout* layout = new QHBoxLayout(item_widget);
        layout->addStretch();
        layout->addWidget(delete_btn);
        layout->setContentsMargins(0, 0, 0, 0);
        item_widget->setLayout(layout);

        tree_widget_->setItemWidget(item, tree_widget_->columnCount() - 1, item_widget);

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

QString TimeWindowCollectionWidget::timeWindowBeginToString(const Utils::TimeWindow& tw) const
{
    return Time::qtFrom(tw.begin()).toString(Time::QT_DATETIME_FORMAT_SHORT.c_str());
}

QString TimeWindowCollectionWidget::timeWindowEndToString(const Utils::TimeWindow& tw) const
{
    return Time::qtFrom(tw.end()).toString(Time::QT_DATETIME_FORMAT_SHORT.c_str());
}

void TimeWindowCollectionWidget::addTimeWindow()
{
    std::unique_ptr<TimeWindowDialog> dialog;

    auto& dbcont_man = dbcont_man_;

    if (dbcont_man.hasMinMaxTimestamp())
    {
        auto time_stamps = dbcont_man.minMaxTimestamp();
        dialog.reset(new TimeWindowDialog(this, std::get<0>(time_stamps), std::get<1>(time_stamps)));
    }
    else
    {
        dialog.reset(new TimeWindowDialog(this));
    }

    if (dialog->exec() == QDialog::Accepted)
    {
        TimeWindow new_tw(dialog->begin(), dialog->end(), dialog->comment());
        collection_.add(new_tw);
        refreshList();

        something_changed_flag_ = true;
    }
}

void TimeWindowCollectionWidget::editTimeWindow(QTreeWidgetItem* item)
{
    int index = item->data(0, Qt::UserRole).toInt();
    TimeWindow& tw = const_cast<TimeWindow&>(collection_.get(index));

    TimeWindowDialog dialog(this, tw.begin(), tw.end(), tw.comment());
    if (dialog.exec() == QDialog::Accepted) 
    {
        tw = TimeWindow(dialog.begin(), dialog.end(), dialog.comment());
        refreshList();

        something_changed_flag_ = true;
    }
}

bool TimeWindowCollectionWidget::somethingChangedFlag() const
{
    return something_changed_flag_;
}
