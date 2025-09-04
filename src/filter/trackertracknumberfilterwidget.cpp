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

#include "trackertracknumberfilterwidget.h"
//#include "timeconv.h"
#include "datasourcemanager.h"
#include "compass.h"
#include "logger.h"
#include "stringconv.h"

#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QCheckBox>
#include <QDateTimeEdit>

using namespace std;
using namespace Utils;

TrackerTrackNumberFilterWidget::TrackerTrackNumberFilterWidget(TrackerTrackNumberFilter& filter)
    : DBFilterWidget(filter), filter_(filter)
{
    update();
}

TrackerTrackNumberFilterWidget::~TrackerTrackNumberFilterWidget() = default;

void TrackerTrackNumberFilterWidget::update()
{
    loginf;

    traced_assert(child_layout_);

    DBFilterWidget::update();

    deleteChildrenFromLayout();

    DataSourceManager& ds_man = COMPASS::instance().dataSourceManager();

    // ds_id -> line_id -> values
    std::map<unsigned int, std::map<unsigned int, std::string>> active_values = filter_.getActiveTrackerTrackNums();
    std::string ds_name;

    for (auto& ds_it : active_values)
    {
        for (auto& line_it : ds_it.second)
        {
            traced_assert(ds_man.hasDBDataSource(ds_it.first));
            ds_name = ds_man.dbDataSource(ds_it.first).name();

            QLineEdit* value_edit = new QLineEdit(line_it.second.c_str());
            value_edit->setProperty("ds_id", ds_it.first);
            value_edit->setProperty("line_id", line_it.first);
            connect(value_edit, &QLineEdit::textEdited, this, &TrackerTrackNumberFilterWidget::valueEditedSlot);

            addNameValuePair(ds_name + " " + String::lineStrFrom(line_it.first) + " Track Number IN", value_edit);

            //loginf << "adding track number filter for '" << (ds_name + " " + String::lineStrFrom(line_it.first)) << "'";
        }
    }

    emit filterContentChanged();
}

void TrackerTrackNumberFilterWidget::valueEditedSlot(const QString& value)
{
    QLineEdit* value_edit = dynamic_cast<QLineEdit*> (sender());
    traced_assert(value_edit);

    unsigned int ds_id = value_edit->property("ds_id").toUInt();
    unsigned int line_id = value_edit->property("line_id").toUInt();

    loginf << "ds_id " << ds_id
           << " value '" << value.toStdString() << "'";

    filter_.setTrackerTrackNum(ds_id, line_id, value.toStdString());
}

//void TrackerTrackNumberFilterWidget::minDateTimeChanged(const QDateTime& datetime)
//{
//    if (update_active_)
//        return;

//    loginf << "value "
//           << datetime.toString(Time::QT_DATETIME_FORMAT.c_str()).toStdString();

//    filter_.minValue(Time::fromString(datetime.toString(Time::QT_DATETIME_FORMAT.c_str()).toStdString()), false);
//}

//void TrackerTrackNumberFilterWidget::maxDateTimeChanged(const QDateTime& datetime)
//{
//    if (update_active_)
//        return;

//    loginf << "value "
//           << datetime.toString(Time::QT_DATETIME_FORMAT.c_str()).toStdString();

//    filter_.maxValue(Time::fromString(datetime.toString(Time::QT_DATETIME_FORMAT.c_str()).toStdString()), false);
//}
