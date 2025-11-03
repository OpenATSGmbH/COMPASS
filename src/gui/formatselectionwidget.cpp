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

#include "formatselectionwidget.h"

#include "logger.h"

FormatSelectionWidget::FormatSelectionWidget(PropertyDataType data_type, Format& format)
    : QPushButton(), data_type_(data_type), format_(format)
{
    logdbg;

    update(data_type, format);

    createMenu();

    connect(&menu_, SIGNAL(triggered(QAction*)), this, SLOT(triggerSlot(QAction*)));
    connect(this, SIGNAL(clicked()), this, SLOT(showMenuSlot()));
}

FormatSelectionWidget::~FormatSelectionWidget() {}

void FormatSelectionWidget::update(PropertyDataType data_type, Format& format)
{
    logdbg << "data type '" << Property::asString(data_type)
           << "' format '" << format << "'";

    if (format_.size() > 0)
    {
        std::string data_type_str = Property::asString(data_type) + +":" + format;
        setText(QString::fromStdString(data_type_str));
    }
    else
        setText("");

    data_type_ = data_type;
    format_ = format;
}

void FormatSelectionWidget::createMenu()
{
    logdbg;
    //    menu_.addAction( "" );

    std::string data_type_str = Property::asString(data_type_);

    for (auto ft_it : format_.getAllFormatOptions().at(data_type_))
    {
        QAction* action = menu_.addAction(QString::fromStdString(ft_it));

        QVariantMap vmap;
        vmap.insert(QString::fromStdString(data_type_str), QVariant(QString::fromStdString(ft_it)));
        action->setData(QVariant(vmap));
    }
    logdbg << "end";
}

void FormatSelectionWidget::showMenuSlot() { menu_.exec(QCursor::pos()); }

void FormatSelectionWidget::triggerSlot(QAction* action)
{
    logdbg;

    QVariantMap vmap = action->data().toMap();
    std::string data_type_str, format_str;

    if (action->text().size() != 0)
    {
        data_type_str = vmap.begin().key().toStdString();
        format_str = vmap.begin().value().toString().toStdString();
    }

    loginf << "got data type '" << data_type_str << "' format '"
           << format_str << "'";

    PropertyDataType data_type = Property::asDataType(data_type_str);
    format_.set(data_type, format_str);

    if (format_.size() > 0)
    {
        std::string tmp_str = data_type_str + ":" + format_;
        setText(QString::fromStdString(tmp_str));
    }
    else
        setText("");

    //  emit selectionChanged();
}
