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

#include "datatypeformatselectionwidget.h"
#include "traced_assert.h"
#include "logger.h"

DataTypeFormatSelectionWidget::DataTypeFormatSelectionWidget(std::string& data_type_str,
                                                             Format& format)
    : QPushButton(), data_type_str_(&data_type_str), format_(&format)
{
    logdbg;

    pointers_set_ = true;

    showValues();

    createMenu();

    connect(this, &DataTypeFormatSelectionWidget::clicked, this, &DataTypeFormatSelectionWidget::showMenuSlot);

    setDisabled(false);
}

DataTypeFormatSelectionWidget::DataTypeFormatSelectionWidget()
{
    pointers_set_ = false;

    connect(this, &DataTypeFormatSelectionWidget::clicked, this, &DataTypeFormatSelectionWidget::showMenuSlot);

    setDisabled(true);
}

DataTypeFormatSelectionWidget::~DataTypeFormatSelectionWidget() {}


void DataTypeFormatSelectionWidget::update(std::string& data_type_str, Format& format)
{
    data_type_str_ = &data_type_str;
    format_ = &format;

    pointers_set_ = true;

    showValues();
    createMenu();

    setDisabled(false);
}

void DataTypeFormatSelectionWidget::DataTypeFormatSelectionWidget::clear()
{
    pointers_set_ = false;

    data_type_str_ = nullptr;
    format_ = nullptr;

    setText("");
    menu_ = nullptr;

    setDisabled(true);
}

void DataTypeFormatSelectionWidget::showValues()
{
    traced_assert(pointers_set_);

    if (data_type_str_->size() && format_->size())
        setText(QString::fromStdString(*data_type_str_ + ":" + *format_));
    else
        setText("");
}

void DataTypeFormatSelectionWidget::createMenu()
{
    logdbg;

    traced_assert(pointers_set_);

    menu_.reset(new QMenu());

    for (auto dt_it : format_->getAllFormatOptions())
    {
        logdbg << "dt "
               << static_cast<unsigned>(dt_it.first);
        std::string data_type_str = Property::asString(dt_it.first);
        logdbg << "dt str " << data_type_str;
        QMenu* m2 = menu_->addMenu(QString::fromStdString(data_type_str));

        for (auto ft_it : dt_it.second)
        {
            logdbg << "format " << ft_it;
            QAction* action = m2->addAction(QString::fromStdString(ft_it));

            QVariantMap vmap;
            vmap.insert(QString::fromStdString(data_type_str),
                        QVariant(QString::fromStdString(ft_it)));
            action->setData(QVariant(vmap));
        }
    }

    connect(menu_.get(), &QMenu::triggered, this, &DataTypeFormatSelectionWidget::triggerSlot);

    logdbg << "end";
}

void DataTypeFormatSelectionWidget::showMenuSlot()
{
    logdbg;

    traced_assert(pointers_set_);
    traced_assert(menu_);

    menu_->exec(QCursor::pos());
}

void DataTypeFormatSelectionWidget::triggerSlot(QAction* action)
{
    loginf;

    traced_assert(pointers_set_);

    QVariantMap vmap = action->data().toMap();
    std::string data_type_str, format_str;

    if (action->text().size() != 0)
    {
        data_type_str = vmap.begin().key().toStdString();
        format_str = vmap.begin().value().toString().toStdString();
    }

    loginf << "got data type '" << data_type_str
           << "' format '" << format_str << "'";

    if (data_type_str.size() && format_str.size())
    {
        PropertyDataType data_type = Property::asDataType(data_type_str);
        *data_type_str_ = data_type_str;
        format_->set(data_type, format_str);

        std::string tmp_str = data_type_str + ":" + *format_;
        setText(QString::fromStdString(tmp_str));
    }
    else
    {
        format_->set(PropertyDataType::BOOL, "");
        setText("");
    }
}
