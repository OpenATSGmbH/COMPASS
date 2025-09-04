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

#include "unitselectionwidget.h"

#include "dimension.h"
#include "logger.h"
#include "unit.h"
#include "unitmanager.h"

UnitSelectionWidget::UnitSelectionWidget(std::string& dimension, std::string& unit)
    : QPushButton(), dimension_(&dimension), unit_(&unit)
{
    logdbg << "start";

    pointers_set_ = true;

    update(dimension, unit);

    createMenu();

    connect(this, &UnitSelectionWidget::clicked, this, &UnitSelectionWidget::showMenuSlot);
}

UnitSelectionWidget::UnitSelectionWidget()
{
    pointers_set_ = false;
    setDisabled(true);

    createMenu();

    connect(this, &UnitSelectionWidget::clicked, this, &UnitSelectionWidget::showMenuSlot);
}

UnitSelectionWidget::~UnitSelectionWidget() {}

void UnitSelectionWidget::createMenu()
{
    menu_.addAction("");

    for (auto& it : UnitManager::instance().dimensions())
    {
        const std::map<std::string, Unit*>& units = it.second->units();

        if (units.size() > 0)
        {
            //      loginf << "unit " << it->first;
            QMenu* m2 = menu_.addMenu(QString::fromStdString(it.first));

            for (auto& it2 : units)
            {
                //        loginf << "unitunit " << unitit->first;
                QAction* action = m2->addAction(QString::fromStdString(it2.first));

                QVariantMap vmap;
                vmap.insert(QString::fromStdString(it2.first), QVariant(QString::fromStdString(it.first)));
                action->setData(QVariant(vmap));
            }
        }
    }

    connect(&menu_, &QMenu::triggered, this, &UnitSelectionWidget::triggerSlot);
}

void UnitSelectionWidget::update(std::string& dimension, std::string& unit)
{
    dimension_ = &dimension;
    unit_ = &unit;

    pointers_set_ = true;

    if (dimension_->size() > 0)
        setText(QString::fromStdString(*dimension_) + ":" + QString::fromStdString(*unit_));
    else
        setText("");

    setDisabled(false);
}

void UnitSelectionWidget::clear()
{
    pointers_set_ = false;

    dimension_ = nullptr;
    unit_ = nullptr;

    setText("");

    setDisabled(true);
}


void UnitSelectionWidget::showMenuSlot()
{
    loginf;

    traced_assert(pointers_set_);

    menu_.exec(QCursor::pos());
}

void UnitSelectionWidget::triggerSlot(QAction* action)
{
    traced_assert(pointers_set_);

    QVariantMap vmap = action->data().toMap();
    std::string dimension, unit;

    if (action->text().size() != 0)
    {
        dimension = vmap.begin().value().toString().toStdString();
        unit = vmap.begin().key().toStdString();
    }

    loginf << "got dimension " << dimension << " unit " << unit;

    *dimension_ = dimension;
    *unit_ = unit;

    if (dimension_->size() > 0)
        setText(QString::fromStdString(*dimension_) + ":" + QString::fromStdString(*unit_));
    else
        setText("");

    //  emit selectionChanged();
}
