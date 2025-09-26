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

#include "evaluationstandardcombobox.h"
#include "evaluationstandard.h"
#include "evaluationcalculator.h"
#include "logger.h"

using namespace std;

EvaluationStandardComboBox::EvaluationStandardComboBox(EvaluationCalculator& calculator, QWidget* parent)
    : QComboBox(parent), calculator_(calculator)
{
    updateStandards();

    connect(this, SIGNAL(activated(const QString&)),
            this, SLOT(changedStandardSlot(const QString&)));
}

EvaluationStandardComboBox::~EvaluationStandardComboBox()
{
}

void EvaluationStandardComboBox::changedStandardSlot(const QString& standard_name)
{
    string std_name = standard_name.toStdString();

    loginf << "standard '" << std_name << "'";

    if (calculator_.currentStandardName() != std_name)
    {
        loginf << "setting standard '" << std_name << "'";
        calculator_.currentStandardName(std_name);
    }
}

void EvaluationStandardComboBox::setStandardName(const std::string& value)
{
    loginf << "standard '" << value << "'";

    int index = findText(QString(value.c_str()));
    traced_assert(index >= 0);
    setCurrentIndex(index);
}

void EvaluationStandardComboBox::updateStandards()
{
    loginf;

    clear();

    addItem("");

    for (auto std_it = calculator_.standardsBegin(); std_it != calculator_.standardsEnd(); ++std_it)
    {
        addItem((*std_it)->name().c_str());
    }

    if (calculator_.hasCurrentStandard())
    {
        int index = findText(calculator_.currentStandardName().c_str());

        if (index >= 0)
            setCurrentIndex(index);
        else
            setCurrentIndex(0);
    }
    else
        setCurrentIndex(0);
}
