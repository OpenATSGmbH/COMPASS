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

#include "db_context_merge_dialog.h"
#include "db_context_diff.h"
#include "logger.h"

#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>

namespace context
{

DBContextMergeDialog::DBContextMergeDialog(const DBContext& file_context,
                                           const DBContext& db_context,
                                           const DBContextDiff& /*diff*/,
                                           QWidget* parent)
    : QDialog(parent)
    , merged_(file_context) // default: start from file context
{
    setWindowTitle("Merge Context");
    setMinimumSize(600, 400);

    auto* layout = new QVBoxLayout(this);

    auto* label = new QLabel("Merge dialog is not yet implemented.\n\n"
                             "The file definition will be used.");
    label->setWordWrap(true);
    layout->addWidget(label);

    layout->addStretch();

    auto* ok_button = new QPushButton("OK");
    ok_button->setIcon(QIcon());
    ok_button->setToolTip("Accept and close");
    connect(ok_button, &QPushButton::clicked, this, &QDialog::accept);

    auto* button_layout = new QHBoxLayout();
    button_layout->addStretch();
    button_layout->addWidget(ok_button);
    layout->addLayout(button_layout);
}

} // namespace context
