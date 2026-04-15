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

#include "db_context_conflict_dialog.h"
#include "db_context_diff.h"
#include "logger.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>

namespace context
{

DBContextConflictDialog::DBContextConflictDialog(const std::string& context_name,
                                                 const DBContextDiff& diff,
                                                 QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle("Context Conflict");
    setWindowFlags(windowFlags() & ~Qt::WindowCloseButtonHint);
    setMinimumWidth(450);

    auto* main_layout = new QVBoxLayout(this);

    // description
    auto* desc_label = new QLabel(
        "The context '" + QString::fromStdString(context_name) +
        "' differs between Configuration and Database.\n\n"
        "Choose which version to use:");
    desc_label->setWordWrap(true);
    main_layout->addWidget(desc_label);

    // diff summary
    auto* summary_label = new QLabel(QString::fromStdString(diff.summary()));
    summary_label->setWordWrap(true);
    summary_label->setStyleSheet("QLabel { background-color: palette(alternate-base); padding: 6px; }");
    main_layout->addWidget(summary_label);

    main_layout->addSpacing(10);

    // buttons
    auto* button_layout = new QHBoxLayout();
    button_layout->addStretch();

    auto* use_config_button = new QPushButton("Use Configuration");
    use_config_button->setIcon(QIcon());
    use_config_button->setToolTip("Use the Configuration and overwrite the Database");
    connect(use_config_button, &QPushButton::clicked, this, &DBContextConflictDialog::useFileSlot);
    button_layout->addWidget(use_config_button);

    auto* use_db_button = new QPushButton("Use Database");
    use_db_button->setIcon(QIcon());
    use_db_button->setToolTip("Use the Database and overwrite the Configuration");
    connect(use_db_button, &QPushButton::clicked, this, &DBContextConflictDialog::useDatabaseSlot);
    button_layout->addWidget(use_db_button);

    auto* merge_button = new QPushButton("Merge...");
    merge_button->setIcon(QIcon());
    merge_button->setToolTip("Open a merge dialog to resolve conflicts individually");
    merge_button->setToolTip("Open a merge dialog to resolve conflicts individually");
    connect(merge_button, &QPushButton::clicked, this, &DBContextConflictDialog::mergeSlot);
    button_layout->addWidget(merge_button);

    main_layout->addLayout(button_layout);
}

void DBContextConflictDialog::useFileSlot()
{
    loginf << "user chose: use configuration";
    resolution_ = UseFile;
    accept();
}

void DBContextConflictDialog::useDatabaseSlot()
{
    loginf << "user chose: use database definition";
    resolution_ = UseDatabase;
    accept();
}

void DBContextConflictDialog::mergeSlot()
{
    loginf << "user chose: merge (not yet implemented)";
    resolution_ = Merge;
    accept();
}

} // namespace context
