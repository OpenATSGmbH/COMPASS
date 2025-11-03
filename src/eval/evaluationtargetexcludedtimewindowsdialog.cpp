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

#include "evaluationtargetexcludedtimewindowsdialog.h"
#include "traced_assert.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QFormLayout>
#include <QCheckBox>
#include <QLineEdit>
#include <QDateTime>
#include <QDateTimeEdit>
#include <QPlainTextEdit>
#include <QPushButton>

EvaluationTargetExcludedTimeWindowsDialog::EvaluationTargetExcludedTimeWindowsDialog(
    const std::string utn_str,Utils::TimeWindowCollection& collection, std::string comment, QWidget* parent)
    : QDialog(parent), collection_(collection)
{
    setWindowTitle("Edit Evaluation Excluded Time Windows");

    setModal(true);

    setMinimumSize(QSize(600, 400));

    QVBoxLayout* main_layout = new QVBoxLayout();

    QFormLayout* form_layout = new QFormLayout;
    form_layout->setFormAlignment(Qt::AlignLeft | Qt::AlignTop);

    QPlainTextEdit* utn_label = new QPlainTextEdit(parent);
    utn_label->setReadOnly(true);
    utn_label->setPlainText(utn_str.c_str());

    // Optional: limit height to show only a few lines
    int line_count_to_show = 3;
    QFontMetrics metrics(utn_label->font());
    utn_label->setFixedHeight(metrics.lineSpacing() * line_count_to_show + 4); // +4 for padding

    form_layout->addRow("UTNs", utn_label);

    tw_widget_ = new TimeWindowCollectionWidget(collection_);
    form_layout->addRow("Excluded Time Windows", tw_widget_);

    main_layout->addLayout(form_layout);

    main_layout->addWidget(new QLabel("Comment"));

    comment_edit_ = new QLineEdit(comment.c_str());
    comment_edit_->setMinimumHeight(QFontMetrics(comment_edit_->font()).height() * 3);

    main_layout->addWidget(comment_edit_);

    main_layout->addStretch();

    // buttons

    QHBoxLayout* button_layout = new QHBoxLayout();

    QPushButton* cancel_button = new QPushButton("Cancel");
    connect(cancel_button, &QPushButton::clicked, this, &QDialog::reject);
    button_layout->addWidget(cancel_button);

    button_layout->addStretch();

    QPushButton* run_button = new QPushButton("OK");
    connect(run_button, &QPushButton::clicked, this, &QDialog::accept);
    button_layout->addWidget(run_button);

    main_layout->addLayout(button_layout);

    setLayout(main_layout);
}

std::string EvaluationTargetExcludedTimeWindowsDialog::comment() const
{
    traced_assert(comment_edit_);
    return comment_edit_->text().toStdString();
}
