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

#include "evaluationtargetexcludedrequirementsdialog.h"
#include "stringconv.h"
#include "logger.h"
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
#include <QScrollArea>
#include <QGridLayout>
#include <QCheckBox>

using namespace Utils;

EvaluationTargetExcludedRequirementsDialog::EvaluationTargetExcludedRequirementsDialog(
    const std::string utn_str, std::set<std::string> selected_requirements,
    std::set<std::string> available_requirements, std::string comment, QWidget* parent)
    : QDialog(parent)
    , selected_requirements_(std::move(selected_requirements))
    , available_requirements_(std::move(available_requirements))
{
    logdbg << "selected_requirements '"
           << String::compress(selected_requirements_,',') << "'";

    logdbg << "available_requirements '"
           << String::compress(available_requirements_,',') << "'";

    setWindowTitle("Edit Evaluation Excluded Requirements");

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

    main_layout->addLayout(form_layout);

    // add gui items here

    // Scroll area container for checkboxes
    scroll_area_ = new QScrollArea(this);
    scroll_widget_ = new QWidget;
    grid_layout_ = new QGridLayout(scroll_widget_);

    const int columns = 2;
    int index = 0;
    for (const auto& requirement : available_requirements_)
    {
        auto* checkbox = new QCheckBox(QString::fromStdString(requirement), scroll_widget_);
        checkbox->setChecked(selected_requirements_.count(requirement) > 0);

        connect(checkbox, &QCheckBox::stateChanged, this, [this, requirement](int state) {
            if (state == Qt::Checked)
                selected_requirements_.insert(requirement);
            else
                selected_requirements_.erase(requirement);
        });

        int row = index / columns;
        int col = index % columns;
        grid_layout_->addWidget(checkbox, row, col);
        requirement_checkboxes_.push_back(checkbox);
        ++index;
    }

    scroll_widget_->setLayout(grid_layout_);
    scroll_area_->setWidgetResizable(true);
    scroll_area_->setWidget(scroll_widget_);
    main_layout->addWidget(scroll_area_);

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

std::set<std::string> EvaluationTargetExcludedRequirementsDialog::selectedRequirements() const
{
    return selected_requirements_;
}

std::string EvaluationTargetExcludedRequirementsDialog::comment() const
{
    traced_assert(comment_edit_);
    return comment_edit_->text().toStdString();
}
