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

#include "eval/requirement/base/baseconfigwidget.h"
#include "eval/requirement/base/baseconfig.h"
#include "eval/requirement/group.h"

#include "logger.h"

#include <QLabel>
#include <QLineEdit>
#include <QFormLayout>
#include <QCheckBox>
#include <QPlainTextEdit>

using namespace std;

namespace EvaluationRequirement
{

BaseConfigWidget::BaseConfigWidget(BaseConfig& cfg)
    : QWidget(), config_(cfg)
{
    QVBoxLayout* main_layout = new QVBoxLayout();

    form_layout_ = new QFormLayout();

    traced_assert(Group::requirement_type_mapping_.count(config_.classId()));
    form_layout_->addRow("Requirement Type",
                         new QLabel(Group::requirement_type_mapping_.at(config_.classId()).c_str()));

    QLineEdit* name_edit = new QLineEdit (config_.name().c_str());
    connect(name_edit, &QLineEdit::editingFinished, this, &BaseConfigWidget::changedNameSlot);

    form_layout_->addRow("Name", name_edit);

    QLineEdit* short_name_edit = new QLineEdit (config_.shortName().c_str());
    connect(short_name_edit, &QLineEdit::editingFinished, this, &BaseConfigWidget::changedShortNameSlot);

    form_layout_->addRow("Short Name", short_name_edit);

    // comment
    QPlainTextEdit* comment_edit = new QPlainTextEdit (config_.comment().c_str());
    QFontMetrics fm (comment_edit->document()->defaultFont());
    QMargins margins = comment_edit->contentsMargins ();
    int height = fm.lineSpacing () * 5 +
            (comment_edit->document()->documentMargin() + comment_edit->frameWidth())*2 +
            margins.top() + margins.bottom();
    comment_edit->setFixedHeight (height);
    connect(comment_edit, &QPlainTextEdit::textChanged, this, &BaseConfigWidget::changedCommentSlot);

    form_layout_->addRow("Comment", comment_edit);

    main_layout->addLayout(form_layout_);

    main_layout->addStretch();

    setLayout(main_layout);
}

BaseConfigWidget::~BaseConfigWidget()
{

}

void BaseConfigWidget::changedNameSlot()
{
    QLineEdit* edit = dynamic_cast<QLineEdit*>(sender());
    traced_assert(edit);

    string value_str = edit->text().toStdString();

    loginf << "name '" << value_str << "'";

    if (value_str.size())
    {
        config_.name(value_str);
    }
    else
        logerr << "impossible name '" << value_str << "'";
}

void BaseConfigWidget::changedShortNameSlot()
{
    QLineEdit* edit = dynamic_cast<QLineEdit*>(sender());
    traced_assert(edit);

    string value_str = edit->text().toStdString();

    loginf << "name '" << value_str << "'";

    if (value_str.size())
        config_.shortName(value_str);
    else
        logerr << "impossible name '" << value_str << "'";
}

void BaseConfigWidget::changedCommentSlot()
{
    QPlainTextEdit* edit = dynamic_cast<QPlainTextEdit*>(sender());
    traced_assert(edit);

    string value_str = edit->toPlainText().toStdString();

    logdbg << "comment '" << value_str << "'";

    config_.comment(value_str);
}

}
