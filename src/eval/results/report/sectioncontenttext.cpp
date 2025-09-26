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

#include "eval/results/report/sectioncontenttext.h"
#include "evaluationmanager.h"
#include "latexvisitor.h"
#include "logger.h"

#include <QLabel>
#include <QVBoxLayout>

#include "traced_assert.h"

namespace EvaluationResultsReport
{
SectionContentText::SectionContentText(const string& name, Section* parent_section, EvaluationManager& eval_man)
    : SectionContent(name, parent_section, eval_man)
{

}

void SectionContentText::addText (const string& text)
{
    texts_.push_back(text);
}

void SectionContentText::addToLayout (QVBoxLayout* layout)
{
    traced_assert(layout);

    for (auto& text : texts_)
    {
        QLabel* label = new QLabel((text+"\n\n").c_str());
        label->setWordWrap(true);

        layout->addWidget(label);
    }
}

void SectionContentText::accept(LatexVisitor& v)
{
    loginf;
    v.visit(this);
}

const vector<string>& SectionContentText::texts() const
{
    return texts_;
}

}
