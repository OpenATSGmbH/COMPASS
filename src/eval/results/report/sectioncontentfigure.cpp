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

#include "eval/results/report/sectioncontentfigure.h"
#include "eval/results/report/section.h"
#include "eval/results/report/section_id.h"

#include "evaluationmanager.h"
#include "latexvisitor.h"
#include "logger.h"
#include "stringconv.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QApplication>
#include <QThread>

using namespace Utils;

namespace EvaluationResultsReport
{

SectionContentFigure::SectionContentFigure(
    const string& name, const string& caption,
    std::function<std::shared_ptr<nlohmann::json::object_t>(void)> viewable_fnc,
    Section* parent_section, EvaluationManager& eval_man, int render_delay_msec)
    : SectionContent(name, parent_section, eval_man), caption_(caption),
      render_delay_msec_(render_delay_msec), viewable_fnc_(viewable_fnc)
{
   //assert (viewable_data_);
}

void SectionContentFigure::addToLayout (QVBoxLayout* layout)
{
    traced_assert(layout);

    QHBoxLayout* fig_layout = new QHBoxLayout();

    fig_layout->addWidget(new QLabel(("Figure: "+caption_).c_str()));

    fig_layout->addStretch();

    QPushButton* view_button = new QPushButton("View");
    connect (view_button, &QPushButton::clicked, this, &SectionContentFigure::viewSlot);
    fig_layout->addWidget(view_button);

    layout->addLayout(fig_layout);
}

void SectionContentFigure::accept(LatexVisitor& v)
{
    loginf;
    v.visit(this);
}

void SectionContentFigure::viewSlot()
{
    loginf;
    view();
}

void SectionContentFigure::view() const
{
    eval_man_.setViewableDataConfig(*viewable_fnc_());

    if (render_delay_msec_ > 0)
    {
        boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();
        while ((boost::posix_time::microsec_clock::local_time() - start_time).total_milliseconds()
               < render_delay_msec_)
        {
            QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
            QThread::msleep(10);
        }
    }
}

std::string SectionContentFigure::getSubPath() const
{
    traced_assert(parent_section_);

    return EvaluationResultsReport::SectionID::sectionID2Path(parent_section_->compoundResultsHeading());
}
}
