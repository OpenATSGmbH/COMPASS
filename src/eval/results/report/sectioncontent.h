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

#pragma once

#include <string>

class EvaluationManager;

class QWidget;
class QVBoxLayout;
class LatexVisitor;

namespace EvaluationResultsReport
{
    using namespace std;

    class Section;

    class SectionContent
    {
    public:
        SectionContent(const string& name, Section* parent_section, EvaluationManager& eval_man);

        string name() const;

        virtual void addToLayout (QVBoxLayout* layout) = 0; // add content to layout

        virtual void accept(LatexVisitor& v) = 0; // can not be const since on-demand tables

    protected:
        string name_;
        Section* parent_section_ {nullptr};
        EvaluationManager& eval_man_;

    };

}
