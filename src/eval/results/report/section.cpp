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

#include "eval/results/report/section.h"
#include "eval/results/report/section_id.h"
#include "eval/results/report/sectioncontent.h"
#include "eval/results/report/sectioncontenttext.h"
#include "eval/results/report/sectioncontenttable.h"
#include "eval/results/report/sectioncontentfigure.h"
#include "latexvisitor.h"
#include "logger.h"

#include <QVBoxLayout>

namespace EvaluationResultsReport
{
    /**
    */
    Section::Section(const string& heading, 
                     const string& parent_heading, 
                     TreeItem* parent_item,
                     EvaluationManager& eval_man)
    :   TreeItem(heading, parent_item), heading_(heading), parent_heading_(parent_heading), eval_man_(eval_man)
    {
    }

    /**
    */
    TreeItem* Section::child(int row)
    {
        if (row < 0 || row >= sub_sections_.size())
            return nullptr;

        return sub_sections_.at(row).get();
    }

    /**
    */
    int Section::childCount() const
    {
        return sub_sections_.size();
    }

    /**
    */
    int Section::columnCount() const
    {
        return 1;
    }

    /**
    */
    QVariant Section::data(int column) const
    {
        traced_assert(column == 0);

        return heading_.c_str();
    }

    /**
    */
    int Section::row() const
    {
        return 0;
    }

    /**
    */
    string Section::heading() const
    {
        return heading_;
    }

    /**
    */
    string Section::compoundHeading() const
    {
        if (parent_heading_.size())
            return parent_heading_+":"+heading_;
        else
            return heading_;
    }

    /**
    */
    string Section::compoundResultsHeading() const
    {
        string tmp;

        if (parent_heading_.size())
            tmp = SectionID::sectionID(parent_heading_, heading_);
        else
            tmp = heading_;

        return SectionID::sectionIDWithoutResults(tmp);
    }

    /**
    */
    bool Section::hasSubSection (const std::string& heading)
    {
        return findSubSection(heading) != nullptr;
    }

    /**
    */
    Section& Section::getSubSection (const std::string& heading)
    {
        traced_assert(hasSubSection(heading));

        Section* tmp = findSubSection (heading);
        traced_assert(tmp);
        return *tmp;
    }

    /**
    */
    void Section::addSubSection (const std::string& heading)
    {
        logdbg << "heading '" << heading_ << "': adding " << heading;

        traced_assert(!hasSubSection(heading));

        sub_sections_.push_back(make_shared<Section>(heading, compoundHeading(), this, eval_man_));
        traced_assert(hasSubSection(heading));
    }

    /**
    */
    QWidget* Section::getContentWidget()
    {
        if (!content_widget_)
        {
            createContentWidget();
            traced_assert(content_widget_);
        }

        return content_widget_.get();
    }

    /**
    */
    bool Section::hasText (const std::string& name)
    {
        return findText(name) != nullptr;
    }

    /**
    */
    SectionContentText& Section::getText (const std::string& name)
    {
        SectionContentText* tmp = findText (name);
        traced_assert(tmp);
        return *tmp;
    }

    /**
    */
    void Section::addText (const std::string& name)
    {
        traced_assert(!hasText(name));
        content_.push_back(make_shared<SectionContentText>(name, this, eval_man_));
        traced_assert(hasText(name));
    }

    /**
    */
    bool Section::hasTable (const std::string& name)
    {
        return findTable(name) != nullptr;
    }

    /**
    */
    SectionContentTable& Section::getTable (const std::string& name)
    {
        SectionContentTable* tmp = findTable (name);
        traced_assert(tmp);
        return *tmp;
    }

    /**
    */
    std::vector<std::string> Section::getTableNames() const
    {
        std::vector<std::string> names;

        for (auto& cont_it : content_)
        {
            auto table = dynamic_cast<SectionContentTable*>(cont_it.get());

            if (table)
                names.push_back(table->name());
        }

        return names;
    }

    /**
    */
    void Section::addTable (const std::string& name, 
                            unsigned int num_columns,
                            vector<string> headings, 
                            bool sortable, 
                            unsigned int sort_column, 
                            Qt::SortOrder order)
    {
        traced_assert(!hasTable(name));
        content_.push_back(make_shared<SectionContentTable>(name, num_columns, headings, this, eval_man_,
                                                            sortable, sort_column, order));
        traced_assert(hasTable(name));
    }

    /**
    */
    bool Section::hasFigure (const std::string& name)
    {
        return findFigure(name) != nullptr;
    }

    /**
    */
    SectionContentFigure& Section::getFigure (const std::string& name)
    {
        SectionContentFigure* tmp = findFigure (name);
        traced_assert(tmp);
        return *tmp;
    }

    /**
    */
    void Section::addFigure (const std::string& name, const string& caption,
                             std::function<std::shared_ptr<nlohmann::json::object_t>(void)> viewable_fnc,
                             int render_delay_msec)
    {
        traced_assert(!hasFigure(name));
        content_.push_back(make_shared<SectionContentFigure>(name, caption, viewable_fnc, this, eval_man_, render_delay_msec));
        traced_assert(hasFigure(name));
    }

    /**
    */
    std::vector<SectionContentFigure*> Section::getFigures() const
    {
        std::vector<SectionContentFigure*> figures;
        for (auto& cont_it : content_)
        {
            auto f = dynamic_cast<SectionContentFigure*>(cont_it.get());
            if (f)
                figures.push_back(f);
        }
        return figures;
    }

    /**
    */
    unsigned int Section::numSections()
    {
        unsigned int num = 1; // me

        for (auto& sec_it : sub_sections_)
            num += sec_it->numSections();

        return num;
    }

    /**
    */
    void Section::addSectionsFlat (vector<shared_ptr<Section>>& result, 
                                   bool include_target_details,
                                   bool report_skip_targets_wo_issues)
    {
        if (!include_target_details && compoundHeading() == SectionID::targetResultsID())
            return;

        if (report_skip_targets_wo_issues && perTargetSection())
        {
            if (perTargetWithIssues())
            {
                logdbg << "not skipping section " << compoundHeading();
            }
            else
            {
                logdbg << "skipping section " << compoundHeading();
                return;
            }
        }

        for (auto& sec_it : sub_sections_)
        {
            if (!include_target_details && sec_it->compoundHeading() == SectionID::targetResultsID())
                continue;

            if (report_skip_targets_wo_issues && sec_it->perTargetSection())
            {
                if (sec_it->perTargetWithIssues())
                {
                      logdbg << "not skipping sub-section " << compoundHeading();
                }
                else
                {
                    logdbg << "skipping sub-section" << compoundHeading();
                    continue;
                }
            }

            result.push_back(sec_it);
            sec_it->addSectionsFlat(result, include_target_details, report_skip_targets_wo_issues);
        }
    }

    /**
    */
    void Section::accept(LatexVisitor& v) const
    {
        loginf;
        v.visit(this);
    }

    /**
    */
    const vector<shared_ptr<SectionContent>>& Section::content() const
    {
        return content_;
    }

    /**
    */
    bool Section::perTargetWithIssues() const
    {
        return per_target_section_with_issues_;
    }

    /**
    */
    void Section::perTargetWithIssues(bool value)
    {
        per_target_section_with_issues_ = value;
    }

    /**
    */
    bool Section::perTargetSection() const
    {
        return per_target_section_;
    }

    /**
    */
    void Section::perTargetSection(bool value)
    {
        per_target_section_ = value;
    }

    /**
    */
    Section* Section::findSubSection (const std::string& heading)
    {
        for (auto& sec_it : sub_sections_)
        {
            if (sec_it->heading() == heading)
                return sec_it.get();
        }

        return nullptr;
    }

    /**
    */
    SectionContentText* Section::findText (const std::string& name)
    {
        SectionContentText* tmp;

        for (auto& cont_it : content_)
        {
            tmp = dynamic_cast<SectionContentText*>(cont_it.get());

            if (tmp && tmp->name() == name)
                return tmp;
        }

        return nullptr;
    }

    /**
    */
    SectionContentTable* Section::findTable (const std::string& name)
    {
        SectionContentTable* tmp;

        for (auto& cont_it : content_)
        {
            tmp = dynamic_cast<SectionContentTable*>(cont_it.get());

            if (tmp && tmp->name() == name)
                return tmp;
        }

        return nullptr;
    }

    /**
    */
    SectionContentFigure* Section::findFigure (const std::string& name)
    {
        SectionContentFigure* tmp;

        for (auto& cont_it : content_)
        {
            tmp = dynamic_cast<SectionContentFigure*>(cont_it.get());

            if (tmp && tmp->name() == name)
                return tmp;
        }

        return nullptr;
    }

    /**
    */
    void Section::createContentWidget()
    {
        traced_assert(!content_widget_);

        content_widget_.reset(new QWidget());

        QVBoxLayout* layout = new QVBoxLayout();

        for (auto& cont_it : content_)
            cont_it->addToLayout(layout);

        //layout->addStretch();

        content_widget_->setLayout(layout);
    }
}
