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

#include "evaluationtargetdata.h"
#include "evaluationtarget.h"
#include "dbcontentaccessor.h"

#include <memory>
#include <map>

#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/random_access_index.hpp>
#include <boost/multi_index_container.hpp>

class EvaluationCalculator;
class DBContent;
class EvaluationManager;
class DBContentManager;
class Buffer;

struct target_tag
{
};

namespace ResultReport
{
    class Report;
    class SectionContentTable;
}

typedef boost::multi_index_container<
    EvaluationTargetData,
    boost::multi_index::indexed_by<
        boost::multi_index::random_access<>,  // this index represents insertion order
        boost::multi_index::ordered_unique<boost::multi_index::tag<target_tag>,
            boost::multi_index::member<
        EvaluationTargetData, const unsigned int, &EvaluationTargetData::utn_> >
        > >
    TargetCache;

/**
 */
class EvaluationData
{
public:
    typedef EvaluationTarget::InterestEnabledFunc InterestEnabledFunc;

    EvaluationData(EvaluationCalculator& calculator,
                   EvaluationManager& eval_man,
                   DBContentManager& dbcont_man);

    void setBuffers(std::map<std::string, std::shared_ptr<Buffer>> buffers);
    void addReferenceData (const std::string& dbcontent_name, unsigned int line_id);
    void addTestData (const std::string& dbcontent_name, unsigned int line_id);
    void finalize ();

    void updateToChanges();

    bool hasTargetData (unsigned int utn);
    const EvaluationTargetData& targetData(unsigned int utn);
    unsigned int size() { return target_data_.size(); }

    typedef TargetCache::index<target_tag>::type TargetCacheIterator;
    TargetCacheIterator::const_iterator begin() { return target_data_.get<target_tag>().begin(); }
    TargetCacheIterator::const_iterator end() { return target_data_.get<target_tag>().end(); }

    void clear();

    std::map<unsigned int, EvaluationTarget> toTargets() const;

    void addToReport(std::shared_ptr<ResultReport::Report> report) const;
    void addConstraintsSection(std::shared_ptr<ResultReport::Report> report) const;
    void addTargetSection(std::shared_ptr<ResultReport::Report> report) const;

    void postprocessTargetsTable(ResultReport::SectionContentTable& table) const;
    void fillTargetsTable(const std::map<unsigned int, EvaluationTarget>& targets,
                          ResultReport::SectionContentTable& table,
                          const InterestEnabledFunc & interest_enabled_func) const;
    bool hasTargetTableTooltip(int col) const;
    std::string targetTableToolTip(const EvaluationTarget& target,
                                   int col,
                                   const InterestEnabledFunc & interest_enabled_func) const;

    enum Columns
    {
        ColUse = 0, 
        ColUTN, 
        ColComment, 
        ColCategory,
        ColInterest, 
        ColNumUpdates,
        ColNumRef,
        ColNumTest,
        ColBegin, 
        ColEnd, 
        ColDuration,
        ColACIDs, 
        ColACADs, 
        ColMode3A//, 
        //ColModeCMin, 
        //ColModeCMax
    };

    // ref
    unsigned int ref_line_id_;

    // tst
    unsigned int tst_line_id_;

    static const std::string TargetsSectionID;
    static const std::string TargetsTableName;
    static const std::string ContentPropertyTargets;

    static const std::string ConstraintsSectionID;
    static const std::string ConstraintsTSTableName;
    static const std::string ConstraintsReqTableName;
    static const std::string ConstraintsPlaceholderTxtName;

protected:
    std::pair<nlohmann::json, unsigned int> rawCellData(const EvaluationTarget& target, 
                                                        int column,
                                                        const InterestEnabledFunc & interest_enabled_func) const;
    unsigned int rowStyle(const EvaluationTarget& target) const;
    unsigned int columnStyle(int column) const;

    EvaluationCalculator& calculator_;
    EvaluationManager&    eval_man_;
    DBContentManager&     dbcont_man_;

    QStringList table_columns_ { "Use", 
                                 "UTN", 
                                 "Comment", 
                                 "Category", 
                                 "Interest",
                                 "#Updates", 
                                 "#Ref", 
                                 "#Tst",
                                 "Begin", 
                                 "End", 
                                 "Duration",
                                 "ACIDs", 
                                 "ACADs",
                                 "M3/A"//, 
                                 //"MC Min", 
                                 //"MC Max" 
                                };
    
    std::vector<int> main_columns_    { ColUse, ColUTN, ColComment, ColCategory, ColInterest };
    std::vector<int> duration_columns_{ ColNumUpdates, ColNumRef, ColNumTest, ColBegin, ColEnd, ColDuration };
    std::vector<int> mode_s_columns_  { ColACIDs, ColACADs };
    std::vector<int> mode_ac_columns_ { ColMode3A, /*ColModeCMin, ColModeCMax*/ };

    std::shared_ptr<dbContent::DBContentAccessor> accessor_;

    TargetCache target_data_;
    bool finalized_ {false};

    unsigned int unassociated_ref_cnt_ {0};
    unsigned int associated_ref_cnt_ {0};

    unsigned int unassociated_tst_cnt_ {0};
    unsigned int associated_tst_cnt_ {0};
};
