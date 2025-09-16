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

#include "evaluationdata.h"
#include "evaluationcalculator.h"
#include "evaluationstandard.h"
#include "evaluationtarget.h"
#include "requirement/base/baseconfig.h"
#include "requirement/group.h"
#include "evaluationmanager.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/dbcontent.h"
#include "buffer.h"

#include "task/result/report/report.h"
#include "task/result/report/section.h"
#include "task/result/report/sectioncontenttable.h"

#include "util/async.h"
//#include "util/stringmat.h"

#include <QApplication>
#include <QThread>
#include <QProgressDialog>
#include <QLabel>

#include "util/tbbhack.h"

//#include "boost/date_time/posix_time/posix_time.hpp"

//#include <sstream>
//#include <future>

using namespace std;
using namespace Utils;
using namespace nlohmann;
using namespace boost::posix_time;

const std::string EvaluationData::SectionID              = "Overview:Targets";
const std::string EvaluationData::TargetsTableName       = "Evaluated Targets";
const std::string EvaluationData::ContentPropertyTargets = "targets";

/**
 */
EvaluationData::EvaluationData(EvaluationCalculator& calculator,
                               EvaluationManager& eval_man,
                               DBContentManager& dbcont_man)
:   calculator_(calculator)
,   eval_man_  (eval_man)
,   dbcont_man_(dbcont_man)
{
    accessor_ = make_shared<dbContent::DBContentAccessor>();
}

/**
 */
void EvaluationData::setBuffers(std::map<std::string, std::shared_ptr<Buffer>> buffers)
{
    loginf;

    accessor_->clear();
    accessor_->add(buffers);
}

/**
 */
void EvaluationData::addReferenceData (const std::string& dbcontent_name, unsigned int line_id)
{
    loginf << "dbcontent " << dbcontent_name;

    if (!dbcont_man_.hasAssociations())
    {
        logwrn << "dbcontent has no associations";

        return;
    }

    ref_line_id_ = line_id;
    traced_assert(ref_line_id_ <= 3);

    set<unsigned int> active_srcs = calculator_.activeDataSourcesRef();
    bool use_active_srcs = (calculator_.dbContentNameRef() == calculator_.dbContentNameTst());
    
    double min_std_dev_xy = calculator_.currentStandard().referenceMinAccuracy();

    unsigned int num_skipped {0};

    traced_assert(accessor_->hasMetaVar<ptime>(dbcontent_name, DBContent::meta_var_timestamp_));
    traced_assert(accessor_->hasMetaVar<unsigned int>(dbcontent_name, DBContent::meta_var_ds_id_));
    traced_assert(accessor_->hasMetaVar<unsigned int>(dbcontent_name, DBContent::meta_var_line_id_));
    traced_assert(accessor_->hasMetaVar<unsigned int>(dbcontent_name, DBContent::meta_var_utn_));
    traced_assert(accessor_->hasMetaVar<double>(dbcontent_name, DBContent::meta_var_max_stddev_xy));

    NullableVector<ptime>& ts_vec =
        accessor_->getMetaVar<ptime>(dbcontent_name, DBContent::meta_var_timestamp_);
    NullableVector<unsigned int>& ds_ids =
        accessor_->getMetaVar<unsigned int>(dbcontent_name, DBContent::meta_var_ds_id_);
    NullableVector<unsigned int>& line_ids =
        accessor_->getMetaVar<unsigned int>(dbcontent_name, DBContent::meta_var_line_id_);
    NullableVector<unsigned int>& utn_vec =
        accessor_->getMetaVar<unsigned int>(dbcontent_name, DBContent::meta_var_utn_);

    NullableVector<double>& max_stddev_xy_vec =
        accessor_->getMetaVar<double>(dbcontent_name, DBContent::meta_var_max_stddev_xy);

    unsigned int buffer_size = ts_vec.contentSize();

    ptime timestamp;
    // vector<unsigned int> utn_vec;

    unsigned int utn;

    loginf << "adding target data";
    loginf << "use_active_srcs " << use_active_srcs
           << " active_srcs " << String::compress(active_srcs, ',');

    for (unsigned int cnt = 0; cnt < buffer_size; ++cnt)
    {
        traced_assert(!ds_ids.isNull(cnt));

        if (use_active_srcs &&
            !active_srcs.count(ds_ids.get(cnt)))  // skip those entries not for tst src
        {
            ++num_skipped;
            continue;
        }

        traced_assert(!line_ids.isNull(cnt));

        if (line_ids.get(cnt) != ref_line_id_)
        {
            ++num_skipped;
            continue;
        }

        if (ts_vec.isNull(cnt))
        {
            ++num_skipped;
            continue;
        }

        timestamp = ts_vec.get(cnt);

        if (utn_vec.isNull(cnt))
        {
            ++unassociated_ref_cnt_;
            continue;
        }

        utn = utn_vec.get(cnt);
        if (!dbcont_man_.existsTarget(utn))
        {
            logerr << "ignoring unknown utn " << utn;
            continue;
        }

        if (!max_stddev_xy_vec.isNull(cnt) && max_stddev_xy_vec.get(cnt) > min_std_dev_xy)
        {
            ++num_skipped;
            continue;
        }

        if (!hasTargetData(utn))
            target_data_.emplace_back(utn, *this, accessor_, calculator_, eval_man_, dbcont_man_);

        traced_assert(hasTargetData(utn));

        auto tr_tag_it = target_data_.get<target_tag>().find(utn);
        auto index_it = target_data_.project<0>(tr_tag_it);  // get iterator for random access

        target_data_.modify(
            index_it, [timestamp, cnt](EvaluationTargetData& t) { t.addRefIndex(timestamp, cnt); });

        ++associated_ref_cnt_;
    }

    loginf << "num targets " << target_data_.size()
           << " ref associated cnt " << associated_ref_cnt_ << " unassoc " << unassociated_ref_cnt_
           << " num_skipped " << num_skipped;
}

/**
 */
void EvaluationData::addTestData (const std::string& dbcontent_name, unsigned int line_id)
{
    loginf << "dbcontent " << dbcontent_name;

    if (!dbcont_man_.hasAssociations())
    {
        logwrn << "dbcontent has no associations";
        return;
    }

    tst_line_id_ = line_id;
    traced_assert(tst_line_id_ <= 3);

    set<unsigned int> active_srcs = calculator_.activeDataSourcesTst();
    bool use_active_srcs = (calculator_.dbContentNameRef() == calculator_.dbContentNameTst());
    unsigned int num_skipped {0};

    if (accessor_->hasMetaVar<ptime>(dbcontent_name, DBContent::meta_var_timestamp_) &&
        accessor_->hasMetaVar<unsigned int>(dbcontent_name, DBContent::meta_var_ds_id_) &&
        accessor_->hasMetaVar<unsigned int>(dbcontent_name, DBContent::meta_var_line_id_) &&
        accessor_->hasMetaVar<unsigned int>(dbcontent_name, DBContent::meta_var_utn_))
    {
        NullableVector<ptime>& ts_vec = accessor_->getMetaVar<ptime>(
                    dbcontent_name, DBContent::meta_var_timestamp_);
        NullableVector<unsigned int>& ds_ids = accessor_->getMetaVar<unsigned int>(
                    dbcontent_name, DBContent::meta_var_ds_id_);
        NullableVector<unsigned int>& line_ids = accessor_->getMetaVar<unsigned int>(
                    dbcontent_name, DBContent::meta_var_line_id_);
        NullableVector<unsigned int>& utn_vec = accessor_->getMetaVar<unsigned int>(
                    dbcontent_name, DBContent::meta_var_utn_);

        unsigned int buffer_size = ts_vec.contentSize();

        boost::posix_time::ptime timestamp;
        //vector<unsigned int> utn_vec;

        unsigned int utn;

        loginf << "adding target data";

        loginf << "use_active_srcs " << use_active_srcs << " active_srcs "
               << String::compress(active_srcs, ',');

        for (unsigned int cnt=0; cnt < buffer_size; ++cnt)
        {
            traced_assert(!ds_ids.isNull(cnt));

            if (use_active_srcs && !active_srcs.count(ds_ids.get(cnt))) // skip those entries not for tst src
            {
                ++num_skipped;
                continue;
            }

            traced_assert(!line_ids.isNull(cnt));

            if (line_ids.get(cnt) != tst_line_id_)
            {
                ++num_skipped;
                continue;
            }

            if (ts_vec.isNull(cnt))
            {
                ++num_skipped;
                continue;
            }

            timestamp = ts_vec.get(cnt);

            if (utn_vec.isNull(cnt))
            {
                ++unassociated_tst_cnt_;
                continue;
            }

            utn = utn_vec.get(cnt);
            if (!dbcont_man_.existsTarget(utn))
            {
                logerr << "ignoring unknown utn " << utn;
                continue;
            }

            if (!hasTargetData(utn))
                target_data_.emplace_back(utn, *this, accessor_, calculator_, eval_man_, dbcont_man_);

            traced_assert(hasTargetData(utn));

            auto tr_tag_it = target_data_.get<target_tag>().find(utn);
            auto index_it = target_data_.project<0>(tr_tag_it); // get iterator for random access

            target_data_.modify(index_it, [timestamp, cnt](EvaluationTargetData& t) { t.addTstIndex(timestamp, cnt); });

            ++associated_tst_cnt_;
        }
    }

    loginf << "num targets " << target_data_.size()
           << " tst associated cnt " << associated_tst_cnt_ << " unassoc " << unassociated_tst_cnt_
           << " num_skipped " << num_skipped;
}

/**
 */
void EvaluationData::finalize ()
{
    loginf;

    traced_assert(!finalized_);

    unsigned int num_targets = target_data_.size();

    QApplication::setOverrideCursor(Qt::WaitCursor);
    {
        calculator_.updateSectorLayers();
    }
    QApplication::restoreOverrideCursor();

    if (!num_targets)
    {
        logerr << "no targets loaded";
    }
    else
    {
        auto task = [&] (int cnt) { target_data_[cnt].finalize(); return true; };

        Utils::Async::waitDialogAsyncArray(task, (int) num_targets, "Finalizing data");
    }

    finalized_ = true;
}

/**
 */
void EvaluationData::updateToChanges()
{
    for (auto& target : target_data_)
        target.updateToChanges();
}

/**
 */
bool EvaluationData::hasTargetData (unsigned int utn)
{
    return target_data_.get<target_tag>().find(utn) != target_data_.get<target_tag>().end();
}

/**
 */
const EvaluationTargetData& EvaluationData::targetData(unsigned int utn)
{
    traced_assert(hasTargetData(utn));

    return *target_data_.get<target_tag>().find(utn);
}

/**
 */
void EvaluationData::clear()
{
    accessor_->clear();

    target_data_.clear();
    finalized_ = false;

    unassociated_ref_cnt_ = 0;
    associated_ref_cnt_ = 0;

    unassociated_tst_cnt_ = 0;
    associated_tst_cnt_ = 0;
}

// /**
//  */
// QVariant EvaluationData::data(const QModelIndex& index, int role) const
// {
//     if (!index.isValid() || !finalized_)
//         return QVariant();

//     switch (role)
//     {
//     case Qt::CheckStateRole:
//     {
//         if (index.column() == 0)  // selected special case
//         {
//             traced_assert(index.row() >= 0);
//             traced_assert(index.row() < (int)target_data_.size());

//             const EvaluationTargetData& target = target_data_.at(index.row());

//             if (dbcont_man_.utnUseEval(target.utn_))
//                 return Qt::Checked;
//             else
//                 return Qt::Unchecked;
//         }
//         else
//             return QVariant();
//     }
//     case Qt::BackgroundRole:
//     {
//         traced_assert(index.row() >= 0);
//         traced_assert(index.row() < (int)target_data_.size());

//         const EvaluationTargetData& target = target_data_.at(index.row());

//         traced_assert(index.column() < table_columns_.size());
//         std::string col_name = table_columns_.at(index.column()).toStdString();

//         if (!dbcont_man_.utnUseEval(target.utn_))
//             return QBrush(Qt::lightGray);
//         else if (col_name == "Interest")
//         {
//             const auto& ifactors = target.interestFactors();

//             double interest = target.enabledInterestFactorsSum();

//             return ifactors.empty() ? QVariant() : EvaluationTargetData::colorForInterestFactorSum(interest);
//         }
//         else
//             return QVariant();

//     }
//     case Qt::DisplayRole:
//     case Qt::EditRole:
//     {
//         logdbg << "display role: row " << index.row() << " col " << index.column();

//         traced_assert(index.row() >= 0);
//         traced_assert(index.row() < (int)target_data_.size());

//         const EvaluationTargetData& target = target_data_.at(index.row());

//         logdbg << "got utn " << target.utn_;

//         traced_assert(index.column() < table_columns_.size());
//         std::string col_name = table_columns_.at(index.column()).toStdString();

//         if (col_name == "Use")
//         {
//             return QVariant();
//         }
//         else if (col_name == "UTN")
//         {
//             return target.utn_;
//         }
//         else if (col_name == "Comment")
//         {
//             return dbcont_man_.utnComment(target.utn_).c_str();
//         }
//         else if (col_name == "Interest")
//         {
//             const auto& ifactors = target.interestFactors();

//             return ifactors.empty() ? "" : QString::number(target.enabledInterestFactorsSum(), 'f', 3);
//         }
//         else if (col_name == "Begin")
//         {
//             return target.timeBeginStr().c_str();
//         }
//         else if (col_name == "End")
//         {
//             return target.timeEndStr().c_str();
//         }
//         else if (col_name == "#All")
//         {
//             return target.numUpdates();
//         }
//         else if (col_name == "#Ref")
//         {
//             return target.numRefUpdates();
//         }
//         else if (col_name == "#Tst")
//         {
//             return target.numTstUpdates();
//         }
//         else if (col_name == "ACIDs")
//         {
//             return target.acidsStr().c_str();
//         }
//         else if (col_name == "ACADs")
//         {
//             return target.acadsStr().c_str();
//         }
//         else if (col_name == "M3/A")
//         {
//             return target.modeACodesStr().c_str();
//         }
//         else if (col_name == "MC Min")
//         {
//             return target.modeCMinStr().c_str();
//         }
//         else if (col_name == "MC Max")
//         {
//             return target.modeCMaxStr().c_str();
//         }
//     }
//     case Qt::UserRole: // to find the checkboxes
//     {
//         if (index.column() == 0)
//         {
//             traced_assert(index.row() >= 0);
//             traced_assert(index.row() < (int)target_data_.size());

//             const EvaluationTargetData& target = target_data_.at(index.row());

//             return target.utn_;
//         }
//         else if (index.column() == 2) // comment
//         {
//             traced_assert(index.row() >= 0);
//             traced_assert(index.row() < (int)target_data_.size());

//             const EvaluationTargetData& target = target_data_.at(index.row());
//             return ("comment_"+to_string(target.utn_)).c_str();
//         }
//     }
//     case Qt::ToolTipRole:
//     {
//         logdbg << "tooltip role: row " << index.row() << " col " << index.column();

//         traced_assert(index.row() >= 0);
//         traced_assert(index.row() < (int)target_data_.size());

//         const EvaluationTargetData& target = target_data_.at(index.row());

//         logdbg << "got utn " << target.utn_;

//         traced_assert(index.column() < table_columns_.size());
//         std::string col_name = table_columns_.at(index.column()).toStdString();

//         if (col_name == "Interest")
//         {
//             return target.enabledInterestFactorsStr().c_str();
//         }
//     }
//     default:
//     {
//         return QVariant();
//     }
//     }
// }

// bool EvaluationData::setData(const QModelIndex &index, const QVariant& value, int role)
// {
//     if (!index.isValid() /*|| role != Qt::EditRole*/)
//         return false;

//     if (role == Qt::CheckStateRole && index.column() == 0)
//     {
//         traced_assert(index.row() >= 0);
//         traced_assert(index.row() < (int)target_data_.size());

//         auto it = target_data_.begin()+index.row();

//         bool checked = (Qt::CheckState)value.toInt() == Qt::Checked;
//         loginf << "utn " << it->utn_ <<" check state " << checked;

//         dbcont_man_.utnUseEval(it->utn_, checked);

//         emit dataChanged(index, EvaluationData::index(index.row(), columnCount()-1));
//         return true;
//     }
//     else if (role == Qt::EditRole && index.column() == 2) // comment
//     {
//         traced_assert(index.row() >= 0);
//         traced_assert(index.row() < (int)target_data_.size());

//         auto it = target_data_.begin()+index.row();

//         dbcont_man_.utnComment(it->utn_, value.toString().toStdString());
//         return true;
//     }

//     return false;
// }

/**
 */
std::map<unsigned int, EvaluationTarget> EvaluationData::toTargets() const
{
    std::map<unsigned int, EvaluationTarget> targets;
    for (const auto& target : target_data_)
        targets.emplace(target.utn_, target.toTarget());

    return targets;
}

/**
 */
void EvaluationData::addToReport(std::shared_ptr<ResultReport::Report> report) const
{
    //add target section
    auto& section = report->getSection(SectionID);

    //add target table
    std::vector<std::string> headings;
    for (const auto& h : table_columns_)
        headings.push_back(h.toStdString());

    auto& table = section.addTable(TargetsTableName, headings.size(), headings);
    table.setOnDemand();      // on demand content
    table.setLockStateSafe(); // can be reloaded and exported in lock state
    table.enableTooltips();   // shows custom tooltips
    table.setMaxRowCount(-1); // override row count

    //setup column groups
    table.setColumnGroup("Duration", duration_columns_, false);
    table.setColumnGroup("Mode S"  , mode_s_columns_  , true );
    table.setColumnGroup("Mode A/C", mode_ac_columns_ , false);
}

/**
 */
void EvaluationData::postprocessTargetsTable(ResultReport::SectionContentTable& table) const
{
}

namespace
{
    /**
     */
    std::pair<nlohmann::json, unsigned int> interestData(const EvaluationTarget& target,
                                                         const EvaluationData::InterestEnabledFunc & interest_enabled_func)
    {
        if (!target.useInEval())
            return std::make_pair("-", 0);

        size_t num_contributors;
        auto interest = target.totalInterest(interest_enabled_func, &num_contributors);

        if (num_contributors == 0)
            return std::make_pair("-", 0);

        return std::make_pair(QString::number(interest, 'f', 3).toStdString(),
                              EvaluationTargetData::bgStyleForInterestFactorSum(interest));
    }
}

/**
 */
std::pair<nlohmann::json, unsigned int> EvaluationData::rawCellData(const EvaluationTarget& target, 
                                                                    int column,
                                                                    const InterestEnabledFunc & interest_enabled_func) const
{
    switch (column)
    {
        case ColUse:
            return std::make_pair(dbContent::TargetModel::iconForTarget(target, true), 0);
        case ColUTN: 
            return std::make_pair(target.utn_, 0);
        case ColComment:
            return std::make_pair(target.comment(), 0);
        case ColCategory:
            return std::make_pair(target.emitterCategoryStr(), 0);
        case ColInterest: 
            return interestData(target, interest_enabled_func);
        case ColNumUpdates:
            return std::make_pair(target.numUpdates(), 0);
        case ColNumRef:
            return std::make_pair(target.refCount(), 0);
        case ColNumTest:
            return std::make_pair(target.testCount(), 0);
        case ColBegin:
            return std::make_pair(target.timeBeginStr(), 0);
        case ColEnd:
            return std::make_pair(target.timeEndStr(), 0);
        case ColDuration:
            return std::make_pair(target.timeDurationStr(), 0);
        case ColACIDs:
            return std::make_pair(target.aircraftIdentificationsStr(), 0);
        case ColACADs: 
            return std::make_pair(target.aircraftAddressesStr(), 0);
        case ColMode3A: 
            return std::make_pair(target.modeACodesStr(), 0);
        //case ColModeCMin: 
        //    return std::make_pair(target.hasModeC() ? target.modeCMinStr() : "", 0);
        //case ColModeCMax:
        //    return std::make_pair(target.hasModeC() ? target.modeCMaxStr() : "", 0);
    }
    return nlohmann::json();
}

/**
 */
unsigned int EvaluationData::rowStyle(const EvaluationTarget& target) const
{
    if (!target.useInEval())
        return ResultReport::CellStyleBGColorGray;

    return 0;
}

/**
 */
unsigned int EvaluationData::columnStyle(int column) const
{
    if (column == ColUse)
        return ResultReport::CellStyleIcon;

    return 0;
}

/**
 */
void EvaluationData::fillTargetsTable(const std::map<unsigned int, EvaluationTarget>& targets,
                                      ResultReport::SectionContentTable& table,
                                      const InterestEnabledFunc & interest_enabled_func) const
{
    const int nc = (int)table.numColumns();

    for (int c = 0; c < nc; ++c)
        table.setColumnStyle(c, columnStyle(c));

    int r = 0;
    for (const auto& t : targets)
    {
        auto row = nlohmann::json::array();
        for (int c = 0; c < nc; ++c)
        {
            auto data = rawCellData(t.second, c, interest_enabled_func);

            row.push_back(data.first);

            if (data.second != 0)
                table.setCellStyle(r, c, data.second);
        }

        table.addRow(row, ResultReport::SectionContentViewable().setOnDemand(), "", "", {}, rowStyle(t.second));

        ++r;
    }
}

/**
 */
bool EvaluationData::hasTargetTableTooltip(int col) const
{
    return (col == ColInterest);
}

/**
 */
std::string EvaluationData::targetTableToolTip(const EvaluationTarget& target,
                                               int col,
                                               const InterestEnabledFunc & interest_enabled_func) const
{
    if (col == ColInterest)
        return EvaluationTargetData::enabledInterestFactorsString(target.interestFactors(), interest_enabled_func);

    return "";
}
