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

#include "targetmodel.h"
#include "compass.h"
#include "dbinterface.h"
#include "dbcontentmanager.h"
#include "evaluationmanager.h"
#include "evaluationtargetfilter.h"
#include "logger.h"
#include "reconstructortarget.h"
#include "util/files.h"

#include <QApplication>
#include <QThread>
#include <QProgressDialog>
#include <QLabel>

using namespace std;
using namespace Utils;
using namespace nlohmann;

namespace dbContent {

const QColor very_light_gray (230, 230, 230);

/**
 */
TargetModel::TargetModel(DBContentManager& dbcont_manager)
    :dbcont_manager_(dbcont_manager)
{
}

/**
 */
TargetModel::~TargetModel() = default;

/**
 */
void TargetModel::clear()
{
    beginResetModel();

    target_data_.clear();

    endResetModel();
}

/**
 */
std::string TargetModel::iconForTarget(const Target& target, 
                                       bool add_placeholder_txt)
{
    if (!target.useInEval())
        return add_placeholder_txt ? "delete.png;No" : "delete.png";

    // could be used

    if (COMPASS::instance().evaluationManager().useTimestampFilter()
        || target.evalExcludedTimeWindows().size()
        || target.evalExcludedRequirements().size())
            return add_placeholder_txt ? "partial_done.png;Partial" : "partial_done.png";

    return add_placeholder_txt ? "done.png;Yes" : "done.png";
}

/**
 */
QVariant TargetModel::data(const QModelIndex& index, int role) const
{
    if (!index.isValid())
        return QVariant();

    traced_assert(index.row() >= 0);
    traced_assert(static_cast<size_t>(index.row()) < target_data_.size());

    const Target& target = target_data_.at(index.row());

    switch (role)
    {
        case Qt::BackgroundRole:
        {
            if (!target.useInEval())
                return QBrush(Qt::lightGray);
            if (COMPASS::instance().evaluationManager().useTimestampFilter()
                    || target.evalExcludedTimeWindows().size()
                    || target.evalExcludedRequirements().size())
                return QBrush(very_light_gray);
            else
                return QVariant();

        }
        case Qt::DisplayRole:
        {
            logdbg << "display role: row " << index.row() << " col " << index.column();

            traced_assert(index.column() < table_columns_.size());
            int col = index.column();
            traced_assert(col >= 0);

            return getCellContent(target, (Columns) col);
        }
        case Qt::UserRole: // to find the checkboxes
        {
            if (index.column() == ColUseInEval)
            {
                return target.utn_;
            }
            else if (index.column() == ColComment) // comment
            {
                return ("comment_"+to_string(target.utn_)).c_str();
            }
        }
        case Qt::DecorationRole:
        {
            if (index.column() == ColUseInEval)  // selected special case
            {
                return Utils::Files::IconProvider::getIcon(TargetModel::iconForTarget(target));
            }
            else
                return QVariant();
        }
        case Qt::ToolTipRole:
        {
            QString tooltip_html;
            tooltip_html += "<html><body><table>";

            // Helper lambda to append any (label,value) pair as a right-aligned row
            auto append_row = [&](const QString& label, const QString& value, bool bold_label = true)
            {
                QString tmp_value = value;
                tmp_value.replace("\n", "<br/>");

                tooltip_html += "<tr>"
                                "<td>" + (bold_label ? "<b>" + label + "</b>" : label) + ":</td>"
                                + "<td align=\"left\">" + tmp_value + "</td>"
                                + "</tr>";
            };

            if (!show_eval_columns_)
            {
                QString eval_value;
                if (!target.useInEval())
                    eval_value = "No";
                else if (COMPASS::instance().evaluationManager().useTimestampFilter()
                           || target.evalExcludedTimeWindows().size()
                           || target.evalExcludedRequirements().size())
                    eval_value = "Partial";
                else
                    eval_value = "Yes";

                append_row("Eval", eval_value);

                append_row(table_columns_.at(ColUseEvalDetails),
                           getCellContent(target, ColUseEvalDetails).toString());
            }

            if (!show_duration_columns_)
            {
                for (auto column : duration_columns_)
                    append_row(table_columns_.at(column), getCellContent(target, (Columns)column).toString());
            }

            if (!show_mode_s_columns_)
            {
                for (auto column : mode_s_columns_)
                    append_row(table_columns_.at(column), getCellContent(target, (Columns)column).toString());
            }

            if (!show_mode_ac_columns_)
            {
                for (auto column : mode_ac_columns_)
                    append_row(table_columns_.at(column), getCellContent(target, (Columns)column).toString());
            }

            if (!show_adsb_columns_)
            {
                for (auto column : adsb_columns_)
                    append_row(table_columns_.at(column), getCellContent(target, (Columns)column).toString());
            }

            tooltip_html += "</table></body></html>";
            return tooltip_html;
        }
        default:
        {
            return QVariant();
        }
    }
}

/**
 */
bool TargetModel::setData(const QModelIndex &index, const QVariant& value, int role)
{
    if (!index.isValid() /*|| role != Qt::EditRole*/)
        return false;

    if (role == Qt::EditRole && index.column() == ColComment) // comment
    {
        traced_assert(index.row() >= 0);
        traced_assert(static_cast<size_t>(index.row()) < target_data_.size());

        auto it = target_data_.begin() + index.row();

        loginf << "utn " << it->utn_ <<" comment '" << value.toString().toStdString() << "'";

        setTargetComment(it->utn_, value.toString().toStdString());

        return true;
    }

    return false;
}

/**
 */
QVariant TargetModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
    {
        traced_assert(section < table_columns_.size());
        return table_columns_.at(section);
    }

    return QVariant();
}

/**
 */
QModelIndex TargetModel::index(int row, int column, const QModelIndex& parent) const
{
    return createIndex(row, column);
}

/**
 */
int TargetModel::rowCount(const QModelIndex& parent) const
{
    return target_data_.size();
}

/**
 */
int TargetModel::columnCount(const QModelIndex& parent) const
{
    return table_columns_.size();
}

/**
 */
QModelIndex TargetModel::parent(const QModelIndex& index) const
{
    return QModelIndex();
}

/**
 */
Qt::ItemFlags TargetModel::flags(const QModelIndex &index) const
{
    if (!index.isValid())
        return Qt::ItemIsEnabled;

    traced_assert(index.column() < table_columns_.size());

    if (index.column() == ColComment) // comment
        return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
    else
        return QAbstractItemModel::flags(index);
}

QVariant TargetModel::getCellContent(const Target& target, Columns col) const
{
    switch(col)
    {
    case ColUTN:
        return target.utn_;
    case ColComment:
        return target.comment().c_str();
    case ColCategory:
        return target.emitterCategoryStr().c_str();
    case ColUseInEval:
        return QVariant();
    case ColUseEvalDetails:
    {
        QString tmp;

        if (target.evalExcludedTimeWindows().size())
            tmp = target.evalExcludedTimeWindows().asString().c_str();

        if (target.evalExcludedRequirements().size())
        {
            if (tmp.size())
                tmp += "\n";

            tmp += String::compress(target.evalExcludedRequirements(),',').c_str();
        }

        return tmp;
    }
    case ColNumUpdates:
        return target.numUpdates();
    case ColBegin:
        return target.timeBeginStr().c_str();
    case ColEnd:
        return target.timeEndStr().c_str();
    case ColDuration:
        return target.timeDurationStr().c_str();
    case ColACIDs:
        return target.aircraftIdentificationsStr().c_str();
    case ColACADs:
        return target.aircraftAddressesStr().c_str();
    case ColMode3A:
        return target.modeACodesStr().c_str();
    case ColModeCMin:
        if (target.hasModeC())
            return target.modeCMin();
        else
            return "";
    case ColModeCMax:
        if (target.hasModeC())
            return target.modeCMax();
        else
            return "";
    case ColADSBCount:
        return target.adsbCount() ? target.adsbCount() : QVariant();
    case ColADSBMOPS:
        return target.adsbMopsStr().c_str();
    }

    return QVariant();
}

/**
 */
const dbContent::Target& TargetModel::getTargetOf (const QModelIndex& index)
{
    traced_assert(index.isValid());

    traced_assert(index.row() >= 0);
    traced_assert(static_cast<size_t>(index.row()) < target_data_.size());

    const dbContent::Target& target = target_data_.at(index.row());

    return target;
}

/**
 */
void TargetModel::setEvalUseTarget (unsigned int utn, bool value)
{
    loginf << "utn " << utn << " value " << value;

    traced_assert(existsTarget(utn));

    // search if checkbox can be found
    // QModelIndexList items = match(
    //             index(0, ColUseEval),
    //             Qt::UserRole,
    //             QVariant(utn),
    //             1, // look *
    //             Qt::MatchExactly); // look *

    // traced_assert(items.size() == 1);

    // setData(items.at(0), {value ? Qt::Checked: Qt::Unchecked}, Qt::CheckStateRole);

    target(utn).useInEval(value);

    updateEvalUseColumn();
    emit targetEvalUsageChangedSignal();

    updateToDB(utn);
}

void TargetModel::setEvalUseTarget (std::set<unsigned int> utns, bool value)
{
    for (auto utn : utns)
        target(utn).useInEval(value);

    updateEvalUseColumn();
    emit targetEvalUsageChangedSignal();

    updateToDB(utns);
}

/**
 */
void TargetModel::setAllUseTargets (bool value)
{
    loginf << "value " << value;

    for (auto target_it = target_data_.begin(); target_it != target_data_.end(); ++target_it)
        target_data_.modify(target_it, [value](Target& p) { p.useInEval(value); });

    updateEvalUseColumn();
    emit targetEvalUsageChangedSignal();

    saveToDB();
}

/**
 */
void TargetModel::setTargetComment (unsigned int utn, std::string comment)
{
    loginf << "utn " << utn << " comment '" << comment << "'";

    traced_assert(existsTarget(utn));

    // search if comment can be found can be found, check in COLUMN 2!
    // QModelIndexList items = match(
    //             index(0, ColComment),
    //             Qt::UserRole,
    //             QVariant(("comment_"+to_string(utn)).c_str()),
    //             1, // look *
    //             Qt::MatchExactly); // look *

    //  loginf << "size " << items.size();

    // traced_assert(items.size() == 1);
    // setData(items.at(0), comment.c_str(), Qt::EditRole);

    target(utn).comment(comment);

    updateCommentColumn();
    emit targetInfoChangedSignal();

    updateToDB(utn);
}

void TargetModel::setTargetComment (std::set<unsigned int> utns, std::string comment)
{
    for (auto utn : utns)
        target(utn).comment(comment);

    updateCommentColumn();
    emit targetInfoChangedSignal();

    updateToDB(utns);
}

/**
 */
void TargetModel::clearAllTargetComments ()
{
    loginf;

    for (auto target_it = target_data_.begin(); target_it != target_data_.end(); ++target_it)
        target_data_.modify(target_it, [](Target& p) { p.comment(""); });


    updateCommentColumn();
    emit targetInfoChangedSignal();

    saveToDB();
}

void TargetModel::setEvalExcludeTimeWindows(
    std::set<unsigned int> utns, const Utils::TimeWindowCollection& collection)
{
    for (auto utn : utns)
        target(utn).evalExcludedTimeWindows(collection);

    updateEvalDetailsColumn();
    emit targetEvalFullChangeSignal();

    updateToDB(utns);
}

void TargetModel::clearEvalExcludeTimeWindows(std::set<unsigned int> utns)
{
    for (auto utn : utns)
        target(utn).clearEvalExcludedTimeWindows();

    updateEvalDetailsColumn();
    emit targetEvalFullChangeSignal();

    updateToDB(utns);
}

void TargetModel::clearAllEvalExcludeTimeWindows()
{
    loginf;

    for (auto target_it = target_data_.begin(); target_it != target_data_.end(); ++target_it)
        target_data_.modify(target_it, [](Target& p) { p.clearEvalExcludedTimeWindows(); });

    updateEvalDetailsColumn();
    emit targetEvalFullChangeSignal();

    saveToDB();
}

void TargetModel::setEvalExcludeRequirements(std::set<unsigned int> utns, const std::set<std::string>& excl_req)
{
    for (auto utn : utns)
        target(utn).evalExcludedRequirements(excl_req);

    updateEvalDetailsColumn();
    emit targetEvalUsageChangedSignal();

    updateToDB(utns);
}

void TargetModel::clearEvalExcludeRequirements(std::set<unsigned int> utns)
{
    for (auto utn : utns)
        target(utn).clearEvalExcludedRequirements();

    updateEvalDetailsColumn();
    emit targetEvalUsageChangedSignal();

    updateToDB(utns);
}

void TargetModel::clearAllEvalExcludeRequirements()
{
    loginf;

    for (auto target_it = target_data_.begin(); target_it != target_data_.end(); ++target_it)
        target_data_.modify(target_it, [](Target& p) { p.clearEvalExcludedRequirements(); });

    updateEvalDetailsColumn();
    emit targetEvalUsageChangedSignal();

    saveToDB();
}

/**
 */
void TargetModel::setUseByFilter ()
{
    loginf;

    COMPASS::instance().evaluationManager().targetFilter().setUse(target_data_);

    updateEvalUseColumn();
    emit targetEvalUsageChangedSignal();

    saveToDB();
}

/**
 */
bool TargetModel::hasTargetsInfo()
{
    return target_data_.size();
}

/**
 */
void TargetModel::deleteAllTargets()
{
    clear();

    COMPASS::instance().dbInterface().clearTargetsTable();

    emit targetsDeletedSignal();
}

/**
 */
bool TargetModel::existsTarget(unsigned int utn) const
{
    auto tr_tag_it = target_data_.get<target_tag>().find(utn);

    return tr_tag_it != target_data_.get<target_tag>().end();
}

/**
 */
void TargetModel::createNewTargets(const std::map<unsigned int, dbContent::ReconstructorTarget>& targets)
{
    beginResetModel();

    traced_assert(!target_data_.size());

    unsigned int utn = 0;

    for (auto& tgt_it : targets)
    {
        //cont_man.createNewTarget(tgt_it.first);
        utn = tgt_it.first;

        target_data_.push_back({utn, nlohmann::json::object()});

        traced_assert(existsTarget(utn));

        dbContent::Target& tgt = target(utn);

                //target.useInEval(tgt_it.second.use_in_eval_);

                //if (tgt_it.second.comment_.size())
                //    target.comment(tgt_it.second.comment_);

        tgt.aircraftAddresses(tgt_it.second.acads_);
        tgt.aircraftIdentifications(tgt_it.second.acids_);
        tgt.modeACodes(tgt_it.second.mode_as_);

        if (tgt_it.second.ecat_ && *tgt_it.second.ecat_ != (unsigned int) dbContent::Target::Category::Unknown)
            tgt.targetCategory(dbContent::Target::fromECAT(tgt_it.second.ecat_));

        if (tgt_it.second.hasTimestamps())
        {
            tgt.timeBegin(tgt_it.second.total_timestamp_min_);
            tgt.timeEnd(tgt_it.second.total_timestamp_max_);
        }

        if (tgt_it.second.hasModeC())
            tgt.modeCMinMax(*tgt_it.second.mode_c_min_, *tgt_it.second.mode_c_max_);

        if (tgt_it.second.latitude_min_ && tgt_it.second.latitude_max_
            && tgt_it.second.longitude_min_ && tgt_it.second.longitude_max_)
            tgt.setPositionBounds(*tgt_it.second.latitude_min_, *tgt_it.second.latitude_max_,
                                  *tgt_it.second.longitude_min_, *tgt_it.second.longitude_max_);

                // set counts
        for (auto& count_it : tgt_it.second.getDBContentCounts())
            tgt.dbContentCount(count_it.first, count_it.second);

                // set adsb stuff
        tgt.adsbCount(tgt_it.second.adsb_count_);
        tgt.adsbMOPSCount(tgt_it.second.adsb_mops_count_);

        //++utn;
    }

    endResetModel();
}

/**
 */
dbContent::Target& TargetModel::target(unsigned int utn)
{
    auto tr_tag_it = target_data_.get<target_tag>().find(utn);

    traced_assert(tr_tag_it != target_data_.get<target_tag>().end());

    return const_cast<dbContent::Target&> (*tr_tag_it); // ok since key utn_ can not be modified, still const
}

/**
 */
const dbContent::Target& TargetModel::target(unsigned int utn) const
{
    auto tr_tag_it = target_data_.get<target_tag>().find(utn);

    traced_assert(tr_tag_it != target_data_.get<target_tag>().end());

    return const_cast<const dbContent::Target&> (*tr_tag_it);
}

/**
 */
unsigned int TargetModel::size() const
{
    return target_data_.size();
}

/**
 */
// void TargetModel::removeDBContentFromTargets(const std::string& dbcont_name)
// {
//     for (auto target_it = target_data_.begin(); target_it != target_data_.end(); ++target_it)
//         target_data_.modify(target_it, [dbcont_name](Target& p) { p.clearDBContentCount(dbcont_name); });
// }

// void TargetModel::storeTargetsEvalInfo()
// {
//     for (auto target_it = target_data_.begin(); target_it != target_data_.end(); ++target_it)
//         target_data_.modify(target_it, [](Target& p) { p.storeEvalutionInfo(); });
// }

/**
 */
nlohmann::json TargetModel::asJSON() const
{
    nlohmann::json data;

    for (auto& target : target_data_)
    {
        data[to_string(target.utn_)] = target.info();
        data[to_string(target.utn_)]["utn"] = target.utn_;
    }

    return data;
}

/**
 */
nlohmann::json TargetModel::targetAsJSON(unsigned int utn) const
{
    if (!existsTarget(utn))
        return {};

    const auto& t = target(utn);

    nlohmann::json data;
    data["utn" ] = utn;
    data["info"] = t.info();

    return data;
}

/**
 */
nlohmann::json TargetModel::targetStatsAsJSON() const
{
    nlohmann::json data;

    size_t num_targets = target_data_.size();

    size_t associated = 0;
    size_t min_size   = std::numeric_limits<size_t>::max();
    size_t max_size   = 0;
    size_t mean_size  = 0;

    for (const auto& target : target_data_)
    {
        size_t nu = target.numUpdates();

        associated += nu;
        mean_size  += nu;

        if (nu > max_size) max_size = nu;
        if (nu < min_size) min_size = nu;
    }

    if (num_targets == 0)
        min_size = 0;
    else
        mean_size /= num_targets;

    data["num_targets"  ] = num_targets;
    data["min_updates"  ] = min_size;
    data["max_updates"  ] = max_size;
    data["avg_updates"  ] = mean_size;
    data["total_updates"] = associated;

    return data;
}

/**
 */
nlohmann::json TargetModel::utnsAsJSON() const
{
    nlohmann::json data;

    auto utn_arr = nlohmann::json::array();

    for (auto& target : target_data_)
    {
        utn_arr.push_back(target.utn_);
    }

    data["utns"] = utn_arr;

    return data;
}

/**
 */
void TargetModel::loadFromDB()
{
    loginf;

    beginResetModel();

    for (auto& target : COMPASS::instance().dbInterface().loadTargets())
    {
        target_data_.push_back({target->utn_, target->info()});
    }

    for (auto& target : target_data_)
        traced_assert(existsTarget(target.utn_));

    endResetModel();

    loginf << "loaded " << target_data_.size() << " targets";
}

/**
 */
void TargetModel::saveToDB()
{
    loginf << "saving " << target_data_.size() << " targets";

    std::map<unsigned int, nlohmann::json> targets_info;

    for (auto& target : target_data_)
        targets_info[target.utn_] = target.info();

    COMPASS::instance().dbInterface().saveTargets(targets_info);
}

/**
 */
void TargetModel::updateToDB(unsigned int utn)
{
    loginf << "saving utn " << utn;

    auto tr_tag_it = target_data_.get<target_tag>().find(utn);

    traced_assert(tr_tag_it != target_data_.get<target_tag>().end());

    std::map<unsigned int, nlohmann::json> targets_info;

    targets_info[tr_tag_it->utn_] = tr_tag_it->info();

    COMPASS::instance().dbInterface().updateTargets(targets_info);
}

void TargetModel::updateToDB(std::set<unsigned int> utns)
{
    loginf << "saving utns " << String::compress(utns,',');

    std::map<unsigned int, nlohmann::json> targets_info;

    for (auto utn : utns)
    {
        const auto& tgt = target(utn);

        targets_info[tgt.utn_] = tgt.info();
    }

    COMPASS::instance().dbInterface().updateTargets(targets_info);
}

/**
 */
void TargetModel::showMainColumns(bool show)
{
    show_main_columns_ = show;
}

/**
 */
void TargetModel::showEvalColumns(bool show)
{
    show_eval_columns_ = show;
}

/**
 */
void TargetModel::showDurationColumns(bool show)
{
    show_duration_columns_ = show;
}

/**
 */
void TargetModel::showModeSColumns(bool show)
{
    show_mode_s_columns_ = show;
}

/**
 */
void TargetModel::showModeACColumns(bool show)
{
    show_mode_ac_columns_ = show;
}

void TargetModel::showADSBColumns(bool show)
{
    show_adsb_columns_ = show;
}

void TargetModel::updateCommentColumn()
{
    emit dataChanged(index(0, ColComment), index(this->rowCount(), ColComment), {Qt::DisplayRole});
}

void TargetModel::updateEvalUseColumn()
{
    emit dataChanged(index(0, ColUseInEval), index(this->rowCount(), ColUseInEval), {Qt::DecorationRole});
}
void TargetModel::updateEvalDetailsColumn()
{
    emit dataChanged(index(0, ColUseEvalDetails), index(this->rowCount(), ColUseEvalDetails), {Qt::DisplayRole});
}

}
