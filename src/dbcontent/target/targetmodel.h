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

#include "target.h"
#include "json_fwd.hpp"

#include <QAbstractItemModel>

#include <map>

#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/random_access_index.hpp>
#include <boost/multi_index_container.hpp>

class DBContentManager;

namespace dbContent {

    class ReconstructorTarget;

struct target_tag
{
};

typedef boost::multi_index_container<
    Target,
    boost::multi_index::indexed_by<
        boost::multi_index::random_access<>,  // this index represents insertion order
        boost::multi_index::ordered_unique<boost::multi_index::tag<target_tag>,
            boost::multi_index::member<
        Target, const unsigned int, &Target::utn_> >
        > >
    TargetCache;

class TargetModel : public QAbstractItemModel
{
    Q_OBJECT

signals:
    void targetInfoChangedSignal();
    void targetEvalUsageChangedSignal(); // for eval use flag, or disabled requirements (only)
    void targetEvalFullChangeSignal(); // for full changes
    void targetsDeletedSignal();

public:
    enum Columns
    {
        ColUTN = 0,
        ColComment,
        ColCategory,
        ColUseInEval,
        ColUseEvalDetails,
        ColNumUpdates,
        ColBegin,
        ColEnd,
        ColDuration,
        ColACIDs,
        ColACADs,
        ColMode3A,
        ColModeCMin,
        ColModeCMax,
        ColADSBCount,
        ColADSBMOPS
    };

    TargetModel(DBContentManager& dbcont_manager);
    virtual ~TargetModel();

    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    bool setData(const QModelIndex &index, const QVariant& value, int role) override;

    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;
    QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;
    QModelIndex parent(const QModelIndex& index) const override;

    Qt::ItemFlags flags(const QModelIndex &index) const override;

    QVariant getCellContent(const Target& target, Columns col) const;

    const dbContent::Target& getTargetOf (const QModelIndex& index);

    void setTargetComment (unsigned int utn, std::string comment);
    void setTargetComment (std::set<unsigned int> utns, std::string comment);
    void clearAllTargetComments();

    void setEvalUseTarget (unsigned int utn, bool value);
    void setEvalUseTarget (std::set<unsigned int> utns, bool value);
    void setAllUseTargets (bool value);

    void setEvalExcludeTimeWindows(std::set<unsigned int> utns, const Utils::TimeWindowCollection& collection);
    void clearEvalExcludeTimeWindows(std::set<unsigned int> utns);
    void clearAllEvalExcludeTimeWindows();

    void setEvalExcludeRequirements(std::set<unsigned int> utns, const std::set<std::string>& excl_req);
    void clearEvalExcludeRequirements(std::set<unsigned int> utns);
    void clearAllEvalExcludeRequirements();

    void setUseByFilter ();

    bool hasTargetsInfo();
    void deleteAllTargets();
    bool existsTarget(unsigned int utn) const;
    void createNewTargets(const std::map<unsigned int, dbContent::ReconstructorTarget>& targets);
    dbContent::Target& target(unsigned int utn);
    const dbContent::Target& target(unsigned int utn) const;
    unsigned int size() const;

    nlohmann::json asJSON() const;
    nlohmann::json targetAsJSON(unsigned int utn) const;
    nlohmann::json targetStatsAsJSON() const;
    nlohmann::json utnsAsJSON() const;

    void loadFromDB();
    void saveToDB();
    void updateToDB(unsigned int utn);
    void updateToDB(std::set<unsigned int> utns);

    void clear();

    const QStringList& tableHeaders() const { return table_columns_; }

    const std::vector<int>& mainColumns() const { return main_columns_; }
    const std::vector<int>& evalColumns() const { return eval_columns_; }
    const std::vector<int>& durationColumns() const { return duration_columns_; }
    const std::vector<int>& modeSColumns() const { return mode_s_columns_; }
    const std::vector<int>& modeACColumns() const { return mode_ac_columns_; }
    const std::vector<int>& adsbColumns() const { return adsb_columns_; }

    bool showMainColumns() { return show_main_columns_; }
    bool showEvalColumns() { return show_eval_columns_; }
    bool showDurationColumns() { return show_duration_columns_; }
    bool showModeSColumns() { return show_mode_s_columns_; }
    bool showModeACColumns() { return show_mode_ac_columns_; }
    bool showADSBColumns() { return show_adsb_columns_; }

    void showMainColumns(bool show);
    void showEvalColumns(bool show);
    void showDurationColumns(bool show);
    void showModeSColumns(bool show);
    void showModeACColumns(bool show);
    void showADSBColumns(bool show);

    void updateCommentColumn();
    void updateEvalUseColumn();
    void updateEvalDetailsColumn();

    static std::string iconForTarget(const Target& target, bool add_placeholder_txt = false);

protected:
    DBContentManager& dbcont_manager_;

    QStringList               table_columns_   { "UTN", "Comment", "Category", "Eval", "Eval Excluded",
                               "#Updates", "Begin", "End", "Duration", "ACIDs", "ACADs", "M3/A", "MC Min", "MC Max",
                               "ADS-B", "MOPS"};
    std::vector<int>          main_columns_    { ColUTN, ColComment, ColCategory, ColUseInEval };
    std::vector<int>          eval_columns_    { ColUseEvalDetails };
    std::vector<int>          duration_columns_{ ColNumUpdates, ColBegin, ColEnd, ColDuration };
    std::vector<int>          mode_s_columns_  { ColACIDs, ColACADs };
    std::vector<int>          mode_ac_columns_ { ColMode3A, ColModeCMin, ColModeCMax };
    std::vector<int>          adsb_columns_    { ColADSBCount, ColADSBMOPS };

    bool show_main_columns_     = true;
    bool show_eval_columns_     = false;
    bool show_duration_columns_ = false;
    bool show_mode_s_columns_   = true;
    bool show_mode_ac_columns_  = false;
    bool show_adsb_columns_     = false;
    
    TargetCache target_data_;
};

}

