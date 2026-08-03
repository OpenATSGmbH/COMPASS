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

#include "dbcontentitem.h"
#include "dbcontentdataindex.h"

#include <map>
#include <tuple>
#include <memory>

#include "json.hpp"

#include <QColor>
#include <QObject>

class DBContentDataSet;
class DBContentManager;
class Buffer;

namespace dbContent
{
    class TargetReportAccessor;
}

namespace context
{
    class DBContextManager;
}

/**
 * Provides DBContent buffer items grouped by certain attributes (e.g. UTN),
 * fed from the current DBContentDataSet (set via setSource + applyChange by the
 * view framework).
 */
class DBContentItemProvider : public QObject
{
    Q_OBJECT

signals:
    void dataAboutToBeResetSignal();
    void dataResetSignal();
    void dataChangedSignal(unsigned int dbc_id, size_t group_idx);
    void dataRefreshedSignal();
    void itemVisibilityChangedSignal();
    void groupingChangedSignal();
    void showItemColorsChangedSignal();

public:
    typedef dbContent::Grouping                            Grouping;
    typedef dbContent::GroupingFlags                       GroupingFlags;
    typedef std::pair<dbContent::ItemGroup*, unsigned int> ItemLocation;
    typedef std::vector<ItemLocation>                      ItemLocations;

    DBContentItemProvider(DBContentManager& dbc_manager,
                          Grouping grouping = Grouping::None);
    virtual ~DBContentItemProvider();

    void update();
    void reset();

    // Source-fed path: setSource points the provider at the current DBContentDataSet
    // (nullptr => no data); applyChange rebuilds the affected contents, keyed by content
    // name (the DBContentDataSet change contract). Driven by the view framework.
    void setSource(const DBContentDataSet* source) { source_ = source; }
    void applyChange(const std::vector<std::string>& names, bool reset, bool last);

    void setGrouping(Grouping grouping,
                     bool run_update = true);
    Grouping grouping() const { return grouping_; }
    std::string groupingAsString() const;
    bool groupingIsNumeric() const;
    bool groupingIsTargetSpecific() const;

    const std::vector<std::unique_ptr<dbContent::ItemGroup>>& itemGroups() const { return item_groups_; }
    const std::map<nlohmann::json, ItemLocations>& itemLocations() const { return item_locations_; }
    const ItemLocations& itemLocations(const nlohmann::json& item_id) const;

    std::string itemName(const nlohmann::json& item_id) const;
    nlohmann::json itemSortValue(const nlohmann::json& item_id) const;
    std::string itemBestAvailableIdentification(const nlohmann::json& item_id) const;

    // --- per-item visibility cache ---
    // Single source of truth for "is item visible?". The DBContentItemModel
    // queries this cache for its check-state column. New ids default to
    // visible (true). Cache is cleared on reset(). Subclasses propagate the
    // change to their backing store via the *_impl() hooks below; external
    // mutations should call setItemVisibleSilent()/setItemsVisibleSilent()
    // and emit itemVisibilityChangedSignal() once the batch completes.

    bool itemVisible(const nlohmann::json& item_id) const;

    void setItemVisible(const nlohmann::json& item_id, bool visible);
    void setItemsVisible(const std::vector<nlohmann::json>& item_ids, bool visible);
    void setAllItemsVisible(bool visible);
    void setSiblingItemsVisible(const nlohmann::json& item_id, bool visible);

    QColor itemColor(const nlohmann::json& item_id) const;

    bool showItemColors() const { return show_item_colors_; }
    void setShowItemColors(bool show);

    static std::string groupingToString(Grouping grouping);
    static Grouping groupingFromString(const std::string& str);
    static bool isGroupingString(const std::string& str);
    static bool isTargetSpecific(Grouping grouping);
    static bool isNumeric(Grouping grouping);
    static std::vector<Grouping> getGroupings(unsigned int flags = std::numeric_limits<unsigned int>::max());

    static const std::string GroupingStrNone;
    static const std::string GroupingStrAircraftAddress;
    static const std::string GroupingStrAircraftID;
    static const std::string GroupingStrTrackNumber;
    static const std::string GroupingStrMode3ACode;
    static const std::string GroupingStrUTN;

    static const Grouping DefaultGrouping;

    /// Fallback ds_type used in ItemGroup::ds_type when the data source is
    /// not registered in the context manager.
    static const std::string DsTypeOther;

protected:
    std::string toString() const;

    void updateInternal(bool grouping_changed);

    virtual void reset_impl() {}
    virtual void contentToBeRebuilt_impl(unsigned int dbc_id) {}
    virtual void rebuildContent_impl(unsigned int dbc_id, size_t group_idx) {}
    virtual void contentRebuilt_impl() {}
    virtual void dataChanged_impl() {}
    virtual void update_impl(bool grouping_changed) {}

    /// Subclass propagates a visibility change to the backing store
    /// (e.g. layer per-item-range hidden flags). Cache is already updated
    /// when these are called.
    virtual void setItemVisible_impl(const nlohmann::json& /*item_id*/, bool /*visible*/) {}
    virtual void setItemsVisible_impl(const std::vector<nlohmann::json>& /*item_ids*/, bool /*visible*/) {}
    virtual void setAllItemsVisible_impl(bool /*visible*/) {}

    /// Update the cache without invoking the *_impl hook and without
    /// emitting itemVisibilityChangedSignal(). Returns true if the cache
    /// value actually changed. Use from subclasses when external state
    /// (e.g. menu actions on a layer scope) implies a visibility change;
    /// emit itemVisibilityChangedSignal() once after a batch.
    bool setItemVisibleSilent(const nlohmann::json& item_id, bool visible);

    std::vector<std::unique_ptr<dbContent::ItemGroup>>& itemGroups() { return item_groups_; }

    /// The current source's full buffer map (empty when no source is set).
    const std::map<std::string, std::shared_ptr<Buffer>>& curBuffers() const;

private:
    void resetData();
    void rebuildContent(unsigned int dbc_id);
    void contentRebuilt();

    void setGroupIDNames(dbContent::ItemGroup& group) const;

    std::function<nlohmann::json(unsigned int)> createGroupFunc(dbContent::TargetReportAccessor& accessor) const;

    static QColor colorFromItemId(const nlohmann::json& item_id);

    // data-access facade over the current source (empty/null when no source is set).
    const DBContentDataIndex::DBContentMap&           curIndices() const;
    std::shared_ptr<Buffer>                           curBuffer(unsigned int dbc_id) const;
    std::shared_ptr<dbContent::TargetReportAccessor>  curTargetReportAccessor(unsigned int dbc_id) const;

    DBContentManager&                                   dbcont_man_;                // static model (names/ids/target model)
    const DBContentDataSet*                             source_ {nullptr};          // current source; null => no data
    context::DBContextManager&                          context_manager_;           // context manager
    Grouping                                            grouping_ = Grouping::None; // item grouping mode
    std::vector<std::unique_ptr<dbContent::ItemGroup>>  item_groups_;               // per (dbcontent, ds, line) item groups
    std::map<nlohmann::json, ItemLocations>             item_locations_;            // per item group locations
    std::map<nlohmann::json, bool>                      item_visibility_;           // per item visibility cache; missing entries default to true
    std::map<nlohmann::json, QColor>                    item_colors_;
    bool                                                show_item_colors_ {false};
};
