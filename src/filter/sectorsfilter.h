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

#include "dbfilter.h"
#include "sectorinsidecache.h"

#include <memory>
#include <string>
#include <vector>

/**
 * Filters target reports by the sector layers of the active Data Context.
 *
 * A report passes if its position is inside any non-exclusion sector of any selected layer,
 * whole layers only. With the surrounding delta the sectors are inflated by the delta in
 * degrees, so reports slightly outside pass too. The altitude limits of the sectors apply to
 * reports with a barometric altitude, a report without one passes the planar check alone.
 *
 * Offline the WHERE clause holds a bounding box over the selected layers plus a call of the
 * database function sector_inside() per layer, see SectorInsideCache::DBFunctionName. In live
 * mode the same prepared shapes are checked in memory. A selected layer the active context does
 * not have contributes no condition.
 */
class SectorsFilter : public DBFilter
{
public:
    /**
     * A selected layer with its bounds, input of sqlFor().
     */
    struct Layer
    {
        std::string                    name;
        SectorInsideCache::LayerBounds bounds;
    };

    SectorsFilter(nlohmann::json& config, FilterManager* parent, IDBVariableResolver& var_resolver);
    virtual ~SectorsFilter();

    virtual std::string getConditionString(const std::string& dbcontent_name,
                                           dbContent::VariableSet& read_set, bool& first) override;
    virtual FilterClause getClause(const std::string& dbcontent_name) override;

    /**
     * Renders the clause for the layers: the bounding box of all layers widened by a positive
     * delta, and one sector_inside() call per layer, joined with OR. Empty without layers or
     * for a DBContent without position.
     */
    static FilterClause sqlFor(IDBVariableResolver& resolver,
                               const std::string& dbcontent_name,
                               const std::vector<Layer>& layers,
                               double delta_deg);

    /**
     * Surrounding delta from the extent of the layers: the fraction of the larger side of the
     * combined bounds (the longitude side scaled by the cosine of the center latitude), clamped
     * to [min_m converted to degrees of latitude, max_deg]. 0 without layers.
     */
    static double extentDeltaDeg(const std::vector<Layer>& layers,
                                 double fraction, double min_m, double max_deg);

    virtual bool filters(const std::string& dbcontent_name) override;
    virtual void reset() override;

    virtual void saveViewPointConditions (nlohmann::json& filters) override;
    virtual void loadViewPointConditions (const nlohmann::json& filters) override;

    virtual bool activeInLiveMode() override;
    virtual std::vector<unsigned int> filterBuffer(const std::string& dbcontent_name,
                                                   std::shared_ptr<Buffer> buffer) override;

    std::vector<std::string> selectedLayers() const;
    bool layerSelected(const std::string& layer_name) const;
    void selectLayer(const std::string& layer_name, bool select);

    bool useDelta() const;
    void useDelta(bool use);

    double deltaDeg() const;
    void deltaDeg(double delta_deg);

    /// the delta applied, 0 if the surrounding delta is off
    double effectiveDeltaDeg() const;

    /// layers offered for selection: the sector layers of the active context, or the layers
    /// of the cache without a filter manager (tests)
    std::vector<std::string> availableLayers() const;

    /// the cache of the database interface, or an own one without a filter manager (tests)
    SectorInsideCache& cache();

    /// refreshes the widget if it exists
    void updateWidget();

    /// removes selected layers and cached layers the active context does not have, called when a
    /// database is opened. Returns the number of removed selections
    unsigned int removeLayersNotInContext();

    /// unusable without sector layers in the active context, usable again when layers appear.
    /// Returns true if the state changed
    bool updateUsability();

    static const std::string FilterName;
    static const std::string ViewPointKeyLayers;
    static const std::string ViewPointKeyUseDelta;
    static const std::string ViewPointKeyDelta;

protected:
    virtual DBFilterWidget* createWidget() override;

    /// the selected layers that exist, their geometry pushed to the cache, with bounds
    std::vector<Layer> activeLayers();

    nlohmann::json layers_;              // selected layer names, array of strings
    bool           use_delta_ {false};
    double         delta_deg_ {0.1};

    std::shared_ptr<SectorInsideCache> own_cache_;
};
