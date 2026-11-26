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

#include "sectorsfilter.h"
#include "sectorsfilterwidget.h"
#include "filtermanager.h"
#include "idbvariableresolver.h"
#include "dbinterface.h"
#include "db_context_manager.h"
#include "sectorlayer.h"
#include "buffer.h"
#include "logger.h"
#include "traced_assert.h"
#include "json.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

using namespace nlohmann;

const std::string SectorsFilter::FilterName           = "Sectors";
const std::string SectorsFilter::ViewPointKeyLayers   = "Layers";
const std::string SectorsFilter::ViewPointKeyUseDelta = "Use Surrounding Delta";
const std::string SectorsFilter::ViewPointKeyDelta    = "Surrounding Delta";

namespace
{
    std::string sqlString(const std::string& value)
    {
        std::string escaped;
        escaped.reserve(value.size() + 2);

        for (char c : value)
        {
            if (c == '\'')
                escaped += '\'';
            escaped += c;
        }

        return "'" + escaped + "'";
    }
}

/**
 */
SectorsFilter::SectorsFilter(nlohmann::json& config, FilterManager* parent, IDBVariableResolver& var_resolver)
    : DBFilter(config, false, parent, var_resolver)
{
    registerParameter("layers", &layers_, json::array());
    registerParameter("use_delta", &use_delta_, false);
    registerParameter("delta_deg", &delta_deg_, 0.1);

    if (!layers_.is_array())
        layers_ = json::array();

    name_ = FilterName;

    createSubConfigurables();

    //the usability is evaluated when the widget is created, when a database is opened and when
    //the sectors change. Not here: the filter manager is created before the context manager.
}

/**
 */
bool SectorsFilter::updateUsability()
{
    const bool no_layers = availableLayers().empty();
    const bool changed   = no_layers != unusable();

    if (changed)
        loginf << "sectors filter " << (no_layers ? "unusable, no sector layers in the active context"
                                                  : "usable again");

    setUnusable(no_layers);

    return changed;
}

SectorsFilter::~SectorsFilter() = default;

/**
 */
bool SectorsFilter::filters(const std::string& dbcontent_name)
{
    //no sector layers in the active context
    if (unusable())
        return false;

    return variableResolver().metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_latitude_) &&
           variableResolver().metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_longitude_);
}

/**
 */
std::string SectorsFilter::getConditionString(const std::string& dbcontent_name,
                                              dbContent::VariableSet& read_set, bool& first)
{
    FilterClause clause = getClause(dbcontent_name);

    if (clause.empty())
        return "";

    std::string condition = (first ? " " : " AND ") + clause.sql;
    first = false;

    return condition;
}

/**
 */
FilterClause SectorsFilter::getClause(const std::string& dbcontent_name)
{
    FilterClause clause;

    if (!active_ || !filters(dbcontent_name))
        return clause;

    std::vector<Layer> layers = activeLayers();

    if (layers.empty())
        return clause;

    return sqlFor(variableResolver(), dbcontent_name, layers, effectiveDeltaDeg());
}

/**
 */
FilterClause SectorsFilter::sqlFor(IDBVariableResolver& resolver,
                                   const std::string& dbcontent_name,
                                   const std::vector<Layer>& layers,
                                   double delta_deg)
{
    FilterClause clause;

    if (layers.empty())
        return clause;

    if (!resolver.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_latitude_) ||
        !resolver.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_longitude_))
        return clause;

    const std::string lat_col = resolver.metaGetVariableDBColumn(dbcontent_name, dbcontent_vars::meta_var_latitude_);
    const std::string lon_col = resolver.metaGetVariableDBColumn(dbcontent_name, dbcontent_vars::meta_var_longitude_);

    //barometric altitude for the altitude limits of the sectors, without it the limits are not checked
    const std::string alt_col = resolver.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_)
                                ? resolver.metaGetVariableDBColumn(dbcontent_name, dbcontent_vars::meta_var_mc_)
                                : std::string("NULL");

    //bounding box over all layers, widened by an inflating delta. a deflating delta keeps the box
    double lat_min = layers.front().bounds.lat_min;
    double lat_max = layers.front().bounds.lat_max;
    double lon_min = layers.front().bounds.lon_min;
    double lon_max = layers.front().bounds.lon_max;

    for (const auto& layer : layers)
    {
        lat_min = std::min(lat_min, layer.bounds.lat_min);
        lat_max = std::max(lat_max, layer.bounds.lat_max);
        lon_min = std::min(lon_min, layer.bounds.lon_min);
        lon_max = std::max(lon_max, layer.bounds.lon_max);
    }

    const double widen = std::max(delta_deg, 0.0);

    std::stringstream ss;
    ss << std::setprecision(10);

    ss << "(" << lat_col << " BETWEEN " << lat_min - widen << " AND " << lat_max + widen
       << " AND " << lon_col << " BETWEEN " << lon_min - widen << " AND " << lon_max + widen
       << " AND ";

    if (layers.size() > 1)
        ss << "(";

    bool first = true;

    for (const auto& layer : layers)
    {
        if (!first)
            ss << " OR ";

        ss << SectorInsideCache::DBFunctionName << "(" << sqlString(layer.name) << ", "
           << lat_col << ", " << lon_col << ", " << alt_col << ", " << delta_deg << ")";

        first = false;
    }

    if (layers.size() > 1)
        ss << ")";

    ss << ")";

    clause.sql = ss.str();

    return clause;
}

/**
 */
double SectorsFilter::extentDeltaDeg(const std::vector<Layer>& layers,
                                     double fraction, double min_m, double max_deg)
{
    if (layers.empty())
        return 0.0;

    double lat_min = layers.front().bounds.lat_min;
    double lat_max = layers.front().bounds.lat_max;
    double lon_min = layers.front().bounds.lon_min;
    double lon_max = layers.front().bounds.lon_max;

    for (const auto& layer : layers)
    {
        lat_min = std::min(lat_min, layer.bounds.lat_min);
        lat_max = std::max(lat_max, layer.bounds.lat_max);
        lon_min = std::min(lon_min, layer.bounds.lon_min);
        lon_max = std::max(lon_max, layer.bounds.lon_max);
    }

    //larger side of the combined bounds, in degrees of latitude
    const double center_lat_rad = (lat_min + lat_max) / 2.0 * M_PI / 180.0;
    const double extent_deg     = std::max(lat_max - lat_min,
                                           (lon_max - lon_min) * std::cos(center_lat_rad));

    //one degree of latitude is 111320 m
    const double min_deg = min_m / 111320.0;

    return std::min(std::max(fraction * extent_deg, min_deg), max_deg);
}

/**
 */
std::vector<SectorsFilter::Layer> SectorsFilter::activeLayers()
{
    std::vector<Layer> layers;

    SectorInsideCache& sector_cache = cache();

    for (const std::string& name : selectedLayers())
    {
        if (filter_manager_)
        {
            auto& ctx_man = filter_manager_->dbContextManager();

            //not in this context
            if (!ctx_man.hasActiveContext() || !ctx_man.hasSectorLayer(name))
                continue;

            std::shared_ptr<SectorLayer> layer = ctx_man.sectorLayer(name);
            traced_assert(layer);

            sector_cache.setLayer(name, layer->sectors());
        }
        else if (!sector_cache.hasLayer(name))
        {
            continue;
        }

        //no non-exclusion sectors
        auto bounds = sector_cache.bounds(name);
        if (!bounds.has_value())
            continue;

        layers.push_back(Layer{name, bounds.value()});
    }

    return layers;
}

/**
 */
std::vector<std::string> SectorsFilter::availableLayers() const
{
    std::vector<std::string> names;

    if (filter_manager_)
    {
        auto& ctx_man = filter_manager_->dbContextManager();

        if (!ctx_man.hasActiveContext())
            return names;

        for (const auto& layer : ctx_man.sectorLayers())
            names.push_back(layer->name());
    }
    else if (own_cache_)
    {
        names = own_cache_->layerNames();
    }

    std::sort(names.begin(), names.end());

    return names;
}

/**
 */
SectorInsideCache& SectorsFilter::cache()
{
    if (filter_manager_)
        return filter_manager_->dbInterface().sectorInsideCache();

    if (!own_cache_)
        own_cache_ = std::make_shared<SectorInsideCache>();

    return *own_cache_;
}

/**
 */
DBFilterWidget* SectorsFilter::createWidget()
{
    //initial state of the widget, DBFilter::widget() applies it after creation
    if (filter_manager_)
        updateUsability();

    return new SectorsFilterWidget(*this);
}

/**
 */
void SectorsFilter::reset()
{
    updateWidget();
}

/**
 */
void SectorsFilter::updateWidget()
{
    if (widget_)
        widget_->update();
}

/**
 */
unsigned int SectorsFilter::removeLayersNotInContext()
{
    const std::vector<std::string> available = availableLayers();

    auto isAvailable = [&available](const std::string& name)
    {
        return std::find(available.begin(), available.end(), name) != available.end();
    };

    unsigned int num_removed = 0;

    for (const std::string& name : selectedLayers())
    {
        if (isAvailable(name))
            continue;

        loginf << "removing selected sector layer '" << name << "', not in the active context";

        selectLayer(name, false);
        ++num_removed;
    }

    //the prepared shapes of other contexts are not needed any more
    SectorInsideCache& sector_cache = cache();

    for (const std::string& name : sector_cache.layerNames())
    {
        if (!isAvailable(name))
            sector_cache.removeLayer(name);
    }

    updateUsability();
    updateWidget();

    return num_removed;
}

/**
 */
void SectorsFilter::saveViewPointConditions (nlohmann::json& filters)
{
    traced_assert(conditions_.size() == 0);

    traced_assert(!filters.contains(name_));
    filters[name_] = json::object();
    json& filter = filters.at(name_);

    filter[ViewPointKeyLayers]   = layers_;
    filter[ViewPointKeyUseDelta] = use_delta_;
    filter[ViewPointKeyDelta]    = delta_deg_;
}

/**
 */
void SectorsFilter::loadViewPointConditions (const nlohmann::json& filters)
{
    traced_assert(conditions_.size() == 0);

    traced_assert(filters.contains(name_));
    const json& filter = filters.at(name_);

    layers_ = json::array();

    if (filter.contains(ViewPointKeyLayers) && filter.at(ViewPointKeyLayers).is_array())
    {
        for (const auto& name : filter.at(ViewPointKeyLayers))
        {
            if (name.is_string())
                layers_.push_back(name.get<std::string>());
        }
    }

    use_delta_ = filter.contains(ViewPointKeyUseDelta) && filter.at(ViewPointKeyUseDelta).is_boolean()
                 ? filter.at(ViewPointKeyUseDelta).get<bool>() : false;

    if (filter.contains(ViewPointKeyDelta) && filter.at(ViewPointKeyDelta).is_number())
        delta_deg_ = filter.at(ViewPointKeyDelta).get<double>();

    //only the layers of the active context count, also updates the widget
    removeLayersNotInContext();
}

/**
 */
bool SectorsFilter::activeInLiveMode()
{
    return true;
}

/**
 */
std::vector<unsigned int> SectorsFilter::filterBuffer(const std::string& dbcontent_name,
                                                      std::shared_ptr<Buffer> buffer)
{
    std::vector<unsigned int> to_be_removed;

    if (!filters(dbcontent_name))
        return to_be_removed;

    std::vector<Layer> layers = activeLayers();

    //no constraint, as in the database clause
    if (layers.empty())
        return to_be_removed;

    const std::string lat_name = variableResolver().metaGetVariableName(dbcontent_name, dbcontent_vars::meta_var_latitude_);
    const std::string lon_name = variableResolver().metaGetVariableName(dbcontent_name, dbcontent_vars::meta_var_longitude_);

    if (!buffer->has<double>(lat_name) || !buffer->has<double>(lon_name))
    {
        logwrn << "dbcontent '" << dbcontent_name << "' buffer without position, not filtered";
        return to_be_removed;
    }

    const double delta_deg = effectiveDeltaDeg();

    std::vector<std::shared_ptr<const SectorInsideCache::PreparedLayer>> prepared;

    for (const auto& layer : layers)
    {
        auto prepared_layer = cache().prepared(layer.name, delta_deg);

        if (prepared_layer)
            prepared.push_back(prepared_layer);
    }

    const NullableVector<double>& lat_vec = buffer->get<double>(lat_name);
    const NullableVector<double>& lon_vec = buffer->get<double>(lon_name);

    //barometric altitude for the altitude limits of the sectors, without it the limits are not checked
    const NullableVector<float>* alt_vec = nullptr;

    if (variableResolver().metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_))
    {
        const std::string alt_name = variableResolver().metaGetVariableName(dbcontent_name, dbcontent_vars::meta_var_mc_);

        if (buffer->has<float>(alt_name))
            alt_vec = &buffer->get<float>(alt_name);
    }

    for (unsigned int cnt = 0; cnt < buffer->size(); ++cnt)
    {
        //a report without position is outside, as the null result of the database function
        if (lat_vec.isNull(cnt) || lon_vec.isNull(cnt))
        {
            to_be_removed.push_back(cnt);
            continue;
        }

        const double lat = lat_vec.get(cnt);
        const double lon = lon_vec.get(cnt);

        const bool   has_alt = alt_vec && !alt_vec->isNull(cnt);
        const double alt     = has_alt ? alt_vec->get(cnt) : 0.0;

        bool inside = false;

        for (const auto& prepared_layer : prepared)
        {
            if (prepared_layer->isInside(lat, lon, has_alt, alt))
            {
                inside = true;
                break;
            }
        }

        if (!inside)
            to_be_removed.push_back(cnt);
    }

    return to_be_removed;
}

/**
 */
std::vector<std::string> SectorsFilter::selectedLayers() const
{
    std::vector<std::string> names;

    for (const auto& name : layers_)
    {
        if (name.is_string())
            names.push_back(name.get<std::string>());
    }

    return names;
}

/**
 */
bool SectorsFilter::layerSelected(const std::string& layer_name) const
{
    for (const auto& name : layers_)
    {
        if (name.is_string() && name.get<std::string>() == layer_name)
            return true;
    }

    return false;
}

/**
 */
void SectorsFilter::selectLayer(const std::string& layer_name, bool select)
{
    if (select)
    {
        if (!layerSelected(layer_name))
            layers_.push_back(layer_name);

        return;
    }

    json remaining = json::array();

    for (const auto& name : layers_)
    {
        if (name.is_string() && name.get<std::string>() != layer_name)
            remaining.push_back(name);
    }

    layers_ = remaining;
}

bool SectorsFilter::useDelta() const
{
    return use_delta_;
}

void SectorsFilter::useDelta(bool use)
{
    use_delta_ = use;
}

double SectorsFilter::deltaDeg() const
{
    return delta_deg_;
}

void SectorsFilter::deltaDeg(double delta_deg)
{
    delta_deg_ = delta_deg;
}

double SectorsFilter::effectiveDeltaDeg() const
{
    return use_delta_ ? delta_deg_ : 0.0;
}
