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

#include "variableviewstashmodel.h"
#include "appstate.h"
#include "viewvariable.h"
#include "viewasyncprocessor.h"
#include "viewlayertreemodel.h"

#include "buffer.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/metavariable.h"
#include "dbcontentmanager.h"
#include "db_context_manager.h"
#include "stringconv.h"
#include "number.h"
#include "timeconv.h"

#include "logger.h"
#include "property_templates.h"
#include "traced_assert.h"

#include "json.hpp"

#include <algorithm>
#include <set>

using namespace std;
using namespace Utils;

/**
*/
VariableViewStashModel::VariableViewStashModel(DBContentManager& dbcont_man,
                                               context::DBContextManager& ctx_man,
                                               AppState& app_state,
                                               size_t num_variables,
                                               bool group_per_datasource)
:   VariableViewModel    (dbcont_man, ctx_man, app_state)
,   stash_               (num_variables)
,   group_per_datasource_(group_per_datasource)
{
}

/**
*/
VariableViewStashModel::~VariableViewStashModel()
{
    //the stash workers capture this - join them before the members die
    shutdownAsyncProcessor();
}

/**
*/
unsigned int VariableViewStashModel::loadedDataCount() const
{
    unsigned int count = 0;

    for (const auto& buf_it : viewData())
        count += buf_it.second->size();

    return count;
}

/**
*/
void VariableViewStashModel::resetVariableData()
{
    //reset stash data
    resetStash();
}

/**
*/
void VariableViewStashModel::resetIntermediateVariableData()
{
    //clear the stash itself
    resetStash();

    //clear data generated from the stash during recomputation
    resetStashDependentData();
}

/**
 * The selection and the manager state read along the way are stable while the worker runs - the
 * load-done / refresh edge (and with it any user or runtime-command interaction) waits for the commit.
 */
bool VariableViewStashModel::launchAsyncRecompute()
{
    //main thread right before the stash is filled, asynchronous or not:
    //the worker must not read Qt-side state
    prepareStashProcessing();

    //live mutates the buffers per tick - synchronous recompute
    if (appState().appMode() == AppMode::LiveRunning)
        return false;

    if (viewData().empty())
        return false;

    asyncProcessor().launch(objectName().toStdString() + " stash update",
        [ this ] ()
        {
            clearIntermediateData();
            updateFromVariables();
        },
        [ this ] ()
        {
            publishDisplayData();

            emit displayDataChangedSignal();
        });

    return true;
}

/**
 * Fills the stash from the buffers and processes it. No Qt models/widgets touched, may run on a
 * worker thread (see launchAsyncRecompute()).
 */
void VariableViewStashModel::updateFromVariables()
{
    logdbg;

    traced_assert(numVariables() == stash_.dataRanges().size());

    for (const auto& buf_it : viewData())
    {
        const auto& dbcontent_name = buf_it.first;
        std::shared_ptr<Buffer> buffer = buf_it.second;
        traced_assert(buffer);

        bool can_update = canUpdate(dbcontent_name);

        logdbg << "dbcont " << dbcontent_name << " canUpdate " << can_update;

        if (can_update)
            updateVariableData(dbcontent_name, *buffer);
    }

    //update the stash (bounds, counts, etc.)
    updateStash();

    //check the stash integrity
    traced_assert(stash_.isValid());

    //invoke derived to process stash data
    processStash(stash_);
}

namespace
{
    /**
    */
    class CanUpdateFunc
    {
    public:
        CanUpdateFunc(const Buffer* buffer,
                      const ViewVariable* view_var,
                      const dbContent::Variable* var)
        :   buffer_       (buffer       )
        ,   view_var_     (view_var     )
        ,   var_          (var          )
        {
            traced_assert(buffer_);
            traced_assert(view_var_);
            traced_assert(var_);
        }
        ~CanUpdateFunc() = default;

        template <typename T, PropertyDataType dtype>
        bool operator ()()
        {
            //data type not supported by view?
            if (view_var_->settings().valid_data_types.count(dtype) == 0)
                return false;

            //variable in buffer?
            in_buffer_ = buffer_->has<T>(var_->name());

            return in_buffer_;
        }

        void error(PropertyDataType dtype)
        {
            logerr << "impossible for property type " << Property::asString(dtype);
            throw std::runtime_error("VariableViewStashModel: CanUpdateFunc: impossible property type " + Property::asString(dtype));
        }

        bool varInBuffer() const { return in_buffer_; }

    private:
        const Buffer*              buffer_;
        const ViewVariable*        view_var_;
        const dbContent::Variable* var_;

        bool in_buffer_ = false;
    };
}

/**
*/
bool VariableViewStashModel::canUpdate(int var_idx, const std::string& dbcontent_name)
{
    const ViewVariable& view_var = variable(var_idx);

    if (!viewData().count(dbcontent_name))
    {
        logdbg << "variable " << view_var.id() << " dbcontent " << dbcontent_name << ": no buffer";
        return false;
    }

    Buffer* buffer = viewData().at(dbcontent_name).get();
    traced_assert(buffer);

    //allow empty variables if configured correctly
    if (view_var.settings().allow_empty_var && view_var.isEmpty())
        return true;

    auto data_var = view_var.getFor(dbcontent_name);
    if (!data_var)
    {
        logdbg << "variable " << view_var.id() << " dbcontent " << dbcontent_name << ": no metavar";
        return false;
    }

    PropertyDataType data_type        = data_var->dataType();
    std::string      current_var_name = data_var->name();

    CanUpdateFunc func(buffer, &view_var, data_var);
    bool ok = property_templates::invokeFunctor(data_type, func);

    // !var should be in buffer if no reload is required!
    // engine loads guarantee this via the read set. Live feed buffers do not
    // (columns no record carried are absent), there the graceful branch applies
    traced_assert(func.varInBuffer() || (reload_needed_ && reload_needed_()) || !dataFulfillsReadSet());

    if (!func.varInBuffer())
        setVariableState(var_idx, VariableState::MissingFromBuffer);

    if (!ok)
    {
        logdbg << "variable " << view_var.id()
               << " dbcontent " << dbcontent_name << ": data var '" << current_var_name
               << "' not in buffer or not supported by view";
    }

    return ok;
}

/**
*/
bool VariableViewStashModel::canUpdate(const std::string& dbcontent_name)
{
    bool ok = true;
    for (size_t i = 0; i < numVariables(); ++i)
    {
        bool can_update = canUpdate((int)i, dbcontent_name);
        ok = ok && can_update;
    }

    return ok;
}

/**
*/
bool VariableViewStashModel::variableIsDateTime(int var_idx) const
{
    const ViewVariable& view_var = variable(var_idx);

    if (!view_var.hasVariable())
        return false;

    auto dtype = view_var.dataType();
    return (dtype.has_value() ? dtype.value() == PropertyDataType::TIMESTAMP : false);
}

/**
*/
void VariableViewStashModel::updateVariableData(const std::string& dbcontent_name,
                                                Buffer& buffer)
{
    loginf << "updating data for " << objectName().toStdString();

    for (size_t i = 0; i < numVariables(); ++i)
    {
        const ViewVariable& view_var = variable((int)i);

        //legit empty variables are handled in a special way, so do not return
        bool is_empty = view_var.settings().allow_empty_var && view_var.isEmpty();
        if (is_empty)
        {
            logdbg << "view_var " << view_var.id() << " empty";
            continue;
        }

        //otherwise return on invalid variable
        if (!view_var.getFor(dbcontent_name))
        {
            logdbg << "view_var " << view_var.variableName()
                   << " not existing in " << dbcontent_name << ", skipping";
            return;
        }
    }

    unsigned int last_size = last_buffer_size_[dbcontent_name];
    unsigned int current_size = buffer.size();

    map<string, vector<unsigned int>> grouped_indexes; // group name -> indexes
    std::string group_name;

    if (group_per_datasource_) // add by DS ID + Line ID
    {
        DBContentManager& dbcontent_man = dbContentManager();
        auto& ds_man = dbContextManager();

        traced_assert(buffer.has<unsigned int>(
            dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_ds_id_).name()));
        traced_assert(buffer.has<unsigned int>(
            dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_line_id_).name()));

        const NullableVector<unsigned int>& ds_ids = buffer.get<unsigned int>(
            dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_ds_id_).name());

        const NullableVector<unsigned int>& line_ids = buffer.get<unsigned int>(
            dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_line_id_).name());

        unsigned int ds_id, line_id;

        const NullableVector<unsigned char>* sensor_sacs {nullptr};
        const NullableVector<unsigned char>* sensor_sics {nullptr};

        if (dbcontent_name == "CAT063") // use sensor sec/sic special case
        {
            traced_assert(buffer.has<unsigned char>(
                dbcontent_man.getVariable(dbcontent_name, dbcontent_vars::var_cat063_sensor_sac_).name()));
            traced_assert(buffer.has<unsigned char>(
                dbcontent_man.getVariable(dbcontent_name, dbcontent_vars::var_cat063_sensor_sic_).name()));

            sensor_sacs = &buffer.get<unsigned char>(
                dbcontent_man.getVariable(dbcontent_name, dbcontent_vars::var_cat063_sensor_sac_).name());
            sensor_sics = &buffer.get<unsigned char>(
                dbcontent_man.getVariable(dbcontent_name, dbcontent_vars::var_cat063_sensor_sic_).name());
        }

        // The group of a row depends only on (ds_id, line_id), of which a dataset has a
        // handful - so resolve the data source and build the group name once per pair
        // and cache the target vector. Doing it per row cost a data source lookup, five
        // string allocations and a string keyed map probe for every one of millions of
        // rows. The map is node based, so the cached pointers stay valid across inserts.
        std::map<std::pair<unsigned int, unsigned int>, std::vector<unsigned int>*> group_cache;

        for (unsigned int index = last_size; index < current_size; ++index)
        {
            traced_assert(!ds_ids.isNull(index));
            traced_assert(!line_ids.isNull(index));

            ds_id = ds_ids.get(index);
            line_id = line_ids.get(index);

            if (sensor_sacs && sensor_sics)
            {
                if(!sensor_sacs->isNull(index) && !sensor_sics->isNull(index))
                    ds_id = Number::dsIdFrom(sensor_sacs->get(index), sensor_sics->get(index));
                else
                    continue;
            }

            const auto cache_key = std::make_pair(ds_id, line_id);

            auto cache_it = group_cache.find(cache_key);
            if (cache_it == group_cache.end())
            {
                std::string ds_type_str;
                std::string ds_name_str;

                if (ds_man.hasDataSource(ds_id))
                {
                    const auto* ds = ds_man.dataSource(ds_id);
                    ds_type_str = ds->dsType();
                    ds_name_str = ds->name();
                }
                else
                {
                    ds_type_str = "Other";
                    ds_name_str = to_string(Number::sacFromDsId(ds_id))+"/"+to_string(Number::sicFromDsId(ds_id));
                }

                group_name = ds_type_str + ":" + ds_name_str + ":"
                           + String::lineStrFrom(line_id) + ":" + dbcontent_name;

                cache_it = group_cache.emplace(cache_key, &grouped_indexes[group_name]).first;
            }

            cache_it->second->push_back(index);
        }
    }
    else // simple add
    {
        group_name = dbcontent_name;

        //single group: resolve it once instead of probing the map per row
        auto& indexes = grouped_indexes[dbcontent_name];

        if (current_size > last_size)
            indexes.reserve(indexes.size() + (current_size - last_size));

        for (unsigned int index = last_size; index < current_size; ++index)
            indexes.push_back(index);
    }

    // add selected flags & rec_nums
    traced_assert(buffer.has<bool>(dbcontent_vars::selected_var_.name()));
    traced_assert(buffer.has<std::uint64_t>(dbcontent_vars::meta_var_rec_num_.name()));

    const NullableVector<bool>&          selected_vec = buffer.get<bool>(dbcontent_vars::selected_var_.name());
    const NullableVector<std::uint64_t>& rec_num_vec  = buffer.get<std::uint64_t>(dbcontent_vars::meta_var_rec_num_.name());

    for (auto& group_it : grouped_indexes)
    {
        group_name = group_it.first;

        if (!group_it.second.size())
            continue;

        auto& group_stash = stash_.groupedDataStash(group_name);

        logdbg << "value counts before:";

        for (size_t i = 0; i < numVariables(); ++i)
            logdbg << "   " << variable((int)i).id() << " " << group_stash.variable_stashes[ i ].values.size();

        std::vector<bool>&          selected_data = group_stash.selected_values;
        std::vector<std::uint64_t>& rec_num_data  = group_stash.record_numbers;

        for (auto index : group_it.second)
        {
            if (selected_vec.isNull(index))
                selected_data.push_back(false);
            else
                selected_data.push_back(selected_vec.get(index));

            traced_assert(!rec_num_vec.isNull(index));
            rec_num_data.push_back(rec_num_vec.get(index));
        }

        //collect data for all variables
        for (size_t i = 0; i < numVariables(); ++i)
            updateVariableData(i, group_name, buffer, group_it.second);

        //check counts
        traced_assert(stash_.isValid());

        logdbg << "value counts after:";

        for (size_t i = 0; i < numVariables(); ++i)
            logdbg << "   " << variable((int)i).id() << " " << group_stash.variable_stashes[ i ].values.size();
    }

    last_buffer_size_[dbcontent_name] = current_size;
}

/**
*/
void VariableViewStashModel::updateVariableData(size_t var_idx,
                                                std::string group_name,
                                                const Buffer& buffer,
                                                const std::vector<unsigned int>& indexes)
{
    logdbg << "group_name "
           << group_name << " indexes size " << indexes.size();

    auto& group_stash = stash_.groupedDataStash(group_name);
    auto& var_stash   = group_stash.variable_stashes.at(var_idx);

    auto& buffer_counts = var_stash.count;
    auto& values        = var_stash.values;

    unsigned int last_size = buffer_counts;

    const ViewVariable& view_var = variable((int)var_idx);
    const dbContent::Variable* data_var = view_var.getFor(buffer.dbContentName());

    bool is_empty = view_var.settings().allow_empty_var && view_var.isEmpty();

    //handle special cases
    if (is_empty)
    {
        logdbg << "adding empty variable to stash";

        //empty variable selected => add zero values (valid values)
        //(a little bit hacky, but we do not want to count the values as Nan or NULL values)
        const double DefaultEmptyVarValue = 0.0;

        std::vector<double> vempty(indexes.size(), DefaultEmptyVarValue);
        values.insert(values.end(), vempty.begin(), vempty.end());

        return;
    }
    else if (!data_var)
    {
        //if not empty and no data var, something is fishy
        logwrn << "could not retrieve data var";
        return;
    }

    PropertyDataType data_type        = data_var->dataType();
    std::string      current_var_name = data_var->name();

    logdbg << "updating, last size " << last_size;

#define UpdateFunc(PDType, DType, Suffix)                                        \
        traced_assert(view_var.settings().valid_data_types.count(PDType) != 0);  \
        traced_assert(buffer.has<DType>(current_var_name));                      \
                                                                                 \
        const NullableVector<DType>& data = buffer.get<DType>(current_var_name); \
                                                                                 \
        appendData(data, values, indexes);

#define NotFoundFunc                                                                                     \
        logerr << "impossible for property type "                                                        \
        << Property::asString(data_type);                                                                \
        throw std::runtime_error("VariableViewStashModel: updateVariableData: impossible property type " \
                                 + Property::asString(data_type));

    SwitchPropertyDataType(data_type, UpdateFunc, NotFoundFunc)

#undef UpdateFunc
#undef NotFoundFunc

    logdbg << "updated size " << buffer_counts;
}

/**
*/
void VariableViewStashModel::resetStash()
{
    stash_.reset();
    last_buffer_size_.clear();
}

/**
*/
void VariableViewStashModel::updateStash()
{
    stash_.update();

    const auto& data_ranges = stash_.dataRanges();

    logdbg;

    for (size_t i = 0; i < numVariables(); ++i)
    {
        logdbg << "   " << variable((int)i).id() << " "
               << "has_min_max " << data_ranges[ i ].has_value() << " "
               << "x_min " << (data_ranges[ i ].has_value() ? std::to_string(data_ranges[ i ]->first) : "-") << " "
               << "x_max " << (data_ranges[ i ].has_value() ? std::to_string(data_ranges[ i ]->second) : "-");
    }
}

/**
 */
void VariableViewStashModel::selectData(double x_min,
                                        double x_max,
                                        double y_min,
                                        double y_max,
                                        int var_x,
                                        int var_y,
                                        bool add_to_selection,
                                        bool correct_datetime)
{
    loginf << "x_min " << x_min << " "
           << "x_max " << x_max << " "
           << "y_min " << y_min << " "
           << "y_max " << y_max << " "
           << "var_x " << var_x << " "
           << "var_y " << var_y << " "
           << "add to selection " << add_to_selection;

    if (correct_datetime)
    {
        if (variableIsDateTime(var_x))
        {
            x_min = Utils::Time::decorrectLongQtUTC((std::int64_t)x_min);
            x_max = Utils::Time::decorrectLongQtUTC((std::int64_t)x_max);
        }
        if (variableIsDateTime(var_y))
        {
            y_min = Utils::Time::decorrectLongQtUTC((std::int64_t)y_min);
            y_max = Utils::Time::decorrectLongQtUTC((std::int64_t)y_max);
        }
    }

    unsigned int sel_cnt = 0;
    string group_name;

    map<unsigned int, vector<std::uint64_t>> selected_rec_nums;

    // Points the user has hidden in the layer panel are not drawn, so they must
    // not be selectable either. The stored hidden ids are the stash group keys,
    // see the leaf payloads' persistenceId().
    auto* layer_model = layerPanelModel();

    const std::set<std::string> hidden_groups =
        layer_model ? layer_model->state().hiddenIds() : std::set<std::string>{};

    // Views that pool selected points into one overlay layer draw them under
    // that id, not under their group id. A selected point is therefore shown
    // when the overlay layer is shown, whatever its group's checkbox says. This
    // keeps a sub-selection inside the already selected data working while all
    // data layers are hidden.
    const std::string selection_layer_id = selectionLayerId();

    const bool has_selection_layer    = !selection_layer_id.empty();
    const bool selection_layer_hidden = has_selection_layer
                                     && hidden_groups.count(selection_layer_id) > 0;

    // collect all selected rec nums per dbcont id
    for (const auto& dbc_stash_it : getStash().groupedStashes())
    {
        group_name = dbc_stash_it.first;
        const auto& dbc_stash = dbc_stash_it.second;

        const bool group_hidden = hidden_groups.count(group_name) > 0;

        // nothing of this group is drawn
        if (group_hidden && (!has_selection_layer || selection_layer_hidden))
            continue;

        const std::vector<double>&        x_values       = dbc_stash.variable_stashes[ var_x ].values;
        const std::vector<double>&        y_values       = dbc_stash.variable_stashes[ var_y ].values;
        const std::vector<std::uint64_t>& rec_num_values = dbc_stash.record_numbers;

        traced_assert(x_values.size() == y_values.size());
        traced_assert(x_values.size() == rec_num_values.size());
        traced_assert(x_values.size() == dbc_stash.selected_values.size());

        double x, y;
        bool in_range;
        std::uint64_t rec_num;

        for (unsigned int cnt=0; cnt < x_values.size(); ++cnt)
        {
            // a selected point is drawn by the overlay layer, an unselected one
            // by its group layer
            const bool in_selection_layer = has_selection_layer && dbc_stash.selected_values[ cnt ];

            if (in_selection_layer ? selection_layer_hidden : group_hidden)
                continue;

            x = x_values.at(cnt);
            y = y_values.at(cnt);
            rec_num = rec_num_values.at(cnt);

            in_range = false;

            if (!std::isnan(x) && !std::isnan(y))
                in_range =  x >= x_min && x <= x_max && y >= y_min && y <= y_max;

            if (in_range)
            {
                selected_rec_nums[Number::recNumGetDBContId(rec_num)].push_back(rec_num);
                ++sel_cnt;
            }
        }
    }

    loginf << "values in range " << sel_cnt;
    sel_cnt = 0;

    DBContentManager& dbcont_man = dbContentManager();

    unsigned int index;

    if (!add_to_selection) // clear all selected
    {
        for (auto& buf_it : viewData())
        {
            traced_assert(buf_it.second->has<bool>(dbcontent_vars::selected_var_.name()));
            NullableVector<bool>& selected_vec = buf_it.second->get<bool>(dbcontent_vars::selected_var_.name());

            selected_vec.setAll(false);
        }
    }

    for (const auto& sel_cont_id_it : selected_rec_nums)
    {
        dbcont_man.existsDBContentWithId(sel_cont_id_it.first);

        string dbcontent_name = dbcont_man.dbContentWithId(sel_cont_id_it.first);

        traced_assert(viewData().count(dbcontent_name));
        auto buffer = viewData().at(dbcontent_name);

        traced_assert(buffer->has<bool>(dbcontent_vars::selected_var_.name()));
        NullableVector<bool>& selected_vec = buffer->get<bool>(dbcontent_vars::selected_var_.name());

        traced_assert(buffer->has<std::uint64_t>(dbcontent_vars::meta_var_rec_num_.name()));
        NullableVector<std::uint64_t>& rec_num_vec = buffer->get<std::uint64_t>(
            dbcontent_vars::meta_var_rec_num_.name());

        std::map<boost::optional<std::uint64_t>, std::vector<unsigned int>> rec_num_indexes =
            rec_num_vec.distinctValuesWithIndexes(0, rec_num_vec.contentSize());
        // rec_num -> index

        for (const auto& rec_num : sel_cont_id_it.second) // iterator over all selected rec_nums
        {
            traced_assert(rec_num_indexes.count(rec_num));
            std::vector<unsigned int>& indexes = rec_num_indexes.at(rec_num);
            traced_assert(indexes.size() == 1);

            index = indexes.at(0);

            selected_vec.set(index, true);

            ++sel_cnt;
        }
    }

    loginf << "values selected " << sel_cnt;

    emit selectionChangedSignal();
}

/**
 */
void VariableViewStashModel::viewInfoJSON(nlohmann::json& info) const
{
    info[ "num_valid"   ] = stash_.valid_count_;
    info[ "num_selected"] = stash_.selected_count_;
    info[ "num_nan"     ] = stash_.nan_value_count_;

    nlohmann::json input_value_info = nlohmann::json::array();

    for (const auto& it : stash_.groupedStashes())
    {
        nlohmann::json dbcont_info;

        dbcont_info[ "dbcont_name"       ] = it.first;

        for (size_t i = 0; i < numVariables(); ++i)
            dbcont_info[ "num_input_" + variable((int)i).id() ] = it.second.variable_stashes[ i ].values.size();

        dbcont_info[ "num_input_valid"] = it.second.valid_count;

        input_value_info.push_back(dbcont_info);
    }

    info[ "input_values" ] = input_value_info;
}
