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

#include "variableviewmodel.h"
#include "variableviewstash.h"
#include "nullablevector.h"
#include "property_templates.h"

#include <functional>
#include <limits>
#include <map>
#include <string>
#include <vector>

class Buffer;

/**
 * Base view model for variable-based views which stash their buffer data on a per-variable basis.
 * The stash also does a lot of counting. The derived class processes it into its display data.
 */
class VariableViewStashModel : public VariableViewModel
{
public:
    VariableViewStashModel(DBContentManager& dbcont_man,
                           context::DBContextManager& ctx_man,
                           AppState& app_state,
                           size_t num_variables,
                           bool group_per_datasource);
    virtual ~VariableViewStashModel();

    /// Tells if the view of the model waits for a reload. A variable that is not in the loaded buffers
    /// is then no error.
    void setReloadNeededFunc(const std::function<bool()>& func) { reload_needed_ = func; }

    /// number of loaded values, over all buffers
    unsigned int loadedDataCount() const;

    void viewInfoJSON(nlohmann::json& info) const override;

protected:
    void resetVariableData() override final;
    void resetIntermediateVariableData() override final;
    void updateFromVariables() override final;

    // Asynchronous recompute (offline), used by the load-done path and by interactive
    // recomputes: the stash fill and processing run on a worker (13+ s on the main thread for
    // large datasets otherwise), publishDisplayData() on the main thread once done.
    bool launchAsyncRecompute() override final;

    /// main thread, before the stash is filled and processed (on a worker or not): snapshot
    /// Qt-side state processStash() needs (e.g. the layer tree's hidden ids)
    virtual void prepareStashProcessing() {}
    /// Builds the data of the display from the stash. Must not touch Qt models or widgets, it may
    /// run on a worker thread (see launchAsyncRecompute()).
    virtual void processStash(const VariableViewStash<double>& stash) = 0;
    /// clear data computed in processStash()
    virtual void resetStashDependentData() = 0;

    /// Selects the values inside the range, add_to_selection keeps the selection from before.
    void selectData(double x_min,
                    double x_max,
                    double y_min,
                    double y_max,
                    int var_x,
                    int var_y,
                    bool add_to_selection,
                    bool correct_datetime = false);

    /// Derived hook: the layer panel id under which selected points are shown,
    /// for views that pool them into one overlay layer instead of leaving them
    /// in their own group layer. Empty if the view does not do that.
    virtual std::string selectionLayerId() const { return {}; }

    bool variableIsDateTime(int var_idx) const;

    const VariableViewStash<double>& getStash() const { return stash_; }

private:
    bool canUpdate(int var_idx, const std::string& dbcontent_name);
    bool canUpdate(const std::string& dbcontent_name);

    void resetStash();
    void updateStash();

    void updateVariableData(const std::string& dbcontent_name, Buffer& buffer);
    void updateVariableData(size_t var_idx,
                            std::string group_name, const Buffer& buffer,
                            const std::vector<unsigned int>& indexes);

    template<typename T>
    void appendData(const NullableVector<T>& data,
                    std::vector<double>& target,
                    std::vector<unsigned int> indexes)
    {
        bool ok;

        size_t num_nan_values = 0;

        for (unsigned int index : indexes)
        {
            if (data.isNull(index))
            {
                target.push_back(std::numeric_limits<double>::signaling_NaN());
            }
            else
            {
                auto v = property_templates::toDouble<T>(data.get(index), &ok);
                if (!ok)
                    throw std::runtime_error("VariableViewStashModel: appendData: data type not supported");

                target.push_back(v);

                if (!std::isfinite(v))
                    ++num_nan_values;
            }
        }

        addNanCount(num_nan_values);
    }

    VariableViewStash<double> stash_;
    bool group_per_datasource_ {false}; // true = DS ID + Line ID, false = DBContent
    std::map<std::string, unsigned int> last_buffer_size_; // dbcontent name -> last buffer size, only used if group_per_datasource_

    std::function<bool()> reload_needed_;
};
