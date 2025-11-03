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

#include "variableviewdatawidget.h"
#include "variableviewstash.h"
#include "nullablevector.h"
#include "util/timeconv.h"
#include "property_templates.h"

#include <limits>

#include <boost/optional.hpp>

class Buffer;

/**
 * Base view data class for variable-based views which stash their buffer data on a per-variable basis.
 * This stash also does a lot of counting and can be processed later on to generate the final view data.
 */
class VariableViewStashDataWidget : public VariableViewDataWidget
{
public:
    VariableViewStashDataWidget(ViewWidget* view_widget,
                                VariableView* view,
                                bool group_per_datasource,
                                QWidget* parent = nullptr, 
                                Qt::WindowFlags f = Qt::WindowFlags());
    virtual ~VariableViewStashDataWidget();

    boost::optional<QRectF> getPlanarVariableBounds(int var_x, 
                                                    int var_y, 
                                                    bool correct_datetime,
                                                    bool fix_small_ranges) const;
    boost::optional<std::pair<double, double>> getVariableBounds(int var, 
                                                                 bool correct_datetime,
                                                                 bool fix_small_ranges) const;

    static const double RangeMinDefault;

protected:
    virtual void resetVariableData() override final;
    virtual void resetIntermediateVariableData() override final;
    virtual void preUpdateVariableDataEvent() override final;
    virtual void postUpdateVariableDataEvent() override final;
    virtual void updateVariableData(const std::string& dbcontent_name,
                                    Buffer& buffer) override final;

    virtual boost::optional<QRectF> getViewBounds() const;

    /// derived behavior during postUpdateVariableDataEvent()
    virtual void processStash(const VariableViewStash<double>& stash) = 0;
    /// clear data computed in processStash() 
    virtual void resetStashDependentData() = 0;

    void viewInfoJSON_impl(nlohmann::json& info) const override;

    void selectData (double x_min, 
                     double x_max, 
                     double y_min, 
                     double y_max,
                     int var_x,
                     int var_y,
                     bool correct_datetime = false);

    const VariableViewStash<double>& getStash() const { return stash_; }

private:
    void resetStash();
    void updateStash();

    void updateVariableData(size_t var_idx,
                            std::string group_name, const Buffer& buffer,
                            const std::vector<unsigned int>& indexes);

    template<typename T>
    void appendData(const NullableVector<T>& data,
                    std::vector<double>& target, 
                    unsigned int last_size,
                    unsigned int current_size)
    {
        bool ok;

        size_t num_nan_values = 0;

        for (unsigned int cnt=last_size; cnt < current_size; ++cnt)
        {
            if (data.isNull(cnt))
            {
                target.push_back(std::numeric_limits<double>::signaling_NaN());
            }
            else
            {
                auto v = property_templates::toDouble<T>(data.get(cnt), &ok);
                if (!ok)
                    throw std::runtime_error("VariableViewStashDataWidget: appendData: data type not supported");

                target.push_back(v);

                if (!std::isfinite(v))
                    ++num_nan_values;
            }
        }

        addNanCount(num_nan_values);
    }

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
                    throw std::runtime_error("VariableViewStashDataWidget: appendData: data type not supported");

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
};
