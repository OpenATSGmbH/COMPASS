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

#include "viewdatawidget.h"
#include "property.h"

#include <map>
#include <vector>

class ViewWidget;
class VariableView;
class Buffer;

/**
 * Base view data class for variable-based views.
 * Implements default load and update behavior for such a view.
 */
class VariableViewDataWidget : public ViewDataWidget
{
public:
    enum class VariableState
    {
        Ok = 0,
        MissingFromBuffer
    };

    VariableViewDataWidget(ViewWidget* view_widget,
                           VariableView* view,
                           QWidget* parent = nullptr, 
                           Qt::WindowFlags f = Qt::WindowFlags());
    virtual ~VariableViewDataWidget();

    bool canUpdate(int var_idx, const std::string& dbcontent_name) const;
    bool canUpdate(const std::string& dbcontent_name) const;

    VariableState variableState(int var_idx) const;
    bool variableOk(int var_idx) const;
    bool variablesOk() const;

    boost::optional<PropertyDataType> variableDataType(int var_idx) const;
    bool variableIsDateTime(int var_idx) const;

protected:
    virtual void loadingStarted_impl() override final;
    virtual void loadingDone_impl() override final;
    virtual void updateData_impl(bool requires_reset) override final;
    virtual void clearData_impl() override final;
    virtual void clearIntermediateRedrawData_impl() override final;
    virtual DrawState redrawData_impl(bool recompute) override final;
    virtual void liveReload_impl() override final;
    virtual bool hasAnnotations_impl() const override final;

    void viewInfoJSON_impl(nlohmann::json& info) const override;

    void setVariableState(int var_idx, VariableState state);

    /*to be implemented in derived classes*/

    /// called on buffer data update (updateData_impl)
    virtual void updateDataEvent(bool requires_reset) {}
    /// called after loading buffers, should return true if it already triggered a redraw (e.g. by reconfiguring the view)
    virtual bool postLoadTrigger() { return false; }
    /// resets all variable related data collected during load (e.g. collected buffer data)
    virtual void resetVariableData() = 0;
    /// reset intermediate data collected during redraw (e.g. intermediate datastructures used for display purpose)
    virtual void resetIntermediateVariableData() = 0;
    /// resets all display related data (e.g. a chart)
    virtual void resetVariableDisplay() = 0;
    /// called before updating from the variables
    virtual void preUpdateVariableDataEvent() = 0;
    /// creates view data from the chosen variables (only needs override if the data is not updated on a per dbcontent-basis)
    virtual void updateFromVariables();
    /// creates view data from the chosen variables for a specific dbcontent and its respective buffer
    virtual void updateVariableData(const std::string& dbcontent_name, Buffer& buffer) {}
    /// creates view data from annotations collected in View
    virtual bool updateFromAnnotations() { return false; }
    /// called after updating from the variables
    virtual void postUpdateVariableDataEvent() = 0;
    /// updates the display (e.g. by updating a chart showing the data)
    virtual DrawState updateVariableDisplay() = 0;

    const VariableView* variableView() const { return variable_view_; }
    VariableView* variableView() { return variable_view_; }

private:
    void resetVariableStates();
    
    VariableView* variable_view_ = nullptr;

    mutable std::vector<VariableState> variable_states_;

    bool shows_annotations_ = false;
};
