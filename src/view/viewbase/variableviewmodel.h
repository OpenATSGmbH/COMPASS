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

#include "viewmodel.h"

#include <vector>

class ViewVariable;

/**
 * Base view model for variable-based views.
 * Implements the default compute steps for such a view: the display data comes from the annotation
 * if one is set, else from the view variables and the buffers.
 */
class VariableViewModel : public ViewModel
{
public:
    enum class VariableState
    {
        Ok = 0,
        MissingFromBuffer
    };

    VariableViewModel(DBContentManager& dbcont_man, context::DBContextManager& ctx_man, AppState& app_state);
    virtual ~VariableViewModel();

    /// the variables of the view, in the order of the view (NOT owned)
    void setVariables(const std::vector<ViewVariable*>& variables);
    size_t numVariables() const { return variables_.size(); }

    void clearAnnotation() override;
    bool hasAnnotation() const override final { return shows_annotation_; }

    VariableState variableState(int var_idx) const;
    bool variableOk(int var_idx) const;
    bool variablesOk() const;

protected:
    void updateFromSource_impl(bool reset, bool last) override final;
    void clearData_impl() override final;
    void recompute_impl() override final;

    /// the derived setAnnotation() has stored an annotation, the display data comes from it
    void setAnnotationActive() { annotation_active_ = true; }
    bool annotationActive() const { return annotation_active_; }

    ViewVariable& variable(int var_idx) const;
    void setVariableState(int var_idx, VariableState state);

    /// resets the counts, the variable states and the intermediate data, the compute phase starts with it
    void clearIntermediateData();

    /*to be implemented in derived classes*/

    /// called on buffer data update (on the load finalize)
    virtual void updateDataEvent(bool requires_reset) {}
    /// resets all variable related data collected during load (e.g. collected buffer data)
    virtual void resetVariableData() = 0;
    /// reset intermediate data collected during a recompute
    virtual void resetIntermediateVariableData() = 0;
    /// resets the display data
    virtual void resetDisplayData() = 0;
    /// Called on the main thread before the display data is computed from the variables. Returns true if
    /// it launched the computation on a worker: clearIntermediateData() and updateFromVariables() in the
    /// work function, publishDisplayData() in the commit.
    virtual bool launchAsyncRecompute() { return false; }
    /// computes the data of the display from the chosen variables and the buffers
    virtual void updateFromVariables() = 0;
    /// takes the display data from the stored annotation, false if there is no valid one
    virtual bool updateFromAnnotation() = 0;
    /// The computed data becomes the display data (main thread). The caller emits displayDataChangedSignal().
    virtual void publishDisplayData() = 0;

private:
    void resetVariableStates();

    std::vector<ViewVariable*> variables_;

    std::vector<VariableState> variable_states_;

    bool annotation_active_ = false; // an annotation is set, see setAnnotationActive()
    bool shows_annotation_  = false; // the display data shows the annotation
};
