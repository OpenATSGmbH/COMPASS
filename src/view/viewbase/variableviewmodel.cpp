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

#include "variableviewmodel.h"
#include "viewvariable.h"
#include "logger.h"
#include "traced_assert.h"

/**
*/
VariableViewModel::VariableViewModel(DBContentManager& dbcont_man,
                                     context::DBContextManager& ctx_man,
                                     AppState& app_state)
:   ViewModel(dbcont_man, ctx_man, app_state)
{
}

/**
*/
VariableViewModel::~VariableViewModel() = default;

/**
*/
void VariableViewModel::setVariables(const std::vector<ViewVariable*>& variables)
{
    for (auto v : variables)
        traced_assert(v);

    variables_ = variables;

    resetVariableStates();
}

/**
*/
ViewVariable& VariableViewModel::variable(int var_idx) const
{
    return *variables_.at(var_idx);
}

/**
*/
void VariableViewModel::clearAnnotation()
{
    annotation_active_ = false;
}

/**
*/
void VariableViewModel::updateFromSource_impl(bool reset, bool last)
{
    // full refresh once the load completes (the buffers mirror the full source by now); the
    // per-content arrivals (last=false) only accumulate, no need to react each time
    if (last)
        updateDataEvent(reset);
}

/**
*/
void VariableViewModel::clearData_impl()
{
    //reset everything
    resetVariableStates();
    resetVariableData();
    resetIntermediateVariableData();
    resetDisplayData();

    shows_annotation_ = false;
}

/**
*/
void VariableViewModel::clearIntermediateData()
{
    resetCounts();

    resetIntermediateVariableData(); // reset any intermediate data generated during a recompute
    resetVariableStates();           // reset var states

    //every variable-based view should support these counts, so init them to zero
    addNullCount(0);
    addNanCount(0);

    shows_annotation_ = false;
}

/**
*/
void VariableViewModel::recompute_impl()
{
    logdbg << "annotation " << annotation_active_;

    if (annotation_active_)
    {
        clearIntermediateData();

        shows_annotation_ = updateFromAnnotation();

        emit displayDataChangedSignal();
        return;
    }

    //offline, the recompute runs on a worker with a main thread commit. the display
    //keeps showing the previous content until the commit. the launch declines (and
    //the synchronous path below runs) for live
    if (launchAsyncRecompute())
        return;

    clearIntermediateData();
    updateFromVariables();
    publishDisplayData();

    emit displayDataChangedSignal();
}

/**
*/
void VariableViewModel::resetVariableStates()
{
    variable_states_.assign(variables_.size(), VariableState::Ok);
}

/**
*/
void VariableViewModel::setVariableState(int var_idx, VariableState state)
{
    variable_states_.at(var_idx) = state;
}

/**
*/
VariableViewModel::VariableState VariableViewModel::variableState(int var_idx) const
{
    return variable_states_.at(var_idx);
}

/**
*/
bool VariableViewModel::variableOk(int var_idx) const
{
    return (variableState(var_idx) == VariableState::Ok);
}

/**
*/
bool VariableViewModel::variablesOk() const
{
    for (size_t i = 0; i < variables_.size(); ++i)
        if (!variableOk((int)i))
            return false;

    return true;
}
