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

#include "variableviewdatawidget.h"
#include "variableview.h"
#include "viewvariable.h"

#include "property_templates.h"

#include "dbcontent/variable/variable.h"

/**
*/
VariableViewDataWidget::VariableViewDataWidget(ViewWidget* view_widget,
                                               VariableView* view,
                                               QWidget* parent,
                                               Qt::WindowFlags f)
:   ViewDataWidget(view_widget, parent, f)
,   variable_view_(view)
{
    traced_assert(variable_view_);

    //init states
    resetVariableStates();
}

/**
*/
VariableViewDataWidget::~VariableViewDataWidget()
{
    logdbg;
}

/**
*/
void VariableViewDataWidget::clearData_impl()
{
    logdbg;

    //reset everything
    resetVariableStates();
    resetVariableData();
    resetIntermediateVariableData();
    resetVariableDisplay();

    shows_annotations_ = false;

    logdbg << "end";
}

/**
*/
void VariableViewDataWidget::clearIntermediateRedrawData_impl()
{
    resetIntermediateVariableData(); // reset any intermediate data generated during redraw
    resetVariableStates();           // reset var states

    //every variable-based view should support these counts, so init them to zero
    addNullCount(0);
    addNanCount(0);

    shows_annotations_ = false;
}

/**
*/
void VariableViewDataWidget::loadingStarted_impl()
{
    logdbg;

    //nothing to do yet

    logdbg << "end";
}

/**
*/
void VariableViewDataWidget::updateData_impl(bool requires_reset)
{
    logdbg;

    //react on data update
    updateDataEvent(requires_reset);

    logdbg << "end";
}

/**
*/
void VariableViewDataWidget::loadingDone_impl()
{
    logdbg;

    //redraw already triggered by post load trigger? => return
    if (postLoadTrigger())
        return;

    //default behavior
    ViewDataWidget::loadingDone_impl();

    logdbg << "end";
}

/**
*/
ViewDataWidget::DrawState VariableViewDataWidget::redrawData_impl(bool recompute)
{
    logdbg << "recompute " << recompute
           << " shows anno " << variable_view_->showsAnnotation();

    //recompute data
    if (recompute)
    {   
        if (variable_view_->showsAnnotation())
        {
            //update from annotations
            shows_annotations_ = updateFromAnnotations();
        }
        else
        {
            //things to do before updating the variables
            preUpdateVariableDataEvent();

            //update from variables
            updateFromVariables();
        
            //things to do after updating the variables
            postUpdateVariableDataEvent();
        }
    }

    //update display
    auto draw_state = updateVariableDisplay();

    logdbg << "end";

    return draw_state;
}

/**
*/
void VariableViewDataWidget::liveReload_impl()
{
    logdbg;

    //@TODO: implement live reload for variable based views

    logdbg << "end";
}

/**
*/
bool VariableViewDataWidget::hasAnnotations_impl() const
{
    return shows_annotations_;
}

/**
*/
void VariableViewDataWidget::viewInfoJSON_impl(nlohmann::json& info) const
{
    //@TODO?
}

/**
*/
void VariableViewDataWidget::resetVariableStates()
{
    variable_states_.assign(variable_view_->numVariables(), VariableState::Ok);
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
            throw std::runtime_error("VariableViewDataWidget: CanUpdateFunc: impossible property type " + Property::asString(dtype));
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
bool VariableViewDataWidget::canUpdate(int var_idx, const std::string& dbcontent_name) const
{
    auto& variable = variable_view_->variable(var_idx);
    
    if (!viewData().count(dbcontent_name))
    {
        logdbg << "variable " << variable.id() << " dbcontent " << dbcontent_name << ": no buffer";
        return false;
    }

    Buffer* buffer = viewData().at(dbcontent_name).get();
    traced_assert(buffer);

    //allow empty variables if configured correctly
    if (variable.settings().allow_empty_var && variable.isEmpty())
        return true;

    auto data_var = variable.getFor(dbcontent_name);
    if (!data_var)
    {
        logdbg << "variable " << variable.id() << " dbcontent " << dbcontent_name << ": no metavar";
        return false;
    }

    PropertyDataType data_type        = data_var->dataType();
    std::string      current_var_name = data_var->name();

    CanUpdateFunc func(buffer, &variable, data_var);
    bool ok = property_templates::invokeFunctor(data_type, func);

    // !var should be in buffer if no reload is required!
    traced_assert(func.varInBuffer() || variable_view_->reloadNeeded());

    if (!func.varInBuffer())
        variable_states_.at(var_idx) = VariableState::MissingFromBuffer;

    if (!ok)
    {
        logdbg << "variable " << variable.id()
               << " dbcontent " << dbcontent_name << ": data var '" << current_var_name
               << "' not in buffer or not supported by view";
    }

    return ok;
}

/**
*/
bool VariableViewDataWidget::canUpdate(const std::string& dbcontent_name) const
{
    bool ok = true;
    for (size_t i = 0; i < variable_view_->numVariables(); ++i)
    {
        bool can_update = canUpdate(i, dbcontent_name);
        ok = ok && can_update;
    }

    return ok;
}

/**
*/
void VariableViewDataWidget::setVariableState(int var_idx, VariableState state)
{
    variable_states_.at(var_idx) = state;
}

/**
*/
VariableViewDataWidget::VariableState VariableViewDataWidget::variableState(int var_idx) const
{
    return variable_states_.at(var_idx);
}

/**
*/
bool VariableViewDataWidget::variableOk(int var_idx) const
{
    return (variableState(var_idx) == VariableState::Ok);
}

/**
*/
bool VariableViewDataWidget::variablesOk() const
{
    for (size_t i = 0; i < variable_view_->numVariables(); ++i)
        if (!variableOk((int)i))
            return false;

    return true;
}

/**
*/
boost::optional<PropertyDataType> VariableViewDataWidget::variableDataType(int var_idx) const
{
    if (!variable_view_->variable(var_idx).hasVariable())
        return {};

    return variable_view_->variable(var_idx).dataType();
}

/**
*/
bool VariableViewDataWidget::variableIsDateTime(int var_idx) const
{
    auto dtype = variableDataType(var_idx);
    return (dtype.has_value() ? dtype.value() == PropertyDataType::TIMESTAMP : false);
}

/**
 * Default behavior:
 *  - check if variables are part of dbcontent via canUpdate()
 *  - if yes trigger updateVariableData()
 * 
 * Override for custom behavior.
 */
void VariableViewDataWidget::updateFromVariables()
{
    logdbg;

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
}
