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

#include "ui_test_cmd.h"
#include "ui_test_setget.h"
#include "ui_test_inject.h"
#include "ui_test_testable.h"
#include "rtcommand_registry.h"
//#include "json_tools.h"

#include <boost/program_options.hpp>

#include <QMainWindow>
#include <QDialog>

REGISTER_RTCOMMAND(ui_test::RTCommandUISet)
REGISTER_RTCOMMAND(ui_test::RTCommandUIGet)
REGISTER_RTCOMMAND(ui_test::RTCommandUIGetJSON)
REGISTER_RTCOMMAND(ui_test::RTCommandUIInject)
REGISTER_RTCOMMAND(ui_test::RTCommandUIRefresh)

namespace ui_test
{

/***************************************************************************************
 * RTCommandObject
 ***************************************************************************************/

/**
 */
void RTCommandUIObject::collectOptions_impl(OptionsDescription& options,
                                            PosOptionsDescription& positional)
{
    //add basic command options here
    ADD_RTCOMMAND_OPTIONS(options)
        ("object,o", po::value<std::string>()->default_value(""), "name of an ui element, object names separated by '.', e.g. 'mainwindow.window1.geographicview1.toolbar'");

    ADD_RTCOMMAND_POS_OPTION(positional, "object")
}

/**
 */
void RTCommandUIObject::assignVariables_impl(const VariablesMap& variables)
{
    RTCOMMAND_GET_QSTRING_OR_THROW(variables, "object", obj)
}

/*************************************************************************
 * RTCommandUIInjection
 *************************************************************************/

/**
 */
void RTCommandUIInjection::collectOptions_impl(OptionsDescription& options,
                                               PosOptionsDescription& positional)
{
    //call base
    RTCommandUIObject::collectOptions_impl(options, positional);

    ADD_RTCOMMAND_OPTIONS(options)
        ("uidelay", po::value<int>()->default_value(-1), "delay added after each injected ui event");
}

/**
 */
void RTCommandUIInjection::assignVariables_impl(const VariablesMap& variables)
{
    //call base
    RTCommandUIObject::assignVariables_impl(variables);

    RTCOMMAND_GET_VAR_OR_THROW(variables, "uidelay", int, injection_delay)
}

/*************************************************************************
 * RTCommandUISet
 *************************************************************************/

/**
 */
bool RTCommandUISet::run_impl()
{
    auto receiver = rtcommand::getCommandReceiverAs<QWidget>(obj.toStdString());
    if (receiver.first != rtcommand::FindObjectErrCode::NoError)
    {
        setResultMessage("Object '" + obj.toStdString() + "' not found");
        return false;
    }

    bool ok = setUIElement(receiver.second, "", value, injection_delay);
    if (!ok)
    {
        setResultMessage("Value '" + value.toStdString() + "' could not be set in object '" + obj.toStdString() + "'");
        return false;
    }

    return true;
}

/**
 */
void RTCommandUISet::collectOptions_impl(OptionsDescription& options,
                                         PosOptionsDescription& positional)
{
    //call base
    RTCommandUIInjection::collectOptions_impl(options, positional);

    ADD_RTCOMMAND_OPTIONS(options)
        ("value,v", po::value<std::string>()->default_value(""), "new value to set, content depending on the addressed ui element");

    ADD_RTCOMMAND_POS_OPTION(positional, "value")
}

/**
 */
void RTCommandUISet::assignVariables_impl(const VariablesMap& variables)
{
    //call base
    RTCommandUIInjection::assignVariables_impl(variables);

    RTCOMMAND_GET_QSTRING_OR_THROW(variables, "value", value)
}

/*************************************************************************
 * RTCommandUIGet
 *************************************************************************/

/**
 */
bool RTCommandUIGet::run_impl()
{
    auto receiver = rtcommand::getCommandReceiverAs<QWidget>(obj.toStdString());
    if (receiver.first != rtcommand::FindObjectErrCode::NoError)
    {
        setResultMessage("Object '" + obj.toStdString() + "' not found");
        return false;
    }

    boost::optional<nlohmann::json> result;
    std::string result_string;

    if (visible)
    {
        //retrieve ui element visibility
        nlohmann::json v;
        v[ "visible" ] = receiver.second->isVisible();

        result        = v;
        result_string = receiver.second->isVisible() ? "true" : "false";
    }
    else
    {
        if (as_json)
        {
            //retrieve ui element value as json
            auto res = getUIElementJSON(receiver.second, "", what);
            if (!res.is_null())
            {
                result = res;
                result_string = res.dump(4);
            }
        }
        else
        {
            //retrieve ui element value as string
            auto res = getUIElement(receiver.second, "", what);
            if (res.has_value())
            {
                result_string = res.value().toStdString();

                //nlohmann::json v;
                //v[ "value" ] = result_string;

                result = result_string;
            } 
        }
    }

    if (!result.has_value())
    {
        setResultMessage("Value could not be retrieved from object '" + obj.toStdString() + "'");
        return false;
    }
    
    setJSONReply(result.value(), result_string);

    return true;
}

/**
 */
void RTCommandUIGet::collectOptions_impl(OptionsDescription& options,
                                         PosOptionsDescription& positional)
{
    //call base
    RTCommandUIObject::collectOptions_impl(options, positional);

    ADD_RTCOMMAND_OPTIONS(options)
        ("what,w", po::value<std::string>()->default_value(""), "which value to retrieve from the ui element (empty = default behavior)")
        ("json", "if present, the result will be returned as a json struct instead of a string")
        ("visible", "if present, the visibility of the ui element will be returned");

    ADD_RTCOMMAND_POS_OPTION(positional, "what")
}

/**
 */
void RTCommandUIGet::assignVariables_impl(const VariablesMap& variables)
{
    //call base
    RTCommandUIObject::assignVariables_impl(variables);

    RTCOMMAND_GET_QSTRING_OR_THROW(variables, "what", what)
    RTCOMMAND_CHECK_VAR(variables, "json", as_json)
    RTCOMMAND_CHECK_VAR(variables, "visible", visible)
}

/*************************************************************************
 * RTCommandUIGetJSON
 *************************************************************************/

/**
 */
bool RTCommandUIGetJSON::run_impl()
{
    auto receiver = rtcommand::getCommandReceiverAs<QWidget>(obj.toStdString());
    if (receiver.first != rtcommand::FindObjectErrCode::NoError)
    {
        setResultMessage("Object '" + obj.toStdString() + "' not found");
        return false;
    }

    boost::optional<nlohmann::json> result;
    std::string result_string;

    if (visible)
    {
        //retrieve ui element visibility
        nlohmann::json v;
        v[ "visible" ] = receiver.second->isVisible();

        result        = v;
        result_string = receiver.second->isVisible() ? "true" : "false";
    }
    else
    {
        //retrieve ui element value as json
        auto res = getUIElementJSON(receiver.second, "", what);
        if (!res.is_null())
        {
            result = res;
            result_string = res.dump(4);
        }
    }

    if (!result.has_value())
    {
        setResultMessage("Value could not be retrieved from object '" + obj.toStdString() + "'");
        return false;
    }
    
    setJSONReply(result.value(), result_string);

    return true;
}

/**
 */
void RTCommandUIGetJSON::collectOptions_impl(OptionsDescription& options,
                                             PosOptionsDescription& positional)
{
    //call base
    RTCommandUIObject::collectOptions_impl(options, positional);

    ADD_RTCOMMAND_OPTIONS(options)
        ("what,w", po::value<std::string>()->default_value(""), "which value to retrieve from the ui element (empty = default behavior)")
        ("visible", "if present, the visibility of the ui element will be returned");

    ADD_RTCOMMAND_POS_OPTION(positional, "what")
}

/**
 */
void RTCommandUIGetJSON::assignVariables_impl(const VariablesMap& variables)
{
    //call base
    RTCommandUIObject::assignVariables_impl(variables);

    RTCOMMAND_GET_QSTRING_OR_THROW(variables, "what", what)
    RTCOMMAND_CHECK_VAR(variables, "visible", visible)
}

/*************************************************************************
 * RTCommandUIInject
 *************************************************************************/

/**
 * Inject UI events into UI objects, e.g.
 * uiinject mouse(left,click,50,40)  => inject a mouse click at (50,40)
 *          mouse(right,click,-1,-1) => inject a mouse click at the middle of the widget
 *          mouse(left,click,50%,50%) => inject a mouse click at the middle of the widget
 *          mouse(left,rect,0,0,100,100) => press the mouse at (0,0), move to (100,100) and release the mouse
 */
bool RTCommandUIInject::run_impl()
{
    auto receiver = rtcommand::getCommandReceiverAs<QWidget>(obj.toStdString());
    if (receiver.first != rtcommand::FindObjectErrCode::NoError)
    {
        setResultMessage("Object '" + obj.toStdString() + "' not found");
        return false;
    }

    bool ok = injectUIEvent(receiver.second, "", event, injection_delay);
    if (!ok)
    {
        setResultMessage("Event '" + event.toStdString() + "' could not be injected into object '" + obj.toStdString() + "'");
        return false;
    }

    return true;
}

/**
 */
void RTCommandUIInject::collectOptions_impl(OptionsDescription& options,
                                            PosOptionsDescription& positional)
{
    //call base
    RTCommandUIInjection::collectOptions_impl(options, positional);

    ADD_RTCOMMAND_OPTIONS(options)
        ("event,e", po::value<std::string>()->default_value(""), "event to inject into the ui element");

    ADD_RTCOMMAND_POS_OPTION(positional, "event")
}

/**
 */
void RTCommandUIInject::assignVariables_impl(const VariablesMap& variables)
{
    //call base
    RTCommandUIInjection::assignVariables_impl(variables);

    RTCOMMAND_GET_QSTRING_OR_THROW(variables, "event", event)
}

/*************************************************************************
 * RTCommandUIRefresh
 *************************************************************************/

/**
 * "Refreshes" the given UI element, whatever this means for the specific type of element.
 * @TODO: This is only implemented for classes derived from UITestable at the moment.
 */
bool RTCommandUIRefresh::run_impl()
{
    auto receiver = rtcommand::getCommandReceiverAs<QWidget>(obj.toStdString());
    if (receiver.first != rtcommand::FindObjectErrCode::NoError)
    {
        setResultMessage("Object '" + obj.toStdString() + "' not found");
        return false;
    }

    //check if object is UITestable
    auto testable = dynamic_cast<UITestable*>(receiver.second);
    if (!testable)
    {
        setResultMessage("Object '" + obj.toStdString() + "' is not ui testable");
        return false;
    }

    //refresh object
    testable->uiRefresh();

    return true;
}

}
