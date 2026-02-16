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

#include "rtcommand.h"
#include "rtcommand_macros.h"

namespace ui_test
{

/**
 * Command targeting a specific QObject.
 */
struct RTCommandUIObject : public rtcommand::RTCommand
{
    QString obj;

    virtual rtcommand::IsValid valid() const override 
    {
        CHECK_RTCOMMAND_INVALID_CONDITION(obj.isEmpty(), "Object path must not be empty")
        
        return rtcommand::RTCommand::valid(); 
    }
    
    DECLARE_RTCOMMAND_OPTIONS
};

/** 
 */
struct RTCommandUIInjection : public RTCommandUIObject
{
    int injection_delay = -1; //delay used for each UI injection
    
    DECLARE_RTCOMMAND_OPTIONS
};

/**
 */
struct RTCommandUISet : public RTCommandUIInjection
{
    QString value;
protected:
    virtual bool run_impl() override;

    DECLARE_RTCOMMAND(uiset, "sets an ui element to the given value")
    DECLARE_RTCOMMAND_OPTIONS
};

/**
 */
struct RTCommandUIGet : public RTCommandUIObject
{
    QString what;
    bool    as_json = false;
    bool    visible = false;
protected:
    virtual bool run_impl() override;

    DECLARE_RTCOMMAND(uiget, "retrieves the value of the given ui element")
    DECLARE_RTCOMMAND_OPTIONS
};

/**
 */
struct RTCommandUIGetJSON : public RTCommandUIObject
{
    QString what;
    bool    visible = false;
protected:
    virtual bool run_impl() override;

    DECLARE_RTCOMMAND(uiget_json, "retrieves the value of the given ui element as json")
    DECLARE_RTCOMMAND_OPTIONS
};

/**
 */
struct RTCommandUIInject : public RTCommandUIInjection
{
    QString event;
protected:
    virtual bool run_impl() override;

    DECLARE_RTCOMMAND(uiinject, "injects an event into the given ui element")
    DECLARE_RTCOMMAND_OPTIONS
};

/**
 */
struct RTCommandUIRefresh : public RTCommandUIObject
{
protected:
    virtual bool run_impl() override;

    DECLARE_RTCOMMAND(uirefresh, "refreshes the given ui element")
};

/**
*/
inline void initUITestCommands()
{
    RTCommandUISet::init();
    RTCommandUIGet::init();
    RTCommandUIGetJSON::init();
    RTCommandUIInject::init();
    RTCommandUIRefresh::init();
}

} // namespace ui_test
