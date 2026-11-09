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

#include "appstate.h"

#include "system.h"
#include "stringconv.h"
#include "logger.h"
#include "traced_assert.h"

#include <QApplication>
#include <QCursor>

#include <iostream>

const bool AppState::is_app_image_ = {Utils::System::appDir() != nullptr};

/**
 */
AppState::AppState()
{
    pdflatex_found_ = Utils::System::exec("which pdflatex").size(); // empty if none
}

/**
 */
AppState::~AppState() = default;

/**
 * Notice: emits appModeAboutToSwitchSignal, then appModeSwitchSignal.
 */
void AppState::appMode(AppMode app_mode)
{
    if (app_mode_ == app_mode)
        return;

    AppMode last_app_mode = app_mode_;

    app_mode_ = app_mode;

    loginf << "app_mode_current " << toString(app_mode_)
           << " previous " << toString(last_app_mode);

    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));

    // Live entry / pause / resume / exit all reduce to the same two steps: switch
    // ASTERIX ingestion (importer first, so no trailing inserts land after a pause),
    // then flip the display. Within a live session ViewManager keeps the LiveDataFeed as
    // the current source across running+paused, so pause = freeze and resume = unfreeze;
    // no reload or cache prime is needed - the feed already holds the data.
    // The importer is connected to appModeAboutToSwitchSignal => it runs before all
    // slots of appModeSwitchSignal.
    emit appModeAboutToSwitchSignal(last_app_mode, app_mode_);
    emit appModeSwitchSignal(last_app_mode, app_mode_);

    QApplication::restoreOverrideCursor();
}

/**
 */
std::string AppState::appModeStr() const
{
    if (!appModes2Strings().count(app_mode_))
    {
        std::cout << "COMPASS: appModeStr: unkown type " << (unsigned int) app_mode_ << std::endl;
        logerr << "unkown type " << (unsigned int) app_mode_;
    }

    traced_assert(appModes2Strings().count(app_mode_) > 0);
    return appModes2Strings().at(app_mode_);
}

/**
 */
const std::map<AppMode, std::string>& AppState::appModes2Strings()
{
    static const auto* map = new std::map<AppMode, std::string>
    {{AppMode::Offline, "Offline Mode"},
        {AppMode::LiveRunning, "Live Mode: Running"},
        {AppMode::LivePaused, "Live Mode: Paused"}};

    return *map;
}

/**
 */
void AppState::expertMode(bool expert_mode)
{
    loginf << "setting expert mode " << expert_mode;

    expert_mode_ = expert_mode;
}

/**
 */
void AppState::liveOverloaded(bool overloaded)
{
    if (live_overloaded_ == overloaded)
        return;

    live_overloaded_ = overloaded;

    loginf << "live overload " << (overloaded ? "detected" : "cleared");

    emit liveOverloadChangedSignal(overloaded);
}

/**
 */
void AppState::sensorStatusTimeHack(bool value)
{
    loginf << "value " << value;

    sensor_status_time_hack_ = value;
}

/**
 */
void AppState::darkMode(bool value)
{
    if (settings_.dark_mode == value)
        return;

    settings_.dark_mode = value;
    emit darkModeChangedSignal(settings_.dark_mode);
}

/**
 */
void AppState::colorMode(unsigned int value)
{
    if (settings_.color_mode == value)
        return;

    settings_.color_mode = value;
    emit colorModeChangedSignal(settings_.color_mode);
}

/**
 */
std::string AppState::lastUsedPath() const
{
    loginf << "return '" << settings_.last_path << "'";
    return settings_.last_path;
}

/**
 */
void AppState::lastUsedPath(const std::string& last_path)
{
    loginf << "set '" << last_path << "'";
    settings_.last_path = last_path;
}

/**
 */
const char* AppState::lineEditInvalidStyle() const
{
    if (settings_.dark_mode)
        return "QLineEdit { background: rgb(255, 50, 50); selection-background-color:"
                          " rgb(255, 100, 100); }";
    else
        return "QLineEdit { background: rgb(255, 100, 100); selection-background-color:"
                          " rgb(255, 200, 200); }";
}

/**
 */
bool AppState::isUnspecificACID(const std::string& acid) const
{
    // whitespace-only identifications carry no information
    if (Utils::String::trim(acid).empty())
        return true;

    // compare against the raw configured forms (e.g. "00000000", "????????")
    for (const auto& it : settings_.unspecific_acids)
    {
        if (it.is_string() && it.get<std::string>() == acid)
            return true;
    }

    return false;
}
