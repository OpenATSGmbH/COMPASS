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

#include "appmode.h"

#include "json.hpp"

#include <QObject>

#include <map>
#include <string>

/**
 * State and global settings of the application.
 * - runtime state: phase, app mode, database opened, expert mode, user interactions, live overload
 * - settings, persisted as config parameters of COMPASS (registered there, config files unchanged)
 * - owned by COMPASS, which forwards its functions for the libraries above core
 */
class AppState : public QObject
{
    Q_OBJECT

signals:
    // emitted right before appModeSwitchSignal, for steps that must run before all its slots
    void appModeAboutToSwitchSignal (AppMode app_mode_previous, AppMode app_mode_current);
    void appModeSwitchSignal (AppMode app_mode_previous, AppMode app_mode_current);
    void liveOverloadChangedSignal(bool overloaded);
    void colorModeChangedSignal(unsigned int mode);
    void darkModeChangedSignal(bool dark);

public:
    /**
     * Persisted settings, registered as config parameters of COMPASS.
     */
    struct Settings
    {
        bool         dark_mode                              = false;
        unsigned int color_mode                             = 0;     // 0=DSType, 1=DBContent, 2=DataSource, 3=DataSourceLine
        std::string  last_path;                                      // last used directory in file dialogs

        bool         disable_live_to_offline_switch         = false;
        bool         disable_menu_config_save               = false;
        bool         disable_geographicview_rotate          = false;
        bool         disable_add_remove_views               = false;
        bool         disable_confirm_reset_views            = false;
        bool         disable_native_dialogs                 = false;

        unsigned int max_live_data_age_cache                = 5;     // minutes
        unsigned int max_live_data_age_db                   = 60;    // minutes

        unsigned int live_overload_max_packets              = 4;     // import backlog above which live ticks are held
        double       live_overload_max_latency_s            = -1.0;  // latency in seconds above which live ticks are held (<= 0 = off)
        bool         live_overload_show_debug_info          = false; // show backlog/latency in the geographic view (debug)

        unsigned int auto_live_running_resume_ask_time      = 60;    // minutes
        unsigned int auto_live_running_resume_ask_wait_time = 1;     // minutes

        unsigned int min_app_width                          = 1600;
        unsigned int min_app_height                         = 800;
        float        app_font_scale                         = 1.0;

        nlohmann::json unspecific_acids; // aircraft identifications carrying no information
    };

    AppState();
    virtual ~AppState();

    Settings& settings() { return settings_; }
    const Settings& settings() const { return settings_; }

    // phase
    AppPhase phase() const { return phase_; }
    void phase(AppPhase phase) { phase_ = phase; }
    bool isShutDown() const { return phase_ == AppPhase::Shutdown; }
    bool isRunning() const { return phase_ == AppPhase::Running; }

    // application mode
    AppMode appMode() const { return app_mode_; }
    void appMode(AppMode app_mode);
    std::string appModeStr() const;

    static const std::map<AppMode, std::string>& appModes2Strings();

    // database
    // set by COMPASS on opening/closing a database
    bool dbOpened() const { return db_opened_; }
    void dbOpened(bool opened) { db_opened_ = opened; }
    bool dbInMem() const { return db_inmem_; }
    void dbInMem(bool in_mem) { db_inmem_ = in_mem; }

    bool dbExportInProgress() const { return db_export_in_progress_; }
    void dbExportInProgress(bool value) { db_export_in_progress_ = value; }

    // interaction
    bool expertMode() const { return expert_mode_; }
    void expertMode(bool expert_mode);

    // false while automated processing (CLI / runtime commands) runs - code must not open
    // modal GUI dialogs when this is false, but resolve non-interactively instead
    bool allowUserInteractions() const { return allow_user_interactions_; }
    void allowUserInteractions(bool value) { allow_user_interactions_ = value; }

    // live overload: while set, live ticks are held (the views keep the last distributed data)
    bool liveOverloaded() const { return live_overloaded_; }
    void liveOverloaded(bool overloaded);

    // debug: force a live overload (runtime only, not persisted; no UI, set from code)
    bool liveOverloadForced() const { return live_overload_forced_; }
    void liveOverloadForced(bool forced) { live_overload_forced_ = forced; }

    bool sensorStatusTimeHack() const { return sensor_status_time_hack_; }
    void sensorStatusTimeHack(bool value);

    // settings with logic
    bool darkMode() const { return settings_.dark_mode; }
    void darkMode(bool value);

    unsigned int colorMode() const { return settings_.color_mode; }
    void colorMode(unsigned int value);

    std::string lastUsedPath() const;
    void lastUsedPath(const std::string& last_path);

    const char* lineEditInvalidStyle() const;

    bool isUnspecificACID(const std::string& acid) const;

    // environment
    static bool isAppImage() { return is_app_image_; }
    bool pdflatexFound() const { return pdflatex_found_; }

private:
    static const bool is_app_image_;

    Settings settings_;

    AppPhase phase_    = AppPhase::Starting;
    AppMode  app_mode_ = AppMode::Offline;

    bool db_opened_               = false;
    bool db_inmem_                = false;
    bool db_export_in_progress_   = false;
    bool expert_mode_             = false;
    bool allow_user_interactions_ = true;
    bool live_overloaded_         = false;
    bool live_overload_forced_    = false; // debug: live ticks held until switched off (not persisted)
    bool sensor_status_time_hack_ = false;
    bool pdflatex_found_          = false;
};
