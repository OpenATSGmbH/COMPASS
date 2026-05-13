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
#include "autoresumedialog.h"

#include <QMainWindow>

#include <memory>

#include "boost/date_time/posix_time/posix_time.hpp"

class QLabel;
class QPushButton;
class QTabWidget;
class QCheckBox;
class QMenu;
class QPushButton;
class QAction;
class QTimer;

class DBSelectionWidget;
class DBSchemaManagerWidget;
class DBContentManagerWidget;
class MainLoadWidget;
class ToolBox;

class COMPASS;

class MainWindow : public QMainWindow
{
    Q_OBJECT
signals:
    void dataLoaded();

public slots:
    void newDBSlot();
    void openExistingDBSlot();
    void openRecentDBSlot();
    void exportDBSlot();
    void clearExistingDBsSlot();
    void closeDBSlot();

    void saveConfigSlot();

    void quitWOConfigSlot();
    void quitSlot();

    void loadButtonSlot();
    void loadingStartedSlot();
    void loadingDoneSlot();

    void livePauseResumeSlot();
    void liveStopSlot();

    void configureDBContentSlot();

    void importAsterixRecordingSlot();
    void importRecentAsterixRecordingSlot();
    void importAsterixFromNetworkSlot();
    void importAsterixFromPCAPSlot();
    void importAsterixFromJSONSlot();
    void importJSONRecordingSlot();

    void importGPSTrailSlot();

    void importViewPointsSlot();

    void calculateRadarPlotPositionsSlot();
    void calculateAssociationsARTASSlot();
    void reconstructReferencesSlot();
    void evaluateSlot();
    void analyseMLATDataSourceSlot();

    void quitRequestedSlot();
    void showAddViewMenuSlot();

    void resetViewsMenuSlot();

    void manageLicensesSlot();

    void appModeSwitchSlot (AppMode app_mode_previous, AppMode app_mode_current);

    void autoResumeTimerSlot();
    void autoResumeResumeSlot();
    void autoResumeStaySlot();

    void toggleDarkModeSlot();

    void toggleFullscreenSlot();

    void updateMenuEnabledState();

public:
    explicit MainWindow(COMPASS& compass);
    virtual ~MainWindow();

    void init();

    void disableConfigurationSaving();

    void openExistingDB(const std::string& filename);
    void createDB(const std::string& filename);
    void createInMemoryDB(const std::string& future_filename = "");
    void createDBFromMemory();

    void updateMenus();
    void updateBottomWidget();

    void loadingStarted();
    void loadingDone();

    void selectTool(const std::string& name);
    void showResult(const std::string& name);

protected:
    void createUI();
    void createMenus();
    void createContextMenu();
    void updateContextMenuTitle();
    void createDebugMenu();

    void updateWindowTitle();

    /// @brief Called when application closes
    void closeEvent(QCloseEvent* event);

    void shutdown();

    void showEvaluationResult();

    QWidget* main_widget_{nullptr};
    QTabWidget* tab_widget_{nullptr};

    QPushButton* add_view_button_{nullptr};

    bool save_configuration_{true};

    // menu

    // file menu
    QAction* new_db_action_ {nullptr};
    QAction* open_existing_db_action_ {nullptr};
    QMenu* open_recent_db_menu_ {nullptr};
    QAction* export_db_action_ {nullptr};
    QAction* close_db_action_ {nullptr};
    QAction* save_config_action_ {nullptr};
    QAction* quit_wo_cfg_sav_action_ {nullptr};

    // context menu
    QMenu* context_menu_ {nullptr};
    QMenu* context_switch_menu_ {nullptr};
    QAction* context_edit_action_ {nullptr};
    QAction* context_compare_action_ {nullptr};
    QAction* context_new_action_ {nullptr};
    QAction* context_copy_action_ {nullptr};
    QAction* context_delete_action_ {nullptr};

    // import menu
    QMenu* import_menu_ {nullptr};

    // configuration menu
    QMenu* config_menu_ {nullptr};
    QAction* license_action_ {nullptr};
    QAction* dark_mode_action_ {nullptr};
    QAction* fullscreen_action_ {nullptr};
    QAction* auto_refresh_views_action_ {nullptr};
    QAction* dbcontent_action_ {nullptr};

    // process menu
    QMenu* process_menu_ {nullptr};
    QMenu* analyze_menu_ {nullptr};
    //QAction* calculate_references_action_ {nullptr};

    // ui menu
    QMenu* ui_menu_ {nullptr};

    bool loading_ {false};
    boost::posix_time::ptime loading_start_time_;

    QLabel* db_label_ {nullptr};
    QLabel* status_label_ {nullptr};
    QPushButton* load_button_ {nullptr};

    QPushButton* live_pause_resume_button_ {nullptr};
    QPushButton* live_stop_button_ {nullptr}; // optional button, may be nullptr

    std::unique_ptr<AutoResumeDialog> auto_resume_dialog_;
    QTimer* auto_resume_timer_ {nullptr};

    ToolBox* tool_box_ = nullptr;

private:
    void showCommandShell();

    COMPASS& compass_;

    unsigned int tool_ds_;
    unsigned int tool_sstatus_;
    unsigned int tool_filters_;
    unsigned int tool_targets_;
    unsigned int tool_reports_;
    unsigned int tool_vp_;
    unsigned int tool_log_;
};

