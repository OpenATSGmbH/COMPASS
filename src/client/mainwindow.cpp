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

#include "mainwindow.h"
#include "mainwindow_commands.h"
#include "compass.h"
#include "configurationmanager.h"
#include "datasourcemanager.h"
#include "datasourcesconfigurationdialog.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/target/targetlistwidget.h"
#include "datasourcestoolwidget.h"
#include "dbcontent/variable/metavariableconfigurationdialog.h"
#include "files.h"
#include "filtermanager.h"
#include "filtermanagerwidget.h"
#include "global.h"
#include "logger.h"
#include "taskmanager.h"
#include "viewmanager.h"
#include "viewpointsimporttask.h"
#include "viewpointsimporttaskdialog.h"
#include "gpstrailimporttask.h"
#include "gpstrailimporttaskdialog.h"
#include "managesectorstask.h"
#include "managesectorstaskdialog.h"
#include "evaluationmanager.h"
#include "compass.h"
#include "fftmanager.h"
#include "fftsconfigurationdialog.h"
#include "ui_test_common.h"
#include "ui_test_cmd.h"
#include "rtcommand_shell.h"
#include "licensemanagerdialog.h"
#include "licensemanager.h"
#include "toolbox.h"
#include "toolboxwidget.h"
#include "viewpointswidget.h"
#include "taskresultswidget.h"

#include "asteriximporttask.h"
#include "asteriximporttaskdialog.h"
#include "jsonimporttask.h"
#include "jsonimporttaskdialog.h"

#include "radarplotpositioncalculatortask.h"
#include "radarplotpositioncalculatortaskdialog.h"
#include "createartasassociationstask.h"
#include "createartasassociationstaskdialog.h"
#include "reconstructortask.h"
#include "reconstructortaskdialog.h"
#include "util/async.h"

#include "logwidget.h"

#if USE_EXPERIMENTAL_SOURCE == true
#include "geometrytreeitem.h"
#include "test_lab.h"
#include "extra_commands.h"
#endif

#include <QApplication>
#include <QFileDialog>
#include <QCloseEvent>
#include <QSettings>
#include <QStackedWidget>
#include <QTabWidget>
#include <QLocale>
#include <QMessageBox>
#include <QPushButton>
#include <QMenu>
#include <QMenuBar>
#include <QThread>
#include <QVBoxLayout>
#include <QLabel>
#include <QCheckBox>
#include <QTimer>
#include <QShortcut>

using namespace Utils;
using namespace std;

MainWindow::MainWindow()
{
    logdbg;

    QLocale::setDefault(QLocale::c());
    setLocale(QLocale::c());

    const char* appdir = getenv("APPDIR");
    if (appdir && COMPASS::instance().disableNativeDialogs())
        QCoreApplication::setAttribute(Qt::AA_DontUseNativeDialogs); // disable native

    setMinimumSize(QSize(COMPASS::instance().minAppWidth(), COMPASS::instance().minAppHeight()));

    QIcon ats_icon(Files::IconProvider::getIcon("ats.png"));
    setWindowIcon(ats_icon);  // for the glory of the empire

    QSettings settings("COMPASS", "Client");
    restoreGeometry(settings.value("MainWindow/geometry").toByteArray());

    // if (settings.value("MainWindow/isMaximized", false).toBool())
    //     showMaximized();

    if (settings.value("MainWindow/isFullScreen", false).toBool())
    {
        // Enter fullscreen with no decorations
        //setWindowFlag(Qt::FramelessWindowHint, true);
        setWindowFlags(Qt::FramelessWindowHint);
        showMaximized();
        showFullScreen(); // Re-show to apply
    }
    else
    {
        // No fullscreen: restore window flags and normal size
        //setWindowFlag(Qt::FramelessWindowHint, false); // Remove frameless
        setWindowFlags(Qt::Window);
        //setWindowFlag(Qt::Window, true);               // Ensure normal window flags

        // if (settings.value("MainWindow/isMaximized", false).toBool())
        // {
        //     loginf << "isMaximized";
        //     showMaximized();
        // }
        // else
        {
            restoreGeometry(settings.value("MainWindow/geometry").toByteArray());
            showNormal();
        }
    }

    //create ui
    createUI();

    //init ui related commands
    ui_test::initUITestCommands();

    //init extra commands
#if USE_EXPERIMENTAL_SOURCE == true
    if (!COMPASS::instance().isAppImage())
        extra::init_extra_commands();
#endif

    main_window::init_commands();
}

MainWindow::~MainWindow()
{
    logdbg;

            // remember: this not called! insert deletes into closeEvent function
}

void MainWindow::createUI()
{
    main_widget_ = new QWidget();

    QVBoxLayout* main_layout = new QVBoxLayout();
    main_layout->setContentsMargins(0, 0, 0, 0);
    main_widget_->setLayout(main_layout);

    QHBoxLayout* content_layout = new QHBoxLayout;
    content_layout->setContentsMargins(0, 0, 0, 0);
    content_layout->setSpacing(0);

    main_layout->addLayout(content_layout);

    // initialize tabs
    tab_widget_ = new QTabWidget();
    tab_widget_->setObjectName("container0");

    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));

    //tab_widget_->setCurrentIndex(0);

    QApplication::restoreOverrideCursor();

    const QString tool_tip = "Add view";

    add_view_button_ = new QPushButton();
    UI_TEST_OBJ_NAME(add_view_button_, tool_tip);
    add_view_button_->setIcon(Files::IconProvider::getIcon("crosshair_fat.png"));
    add_view_button_->setFixedSize(UI_ICON_SIZE);
    add_view_button_->setFlat(UI_ICON_BUTTON_FLAT);
    add_view_button_->setToolTip(tr(tool_tip.toStdString().c_str()));
    add_view_button_->setDisabled(COMPASS::instance().disableAddRemoveViews());

    connect(add_view_button_, &QPushButton::clicked, this, &MainWindow::showAddViewMenuSlot);
    tab_widget_->setCornerWidget(add_view_button_);

    //init some components before using their widgets
    COMPASS::instance().evaluationManager().init();
    COMPASS::instance().viewManager().init(tab_widget_);

    // initialize toolbox
    tool_box_ = new ToolBox(this);
    
    tool_box_->addTool(COMPASS::instance().dataSourceManager().loadWidget()); // 0
    tool_box_->addTool(COMPASS::instance().filterManager().widget()); // 1
    tool_box_->addTool(COMPASS::instance().dbContentManager().targetListWidget()); // 2
    tool_box_->addTool(COMPASS::instance().taskManager().widget()); // 3
    tool_box_->addTool(COMPASS::instance().viewManager().viewPointsWidget()); // 4
    tool_box_->addTool(new LogWidget(COMPASS::instance().logStore())); // 5

    //@TODO: !handle filter check box!
    //QTabBar *tabBar = tab_widget_->tabBar();
    //tabBar->setTabButton(1, QTabBar::LeftSide, COMPASS::instance().filterManager().widget()->filtersCheckBox());
    //tabBar->setTabButton(0, QTabBar::RightSide, new QLabel("label0");

    tool_box_->setMainContent(tab_widget_);
    tool_box_->finalize();
    tool_box_->selectTool("Data Sources");

    // add toolbox and view tab widget
    content_layout->addWidget(tool_box_);

    // bottom widget
    QWidget* bottom_widget = new QWidget();
    bottom_widget->setMaximumHeight(40);

    QHBoxLayout* bottom_layout = new QHBoxLayout();
    bottom_layout->setContentsMargins(2, 2, 2, 2);

    db_label_ = new QLabel();
    bottom_layout->addWidget(db_label_);

    bottom_layout->addStretch();

    // add status & button
    status_label_ = new QLabel();
    bottom_layout->addWidget(status_label_);

    live_pause_resume_button_ = new QPushButton("Pause");
    live_pause_resume_button_->setObjectName("livebutton");
    connect(live_pause_resume_button_, &QPushButton::clicked, this, &MainWindow::livePauseResumeSlot);
    bottom_layout->addWidget(live_pause_resume_button_);

    if (!COMPASS::instance().disableLiveToOfflineSwitch())
    {
        live_stop_button_ = new QPushButton("Stop");
        connect(live_stop_button_, &QPushButton::clicked, this, &MainWindow::liveStopSlot);

        bottom_layout->addWidget(live_stop_button_);
    }

    bottom_layout->addStretch();

    // load button
    load_button_ = new QPushButton("Load");
    connect(load_button_, &QPushButton::clicked, this, &MainWindow::loadButtonSlot);
    bottom_layout->addWidget(load_button_);

    load_button_->setObjectName("reload");

    bottom_widget->setLayout(bottom_layout);

    updateBottomWidget();

    main_layout->addWidget(bottom_widget);

    setCentralWidget(main_widget_);

    // create menus
    createMenus();
    updateMenus();

    tool_box_->adjustSizings();

    updateWindowTitle();

    // connect signal slots
    connect (&COMPASS::instance(), &COMPASS::appModeSwitchSignal,
            this, &MainWindow::appModeSwitchSlot);

    connect(&COMPASS::instance().dbContentManager(), &DBContentManager::loadingDoneSignal,
            this, &MainWindow::loadingDoneSlot);
    connect (&COMPASS::instance().dbContentManager(), &DBContentManager::associationStatusChangedSignal,
            this, &MainWindow::updateMenus);

    connect(&COMPASS::instance().licenseManager(), &LicenseManager::changed,
            this, &MainWindow::updateWindowTitle);

    connect(&COMPASS::instance().evaluationManager(), &EvaluationManager::evaluationDoneSignal,
            this, &MainWindow::showEvaluationResult);
}

void MainWindow::createMenus ()
{
    bool expert_mode = COMPASS::instance().expertMode();

    menuBar()->setObjectName("mainmenu");

    // file menu
    QMenu* file_menu = menuBar()->addMenu("&File");
    file_menu->setObjectName("main_window_file_menu");
    file_menu->setToolTipsVisible(true);

    // db operations
    new_db_action_ = new QAction("&New");
    new_db_action_->setShortcuts(QKeySequence::New);
    new_db_action_->setToolTip("Create a new database");
    connect(new_db_action_, &QAction::triggered, this, &MainWindow::newDBSlot);
    file_menu->addAction(new_db_action_);

    open_existing_db_action_ = new QAction("&Open");
    open_existing_db_action_->setShortcuts(QKeySequence::Open);
    open_existing_db_action_->setToolTip("Open an existing database");
    connect(open_existing_db_action_, &QAction::triggered, this, &MainWindow::openExistingDBSlot);
    file_menu->addAction(open_existing_db_action_);

    open_recent_db_menu_ = file_menu->addMenu("Open Recent");
    open_recent_db_menu_->setToolTip("Open a recent database");

    open_recent_db_menu_->addSeparator();

    export_db_action_ = new QAction("&Export", file_menu);
    export_db_action_->setObjectName("main_window_exportdb_action");
    export_db_action_->setToolTip("Export database into file");
    connect(export_db_action_, &QAction::triggered, this, &MainWindow::exportDBSlot);
    file_menu->addAction(export_db_action_);

    QAction* clear_act = new QAction("Clear");
    connect(clear_act, &QAction::triggered, this, &MainWindow::clearExistingDBsSlot);
    open_recent_db_menu_->addAction(clear_act);

    close_db_action_ = new QAction("&Close");
    close_db_action_->setToolTip("Close opened database");
    connect(close_db_action_, &QAction::triggered, this, &MainWindow::closeDBSlot);
    file_menu->addAction(close_db_action_);

    file_menu->addSeparator();

    // config operations

    if (!COMPASS::instance().disableMenuConfigSave())
    {
        QAction* save_act = new QAction("&Save Config");
        save_act->setShortcut(tr("Ctrl+S"));
        connect(save_act, &QAction::triggered, this, &MainWindow::saveConfigSlot);
        file_menu->addAction(save_act);

        file_menu->addSeparator();
    }

    // quit operations
    quit_wo_cfg_sav_action_ = new QAction("Quit &Without Saving Config");
    quit_wo_cfg_sav_action_->setShortcut(tr("Ctrl+W"));
    quit_wo_cfg_sav_action_->setToolTip("Quit the application withour saving the configuration");
    connect(quit_wo_cfg_sav_action_, &QAction::triggered, this, &MainWindow::quitWOConfigSlot);
    file_menu->addAction(quit_wo_cfg_sav_action_);

    QAction* quit_act = new QAction("&Quit");
    quit_act->setShortcuts(QKeySequence::Quit);
    //QKeySequence("Ctrl+P");
    quit_act->setToolTip("Quit the application");
    connect(quit_act, &QAction::triggered, this, &MainWindow::quitSlot);
    file_menu->addAction(quit_act);

    // import menu

    import_menu_ = menuBar()->addMenu("&Import");
    import_menu_->setToolTipsVisible(true);

    QAction* import_ast_file_action = new QAction("&ASTERIX Recording");
    import_ast_file_action->setShortcut(tr("Ctrl+A"));
    import_ast_file_action->setToolTip("Import ASTERIX Recording File");
    connect(import_ast_file_action, &QAction::triggered, this, &MainWindow::importAsterixRecordingSlot);
    import_menu_->addAction(import_ast_file_action);

    QAction* import_pcap_file_action = new QAction("&PCAP ASTERIX Recording");
    import_pcap_file_action->setShortcut(tr("Ctrl+P"));
    import_pcap_file_action->setToolTip("Import PCAP Recording File");
    connect(import_pcap_file_action, &QAction::triggered, this, &MainWindow::importAsterixFromPCAPSlot);
    import_menu_->addAction(import_pcap_file_action);

    QAction* import_ast_net_action = new QAction("ASTERIX From Network");
    import_ast_net_action->setToolTip("Import ASTERIX From Network");
    connect(import_ast_net_action, &QAction::triggered, this, &MainWindow::importAsterixFromNetworkSlot);
    import_menu_->addAction(import_ast_net_action);

    QAction* import_json_file_action = new QAction("&JSON Recording");
    import_json_file_action->setShortcut(tr("Ctrl+J"));
    import_json_file_action->setToolTip("Import JSON Recording File");
    connect(import_json_file_action, &QAction::triggered, this, &MainWindow::importJSONRecordingSlot);
    import_menu_->addAction(import_json_file_action);

    QAction* import_gps_nmea_action = new QAction("&GPS Trail NMEA");
    import_gps_nmea_action->setShortcut(tr("Ctrl+G"));
    import_gps_nmea_action->setToolTip("Import GPS Trail NMEA File");
    connect(import_gps_nmea_action, &QAction::triggered, this, &MainWindow::importGPSTrailSlot);
    import_menu_->addAction(import_gps_nmea_action);

    QAction* import_vp_file_action = new QAction("&View Points");
    import_vp_file_action->setShortcut(tr("Ctrl+V"));
    import_vp_file_action->setToolTip("Import View Points File");
    connect(import_vp_file_action, &QAction::triggered, this, &MainWindow::importViewPointsSlot);
    import_menu_->addAction(import_vp_file_action);

    // configuration menu
    config_menu_ = menuBar()->addMenu("&Configuration");
    config_menu_->setToolTipsVisible(true);

            // configure operations
    QAction* ds_action = new QAction("Data Sources");
    ds_action->setToolTip("Configure Data Sources");
    connect(ds_action, &QAction::triggered, this, &MainWindow::configureDataSourcesSlot);
    config_menu_->addAction(ds_action);

    QAction* ffts_action = new QAction("FFTs");
    ffts_action->setToolTip("Configure FFTs");
    connect(ffts_action, &QAction::triggered, this, &MainWindow::configureFFTsSlot);
    config_menu_->addAction(ffts_action);

    QAction* meta_action = new QAction("Meta Variables");

    if (expert_mode)
        meta_action->setToolTip("Configure Meta Variables");
    else
        meta_action->setToolTip("Show Meta Variables");

    connect(meta_action, &QAction::triggered, this, &MainWindow::configureMetaVariablesSlot);
    config_menu_->addAction(meta_action);

    sectors_action_ = new QAction("Sectors");
    sectors_action_->setToolTip("Configure Sectors (stored in Database)");
    connect(sectors_action_, &QAction::triggered, this, &MainWindow::configureSectorsSlot);
    sectors_action_->setDisabled(true);
    config_menu_->addAction(sectors_action_);

#if USE_EXPERIMENTAL_SOURCE == true
    config_menu_->addSeparator();

    license_action_ = config_menu_->addAction("Licenses");
    license_action_->setToolTip("Manage licenses");
    connect(license_action_, &QAction::triggered, this, &MainWindow::manageLicensesSlot);
    config_menu_->addAction(license_action_);
#endif

    config_menu_->addSeparator();

    dark_mode_action_ = config_menu_->addAction("Dark Mode");
    dark_mode_action_->setCheckable(true);
    dark_mode_action_->setChecked(COMPASS::instance().darkMode());
    connect(dark_mode_action_, &QAction::toggled, this, &MainWindow::toggleDarkModeSlot);

    fullscreen_action_ = config_menu_->addAction("Fullscreen [F11]");
    fullscreen_action_->setCheckable(true);
    fullscreen_action_->setChecked(isFullScreen());
    connect(fullscreen_action_, &QAction::toggled, this, &MainWindow::toggleFullscreenSlot);

    ViewManager& view_manager = COMPASS::instance().viewManager();

    auto_refresh_views_action_ = config_menu_->addAction("Refresh Views Automatically");
    auto_refresh_views_action_->setCheckable(true);
    auto_refresh_views_action_->setChecked(view_manager.automaticReloadEnabled());
    connect(auto_refresh_views_action_, &QAction::toggled, &view_manager, &ViewManager::enableAutomaticReload);

    // process menu
    process_menu_ = menuBar()->addMenu("&Process");
    process_menu_->setToolTipsVisible(true);

    QAction* calc_radar_plpos_action = new QAction("Calculate Radar Plot Positions");
    calc_radar_plpos_action->setToolTip("Calculate Radar Plot Positions, only needed if Radar Position information"
        " was changed");
    connect(calc_radar_plpos_action, &QAction::triggered, this, &MainWindow::calculateRadarPlotPositionsSlot);
    process_menu_->addAction(calc_radar_plpos_action);

    QAction* assoc_artas_action = new QAction("Calculate ARTAS Target Report Usage");
    assoc_artas_action->setToolTip("Create target report usage based on ARTAS TRI information");
    connect(assoc_artas_action, &QAction::triggered, this, &MainWindow::calculateAssociationsARTASSlot);
    process_menu_->addAction(assoc_artas_action);

    QAction* reconstruct_action = new QAction("Reconstruct References");
    reconstruct_action->setToolTip("Associate Unique Targets andd reconstruct Reference Trajectories");
    connect(reconstruct_action, &QAction::triggered, this, &MainWindow::reconstructReferencesSlot);
    process_menu_->addAction(reconstruct_action);

    QAction* eval_action = new QAction("Evaluate");
    eval_action->setToolTip("Evaluate test against reference data according to defined standards");
    connect(eval_action, &QAction::triggered, this, &MainWindow::evaluateSlot);
    process_menu_->addAction(eval_action);

    // ui menu
    ui_menu_ = menuBar()->addMenu("&UI");
    ui_menu_->setToolTipsVisible(true);

    QAction* reset_views_action = new QAction("Reset Views");
    reset_views_action->setToolTip(
        "Enable all data sources, reset labels,\n"
        "disable all filters and reset Views to startup configuration");
    connect(reset_views_action, &QAction::triggered, this, &MainWindow::resetViewsMenuSlot);
    ui_menu_->addAction(reset_views_action);

    //debug menu (internal)
    createDebugMenu();
}

void MainWindow::updateMenus()
{
    traced_assert(new_db_action_);
    traced_assert(open_existing_db_action_);
    traced_assert(open_recent_db_menu_);
    traced_assert(close_db_action_);

    traced_assert(sectors_action_);

    traced_assert(import_menu_);

    traced_assert(license_action_);

    bool in_live_running        = COMPASS::instance().appMode() == AppMode::LiveRunning;
    bool in_live_paused         = COMPASS::instance().appMode() == AppMode::LivePaused;
    bool in_live                = in_live_running || in_live_paused;
    bool asterix_import_running = COMPASS::instance().taskManager().asterixImporterTask().isRunning();

    open_recent_db_menu_->clear();

            // recent db files
    vector<string> recent_file_list = COMPASS::instance().dbFileList();

    for (auto& fn_it : recent_file_list)
    {
        QAction* file_act = new QAction(fn_it.c_str());
        file_act->setData(fn_it.c_str());
        connect(file_act, &QAction::triggered, this, &MainWindow::openRecentDBSlot);
        open_recent_db_menu_->addAction(file_act);
    }

    if (recent_file_list.size() == 0)
        open_recent_db_menu_->setDisabled(true);
    else // add clear action
    {
        open_recent_db_menu_->addSeparator();

        QAction* clear_file_act = new QAction("Clear");
        connect(clear_file_act, &QAction::triggered, this, &MainWindow::clearExistingDBsSlot);
        open_recent_db_menu_->addAction(clear_file_act);
    }

    bool db_open = COMPASS::instance().dbOpened();

    new_db_action_->setDisabled(db_open || in_live);
    open_existing_db_action_->setDisabled(db_open || in_live);

    if (recent_file_list.size()) // is disabled otherwise
        open_recent_db_menu_->setDisabled(db_open || in_live);

    export_db_action_->setDisabled(!db_open || in_live_running);
    close_db_action_->setDisabled(!db_open || in_live);

    sectors_action_->setDisabled(!db_open || in_live_running);

    import_menu_->setDisabled(!db_open || asterix_import_running || in_live);
    process_menu_->setDisabled(!db_open || asterix_import_running || in_live);

    traced_assert(config_menu_);
    config_menu_->setDisabled(asterix_import_running || in_live);

    for (auto a : config_menu_->actions())
        a->setEnabled(a == license_action_
                      || a == auto_refresh_views_action_
                              || a == dark_mode_action_
                              || a == fullscreen_action_ ? true : db_open);
}

void MainWindow::updateBottomWidget()
{
    COMPASS& compass = COMPASS::instance();

    traced_assert(db_label_);

    if (compass.dbOpened())
        db_label_->setText(("DB: "+compass.lastDbFilename()).c_str());
    else
        db_label_->setText("No Database");

    traced_assert(status_label_);
    status_label_->setText(compass.appModeStr().c_str());

    traced_assert(load_button_);
    traced_assert(live_pause_resume_button_);

    AppMode app_mode = compass.appMode();

    if (!compass.dbOpened())
    {
        load_button_->setHidden(true);

        live_pause_resume_button_->setHidden(true);

        if (live_stop_button_)
            live_stop_button_->setHidden(true);
    }
    else if (app_mode == AppMode::Offline)
    {
        load_button_->setHidden(false);

        live_pause_resume_button_->setHidden(true);

        if (live_stop_button_)
            live_stop_button_->setHidden(true);
    }
    else if (app_mode == AppMode::LivePaused)
    {
        load_button_->setHidden(false);

        live_pause_resume_button_->setHidden(false);
        live_pause_resume_button_->setText("Resume");

        if (live_stop_button_)
            live_stop_button_->setHidden(false);
    }
    else if (app_mode == AppMode::LiveRunning)
    {
        load_button_->setHidden(true);

        live_pause_resume_button_->setHidden(false);
        live_pause_resume_button_->setText("Pause");

        if (live_stop_button_)
            live_stop_button_->setHidden(false);
    }
    else
        logerr << "unknown app mode " << (unsigned int) COMPASS::instance().appMode();
}

void MainWindow::disableConfigurationSaving()
{
    logdbg;
    save_configuration_ = false;

    traced_assert(quit_wo_cfg_sav_action_);
    quit_wo_cfg_sav_action_->setEnabled(save_configuration_);
}

void MainWindow::openExistingDB(const std::string& filename)
{
    loginf << "filename '" << filename << "'";

    COMPASS::instance().openDBFile(filename);

    updateBottomWidget();
    updateMenus();
}

void MainWindow::createDB(const std::string& filename)
{
    loginf << "filename '" << filename << "'";

    COMPASS::instance().createNewDBFile(filename);

    updateBottomWidget();
    updateMenus();
}

void MainWindow::createInMemoryDB(const std::string& future_filename)
{
    loginf << "future filename '" << future_filename << "'";

    COMPASS::instance().createInMemDBFile(future_filename);

    updateBottomWidget();
    updateMenus();
}

void MainWindow::createDBFromMemory()
{
    loginf;

    COMPASS::instance().createNewDBFileFromMemory();

    updateBottomWidget();
    updateMenus();
}

void MainWindow::newDBSlot()
{
    loginf;

    string filename = QFileDialog::getSaveFileName(
                          this, "New Database File", COMPASS::instance().lastUsedPath().c_str()).toStdString();

    if (filename.size() > 0)
        createDB(filename);
}

void MainWindow::openExistingDBSlot()
{
    loginf;

    string filename = QFileDialog::getOpenFileName(
                          this, "Open Database File", COMPASS::instance().lastUsedPath().c_str()).toStdString();

    if (filename.size() > 0)
        openExistingDB(filename);
}

void MainWindow::openRecentDBSlot()
{
    loginf;

    QAction* action = dynamic_cast<QAction*> (QObject::sender());
    traced_assert(action);

    string filename = action->data().toString().toStdString();

    traced_assert(filename.size());

    openExistingDB(filename);
}

void MainWindow::exportDBSlot()
{
    loginf;

    string filename = QFileDialog::getSaveFileName(
                          this, "Export Database File", COMPASS::instance().lastUsedPath().c_str()).toStdString();

    if (filename.size() > 0)
    {
        COMPASS::instance().exportDBFile(filename);
    }
}

void MainWindow::clearExistingDBsSlot()
{
    loginf;

    COMPASS::instance().clearDBFileList();

    updateBottomWidget();
    updateMenus();
}

void MainWindow::closeDBSlot()
{
    loginf;

    COMPASS::instance().closeDB();

    updateBottomWidget();
    updateMenus();
}

void MainWindow::saveConfigSlot()
{
    loginf;

    traced_assert(!COMPASS::instance().disableMenuConfigSave());

    ConfigurationManager::getInstance().saveConfiguration();
}

void MainWindow::quitWOConfigSlot()
{
    loginf;

    save_configuration_ = false;

    shutdown();

    QApplication::quit();
}
void MainWindow::quitSlot()
{
    loginf;

    shutdown();

    QApplication::quit();
}

void MainWindow::importAsterixRecordingSlot()
{
    loginf;

    QFileDialog dialog(this, "Import ASTERIX File(s)");
    dialog.setViewMode(QFileDialog::Detail);
    dialog.setFileMode(QFileDialog::ExistingFiles);
    dialog.setDirectory(COMPASS::instance().lastUsedPath().c_str());

    ASTERIXImportTask& task = COMPASS::instance().taskManager().asterixImporterTask();

    task.source().setSourceType(ASTERIXImportSource::SourceType::FileASTERIX);

    if (dialog.exec())
    {
        QStringList filenames = dialog.selectedFiles();

        vector<string> filenames_vec;

        for (auto& filename : filenames)
        {
            traced_assert(Files::fileExists(filename.toStdString()));
            COMPASS::instance().lastUsedPath(Files::getDirectoryFromPath(filename.toStdString()));

            filenames_vec.push_back(filename.toStdString());
        }

        task.source().addFiles(filenames_vec);

        updateMenus();

        task.runDialog(this);
    }
}

void MainWindow::importRecentAsterixRecordingSlot()
{
    loginf;

    QAction* action = dynamic_cast<QAction*> (QObject::sender());
    traced_assert(action);

    string filename = action->data().toString().toStdString();

    traced_assert(filename.size());

    auto& task = COMPASS::instance().taskManager().asterixImporterTask();

    task.source().setSourceType(ASTERIXImportSource::SourceType::FileASTERIX, {filename});

    updateMenus();

    task.runDialog(this);
}

void MainWindow::importAsterixFromPCAPSlot()
{
    loginf;

    // auto fn = QFileDialog::getOpenFileName(this,
    //                                        "Import PCAP File",
    //                                        COMPASS::instance().lastUsedPath().c_str(),
    //                                        "PCAP Files (*.pcap *.PCAP)");
    // if (fn.isEmpty())
    //     return;

    // ASTERIXImportTask& task = COMPASS::instance().taskManager().asterixImporterTask();

    // task.source().setSourceType(ASTERIXImportSource::SourceType::FilePCAP, { fn.toStdString() });

    // updateMenus();

    // task.runDialog(this);

    QFileDialog dialog(this, "Import PCAP File(s)", QString(), "PCAP Files (*.pcap *.PCAP)");
    dialog.setViewMode(QFileDialog::Detail);
    dialog.setFileMode(QFileDialog::ExistingFiles);
    dialog.setDirectory(COMPASS::instance().lastUsedPath().c_str());

    ASTERIXImportTask& task = COMPASS::instance().taskManager().asterixImporterTask();

    task.source().setSourceType(ASTERIXImportSource::SourceType::FilePCAP);

    if (dialog.exec())
    {
        QStringList filenames = dialog.selectedFiles();

        vector<string> filenames_vec;

        for (auto& filename : filenames)
        {
            traced_assert(Files::fileExists(filename.toStdString()));
            COMPASS::instance().lastUsedPath(Files::getDirectoryFromPath(filename.toStdString()));

            filenames_vec.push_back(filename.toStdString());
        }

        task.source().addFiles(filenames_vec);

        updateMenus();

        task.runDialog(this);
    }
}

void MainWindow::importAsterixFromJSONSlot()
{
    loginf;

    auto fn = QFileDialog::getOpenFileName(this, 
                                           "Import JSON File", 
                                           COMPASS::instance().lastUsedPath().c_str());
    if (fn.isEmpty())
        return;

    ASTERIXImportTask& task = COMPASS::instance().taskManager().asterixImporterTask();

    task.source().setSourceType(ASTERIXImportSource::SourceType::FileJSON, { fn.toStdString() });

    updateMenus();

    task.runDialog(this);
}

void MainWindow::importAsterixFromNetworkSlot()
{
    loginf;

    ASTERIXImportTask& task = COMPASS::instance().taskManager().asterixImporterTask();

    task.source().setSourceType(ASTERIXImportSource::SourceType::NetASTERIX);

    task.runDialog(this);
}

void MainWindow::importJSONRecordingSlot()
{
    string filename = QFileDialog::getOpenFileName(
                          this, "Import JSON File", COMPASS::instance().lastUsedPath().c_str(),
                          "JSON Files (*.json *.zip)").toStdString();

    if (filename.size() > 0)
    {
        COMPASS::instance().taskManager().jsonImporterTask().importFilename(filename); // also adds
        COMPASS::instance().lastUsedPath(Files::getDirectoryFromPath(filename));

        updateMenus();

        COMPASS::instance().taskManager().jsonImporterTask().dialog()->updateSource();
        COMPASS::instance().taskManager().jsonImporterTask().dialog()->show();
    }
}

void MainWindow::importGPSTrailSlot()
{
    string filename = QFileDialog::getOpenFileName(this, "Import GPS Trail",
                                                   COMPASS::instance().lastUsedPath().c_str(),
                                                   "Text Files (*.nmea *.txt)").toStdString();

    if (filename.size() > 0)
    {
        COMPASS::instance().taskManager().gpsTrailImportTask().importFilename(filename);
        COMPASS::instance().lastUsedPath(Files::getDirectoryFromPath(filename));

        updateMenus();

        COMPASS::instance().taskManager().gpsTrailImportTask().dialog()->show();
    }
}

// void MainWindow::importGPSCSVSlot()
// {
//     string filename = QFileDialog::getOpenFileName(this, "Import GPS Trail CSV",
//                                                    COMPASS::instance().lastUsedPath().c_str(),
//                                                    "Text Files (*.csv *.txt)").toStdString();

//     if (filename.size() > 0)
//     {
//         COMPASS::instance().taskManager().gpsImportCSVTask().importFilename(filename);
//         COMPASS::instance().lastUsedPath(Files::getDirectoryFromPath(filename));

//         updateMenus();

//         COMPASS::instance().taskManager().gpsImportCSVTask().dialog()->show();
//     }
// }

void MainWindow::importViewPointsSlot()
{
    string filename = QFileDialog::getOpenFileName(this, "Import View Points",
                                                   COMPASS::instance().lastUsedPath().c_str(), "*.json").toStdString();

    if (filename.size() > 0)
    {
        COMPASS::instance().taskManager().viewPointsImportTask().importFilename(filename);
        COMPASS::instance().lastUsedPath(Files::getDirectoryFromPath(filename));

        updateMenus();

        COMPASS::instance().taskManager().viewPointsImportTask().showDialog();
    }
}

void MainWindow::calculateRadarPlotPositionsSlot()
{
    loginf;

    COMPASS::instance().taskManager().radarPlotPositionCalculatorTask().showDialog();
}

void MainWindow::calculateAssociationsARTASSlot()
{
    loginf;

    COMPASS::instance().taskManager().createArtasAssociationsTask().showDialog();
}

void MainWindow::reconstructReferencesSlot()
{
    loginf;

    COMPASS::instance().taskManager().reconstructReferencesTask().showDialog();
}

void MainWindow::evaluateSlot()
{
    loginf;

    COMPASS::instance().evaluationManager().evaluate(true);
}

void MainWindow::configureDataSourcesSlot()
{
    loginf;

    COMPASS::instance().dataSourceManager().configurationDialog()->show();
}

void MainWindow::configureFFTsSlot()
{
    loginf;

    COMPASS::instance().fftManager().configurationDialog()->show();
}


void MainWindow::configureMetaVariablesSlot()
{
    loginf;

    COMPASS::instance().dbContentManager().metaVariableConfigdialog()->show();
}

void MainWindow::configureSectorsSlot()
{
    loginf;

    COMPASS::instance().taskManager().manageSectorsTask().dialog()->show();
}

void MainWindow::quitRequestedSlot()
{
    shutdown();
    QApplication::quit();
}

void MainWindow::showAddViewMenuSlot()
{
    loginf;

    traced_assert(!COMPASS::instance().disableAddRemoveViews());
    COMPASS::instance().viewManager().showMainViewContainerAddView();
}

void MainWindow::resetViewsMenuSlot()
{
    loginf;

    QMessageBox::StandardButton reply;

    if (COMPASS::instance().disableConfirmResetViews())
        reply = QMessageBox::Yes;
    else
    {

        reply = QMessageBox::question(
            this, "Reset Views",
            "Confirm to enable all data sources, reset labels,\n"
            "disable all filters and reset Views to startup configuration?",
            QMessageBox::Yes|QMessageBox::No);
    }

    if (reply == QMessageBox::Yes)
    {
        QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));

        traced_assert(tab_widget_);
        //int index = tab_widget_->currentIndex();

        {
            QMessageBox msg_box;
            msg_box.setWindowTitle("Resetting");
            msg_box.setText( "Please wait...");
            msg_box.setStandardButtons(QMessageBox::NoButton);
            msg_box.setWindowModality(Qt::ApplicationModal);
            msg_box.show();

            setVisible(false);

            Async::waitAndProcessEventsFor(50);

                    // reset stuff
            COMPASS::instance().dbContentManager().resetToStartupConfiguration();

            COMPASS::instance().dataSourceManager().resetToStartupConfiguration();

            COMPASS::instance().filterManager().resetToStartupConfiguration();

#if USE_EXPERIMENTAL_SOURCE == true
            GeometryTreeItem::clearHiddenIdentifierStrs(); // clears hidden layers
#endif

            COMPASS::instance().viewManager().resetToStartupConfiguration();

                    // set AppMode
            if (COMPASS::instance().appMode() == AppMode::LivePaused)
                COMPASS::instance().appMode(AppMode::LiveRunning);
            else
            {
                COMPASS::instance().viewManager().appModeSwitchSlot(
                    COMPASS::instance().appMode(), COMPASS::instance().appMode());
            }

            msg_box.hide();
        }

        //tab_widget_->setCurrentIndex(index);

        setVisible(true);

        QApplication::restoreOverrideCursor();
    }
}

void MainWindow::appModeSwitchSlot (AppMode app_mode_previous, AppMode app_mode_current)
{
    bool enable_tabs = app_mode_current == AppMode::Offline;

    loginf << "app_mode " << COMPASS::instance().appModeStr()
           << " enable_tabs " << enable_tabs;

    traced_assert(tool_box_);
    if (app_mode_current == AppMode::LiveRunning)
    {
        tool_box_->disableTools({2,3,4});
        selectTool("Data Sources");
    }
    else
        tool_box_->disableTools({}); // enable all

    updateBottomWidget();
    updateMenus();

    if (app_mode_current == AppMode::LivePaused)
    {
        traced_assert(!auto_resume_timer_);

        auto_resume_timer_ = new QTimer();

        connect(auto_resume_timer_, &QTimer::timeout, this, &MainWindow::autoResumeTimerSlot);
        auto_resume_timer_->start(COMPASS::instance().autoLiveRunningResumeAskTime() * 60 * 1000); // min -> ms
    }
    else if (auto_resume_timer_)
    {
        delete auto_resume_timer_;
        auto_resume_timer_ = nullptr;
    }

    if (auto_resume_dialog_)
    {
        auto_resume_dialog_->close();
        auto_resume_dialog_ = nullptr;
    }
}

void MainWindow::autoResumeTimerSlot()
{
    loginf;

    traced_assert(!auto_resume_dialog_);

    auto_resume_dialog_.reset(new AutoResumeDialog(COMPASS::instance().autoLiveRunningResumeAskWaitTime() * 60));

            // min to s
    connect (auto_resume_dialog_.get(), &AutoResumeDialog::resumeSignal, this, &MainWindow::autoResumeResumeSlot);
    connect (auto_resume_dialog_.get(), &AutoResumeDialog::stayPausedSignal, this, &MainWindow::autoResumeStaySlot);

    auto_resume_dialog_->show();
}

void MainWindow::autoResumeResumeSlot()
{
    loginf;

    traced_assert(auto_resume_dialog_);
    auto_resume_dialog_->close();

    auto_resume_dialog_ = nullptr;

    livePauseResumeSlot();
}

void MainWindow::autoResumeStaySlot()
{
    loginf;

    auto_resume_dialog_->close();

    auto_resume_dialog_ = nullptr;

            // restart timer
    auto_resume_timer_->start(COMPASS::instance().autoLiveRunningResumeAskTime() * 60 * 1000); // min -> ms
}

void MainWindow::toggleDarkModeSlot()
{
    loginf;

    COMPASS::instance().darkMode(!COMPASS::instance().darkMode());

    QMessageBox m_warning(QMessageBox::Information, "Dark Mode",
                          "Please restart the applications for the Dark Mode change to take effect.", QMessageBox::Ok);
    m_warning.exec();
}

void MainWindow::toggleFullscreenSlot()
{
    loginf << "isFullScreen " << isFullScreen();

    if (isFullScreen())
    {
        // Exit fullscreen: restore window flags and normal size
        //setWindowFlag(Qt::FramelessWindowHint, false); // Remove frameless
        //setWindowFlag(Qt::Window, true);               // Ensure normal window flags
        setWindowFlags(Qt::Window);

        QSettings settings("COMPASS", "Client");

        // if (settings.value("MainWindow/isMaximized", false).toBool())
        // {
        //     loginf << "isMaximized";
        //     showMaximized();
        // }
        // else
        {
            restoreGeometry(settings.value("MainWindow/geometry").toByteArray());
            showNormal();
        }
    }
    else
    {
        QSettings settings("COMPASS", "Client");
        settings.setValue("MainWindow/geometry", saveGeometry());
        //settings.setValue("MainWindow/isMaximized", isMaximized());

        // Enter fullscreen with no decorations
        //setWindowFlag(Qt::FramelessWindowHint, true);
        setWindowFlags(Qt::FramelessWindowHint);
        showMaximized();
        showFullScreen(); // Re-show to apply
    }
}

void MainWindow::loadButtonSlot()
{
    loginf;

    if (COMPASS::instance().viewManager().getViews().size() == 0)
    {
        QMessageBox m_warning(QMessageBox::Warning, "Loading Not Possible",
                              "There are no Views active, so loading is not possible.",
                              QMessageBox::Ok);

        m_warning.exec();
        return;
    }

    traced_assert(load_button_);

    if (loading_)
    {
        load_button_->setDisabled(true);
        COMPASS::instance().dbContentManager().quitLoading();
        return;
    }

    loading_ = true;
    load_button_->setText("Stop");

    COMPASS::instance().dbContentManager().load("", true);
}

void MainWindow::loadingDoneSlot()
{
    loginf;

    traced_assert(load_button_);

    loading_ = false;
    load_button_->setText("Load");
    load_button_->setDisabled(false);

    emit dataLoaded();
}

void MainWindow::livePauseResumeSlot()
{
    loginf;

    AppMode app_mode = COMPASS::instance().appMode();

    traced_assert(app_mode == AppMode::LivePaused || AppMode::LiveRunning);

    if (app_mode == AppMode::LiveRunning)
        COMPASS::instance().appMode(AppMode::LivePaused);
    else // AppMode::LivePaused)
        COMPASS::instance().appMode(AppMode::LiveRunning);
}

void MainWindow::liveStopSlot()
{
    loginf;

    COMPASS::instance().appMode(AppMode::Offline);

    //transfer memory db to file
    if (COMPASS::instance().canCreateDBFileFromMemory())
        createDBFromMemory();
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    loginf;

    shutdown();
    event->accept();

    logdbg << "done";
}

void MainWindow::shutdown()
{
    QSettings settings("COMPASS", "Client");
    settings.setValue("MainWindow/geometry", saveGeometry());
    //settings.setValue("MainWindow/isMaximized", isMaximized());
    settings.setValue("MainWindow/isFullScreen", isFullScreen());

    COMPASS::instance().viewManager().unsetCurrentViewPoint(); // needed to remove temporary stuff

    if (save_configuration_)
        ConfigurationManager::getInstance().saveConfiguration();
    else
        loginf << "configuration not saved";

    COMPASS::instance().shutdown();

    if (tab_widget_)
    {
        delete tab_widget_;
        tab_widget_ = nullptr;
    }
}

// void MainWindow::keyPressEvent ( QKeyEvent * event )
//{
//    logdbg << "'" << event->text().toStdString() << "'";
//}

void MainWindow::createDebugMenu()
{
    //add debug menu entry
    auto debug_menu = menuBar()->addMenu("Debug");

        //add test lab entries
#if USE_EXPERIMENTAL_SOURCE == true
    {
        TestLabCollection().appendTestLabs(debug_menu);
    }
#endif

        //add command shell
    {
        auto action = debug_menu->addAction("Command Shell");
        connect(action, &QAction::triggered, [ this ] () { this->showCommandShell(); });

        auto shortcut = new QShortcut(this);
        shortcut->setKey(QKeySequence(tr("Ctrl+Alt+S")));
        connect(shortcut, &QShortcut::activated, [ this ] () { this->showCommandShell(); });
    }

    debug_menu->menuAction()->setVisible(!COMPASS::instance().isAppImage());
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_F11) {
        toggleFullscreenSlot();
    } else {
        QMainWindow::keyPressEvent(event); // Let base class handle other keys
    }
}

void MainWindow::showCommandShell()
{
    QDialog dlg(this);
    QHBoxLayout* layout = new QHBoxLayout;
    dlg.setLayout(layout);

    rtcommand::RTCommandShell* shell = new rtcommand::RTCommandShell(&dlg);
    layout->addWidget(shell);

    dlg.resize(800, 600);
    dlg.exec();
}

void MainWindow::manageLicensesSlot()
{
    LicenseManagerDialog dlg(this);
    dlg.resize(1000, 600);
    dlg.exec();
}

void MainWindow::updateWindowTitle()
{
    auto version  = COMPASS::instance().versionString();
    auto licensee = COMPASS::instance().licenseeString();

    auto title = version;
    if (!licensee.empty())
        title += "   |   " + licensee;

    QWidget::setWindowTitle(title.c_str());
}

void MainWindow::loadingStarted()
{
    if (tool_box_)
        tool_box_->loadingStarted();
}

void MainWindow::loadingDone()
{
    if (tool_box_)
        tool_box_->loadingDone();
}

void MainWindow::selectTool(const std::string& name)
{
    loginf << "selecting tool '" << name << "'";

    if (tool_box_)
        tool_box_->selectTool(name);
}

void MainWindow::showResult(const std::string& name)
{
    loginf << "showing result '" << name << "'";

    auto& task_manager = COMPASS::instance().taskManager();

    //select task results tool
    std::string tool_name = task_manager.widget()->toolName();
    selectTool(tool_name);

    //show result of given name
    task_manager.widget()->setReport(name);
}

void MainWindow::showEvaluationResult()
{
    std::string result_name = COMPASS::instance().evaluationManager().lastResultName();

    if (!result_name.empty())
        showResult(result_name);
}
