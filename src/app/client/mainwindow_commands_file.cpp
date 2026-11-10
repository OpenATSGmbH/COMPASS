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

#include "mainwindow_commands_file.h"
#include "mainwindow.h"
#include "compass.h"
#include "compassroot.h"
#include "logger.h"
#include "files.h"
#include "traced_assert.h"

#include <QApplication>
#include <QDialog>
#include <QTimer>

#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>

using namespace std;
using namespace Utils;

namespace main_window
{
// open_db

/**
 */
RTCommandOpenDB::RTCommandOpenDB(CompassRoot& root)
    : rtcommand::RTCommand()
    , root_(root)
{
}

rtcommand::IsValid RTCommandOpenDB::valid() const
{
    CHECK_RTCOMMAND_INVALID_CONDITION(!filename_.size(), "Filename empty")
    CHECK_RTCOMMAND_INVALID_CONDITION(!Files::fileExists(filename_), string("File '")+filename_+"' does not exist")

    return RTCommand::valid();
}

bool RTCommandOpenDB::run_impl()
{
    if (!filename_.size())
    {
        setResultMessage("Filename empty");
        return false;
    }

    if (!Files::fileExists(filename_))
    {
        setResultMessage("File '"+filename_+"' does not exist");
        return false;
    }

    if (root_.compass().dbOpened())
    {
        if (assure_open_ && root_.compass().appState().lastDbFilename() == filename_)
        {
            return true;
        }
        else
        {
            setResultMessage("Database already opened");
            return false;
        }
    }

    if (root_.compass().appState().appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+root_.compass().appState().appModeStr());
        return false;
    }

    MainWindow* main_window = dynamic_cast<MainWindow*> (rtcommand::mainWindow());
    traced_assert(main_window);

    main_window->openExistingDB(filename_);

    return root_.compass().dbOpened();
}

void RTCommandOpenDB::collectOptions_impl(OptionsDescription& options,
                                          PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
    ("filename,f", po::value<std::string>()->required(), "given filename, e.g. ’/data/file1.db’")
        ("assure_open", "Only opens the file if it is not already opened");

    ADD_RTCOMMAND_POS_OPTION(positional, "filename") // give position
}

void RTCommandOpenDB::assignVariables_impl(const VariablesMap& variables)
    {
        RTCOMMAND_GET_VAR_OR_THROW(variables, "filename", std::string, filename_)
        RTCOMMAND_CHECK_VAR(variables, "assure_open", assure_open_)
    }

// open_recent_db

/**
 */
RTCommandOpenRecentDB::RTCommandOpenRecentDB(CompassRoot& root)
    : rtcommand::RTCommand()
    , root_(root)
{
}

std::string RTCommandOpenRecentDB::getPath() const
{
    vector<string> recent_file_list = root_.compass().dbFileList();
    if (recent_file_list.empty())
        return "";

    if (!filename.empty())
    {
        for (const auto& fn : recent_file_list)
        {
            if (boost::filesystem::path(fn).filename() == filename)
                return fn;
        }
        return "";
    }

    if (index >= 0)
    {
        if (index < (int)recent_file_list.size())
            return recent_file_list[index];

        return "";
    }

    return recent_file_list.front();
}

rtcommand::IsValid RTCommandOpenRecentDB::valid() const
{
    auto fn = getPath();

    CHECK_RTCOMMAND_INVALID_CONDITION(fn.empty(), "No recent file found")

    return RTCommand::valid();
}

bool RTCommandOpenRecentDB::run_impl()
{
    auto fn = getPath();

    if (fn.empty())
    {
        setResultMessage("No recent file found");
        return false;
    }

    if (!Files::fileExists(fn))
    {
        setResultMessage("File '"+fn+"' does not exist");
        return false;
    }

    if (root_.compass().dbOpened())
    {
        setResultMessage("Database already opened");
        return false;
    }

    if (root_.compass().appState().appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+root_.compass().appState().appModeStr());
        return false;
    }

    MainWindow* main_window = dynamic_cast<MainWindow*> (rtcommand::mainWindow());
    traced_assert(main_window);

    main_window->openExistingDB(fn);

    return root_.compass().dbOpened();
}

void RTCommandOpenRecentDB::collectOptions_impl(OptionsDescription& options,
                                                PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
    ("filename,f", po::value<std::string>()->default_value(""), "filename listed in the recent file history, e.g. ’file1.db’")
        ("index,i", po::value<int>()->default_value(-1), "index in the recent file history");

    ADD_RTCOMMAND_POS_OPTION(positional, "filename") // give position
}

void RTCommandOpenRecentDB::assignVariables_impl(const VariablesMap& variables)
    {
        RTCOMMAND_GET_VAR_OR_THROW(variables, "filename", std::string, filename)
        RTCOMMAND_GET_VAR_OR_THROW(variables, "index", int, index)
    }

// create db

/**
 */
RTCommandCreateDB::RTCommandCreateDB(CompassRoot& root)
    : rtcommand::RTCommand()
    , root_(root)
{
}

rtcommand::IsValid  RTCommandCreateDB::valid() const
{
    CHECK_RTCOMMAND_INVALID_CONDITION(!filename_.size(), "Filename empty")

    return RTCommand::valid();
}

bool RTCommandCreateDB::run_impl()
{
    if (!filename_.size())
    {
        setResultMessage("Filename empty");
        return false;
    }

    if (root_.compass().dbOpened())
    {
        setResultMessage("Database already opened");
        return false;
    }

    if (root_.compass().appState().appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+root_.compass().appState().appModeStr());
        return false;
    }

    MainWindow* main_window = dynamic_cast<MainWindow*> (rtcommand::mainWindow());
    traced_assert(main_window);

    main_window->createDB(filename_);

    return root_.compass().dbOpened();
}

void RTCommandCreateDB::collectOptions_impl(OptionsDescription& options,
                                            PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
    ("filename,f", po::value<std::string>()->required(), "given filename, e.g. ’/data/file1.db’");

    ADD_RTCOMMAND_POS_OPTION(positional, "filename") // give position
}

void RTCommandCreateDB::assignVariables_impl(const VariablesMap& variables)
{
    RTCOMMAND_GET_VAR_OR_THROW(variables, "filename", std::string, filename_)
}

// close db

/**
 */
RTCommandCloseDB::RTCommandCloseDB(CompassRoot& root)
    : rtcommand::RTCommand()
    , root_(root)
{
}

void RTCommandCloseDB::collectOptions_impl(OptionsDescription& options,
                                           PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
    ("strict", "fail if no database was opened");
}

void RTCommandCloseDB::assignVariables_impl(const VariablesMap& variables)
{
    RTCOMMAND_CHECK_VAR(variables, "strict", strict_)
}

bool RTCommandCloseDB::run_impl()
{
    //lets return true if there's nothing to be done
    if (!root_.compass().dbOpened())
        return !strict_;

    if (root_.compass().appState().appMode() != AppMode::Offline)
        return false;

    MainWindow* main_window = dynamic_cast<MainWindow*> (rtcommand::mainWindow());
    traced_assert(main_window);

    main_window->closeDBSlot();

    return !root_.compass().dbOpened();
}

// quit app

namespace
{
/**
 * Quits when no modal dialog is open. A modal dialog runs its own event loop and
 * keeps references to the managers. A shutdown inside this loop deletes the managers
 * before the dialog, and the dialog destructor then uses a deleted manager. So the
 * dialogs are closed first, and the quit runs from the main event loop.
 */
void quitWhenNoModalDialog(MainWindow* main_window, int retries)
{
    QWidget* modal = QApplication::activeModalWidget();

    if (modal && retries > 0)
    {
        loginf << "closing modal dialog '" << modal->windowTitle().toStdString() << "' before quit";

        if (auto* dialog = qobject_cast<QDialog*>(modal))
            dialog->reject();
        else
            modal->close();

        QTimer::singleShot(100, [main_window, retries] () { quitWhenNoModalDialog(main_window, retries - 1); });
        return;
    }

    if (modal)
        logwrn << "modal dialog '" << modal->windowTitle().toStdString() << "' still open, quitting";

    main_window->quitSlot();
}
}

bool RTCommandQuit::run_impl()
{
    MainWindow* main_window = dynamic_cast<MainWindow*> (rtcommand::mainWindow());
    traced_assert(main_window);

    QTimer::singleShot(100, [main_window] () { quitWhenNoModalDialog(main_window, 50); });

    return true;
}

}
