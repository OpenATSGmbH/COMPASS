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

#include "viewpointsreportgenerator.h"
#include "viewpointsreportgeneratordialog.h"
#include "viewmanager.h"
#include "view.h"
#include "viewpointswidget.h"
#include "viewpointstablemodel.h"
#include "viewpoint.h"
#include "logger.h"
#include "stringconv.h"
#include "dbcontent/dbcontentmanager.h"
#include "compass.h"
#include "global.h"
#include "dbinterface.h"
#include "dbconnection.h"
#include "sqliteconnection.h"
#include "files.h"
#include "latexdocument.h"
#include "latexvisitor.h"
#include "system.h"

#if USE_EXPERIMENTAL_SOURCE == true
#include "geographicview.h"
#endif

#include <QCoreApplication>
#include <QApplication>
#include <QMessageBox>
#include <QUrl>
#include <QDesktopServices>

#include "boost/date_time/posix_time/posix_time.hpp"

using namespace std;
using namespace Utils;

ViewPointsReportGenerator::ViewPointsReportGenerator(const std::string& class_id, const std::string& instance_id,
                                                     ViewManager& view_manager)
    : Configurable(class_id, instance_id, &view_manager), view_manager_(view_manager)
{
    registerParameter("author", &author_, std::string());

    if (!author_.size())
        author_ = System::getUserName();
    if (!author_.size())
        author_ = "User";

    registerParameter("abstract", &abstract_, std::string());

    //@TODO: remove?
    //const DBConnection* db_con = dynamic_cast<const DBConnection*>(&COMPASS::instance().dbInterface().connection());
    //assert (db_con);

    string current_filename = COMPASS::instance().lastDbFilename();

    report_path_ = Files::getDirectoryFromPath(current_filename)+"/report_"
                + Files::getFilenameFromPath(current_filename) + "/";

    report_filename_ = "report.tex";

    loginf << "report path '" << report_path_ << "'"
           << " filename '"  << report_filename_ << "'";

    registerParameter("export_all_unsorted", &export_all_unsorted_, false);
    registerParameter("group_by_type", &group_by_type_, true);
    registerParameter("add_overview_table", &add_overview_table_, true);

    registerParameter("wait_on_map_loading", &wait_on_map_loading_, true);
    registerParameter("add_overview_screenshot", &add_overview_screenshot_, true);

    registerParameter("run_pdflatex", &run_pdflatex_, true);
    registerParameter("open_created_pdf", &open_created_pdf_, false);

    pdflatex_found_ = System::exec("which pdflatex").size(); // empty if none

    if (!pdflatex_found_)
    {
        run_pdflatex_ = false;
        open_created_pdf_ = false;
    }
}


void ViewPointsReportGenerator::generateSubConfigurable(const std::string& class_id,
                                                        const std::string& instance_id)
{
    throw std::runtime_error("ViewPointsReportGenerator: generateSubConfigurable: unknown class_id " +
                             class_id);
}

void ViewPointsReportGenerator::checkSubConfigurables()
{
    // move along sir
}


ViewPointsReportGeneratorDialog& ViewPointsReportGenerator::dialog()
{
    if (!dialog_)
        dialog_.reset(new ViewPointsReportGeneratorDialog(*this));

    return *dialog_;
}

void ViewPointsReportGenerator::run ()
{
    loginf;

    traced_assert(dialog_);
    dialog_->setRunning(true);

    try
    {
        LatexDocument doc (report_path_, report_filename_);
        doc.title("OpenATS COMPASS View Points Report");

        if (author_.size())
            doc.author(author_);

        if (abstract_.size())
            doc.abstract(abstract_);

        LatexVisitor visitor (doc, group_by_type_, add_overview_table_, add_overview_screenshot_, false,
                              false, 18, wait_on_map_loading_);

        cancel_ = false;
        running_ = true;
        pdf_created_ = false;

        QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));

        boost::posix_time::ptime start_time;
        boost::posix_time::ptime stop_time;
        boost::posix_time::time_duration time_diff;

        boost::posix_time::ptime vp_start_time;

        start_time = boost::posix_time::microsec_clock::local_time();

        ViewPointsWidget* vp_widget = view_manager_.viewPointsWidget();
        traced_assert(vp_widget);
        ViewPointsTableModel* table_model = vp_widget->tableModel();
        traced_assert(table_model);

        std::vector<unsigned int> vp_ids;

        if (export_all_unsorted_)
            vp_ids = vp_widget->viewPoints();
        else
            vp_ids = vp_widget->viewedViewPoints();

        string status_str, elapsed_time_str, remaining_time_str;
        DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

        unsigned int vp_cnt = 0;
        unsigned int vp_size = vp_ids.size();
        double ms;
        double ms_per_vp;

#if USE_EXPERIMENTAL_SOURCE == true
        GeographicView::instant_display_ = true;
#endif

        for (auto vp_id : vp_ids)
        {
            QCoreApplication::processEvents();

            if (cancel_)
            {
                loginf << "cancel";
                break;
            }

            traced_assert(table_model->hasViewPoint(vp_id));
            const ViewPoint& view_point = table_model->viewPoint(vp_id);

            loginf << "setting vp " << vp_id;
            view_manager_.setCurrentViewPoint(&view_point);

            while (dbcont_man.loadInProgress())
                QCoreApplication::processEvents();

            // do stuff
            view_point.accept(visitor);
            visitor.imagePrefix("vp_"+to_string(vp_id));

            // visit se views
            for (auto& view_it : view_manager_.getViews())
                view_it.second->accept(visitor);

            // update status
            stop_time = boost::posix_time::microsec_clock::local_time();

            time_diff = stop_time - start_time;
            ms = time_diff.total_milliseconds();
            elapsed_time_str =
                    String::timeStringFromDouble(ms / 1000.0, false);

            status_str = "Writing view point "+to_string(vp_cnt+1)+"/"+to_string(vp_size);

            if (vp_cnt && ms > 0)
            {
                ms_per_vp = ms/(double)vp_cnt;

                remaining_time_str = String::timeStringFromDouble((vp_size-vp_cnt) * ms_per_vp / 1000.0, false);

                loginf << "setting vp " << vp_id
                       << " done after " << elapsed_time_str << " remaining " << remaining_time_str;
            }
            else
            {
                loginf << "setting vp " << vp_id
                       << " done after " << elapsed_time_str;
            }

            dialog_->setElapsedTime(elapsed_time_str);
            dialog_->setProgress(0, vp_size, vp_cnt);
            dialog_->setStatus(status_str);
            dialog_->setRemainingTime(remaining_time_str);

            ++vp_cnt;
        }

        if (cancel_)
        {
            dialog_->setProgress(0, vp_size, 0);
            dialog_->setStatus("Writing view points cancelled");
            dialog_->setRemainingTime(String::timeStringFromDouble(0, false));

            QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
        }
        else // proceed
        {
            dialog_->setProgress(0, vp_size, vp_size);
            dialog_->setStatus("Writing view points done");
            dialog_->setRemainingTime(String::timeStringFromDouble(0, false));

            QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);

            doc.write();

            if (run_pdflatex_)
            {
                std::string command_out;
                std::string command = "cd "+report_path_+" && pdflatex --interaction=nonstopmode "+report_filename_
                        +" | awk 'BEGIN{IGNORECASE = 1}/warning|!/,/^$/;'";

                loginf << "running pdflatex";
                dialog_->setStatus("Running pdflatex");
                dialog_->setRemainingTime("");

                QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);

                logdbg << "cmd '" << command << "'";

                command_out = System::exec(command);

                logdbg << "cmd done";

                // update status
                stop_time = boost::posix_time::microsec_clock::local_time();

                time_diff = stop_time - start_time;
                ms = time_diff.total_milliseconds();
                elapsed_time_str = String::timeStringFromDouble(ms / 1000.0, false);
                dialog_->setElapsedTime(elapsed_time_str);

                while (command_out.find("Rerun to get outlines right") != std::string::npos
                       || command_out.find("Rerun to get cross-references right") != std::string::npos)
                {
                    loginf << "re-running pdflatex";
                    dialog_->setStatus("Re-running pdflatex");

                    QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);

                    command_out = System::exec(command);

                    time_diff = stop_time - start_time;
                    ms = time_diff.total_milliseconds();
                    elapsed_time_str = String::timeStringFromDouble(ms / 1000.0, false);
                    dialog_->setElapsedTime(elapsed_time_str);

                    logdbg << "re-run done";
                }

                loginf << "result '" << command_out << "'";

                if (!command_out.size()) // no warnings
                {
                    pdf_created_ = true;

                    dialog_->setStatus("Running pdflatex done");

                    if (open_created_pdf_)
                    {
                        std::string fullpath = report_path_+report_filename_;

                        if (String::hasEnding(fullpath, ".tex"))
                        {
                            String::replace(fullpath, ".tex", ".pdf");

                            loginf << "opening '" << fullpath << "'";

                            QDesktopServices::openUrl(QUrl(fullpath.c_str()));
                        }
                        else
                            logerr << "opening not possible since wrong file ending";
                    }
                }
                else // show warnings
                {
                    QMessageBox msgBox;
                    msgBox.setText("PDF Latex failed with warnings:\n\n"+QString(command_out.c_str()));
                    msgBox.exec();
                }
            }
        }
        dialog_->setRunning(false);
        dialog_->close();

        QApplication::restoreOverrideCursor();

#if USE_EXPERIMENTAL_SOURCE == true
        GeographicView::instant_display_ = false;
#endif

        if (show_done_)
        {
            QMessageBox msgBox;
            if (cancel_)
                msgBox.setText("Export View Points as PDF Cancelled");
            else
                msgBox.setText("Export View Points as PDF Done");

            msgBox.exec();
        }

        running_ = false;

    }
    catch (exception& e)
    {
        logwrn << "caught exception '" << e.what() << "'";

        dialog_->setProgress(0, 1, 0);
        dialog_->setStatus("Writing report failed");
        dialog_->setRemainingTime(String::timeStringFromDouble(0, false));

        dialog_->setRunning(false);
        dialog_->close();

        QApplication::restoreOverrideCursor();

        running_ = false;

        QMessageBox m_warning(QMessageBox::Warning, "Export PDF Failed",
                              (string("Error message:\n")+e.what()).c_str(),
                              QMessageBox::Ok);
        m_warning.exec();
    }
}

void ViewPointsReportGenerator::cancel ()
{
    loginf;

    cancel_ = true;

    if (!running_)
        dialog_->close();
}

std::string ViewPointsReportGenerator::reportPath() const
{
    return report_path_;
}

void ViewPointsReportGenerator::reportPath(const std::string& path)
{
    loginf << "'" << path << "'";
    report_path_ = path;

    if (dialog_)
        dialog_->updateFileInfo();
}

std::string ViewPointsReportGenerator::reportFilename() const
{
    return report_filename_;
}

void ViewPointsReportGenerator::reportFilename(const std::string& filename)
{
    loginf << "'" << filename << "'";
    report_filename_ = filename;

    if (dialog_)
        dialog_->updateFileInfo();
}

void ViewPointsReportGenerator::reportPathAndFilename(const std::string& str)
{
    report_path_ = Files::getDirectoryFromPath(str) + "/";
    report_filename_ = Files::getFilenameFromPath(str);

    loginf << "path '" << report_path_
           << "' filename '" << report_filename_ << "'";

    if (dialog_)
        dialog_->updateFileInfo();
}

bool ViewPointsReportGenerator::isRunning() const
{
    return running_;
}

void ViewPointsReportGenerator::showDone(bool show_done)
{
    show_done_ = show_done;
}

std::string ViewPointsReportGenerator::author() const
{
    return author_;
}

void ViewPointsReportGenerator::author(const std::string& author)
{
    author_ = author;
}

std::string ViewPointsReportGenerator::abstract() const
{
    return abstract_;
}

void ViewPointsReportGenerator::abstract(const std::string& abstract)
{
    abstract_ = abstract;
}

bool ViewPointsReportGenerator::exportAllUnsorted() const
{
    return export_all_unsorted_;
}

void ViewPointsReportGenerator::exportAllUnsorted(bool value)
{
    export_all_unsorted_ = value;
}

bool ViewPointsReportGenerator::runPDFLatex() const
{
    return run_pdflatex_;
}

void ViewPointsReportGenerator::runPDFLatex(bool value)
{
    run_pdflatex_ = value;
}

bool ViewPointsReportGenerator::pdfLatexFound() const
{
    return pdflatex_found_;
}

bool ViewPointsReportGenerator::openCreatedPDF() const
{
    return open_created_pdf_;
}

void ViewPointsReportGenerator::openCreatedPDF(bool value)
{
    open_created_pdf_ = value;
}

bool ViewPointsReportGenerator::groupByType() const
{
    return group_by_type_;
}

void ViewPointsReportGenerator::groupByType(bool value)
{
    group_by_type_ = value;
}

bool ViewPointsReportGenerator::addOverviewTable() const
{
    return add_overview_table_;
}

void ViewPointsReportGenerator::addOverviewTable(bool value)
{
    add_overview_table_ = value;
}

bool ViewPointsReportGenerator::addOverviewScreenshot() const
{
    return add_overview_screenshot_;
}

void ViewPointsReportGenerator::addOverviewScreenshot(bool value)
{
    add_overview_screenshot_ = value;
}

bool ViewPointsReportGenerator::waitOnMapLoading() const
{
    return wait_on_map_loading_;
}

void ViewPointsReportGenerator::waitOnMapLoading(bool value)
{
    wait_on_map_loading_ = value;
}

