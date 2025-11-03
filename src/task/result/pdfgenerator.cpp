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

#include "eval/results/report/pdfgenerator.h"
#include "eval/results/report/pdfgeneratordialog.h"
#include "evaluationmanager.h"
#include "compass.h"
#include "global.h"
#include "dbinterface.h"
#include "logger.h"
#include "files.h"
#include "stringconv.h"
#include "dbconnection.h"
#include "latexdocument.h"
#include "latexvisitor.h"
#include "latextable.h"

#include "eval/results/report/section.h"
#include "eval/results/report/rootitem.h"
#include "eval/results/evaluationresultsgenerator.h"

#if USE_EXPERIMENTAL_SOURCE == true
#include "geographicview.h"
#endif

#include <QCoreApplication>
#include <QApplication>
#include <QMessageBox>
#include <QUrl>
#include <QDesktopServices>

#include "boost/date_time/posix_time/posix_time.hpp"

#include <system.h>

using namespace std;
using namespace Utils;

namespace EvaluationResultsReport
{

PDFGenerator::PDFGenerator(EvaluationManager& eval_manager, EvaluationManagerSettings& eval_settings)
    : eval_man_(eval_manager), eval_settings_(eval_settings)
{
    report_filename_ = "report.tex";

    pdflatex_found_ = System::exec("which pdflatex").size(); // empty if none
}

PDFGeneratorDialog& PDFGenerator::dialog()
{
    if (!report_path_.size())
    {
        //const DBConnection* db_con = dynamic_cast<const DBConnection*>(&COMPASS::instance().dbInterface().connection());
        //assert (db_con);
        //@TODO: PWa: did not understand why we need to check the connection at this point?
        traced_assert(COMPASS::instance().dbInterface().ready());

        string current_filename = COMPASS::instance().lastDbFilename();

        string sub_path = Files::getFilenameFromPath(current_filename);
        std::replace(sub_path.begin(), sub_path.end(), ' ', '_'); // replace all 'x' to 'y'

        report_path_ = Files::getDirectoryFromPath(current_filename)+"/eval_report_"
                    + sub_path + "/";
        loginf << "report path '" << report_path_ << "'"
               << " filename '"  << report_filename_ << "'";
    }

    if (!dialog_)
        dialog_.reset(new PDFGeneratorDialog(*this, eval_man_, eval_settings_));

    return *dialog_;
}

bool PDFGenerator::pdfLatexFound() const
{
    return pdflatex_found_;
}

void PDFGenerator::run ()
{
    loginf;

    traced_assert(dialog_);
    dialog_->setRunning(true);

    try
    {

        LatexDocument doc (report_path_, report_filename_);
        doc.title("OpenATS COMPASS Evaluation Report");

        if (eval_settings_.report_author_.size())
            doc.author(eval_settings_.report_author_);

        if (eval_settings_.report_abstract_.size())
            doc.abstract(eval_settings_.report_abstract_);

        LatexTable::num_max_rows_ = eval_settings_.report_num_max_table_rows_;
        LatexVisitor visitor (doc, false, false, false,
                              eval_settings_.report_include_target_details_,
                              eval_settings_.report_include_target_tr_details_,
                              eval_settings_.report_num_max_table_col_width_,
                              eval_settings_.report_wait_on_map_loading_);

        cancel_ = false;
        running_ = true;
        pdf_created_ = false;

        QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));

        boost::posix_time::ptime start_time;
        boost::posix_time::ptime stop_time;
        boost::posix_time::time_duration time_diff;

        //boost::posix_time::ptime vp_start_time;

        start_time = boost::posix_time::microsec_clock::local_time();

        traced_assert(eval_man_.hasResults());
        std::shared_ptr<Section> root_section = eval_man_.resultsGenerator().resultsModel().rootItem()->rootSection();


        string status_str, elapsed_time_str, remaining_time_str;

        // create sections
        vector<shared_ptr<Section>> sections;
        sections.push_back(root_section);
        root_section->addSectionsFlat(sections, eval_settings_.report_include_target_details_,
                                      eval_settings_.report_skip_targets_wo_issues_);

        unsigned int num_sections = sections.size();

        unsigned int vp_cnt = 0;
        double ms;
        double ms_per_sec;

#if USE_EXPERIMENTAL_SOURCE == true
        GeographicView::instant_display_ = true;
#endif

        for (auto& sec_it : sections)
        {
            QCoreApplication::processEvents();

            if (cancel_)
            {
                loginf << "cancel";
                break;
            }

            loginf << "section '" << sec_it->heading() << "'";

            sec_it->accept(visitor);

            // update status
            stop_time = boost::posix_time::microsec_clock::local_time();

            time_diff = stop_time - start_time;
            ms = time_diff.total_milliseconds();
            elapsed_time_str =
                    String::timeStringFromDouble(ms / 1000.0, false);

            status_str = "Writing section "+to_string(vp_cnt+1)+"/"+to_string(num_sections);

            if (vp_cnt && ms > 0)
            {
                ms_per_sec = ms/(double)vp_cnt;

                remaining_time_str = String::timeStringFromDouble((num_sections-vp_cnt) * ms_per_sec / 1000.0, false);

                loginf << "section " << sec_it->heading()
                       << " done after " << elapsed_time_str << " remaining " << remaining_time_str;
            }
            else
            {
                loginf << "section " << sec_it->heading()
                       << " done after " << elapsed_time_str;

            }

            dialog_->setElapsedTime(elapsed_time_str);
            dialog_->setProgress(0, num_sections, vp_cnt);
            dialog_->setStatus(status_str);
            dialog_->setRemainingTime(remaining_time_str);

            ++vp_cnt;
        }

        if (cancel_)
        {
            dialog_->setProgress(0, num_sections, 0);
            dialog_->setStatus("Writing section cancelled");
            dialog_->setRemainingTime(String::timeStringFromDouble(0, false));

            QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
        }
        else // proceed
        {
            dialog_->setProgress(0, num_sections, num_sections);
            dialog_->setStatus("Writing sections done");
            dialog_->setRemainingTime(String::timeStringFromDouble(0, false));

            QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);

            doc.write();

            if (eval_settings_.report_run_pdflatex_)
            {
                std::string command_out;
                std::string command = "cd \""+report_path_+"\" && pdflatex --interaction=nonstopmode \""+report_filename_
                        +"\" | awk 'BEGIN{IGNORECASE = 1}/warning|!/,/^$/;'";

                loginf << "running pdflatex";
                dialog_->setStatus("Running pdflatex");
                dialog_->setRemainingTime("");

                QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);

                loginf << "cmd '" << command << "'";

                command_out = System::exec(command);

                logdbg << "cmd done";

                // update status
                stop_time = boost::posix_time::microsec_clock::local_time();

                time_diff = stop_time - start_time;
                ms = time_diff.total_milliseconds();
                elapsed_time_str = String::timeStringFromDouble(ms / 1000.0, false);
                dialog_->setElapsedTime(elapsed_time_str);

                unsigned int run_cnt=0;

                while (run_cnt < 3 || (command_out.find("Rerun to get outlines right") != std::string::npos
                                       || command_out.find("Rerun to get cross-references right") != std::string::npos))
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

                    ++run_cnt;
                }

                loginf << "result '" << command_out << "'";

                if (!command_out.size()) // no warnings
                {
                    pdf_created_ = true;

                    dialog_->setStatus("Running pdflatex done");

                    if (eval_settings_.report_open_created_pdf_)
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

void PDFGenerator::cancel ()
{
    loginf;

    cancel_ = true;

    if (!running_)
        dialog_->close();
}

std::string PDFGenerator::reportPath() const
{
    return report_path_;
}

void PDFGenerator::reportPath(const std::string& path)
{
    loginf << "'" << path << "'";
    report_path_ = path;

    if (dialog_)
        dialog_->updateFileInfo();
}

std::string PDFGenerator::reportFilename() const
{
    return report_filename_;
}

void PDFGenerator::reportFilename(const std::string& filename)
{
    loginf << "'" << filename << "'";
    report_filename_ = filename;

    if (dialog_)
        dialog_->updateFileInfo();
}

void PDFGenerator::reportPathAndFilename(const std::string& str)
{
    report_path_ = Files::getDirectoryFromPath(str) + "/";
    report_filename_ = Files::getFilenameFromPath(str);

    loginf << "path '" << report_path_
           << "' filename '" << report_filename_ << "'";

    if (dialog_)
        dialog_->updateFileInfo();
}

bool PDFGenerator::isRunning() const
{
    return running_;
}

void PDFGenerator::showDone(bool show_done)
{
    show_done_ = show_done;
}


}
