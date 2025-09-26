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

#include "latexvisitor.h"
#include "latexdocument.h"
#include "latexsection.h"
#include "latextable.h"

#include "dbcontent/dbcontentmanager.h"
#include "viewmanager.h"

#include "viewpoint.h"
#include "tableview.h"
#include "tableviewdatawidget.h"
#include "allbuffertablewidget.h"
#include "histogramview.h"
#include "histogramviewdatawidget.h"
#include "scatterplotview.h"
#include "scatterplotviewdatawidget.h"
#include "gridview.h"
#include "gridviewdatawidget.h"

#include "task/result/report/section.h"
#include "task/result/report/sectioncontenttable.h"
#include "task/result/report/sectioncontenttext.h"
#include "task/result/report/sectioncontentfigure.h"

#include "logger.h"
#include "stringconv.h"
#include "json_tools.h"
#include "files.h"
#include "compass.h"

#if USE_EXPERIMENTAL_SOURCE == true
#include "ViewerWidget.h"
#include "geographicview.h"
#include "geographicviewdatawidget.h"
#endif

#include <QCoreApplication>
#include <QApplication>
#include <QThread>

#include <sstream>

#include <QPixmap>

using namespace std;
using namespace Utils;

/**
 */
LatexVisitor::LatexVisitor(LatexDocument& report, 
                           bool group_by_type, 
                           bool add_overview_table,
                           bool add_overview_screenshot, 
                           bool include_target_details,
                           bool include_target_tr_details, 
                           unsigned int max_table_col_width, 
                           bool wait_on_map_loading)
:   report_                   (report                   )
,   group_by_type_            (group_by_type            )
,   add_overview_table_       (add_overview_table       )
,   add_overview_screenshot_  (add_overview_screenshot  )
,   include_target_details_   (include_target_details   )
,   include_target_tr_details_(include_target_tr_details)
,   max_table_col_width_      (max_table_col_width      )
,   wait_on_map_loading_      (wait_on_map_loading      )
{
}

/**
 */
void LatexVisitor::visit(const ViewPoint* e)
{
    traced_assert(e);

    loginf << "ViewPoint id " << e->id(); 

    const nlohmann::json& j_data = e->data();

    traced_assert(j_data.contains(ViewPoint::VP_NAME_KEY));
    string name = String::latexString(j_data.at(ViewPoint::VP_NAME_KEY));

    traced_assert(j_data.contains(ViewPoint::VP_TYPE_KEY));
    string type = String::latexString(j_data.at(ViewPoint::VP_TYPE_KEY));

    traced_assert(j_data.contains(ViewPoint::VP_STATUS_KEY));
    string status = j_data.at(ViewPoint::VP_STATUS_KEY);

    string comment;

    if (j_data.contains(ViewPoint::VP_COMMENT_KEY))
        comment = String::latexString(j_data.at(ViewPoint::VP_COMMENT_KEY));

    // overview table
    if (add_overview_table_)
        report_.getSection("Overview"); // add before other section

    if (group_by_type_)
        current_section_name_ = "View Points:"+type+":ID "+to_string(e->id())+" "+name;
    else
        current_section_name_ = "View Points:ID "+to_string(e->id())+" "+name;

    LatexSection& sec = report_.getSection(current_section_name_);

    // set label
    std::string current_section_label = "sec:view-point-"+to_string(e->id());
    //boost::replace_all(current_section_label, " ", "-");
    sec.label(current_section_label);

    // details table

    sec.addTable("Info", 2, {"Key","Value"}, "| l | X |");
    LatexTable& info_table = sec.getTable("Info");

    info_table.addRow({ViewPoint::VP_ID_KEY, to_string(e->id())});
    info_table.addRow({ViewPoint::VP_NAME_KEY, j_data.at(ViewPoint::VP_NAME_KEY)});
    info_table.addRow({ViewPoint::VP_TYPE_KEY, j_data.at(ViewPoint::VP_TYPE_KEY)});
    info_table.addRow({ViewPoint::VP_STATUS_KEY, status});

    for (auto& j_it : j_data.items())
    {
        if (!j_it.value().is_primitive())
            continue;

        if (j_it.key() == ViewPoint::VP_ID_KEY || j_it.key() == ViewPoint::VP_NAME_KEY
                || j_it.key() == ViewPoint::VP_TYPE_KEY || j_it.key() == ViewPoint::VP_STATUS_KEY)
            continue;

        info_table.addRow({j_it.key(), JSON::toString(j_it.value())});
    }

    // write overview table entry

    if (add_overview_table_)
    {
        LatexSection& overview_sec = report_.getSection("Overview");

        if (!overview_sec.hasTable("ViewPoints Overview"))
            overview_sec.addTable("ViewPoints Overview", 6,
                                  {ViewPoint::VP_ID_KEY , ViewPoint::VP_NAME_KEY, ViewPoint::VP_TYPE_KEY,
                                   ViewPoint::VP_STATUS_KEY, ViewPoint::VP_COMMENT_KEY, ""},
                                  "| l | l | l | l | X | l |", false);

        LatexTable& overview_table = overview_sec.getTable("ViewPoints Overview");

        overview_table.addRow({
                                  to_string(e->id()),
                                  name,
                                  type,
                                  status,
                                  comment,
                                  R"(\hyperref[)"+current_section_label+"]{Link}"
                              });
    }
}

/**
 */
void LatexVisitor::visit(TableView* e)
{
    traced_assert(e);

    loginf << "TableView " << e->instanceId();

    if (ignore_table_views_)
        return;

    AllBufferTableWidget* allbuf = e->getDataWidget()->getAllBufferTableWidget();
    traced_assert(allbuf);

    std::vector<std::vector<std::string>> data = allbuf->getSelectedText();

    if (data.size())
    {
        LatexSection& sec = report_.getSection(current_section_name_);

        sec.addTable(e->instanceId(), data.at(0).size(), data.at(0));
        LatexTable& table = sec.getTable(e->instanceId());
        table.setWideTable(true);

        data.erase(data.begin()); // remove first header row#

        for (auto& row_it : data)
            table.addRow(row_it);
    }
    else
    {
        data = allbuf->getText();

        if (data.size())
        {
            LatexSection& sec = report_.getSection(current_section_name_);

            sec.addTable(e->instanceId(), data.at(0).size(), data.at(0));
            LatexTable& table = sec.getTable(e->instanceId());
            table.setWideTable(true);

            data.erase(data.begin()); // remove first header row#

            for (auto& row_it : data)
                table.addRow(row_it);
        }
    }
}

/**
 */
void LatexVisitor::visit(HistogramView* e)
{
    traced_assert(e);

    loginf << "HistogramView " << e->instanceId();

    std::string screenshot_path = report_.path()+"/screenshots";

    loginf << "path '" << screenshot_path << "'";

    if (!screenshot_folder_created_)
    {
        bool ret = Files::createMissingDirectories(screenshot_path);

        if (!ret)
            throw runtime_error("LatexVisitor: visit: HistogramView: unable to create directories for '"
                                +screenshot_path+"'");

        screenshot_folder_created_ = true;
    }

    e->showInTabWidget();

    HistogramViewDataWidget* data_widget = e->getDataWidget();
    traced_assert(data_widget);

    loginf << "start" << e->instanceId() 
           << " has visible content: " << data_widget->hasVisibleContent()
           << " has data " << data_widget->hasData()
           << " has annotations " << data_widget->hasAnnotations()
           << " is drawn " << data_widget->isDrawn();

    if (!data_widget->hasVisibleContent())
        return;

    // normal screenshot
    QPixmap pmap = data_widget->renderPixmap();

    QImage screenshot = pmap.toImage();

    std::string image_path = screenshot_path+"/"+image_prefix_+"_"+e->instanceId()+".jpg";
    traced_assert(!screenshot.isNull());

    loginf << "saving screenshot as '" << image_path << "'";
    bool ret = Files::createMissingDirectories(Files::getDirectoryFromPath(image_path));

    if (!ret)
        throw runtime_error("LatexVisitor: visit: HistogramView: unable to create directories for '"
                            +image_path+"'");

    ret = screenshot.save(image_path.c_str(), "JPG"); // , 50
    traced_assert(ret);

    LatexSection& sec = report_.getSection(current_section_name_);

    // add normal screenshot after overview
    sec.addImage(image_path, e->instanceId());
}

#if USE_EXPERIMENTAL_SOURCE == true

/**
 */
void LatexVisitor::visit(GeographicView* e)
{
    traced_assert(e);
    loginf << "GeographicView " << e->instanceId();

    std::string screenshot_path = report_.path()+"/screenshots";

    loginf << "path '" << screenshot_path << "'";

    if (!screenshot_folder_created_)
    {
        bool ret = Files::createMissingDirectories(screenshot_path);

        if (!ret)
            throw runtime_error("LatexVisitor: visit: GeographicView: unable to create directories for '"
                                +screenshot_path+"'");

        screenshot_folder_created_ = true;
    }

    e->showInTabWidget();

    GeographicViewDataWidget* data_widget = e->getDataWidget();
    traced_assert(data_widget);

    loginf << "start" << e->instanceId() 
           << " has screenshot content: " << data_widget->hasScreenshotContent()
           << " has data " << data_widget->hasData()
           << " has annotations " << data_widget->hasAnnotations()
           << " is drawn " << data_widget->isDrawn();

    if (!data_widget->hasScreenshotContent())
        return;

    if (wait_on_map_loading_)
        data_widget->waitUntilMapLoaded();

    const int RenderDelayMSec = 2000;

    //wait a little for e.g. geoimages to warp and render correctly
    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();
    while ((boost::posix_time::microsec_clock::local_time() - start_time).total_milliseconds()
            < RenderDelayMSec)
    {
        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
        QThread::msleep(10);
    }

    //data_widget->clearMouseCoordinates();

    // normal screenshot

    QImage screenshot = data_widget->osgViewerWidget()->grabFrameBuffer();

    std::string image_path = screenshot_path+"/"+image_prefix_+"_"+e->instanceId()+".jpg";
    traced_assert(!screenshot.isNull());

    loginf << "saving screenshot as '" << image_path << "'";
    bool ret = Files::createMissingDirectories(Files::getDirectoryFromPath(image_path));

    if (!ret)
        throw runtime_error("LatexVisitor: visit: GeographicView: unable to create directories for '"
                            +image_path+"'");

    ret = screenshot.save(image_path.c_str(), "JPG"); // , 50
    traced_assert(ret);

    LatexSection& sec = report_.getSection(current_section_name_);

    if (add_overview_screenshot_) // overview screenshot
    {
        data_widget->zoomToDataSlot(); // TODO wrong
        data_widget->addDataMarker();

        QImage overview_screenshot = data_widget->osgViewerWidget()->grabFrameBuffer();

        data_widget->removeDataMarker();

        std::string overview_image_path = screenshot_path+"/"+image_prefix_+"_overview_"+e->instanceId()+".jpg";
        traced_assert(!overview_screenshot.isNull());

        loginf << "saving overview screenshot as '" << overview_image_path << "'";
        ret = overview_screenshot.save(overview_image_path.c_str(), "JPG"); // , 50
        traced_assert(ret);

        sec.addImage(overview_image_path, e->instanceId()+" Overview");
    }

    // add normal screenshot after overview
    sec.addImage(image_path, e->instanceId());
}

#endif

/**
 */
void LatexVisitor::visit(ScatterPlotView* e)
{
    traced_assert(e);

    loginf << "ScatterPlotView " << e->instanceId();

    std::string screenshot_path = report_.path()+"/screenshots";

    loginf << "path '" << screenshot_path << "'";

    if (!screenshot_folder_created_)
    {
        bool ret = Files::createMissingDirectories(screenshot_path);

        if (!ret)
            throw runtime_error("LatexVisitor: visit: ScatterPlotView: unable to create directories for '"
                                +screenshot_path+"'");

        screenshot_folder_created_ = true;
    }

    e->showInTabWidget();

    ScatterPlotViewDataWidget* data_widget = e->getDataWidget();
    traced_assert(data_widget);

    loginf << "start" << e->instanceId() 
           << " has visible content: " << data_widget->hasVisibleContent()
           << " has data " << data_widget->hasData()
           << " has annotations " << data_widget->hasAnnotations()
           << " is drawn " << data_widget->isDrawn();

    if (!data_widget->hasVisibleContent())
        return;

    // normal screenshot
    QPixmap pmap = data_widget->renderPixmap();

    QImage screenshot = pmap.toImage();

    std::string image_path = screenshot_path+"/"+image_prefix_+"_"+e->instanceId()+".jpg";
    traced_assert(!screenshot.isNull());

    loginf << "saving screenshot as '" << image_path << "'";
    bool ret = Files::createMissingDirectories(Files::getDirectoryFromPath(image_path));

    if (!ret)
        throw runtime_error("LatexVisitor: visit: ScatterPlotView: unable to create directories for '"
                            +image_path+"'");

    ret = screenshot.save(image_path.c_str(), "JPG"); // , 50
    traced_assert(ret);

    LatexSection& sec = report_.getSection(current_section_name_);

    // add normal screenshot after overview
    sec.addImage(image_path, e->instanceId());
}

/**
 */
void LatexVisitor::visit(GridView* e)
{
    traced_assert(e);

    loginf << "GridView " << e->instanceId();

    std::string screenshot_path = report_.path()+"/screenshots";

    loginf << "path '" << screenshot_path << "'";

    if (!screenshot_folder_created_)
    {
        bool ret = Files::createMissingDirectories(screenshot_path);

        if (!ret)
            throw runtime_error("LatexVisitor: visit: GridView: unable to create directories for '"
                                +screenshot_path+"'");

        screenshot_folder_created_ = true;
    }

    e->showInTabWidget();

    GridViewDataWidget* data_widget = e->getDataWidget();
    traced_assert(data_widget);

    loginf << "start" << e->instanceId() 
           << " has visible content: " << data_widget->hasVisibleContent()
           << " has data " << data_widget->hasData()
           << " has annotations " << data_widget->hasAnnotations()
           << " is drawn " << data_widget->isDrawn();

    if (!data_widget->hasVisibleContent())
        return;

    // normal screenshot
    auto screenshot = data_widget->renderData();

    std::string image_path = screenshot_path+"/"+image_prefix_+"_"+e->instanceId()+".jpg";
    traced_assert(!screenshot.isNull());

    loginf << "saving screenshot as '" << image_path << "'";
    bool ret = Files::createMissingDirectories(Files::getDirectoryFromPath(image_path));

    if (!ret)
        throw runtime_error("LatexVisitor: visit: GridView: unable to create directories for '"
                            +image_path+"'");

    ret = screenshot.save(image_path.c_str(), "JPG"); // , 50
    traced_assert(ret);

    LatexSection& sec = report_.getSection(current_section_name_);

    // add normal screenshot after overview
    sec.addImage(image_path, e->instanceId());
}

/**
 */
void LatexVisitor::imagePrefix(const std::string& image_prefix)
{
    image_prefix_ = image_prefix;
}
