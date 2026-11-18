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

#include "report/sectioncontentfigure.h"
#include "report/section.h"
#include "report/report.h"
#include "report/sectionid.h"
#include "report/reportexporter.h"
#include "taskresult.h"

#include "itaskprovider.h"
#include "viewmanager.h"
#include "viewpointgenerator.h"
#include "reportfigure.h"
#include "viewrenderwidget.h"
#include "ui_test_common.h"

#include "logger.h"
#include "stringconv.h"
#include "files.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QApplication>
#include <QThread>
#include <QImage>

#include <algorithm>

namespace ResultReport
{

const std::string SectionContentFigure::FieldFigureType      = "figure_type";
const std::string SectionContentFigure::FieldCaption         = "caption";
const std::string SectionContentFigure::FieldRenderDelayMSec = "render_delay_msec";
const std::string SectionContentFigure::FieldViewable        = "viewable";

const std::string SectionContentFigure::FieldDocPath         = "path";
const std::string SectionContentFigure::FieldDocData         = "data_base64";

/**
 * Ctor passing an existing viewable.
 */
SectionContentFigure::SectionContentFigure(unsigned int id,
                                           FigureType figure_type,
                                           const std::string& name, 
                                           const SectionContentViewable& viewable,
                                           Section* parent_section)
:   SectionContent(ContentType::Figure, id, name, parent_section)
,   fig_type_     (figure_type)
{
    setViewable(viewable);
}

/**
 */
SectionContentFigure::SectionContentFigure(Section* parent_section)
:   SectionContent(ContentType::Figure, parent_section)
{
}

/**
 */
void SectionContentFigure::setViewable(const SectionContentViewable& viewable)
{
    caption_           = viewable.caption;
    render_delay_msec_ = viewable.render_delay_msec;
    viewable_fnc_      = viewable.viewable_func;
}

/**
 */
void SectionContentFigure::setViewableFunc(const SectionContentViewable::ViewableFunc& func)
{
    viewable_fnc_ = func;
}

/**
 */
std::string SectionContentFigure::resourceExtension() const
{
    return ReportExporter::ExportImageFormat;
}

/**
 */
void SectionContentFigure::addContentUI(QVBoxLayout* layout, 
                                        bool force_ui_reset)
{
    traced_assert(layout);

    if (isLocked())
    {
        layout->addWidget(lockStatePlaceholderWidget());
        return;
    }

    QLabel* label = new QLabel(("Figure: " + name()).c_str());

    QPushButton* view_button = new QPushButton("View");
    view_button->setIcon(QIcon());
    view_button->setToolTip("Show the figure in the views");
    UI_TEST_OBJ_NAME(view_button, QString::fromStdString("Figure " + name() + " View Button"));

    QObject::connect (view_button, &QPushButton::clicked, [ this ] () { this->view(false, true); });

    const auto entries = renderEntries();

    if (entries.empty())
    {
        QHBoxLayout* fig_layout = new QHBoxLayout();

        fig_layout->addWidget(label);
        fig_layout->addStretch();
        fig_layout->addWidget(view_button);

        layout->addLayout(fig_layout);
        return;
    }

    layout->addWidget(label);

    const bool geographic = std::any_of(entries.begin(), entries.end(),
                                        [] (const report_figure::RenderEntry& e) { return e.geographic; });

    const auto* ctx_man = geographic ? sectorSource() : nullptr;

    for (const auto& entry : entries)
    {
        ViewRenderWidget* render_widget = report_figure::createRenderWidget(entry, ctx_man).release();
        UI_TEST_OBJ_NAME(render_widget, QString::fromStdString("Figure " + name() + " " + entry.image_name));

        layout->addWidget(render_widget);
    }

    QHBoxLayout* button_layout = new QHBoxLayout();

    button_layout->addStretch();
    button_layout->addWidget(view_button);

    layout->addLayout(button_layout);
}

/**
 */
const context::DBContextManager* SectionContentFigure::sectorSource()
{
    auto result = taskResult();
    if (!result)
        return nullptr;

    return &result->taskManager().viewManager().dbContextManager();
}

/**
 */
std::vector<report_figure::RenderEntry> SectionContentFigure::renderEntries() const
{
    //the content of an on-demand figure is not built only to draw it, such a figure stays with the views
    if (isOnDemand())
        return {};

    auto content = viewableContent();
    if (!content)
        return {};

    return report_figure::selectRenderEntries(nlohmann::json(*content));
}

/**
 */
bool SectionContentFigure::view(bool load_blocking, bool open_view_deck) const
{
    loginf << "viewing figure '" << name() << "'";

    if (isLocked())
    {
        loginf << "on-demand figure is locked";
        return false;
    }

    auto content = viewableContent();
    if (!content || content->empty())
    {
        loginf << "no content";
        report_->unsetCurrentViewable();
        return false;
    }

    //view content
    logdbg << "content: " << nlohmann::json(*content).dump(4);
    report_->setCurrentViewable(*content, load_blocking, open_view_deck);

    return true;
}

/**
 */
void SectionContentFigure::executeRenderDelay() const
{
    if (render_delay_msec_ > 0)
    {
        boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();
        while ((boost::posix_time::microsec_clock::local_time() - start_time).total_milliseconds()
               < render_delay_msec_)
        {
            QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
            QThread::msleep(1);
        }
    }
}

/**
 */
std::shared_ptr<nlohmann::json::object_t> SectionContentFigure::viewableContent() const
{
    //load on-demand content if needed (@TODO: eeewwwwww)
    auto this_unconst = const_cast<SectionContentFigure*>(this);
    this_unconst->loadOnDemandIfNeeded();

    //still no viewable? => return empty data
    if (!viewable_fnc_)
        return std::shared_ptr<nlohmann::json::object_t>();

    //call viewable function
    return viewable_fnc_();
}

/**
 */
void SectionContentFigure::clearContent_impl()
{
    viewable_fnc_ = {};
}

/**
 */
void SectionContentFigure::toJSON_impl(nlohmann::json& j) const
{
    //call base
    SectionContent::toJSON_impl(j);

    j[ FieldFigureType      ] = fig_type_;
    j[ FieldCaption         ] = caption_;
    j[ FieldRenderDelayMSec ] = render_delay_msec_;

    nlohmann::json jviewable = nlohmann::json::object();
    if (!isOnDemand())
    {
        auto viewable = viewableContent();
        if (viewable)
            jviewable = *viewable;
    }

    j[ FieldViewable ] = jviewable;
}

/**
 */
bool SectionContentFigure::fromJSON_impl(const nlohmann::json& j)
{
    //call base
    if (!SectionContent::fromJSON_impl(j))
        return false;

    if (!j.is_object()                    ||
        !j.contains(FieldFigureType)      ||
        !j.contains(FieldCaption)         ||
        !j.contains(FieldRenderDelayMSec) ||
        !j.contains(FieldViewable))
    {
        logerr << "section content figure does not obtain needed fields";
        return false;
    }

    fig_type_          = j[ FieldFigureType      ];
    caption_           = j[ FieldCaption         ];
    render_delay_msec_ = j[ FieldRenderDelayMSec ];

    std::shared_ptr<nlohmann::json::object_t> jviewable(new nlohmann::json::object_t);
    *jviewable = j[ FieldViewable ];

    viewable_fnc_ = [ = ] () { return jviewable; };

    return true;
}

/**
 */
ResultT<std::vector<SectionContentFigure::ImageResource>> SectionContentFigure::obtainImages(const std::string* resource_dir) const
{
    traced_assert(!isLocked());

    std::vector<SectionContentFigure::ImageResource> resources;

    auto renderings = taskResult()->renderFigure(*this);

    for (const auto& r : renderings)
    {
        ImageResource img_res;
        img_res.data = r.first;
        img_res.name = r.second;
        
        if (resource_dir)
        {
            auto res = prepareResource(*resource_dir, ResourceDir::Screenshots, r.second);
            if (!res.ok())
                return res;

            if (!r.first.save(QString::fromStdString(res.result().path), nullptr, ReportExporter::ExportImageQuality))
                return Result::failed("Could not store resource for content '" + name() + "'");

            img_res.link = res.result().link;
            img_res.path = res.result().path;
        }
        
        resources.push_back(img_res);
    }

    return ResultT<std::vector<SectionContentFigure::ImageResource>>::succeeded(resources);
}

/**
 */
Result SectionContentFigure::toJSONDocument_impl(nlohmann::json& j,
                                                 const std::string* resource_dir,
                                                 ReportExportMode export_style) const
{
    j = nlohmann::json::array();

    auto resources = obtainImages(resource_dir);
    if (!resources.ok())
        return resources;

    for (const auto& r : resources.result())
    {
        nlohmann::json j_fig;
        SectionContent::toJSONDocument_impl(j_fig, resource_dir, export_style);

        if (resource_dir)
        {
            //store link to file
            j_fig[ FieldDocPath ] = r.link;
        }
        else
        {
            //store data as json encoding
            j_fig[ FieldDocData ] = ViewPointGenFeatureGeoImage::imageToByteString(r.data, "PNG");
        }

        j.push_back(j_fig);
    }

    return Result::succeeded();
}

}
