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

#include "report/sectioncontent.h"

#include <QObject>
#include <QImage>

#include "json_fwd.hpp"

namespace report_figure
{
    struct RenderEntry;
}

namespace context
{
    class DBContextManager;
}

namespace ResultReport
{

/**
 */
class SectionContentFigure : public SectionContent
{
public:
    enum class FigureType
    {
        Section = 0, // figure referenced by section (e.g. rendered in pdf report)
        Hidden       // hidden figure referenced by content (e.g. by a table, not rendered in pdf report)
    };

    /// How the report GUI shows a figure and whether the export writes it, decided by its view point
    enum class FigureKind
    {
        Simple,        // no load_data: render widgets in the report GUI, one image per annotation in the export
        View,          // load_data: name and "View" button in the report GUI, left out of the export
        CannotBeDrawn  // no load_data, but no render widget can draw its view point: no image in the export
    };

    struct Classification
    {
        FigureKind                              kind = FigureKind::CannotBeDrawn;
        std::vector<report_figure::RenderEntry> render_entries; // one per annotation, empty unless simple
    };

    struct ImageResource
    {
        std::string name;
        std::string path;
        std::string link;
        QImage      data;
    };

    SectionContentFigure(unsigned int id,
                         FigureType figure_type,
                         const std::string& name, 
                         const SectionContentViewable& viewable,
                         Section* parent_section);
    SectionContentFigure(Section* parent_section);

    void setViewable(const SectionContentViewable& viewable);
    void setViewableFunc(const SectionContentViewable::ViewableFunc& func);

    virtual std::string resourceExtension() const override;

    virtual void addContentUI(QVBoxLayout* layout,
                              bool force_ui_reset) override;

    /// open_view_deck: a collapsed View Deck opens to show the figure (View button)
    bool view (bool load_blocking = false, bool open_view_deck = false) const;

    FigureType figureType() const { return fig_type_; }
    const std::string& caption() const { return caption_; }

    ResultT<std::vector<ImageResource>> obtainImages(const std::string* resource_dir) const;

    /// stored view point of the figure, loads on-demand content if needed (empty pointer if none)
    std::shared_ptr<nlohmann::json::object_t> viewableContent() const;

    /// kind of the figure and the render widgets that draw a simple figure, loads on-demand content first
    Classification classify() const;
    /// the sectors the geographic grids of the render widgets draw, nullptr without a task result
    const context::DBContextManager* sectorSource();

    static const std::string FieldFigureType;
    static const std::string FieldCaption;
    static const std::string FieldViewable;

    static const std::string FieldDocPath;
    static const std::string FieldDocData;

protected:
    void clearContent_impl() override final;

    void toJSON_impl(nlohmann::json& j) const override final;
    bool fromJSON_impl(const nlohmann::json& j) override final;
    Result toJSONDocument_impl(nlohmann::json& j,
                               const std::string* resource_dir,
                               ReportExportMode export_style) const override final;

    FigureType  fig_type_ = FigureType::Section;
    std::string caption_;

    SectionContentViewable::ViewableFunc viewable_fnc_;
};

}
