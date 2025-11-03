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

#include "task/result/report/sectioncontent.h"

#include <QObject>
#include <QImage>

#include "json_fwd.hpp"

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

    bool view (bool load_blocking = false) const;
    void executeRenderDelay() const;

    FigureType figureType() const { return fig_type_; }
    const std::string& caption() const { return caption_; }
    int renderDelayMSec() const { return render_delay_msec_; }

    ResultT<std::vector<ImageResource>> obtainImages(const std::string* resource_dir) const;

    static const std::string FieldFigureType;
    static const std::string FieldCaption;
    static const std::string FieldRenderDelayMSec;
    static const std::string FieldViewable;

    static const std::string FieldDocPath;
    static const std::string FieldDocData;

protected:
    std::shared_ptr<nlohmann::json::object_t> viewableContent() const;

    void clearContent_impl() override final;

    void toJSON_impl(nlohmann::json& j) const override final;
    bool fromJSON_impl(const nlohmann::json& j) override final;
    Result toJSONDocument_impl(nlohmann::json& j,
                               const std::string* resource_dir,
                               ReportExportMode export_style) const override final;

    FigureType  fig_type_          = FigureType::Section;
    std::string caption_;
    int         render_delay_msec_ = 0;

    SectionContentViewable::ViewableFunc viewable_fnc_;
};

}
