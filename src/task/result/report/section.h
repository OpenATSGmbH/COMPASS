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

#include "task/result/report/treeitem.h"
#include "task/result/report/sectioncontent.h"
#include "property.h"

#include <QWidget>

#include "json_fwd.hpp"

#include <memory>
#include <vector>
#include <bitset>

#include <boost/optional.hpp>

class TaskManager;
class DBInterface;

namespace ResultReport
{

class Report;
class SectionContentText;
class SectionContentTable;
class SectionContentFigure;
struct SectionContentViewable;

/**
 */
class Section : public TreeItem
{
public:
    enum ContentInfoFlag
    {
        ContentAvailable        = 1 << 0,
        ContentLoaded           = 1 << 1,
        ContentOnDemand         = 1 << 2,
        ContentOnDemandComplete = 1 << 3
    };

    typedef std::bitset<32> ExportFlags;

    Section(const std::string& heading, 
            const std::string& parent_heading, 
            TreeItem* parent_item,
            Report* report);
    Section(TreeItem* parent_item,
            Report* report);

    Section* parentSection();
    const Section* parentSection() const;

    virtual TreeItem* child(int row) override;
    virtual int childCount() const override;
    virtual int columnCount() const override;
    virtual QVariant data(int column) const override;
    virtual int row() const override;

    std::string heading() const;
    std::string compoundHeading() const; // "head1:head2" or "head1", starts with "Results"
    std::string compoundResultsHeading() const; // without "Results", can be ""

    bool hasSubSection (const std::string& heading);
    Section& getSubSection (const std::string& heading);
    Section& addSubSection (const std::string& heading);
    std::vector<std::shared_ptr<Section>> subSections(bool recursive) const;
    std::string relativeID(const std::string& id) const;

    QWidget* getContentWidget(bool preload_ondemand_contents);
    void updateContents(bool recursive);

    bool hasText (const std::string& name);
    SectionContentText& getText (const std::string& name);
    SectionContentText& addText (const std::string& name);
    size_t numTexts() const;

    bool hasTable (const std::string& name);
    SectionContentTable& getTable (const std::string& name);
    SectionContentTable& addTable (const std::string& name, 
                                   unsigned int num_columns, 
                                   const std::vector<std::string> headings,
                                   bool sortable=true, 
                                   unsigned int sort_column=0, 
                                   Qt::SortOrder order=Qt::AscendingOrder);
    std::vector<std::string> getTableNames() const;
    size_t numTables() const;

    bool hasFigure (const std::string& name);
    SectionContentFigure& getFigure (const std::string& name);
    SectionContentFigure& addFigure (const std::string& name, 
                                     const SectionContentViewable& viewable);
    std::vector<SectionContentFigure*> getFigures();
    size_t numFigures() const;
    
    bool hasAnyContent(const std::string& name) const;
    bool hasContent(const std::string& name, SectionContentType type) const;

    unsigned int contentInfo(const std::string& name, SectionContentType type) const;
    unsigned int contentID(const std::string& name, SectionContentType type) const;

    std::shared_ptr<SectionContent> retrieveContent(unsigned int id,
                                                    bool show_dialog = false) const;

    unsigned int totalNumSections() const;
    unsigned int totalNumSections(const std::function<bool(const Section&)>& func) const;
    std::map<SectionContentType, unsigned int> totalNumContents() const;
    std::map<SectionContentType, unsigned int> totalNumContents(const std::function<bool(const Section&)>& func_section) const;

    std::vector<std::shared_ptr<SectionContent>> sectionContent(bool with_hidden_content = false) const;
    std::vector<std::shared_ptr<SectionContent>> recursiveContent(bool with_hidden_content = false) const;

    bool perTargetSection() const; // to be used for utn and sub-sections
    void perTargetSection(bool value);

    bool perTargetWithIssues() const; // te be set if requirement (any) requirement failed
    void perTargetWithIssues(bool value);

    Report* report() { return report_; }
    const Report* report() const { return report_; }

    nlohmann::json jsonConfig() const;
    bool configure(const nlohmann::json& j);

    static const std::string DBTableName;
    static const Property    DBColumnSectionID;
    static const Property    DBColumnReportID;
    static const Property    DBColumnJSONContent;

    static const std::string FieldHeading;
    static const std::string FieldParentHeading;
    static const std::string FieldPerTarget;
    static const std::string FieldPerTargetWithIssues;
    static const std::string FieldSubSections;
    static const std::string FieldContentIDs;
    static const std::string FieldContentNames;
    static const std::string FieldContentTypes;
    static const std::string FieldHiddenContentIDs;

    static const std::string FieldDocContents;

    static const std::string FieldConfigContentConfigs;
    static const std::string FieldConfigContentID;
    static const std::string FieldConfigContentType;
    static const std::string FieldConfigContentConfig;

    static void setCurrentContentID(unsigned int id);

protected:
    friend class SectionContentTable;
    friend class DBInterface;
    friend class Report;

    void toJSON_impl(nlohmann::json& j) const override final;
    bool fromJSON_impl(const nlohmann::json& j) override final;
    Result toJSONDocument_impl(nlohmann::json& j, 
                               const std::string* resource_dir,
                               ReportExportMode export_style) const override final;

    Section* findSubSection (const std::string& heading); // nullptr if not found
    boost::optional<size_t> findContent(const std::string& name, SectionContentType type) const;
    boost::optional<size_t> findAnyContent(const std::string& name) const;
    std::vector<size_t> findContents(SectionContentType type) const;
    size_t numContents(SectionContentType type) const;

    void createContentWidget(bool preload_ondemand_contents);
    void recreateContentUI(bool force_ui_reset, 
                           bool preload_ondemand_contents);

    unsigned int addHiddenFigure(const SectionContentViewable& viewable);

    std::shared_ptr<SectionContent> loadOrGetContent(size_t idx, 
                                                     bool is_hidden_content,
                                                     bool show_dialog = false) const;
    static unsigned int newContentID();

    std::string heading_;          // name same as heading
    std::string parent_heading_;   // e.g. "head1:head2" or ""

    Report* report_ = nullptr;

    bool per_target_section_ {false};
    bool per_target_section_with_issues_ {false};

    std::vector<int>                                     content_types_;
    std::vector<std::string>                             content_names_;
    std::vector<unsigned int>                            content_ids_;
    mutable std::vector<std::shared_ptr<SectionContent>> content_;

    std::vector<unsigned int>                            hidden_content_ids_;
    mutable std::vector<std::shared_ptr<SectionContent>> hidden_content_;

    std::unique_ptr<QWidget> content_widget_container_;
    QVBoxLayout*             content_widget_container_layout_ = nullptr;
    QWidget*                 content_widget_                  = nullptr;

    std::vector<std::shared_ptr<Section>> sub_sections_;

    static unsigned int current_content_id_;
};

}
