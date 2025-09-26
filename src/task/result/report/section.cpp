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

#include "task/result/report/section.h"
#include "task/result/report/sectionid.h"
#include "task/result/report/sectioncontent.h"
#include "task/result/report/sectioncontenttext.h"
#include "task/result/report/sectioncontenttable.h"
#include "task/result/report/sectioncontentfigure.h"
#include "task/result/report/report.h"

#include "asynctask.h"
#include "logger.h"

#include <QVBoxLayout>
#include <QApplication>
#include <QThread>

namespace ResultReport
{

const std::string Section::DBTableName         = "report_sections";
const Property    Section::DBColumnSectionID   = Property("section_id"  , PropertyDataType::UINT);
const Property    Section::DBColumnReportID    = Property("report_id"   , PropertyDataType::UINT);
const Property    Section::DBColumnJSONContent = Property("json_content", PropertyDataType::JSON);

const std::string Section::FieldHeading             = "heading";
const std::string Section::FieldParentHeading       = "parent_heading";
const std::string Section::FieldPerTarget           = "per_target_section";
const std::string Section::FieldPerTargetWithIssues = "per_target_section_with_issues";
const std::string Section::FieldSubSections         = "sub_sections";
const std::string Section::FieldContentIDs          = "content_ids";
const std::string Section::FieldContentNames        = "content_names";
const std::string Section::FieldContentTypes        = "content_types";
const std::string Section::FieldHiddenContentIDs    = "hidden_content_ids";

const std::string Section::FieldDocContents         = "contents";

const std::string Section::FieldConfigContentConfigs = "content_configs";
const std::string Section::FieldConfigContentID      = "content_id";
const std::string Section::FieldConfigContentType    = "content_type";
const std::string Section::FieldConfigContentConfig  = "content_config";

unsigned int Section::current_content_id_ = 0;

/**
*/
Section::Section(const std::string& heading, 
                 const std::string& parent_heading, 
                 TreeItem* parent_item,
                 Report* report)
:   TreeItem       (heading, parent_item)
,   heading_       (heading)
,   parent_heading_(parent_heading)
,   report_        (report)
{
}

/**
*/
Section::Section(TreeItem* parent_item,
                 Report* report)
:   TreeItem (parent_item)
,   report_  (report)
{
}

void clearUI();

/**
 */
Section* Section::parentSection()
{
    return dynamic_cast<Section*>(parent_item_);
}

/**
 */
const Section* Section::parentSection() const
{
    return dynamic_cast<const Section*>(parent_item_);
}

/**
*/
unsigned int Section::newContentID()
{
    return current_content_id_++;
}

/**
*/
void Section::setCurrentContentID(unsigned int id)
{
    current_content_id_ = id;
}

/**
*/
TreeItem* Section::child(int row)
{
    if (row < 0 || row >= (int)sub_sections_.size())
        return nullptr;

    return sub_sections_.at(row).get();
}

/**
*/
int Section::childCount() const
{
    return sub_sections_.size();
}

/**
*/
int Section::columnCount() const
{
    return 1;
}

/**
*/
QVariant Section::data(int column) const
{
    traced_assert(column == 0);

    return heading_.c_str();
}

/**
*/
int Section::row() const
{
    return 0;
}

/**
*/
std::string Section::heading() const
{
    return heading_;
}

/**
*/
std::string Section::compoundHeading() const
{
    if (parent_heading_.size())
        return parent_heading_+":"+heading_;
    else
        return heading_;
}

/**
*/
std::string Section::compoundResultsHeading() const
{
    std::string tmp;

    if (parent_heading_.size())
        tmp = SectionID::sectionID(parent_heading_, heading_);
    else
        tmp = heading_;

    return SectionID::sectionIDWithoutResults(tmp);
}

/**
*/
bool Section::hasSubSection (const std::string& heading)
{
    return findSubSection(heading) != nullptr;
}

/**
*/
Section& Section::getSubSection (const std::string& heading)
{
    traced_assert(hasSubSection(heading));

    Section* tmp = findSubSection (heading);
    traced_assert(tmp);
    return *tmp;
}

/**
*/
void Section::addSubSection (const std::string& heading)
{
    logdbg << "Section " << heading_ << ": addSubSection: adding " << heading;

    traced_assert(!hasSubSection(heading));

    sub_sections_.push_back(std::make_shared<Section>(heading, compoundHeading(), this, report_));
    traced_assert(hasSubSection(heading));
}

/**
*/
std::vector<std::shared_ptr<Section>> Section::subSections(bool recursive) const
{
    std::vector<std::shared_ptr<Section>> sections = sub_sections_;

    if (recursive)
    {
        for (const auto& s : sub_sections_)
        {
            auto subs = s->subSections(true);
            sections.insert(sections.end(), subs.begin(), subs.end());
        }
    }

    return sections;
}

/**
*/
std::string Section::relativeID(const std::string& id) const
{
    if(id.empty())
        return id;

    QString id0    = QString::fromStdString(id);
    QString id1    = QString::fromStdString(this->id());

    if (id0 == id1)
        return id;

    QString prefix = id1 + QString::fromStdString(SectionID::Sep);

    if (!id0.startsWith(prefix))
        return id;

    return id0.mid(prefix.count(), -1).toStdString();
}

/**
*/
QWidget* Section::getContentWidget(bool preload_ondemand_contents)
{
    if (!content_widget_container_)
    {
        createContentWidget(preload_ondemand_contents);
        traced_assert(content_widget_container_);
    }

    return content_widget_container_.get();
}

/**
*/
void Section::updateContents(bool recursive)
{
    //clear all loaded demand content
    for (auto& c : content_)
        if (c)
            c->clearOnDemandContent();

    for (auto& c : hidden_content_)
        if (c)
            c->clearOnDemandContent();

    //recreate section ui
    recreateContentUI(true, false);

    //apply recursively?
    if (recursive)
    {
        for (auto& sub_section : sub_sections_)
            sub_section->updateContents(recursive);
    }
}

/**
*/
bool Section::hasText (const std::string& name)
{
    return hasContent(name, SectionContentType::Text);
}

/**
*/
SectionContentText& Section::getText (const std::string& name)
{
    auto idx = findContent(name, SectionContentType::Text);
    traced_assert(idx.has_value());

    auto c = loadOrGetContent(idx.value(), false);
    traced_assert(c);

    SectionContentText* tmp = dynamic_cast<SectionContentText*>(c.get());
    traced_assert(tmp);

    return *tmp;
}

/**
*/
SectionContentText& Section::addText(const std::string& name)
{
    traced_assert(!hasText(name));
    auto id = Section::newContentID();
    content_ids_.push_back(id);
    content_names_.push_back(name);

    auto ptr = std::make_shared<SectionContentText>(id, name, this);
    
    content_.push_back(ptr);
    content_types_.push_back((int)content_.back()->contentType());
    traced_assert(hasText(name));

    return *ptr;
}

/**
*/
size_t Section::numTexts() const
{
    return numContents(SectionContentType::Text);
}

/**
*/
bool Section::hasTable(const std::string& name)
{
    return hasContent(name, SectionContentType::Table);
}

/**
*/
SectionContentTable& Section::getTable(const std::string& name)
{
    auto idx = findContent(name, SectionContentType::Table);
    traced_assert(idx.has_value());

    auto c = loadOrGetContent(idx.value(), false);
    traced_assert(c);

    SectionContentTable* tmp = dynamic_cast<SectionContentTable*>(c.get());
    traced_assert(tmp);

    return *tmp;
}

/**
*/
std::vector<std::string> Section::getTableNames() const
{
    std::vector<std::string> names;

    auto idxs = findContents(SectionContentType::Table);

    for (auto idx : idxs)
        names.push_back(content_names_.at(idx));

    return names;
}

/**
*/
SectionContentTable& Section::addTable(const std::string& name, 
                                       unsigned int num_columns,
                                       std::vector<std::string> headings, 
                                       bool sortable, 
                                       unsigned int sort_column, 
                                       Qt::SortOrder order)
{
    traced_assert(!hasTable(name));
    auto id = Section::newContentID();
    content_ids_.push_back(id);
    content_names_.push_back(name);

    auto ptr = std::make_shared<SectionContentTable>(id, 
                                                     name, 
                                                     num_columns, 
                                                     headings, 
                                                     this, 
                                                     sortable, 
                                                     sort_column, 
                                                     order);

    content_.push_back(ptr);

    content_types_.push_back((int)content_.back()->contentType());
    traced_assert(hasTable(name));

    return *ptr;
}

/**
*/
size_t Section::numTables() const
{
    return numContents(SectionContentType::Table);
}

/**
*/
bool Section::hasFigure (const std::string& name)
{
    return hasContent(name, SectionContentType::Figure);
}

/**
*/
SectionContentFigure& Section::getFigure (const std::string& name)
{
    auto idx = findContent(name, SectionContentType::Figure);
    traced_assert(idx.has_value());

    auto c = loadOrGetContent(idx.value(), false);
    traced_assert(c);

    SectionContentFigure* tmp = dynamic_cast<SectionContentFigure*>(c.get());
    traced_assert(tmp);

    return *tmp;
}

/**
*/
SectionContentFigure& Section::addFigure(const std::string& name, 
                                         const SectionContentViewable& viewable)
{
    traced_assert(!name.empty());
    traced_assert(!hasFigure(name));

    unsigned int id = Section::newContentID();
    content_ids_.push_back(id);
    content_names_.push_back(name);

    auto ptr = std::make_shared<SectionContentFigure>(id, 
                                                      SectionContentFigure::FigureType::Section,
                                                      name, 
                                                      viewable, 
                                                      this);

    content_.push_back(ptr);
    content_types_.push_back((int)content_.back()->contentType());
    traced_assert(hasFigure(name));

    return *ptr;
}

/**
*/
unsigned int Section::addHiddenFigure(const SectionContentViewable& viewable)
{
    unsigned int id = Section::newContentID();
    hidden_content_ids_.push_back(id);
    hidden_content_.push_back(std::make_shared<SectionContentFigure>(id, 
                                                                    SectionContentFigure::FigureType::Hidden,
                                                                    "", 
                                                                    viewable, 
                                                                    this));
    return id;
}

/**
*/
std::vector<SectionContentFigure*> Section::getFigures()
{
    std::vector<SectionContentFigure*> figures;

    auto idxs = findContents(SectionContentType::Figure);

    for (auto idx : idxs)
    {
        auto c = loadOrGetContent(idx, false);
        traced_assert(c);

        auto fig = dynamic_cast<SectionContentFigure*>(c.get());
        traced_assert(fig);

        figures.push_back(fig);
    }

    return figures;
}

/**
*/
size_t Section::numFigures() const
{
    return numContents(SectionContentType::Figure);
}

/**
*/
unsigned int Section::totalNumSections() const
{
    unsigned int num = 1; // me

    for (auto& sec_it : sub_sections_)
        num += sec_it->totalNumSections();

    return num;
}

/**
*/
unsigned int Section::totalNumSections(const std::function<bool(const Section&)>& func) const
{
    if (!func(*this))
        return 0;

    unsigned int num = 1; // me

    for (auto& sec_it : sub_sections_)
        num += sec_it->totalNumSections(func);

    return num;
}

/**
*/
std::map<SectionContentType, unsigned int> Section::totalNumContents() const
{
    std::map<SectionContentType, unsigned int> counts;

    for (size_t i = 0; i < content_types_.size(); ++i)
        counts[(SectionContentType)content_types_[i]]++;

    for (const auto& sec_it : sub_sections_)
    {
        auto sub_counts = sec_it->totalNumContents();
        for (const auto& it : sub_counts)
            counts[it.first] += it.second;
    }

    return counts;
}

/**
*/
std::map<SectionContentType, unsigned int> Section::totalNumContents(const std::function<bool(const Section&)>& func_section) const
{
    std::map<SectionContentType, unsigned int> counts;

    if (!func_section(*this))
        return counts;

    for (size_t i = 0; i < content_types_.size(); ++i)
        counts[(SectionContentType)content_types_[i]]++;    

    for (const auto& sec_it : sub_sections_)
    {
        auto sub_counts = sec_it->totalNumContents(func_section);
        for (const auto& it : sub_counts)
            counts[it.first] += it.second;
    }

    return counts;
}

/**
*/
std::vector<std::shared_ptr<SectionContent>> Section::sectionContent(bool with_hidden_content) const
{
    std::vector<std::shared_ptr<SectionContent>> content;

    size_t n = content_.size();
    for (size_t i = 0; i < n; ++i)
    {
        auto c = loadOrGetContent(i, false, false);
        if (c) content.push_back(c);
    }

    if (with_hidden_content)
    {
        size_t ne = hidden_content_.size();
        for (size_t i = 0; i < ne; ++i)
        {
            auto c = loadOrGetContent(i, true, false);
            if (c) content.push_back(c);
        }
    }

    return content;
}

/**
*/
std::vector<std::shared_ptr<SectionContent>> Section::recursiveContent(bool with_hidden_content) const
{
    std::vector<std::shared_ptr<SectionContent>> sec_content_rec = sectionContent(with_hidden_content);

    for (const auto& s : sub_sections_)
    {
        auto sec_content = s->recursiveContent(with_hidden_content);
        sec_content_rec.insert(sec_content_rec.end(), sec_content.begin(), sec_content.end());
    }

    return sec_content_rec;
}

/**
*/
bool Section::perTargetWithIssues() const
{
    return per_target_section_with_issues_;
}

/**
*/
void Section::perTargetWithIssues(bool value)
{
    per_target_section_with_issues_ = value;
}

/**
*/
bool Section::perTargetSection() const
{
    return per_target_section_;
}

/**
*/
void Section::perTargetSection(bool value)
{
    per_target_section_ = value;
}

/**
*/
Section* Section::findSubSection(const std::string& heading)
{
    for (auto& sec_it : sub_sections_)
    {
        if (sec_it->heading() == heading)
            return sec_it.get();
    }

    return nullptr;
}

/**
*/
boost::optional<size_t> Section::findContent(const std::string& name, SectionContentType type) const
{
    for (size_t i = 0; i < content_.size(); ++i)
    {
        //loginf << "name: " << content_names_[ i ] << " vs " << name << " - type: " << content_types_[ i ] << " vs " << (int)type;

        if (content_names_[ i ] == name && (SectionContentType)content_types_[ i ] == type)
            return i;
    }

    return boost::optional<size_t>();
}

/**
*/
boost::optional<size_t> Section::findAnyContent(const std::string& name) const
{
    for (size_t i = 0; i < content_.size(); ++i)
    {
        if (content_names_[ i ] == name)
            return i;
    }

    return boost::optional<size_t>();
}

/**
*/
std::vector<size_t> Section::findContents(SectionContentType type) const
{
    std::vector<size_t> idxs;

    for (size_t i = 0; i < content_.size(); ++i)
    {
        if ((SectionContentType)content_types_[ i ] == type)
            idxs.push_back(i);
    }

    return idxs;
}

/**
*/
bool Section::hasContent(const std::string& name, SectionContentType type) const
{
    return findContent(name, type).has_value();
}

/**
*/
bool Section::hasAnyContent(const std::string& name) const
{
    return findAnyContent(name).has_value();
}

/**
 */
unsigned int Section::contentID(const std::string& name, SectionContentType type) const
{
    auto idx = findContent(name, type);
    traced_assert(idx.has_value());

    return content_ids_.at(idx.value());
}

/**
*/
unsigned int Section::contentInfo(const std::string& name, SectionContentType type) const
{
    unsigned int flags = 0;

    //has content of the given name?
    auto idx = findContent(name, type);
    if (!idx.has_value())
        return flags;

    flags |= ContentInfoFlag::ContentAvailable;

    const auto& c = content_.at(idx.value());

    //not yet loaded from db?
    if (!c)
        return flags;

    flags |= ContentInfoFlag::ContentLoaded;

    if (!c->isOnDemand())
        return flags;

    flags |= ContentInfoFlag::ContentOnDemand;

    if (!c->isComplete())
        return flags;

    flags |= ContentInfoFlag::ContentOnDemandComplete;

    return flags;
}

/**
*/
size_t Section::numContents(SectionContentType type) const
{
    size_t num = 0;

    for (size_t i = 0; i < content_.size(); ++i)
    {
        if ((SectionContentType)content_types_[ i ] == type)
            ++num;
    }

    return num;
}

/**
*/
void Section::createContentWidget(bool preload_ondemand_contents)
{
    loginf << "creating content widget for section '" << name() << "'";

    traced_assert(!content_widget_container_);

    content_widget_container_.reset(new QWidget());

    content_widget_container_layout_ = new QVBoxLayout;
    content_widget_container_layout_->setContentsMargins(0, 0, 0, 0);

    content_widget_container_->setLayout(content_widget_container_layout_);
    
    recreateContentUI(false, preload_ondemand_contents);
}

/**
*/
void Section::recreateContentUI(bool force_ui_reset,
                                bool preload_ondemand_contents)
{
    //container not yet created => return
    if (!content_widget_container_)
        return;

    traced_assert(content_widget_container_layout_);

    //destroy old content widget
    if (content_widget_)
    {
        content_widget_container_layout_->removeWidget(content_widget_);
        delete content_widget_;
        content_widget_ = nullptr;
    }

    //create new one
    content_widget_ = new QWidget;

    QVBoxLayout* layout = new QVBoxLayout();

    //preload all needed contents as async task with dialog
    auto cb = [ this ] (const AsyncTaskState&, AsyncTaskProgressWrapper&) 
    { 
        for (size_t i = 0; i < this->content_.size(); ++i)
            this->loadOrGetContent(i, false);

        return true;
    };

    AsyncFuncTask task(cb, "Loading", "Loading section contents", false);
    task.runAsyncDialog();

    //preload some on-demand resources if desired
    //note: not possible on other thread because it e.g. might trigger ui stuff on reload
    if (preload_ondemand_contents)
    {
        for (size_t i = 0; i < this->content_.size(); ++i)
        {
            auto c = this->loadOrGetContent(i, false);
            if (c->isOnDemand() &&
                !c->isLocked() &&
                c->contentType() == SectionContentType::Table)
            {
                c->loadOnDemandIfNeeded();
            }
        }
    }

    //add contents to layout
    for (size_t i = 0; i < content_.size(); ++i)
        loadOrGetContent(i, false)->addContentUI(layout, force_ui_reset);

    //layout->addStretch();

    content_widget_->setLayout(layout);

    content_widget_container_layout_->addWidget(content_widget_);
}

namespace 
{
    boost::optional<size_t> findContentInVector(const std::vector<unsigned int>& ids,
                                                unsigned int id)
    {
        auto it = std::lower_bound(ids.begin(), ids.end(), id);
        if (it == ids.end() || *it != id)
            return {};

        return std::distance(ids.begin(), it);
    }
}

/**
*/
std::shared_ptr<SectionContent> Section::retrieveContent(unsigned int id, 
                                                         bool show_dialog) const
{
    bool in_hidden = false;

    auto idx = findContentInVector(content_ids_, id);
    if (!idx.has_value())
    {
        idx = findContentInVector(hidden_content_ids_, id);
        in_hidden = true;
    }

    if (!idx.has_value())
        return std::shared_ptr<SectionContent>();

    return loadOrGetContent(idx.value(), in_hidden, show_dialog);
}

/**
*/
std::shared_ptr<SectionContent> Section::loadOrGetContent(size_t idx, 
                                                          bool is_hidden_content,
                                                          bool show_dialog) const
{
    auto& c_ptr = is_hidden_content ? hidden_content_.at(idx) : content_.at(idx);
    auto  id    = is_hidden_content ? hidden_content_ids_.at(idx) : content_ids_.at(idx);

    if (c_ptr)
        return c_ptr;

    Section* s = const_cast<Section*>(this); //@TODO
    auto c = report_->loadContent(s, id, show_dialog);
    traced_assert(c);

    c_ptr = c;

    return c_ptr;
}

/**
*/
void Section::toJSON_impl(nlohmann::json& j) const
{
    j[ FieldHeading             ] = heading_;
    j[ FieldParentHeading       ] = parent_heading_;
    j[ FieldPerTarget           ] = per_target_section_;
    j[ FieldPerTargetWithIssues ] = per_target_section_with_issues_;
    j[ FieldContentIDs          ] = content_ids_;
    j[ FieldContentNames        ] = content_names_;
    j[ FieldContentTypes        ] = content_types_;
    j[ FieldHiddenContentIDs    ] = hidden_content_ids_;

    nlohmann::json j_subsections = nlohmann::json::array();

    for (const auto& s : sub_sections_)
    {
        nlohmann::json js = s->toJSON();
        j_subsections.push_back(js);
    }

    j[ FieldSubSections ] = j_subsections;
}

/**
*/
bool Section::fromJSON_impl(const nlohmann::json& j)
{
    if (!j.is_object()                        ||
        !j.contains(FieldHeading)             ||
        !j.contains(FieldParentHeading)       ||
        !j.contains(FieldPerTarget)           ||
        !j.contains(FieldPerTargetWithIssues) ||
        !j.contains(FieldSubSections)         ||
        !j.contains(FieldContentIDs)          ||
        !j.contains(FieldContentNames)        ||
        !j.contains(FieldContentTypes)        ||
        !j.contains(FieldHiddenContentIDs))
    {
        logerr << "section does not obtain needed fields";
        return false;
    }

    heading_                        = j[ FieldHeading             ];
    parent_heading_                 = j[ FieldParentHeading       ];
    per_target_section_             = j[ FieldPerTarget           ];
    per_target_section_with_issues_ = j[ FieldPerTargetWithIssues ];
    content_ids_                    = j[ FieldContentIDs          ].get<std::vector<unsigned int>>();
    content_names_                  = j[ FieldContentNames        ].get<std::vector<std::string>>();
    content_types_                  = j[ FieldContentTypes        ].get<std::vector<int>>();
    hidden_content_ids_             = j[ FieldHiddenContentIDs    ].get<std::vector<unsigned int>>();

    content_.resize(content_ids_.size());
    hidden_content_.resize(hidden_content_ids_.size());

    traced_assert(content_ids_.size() == content_names_.size());
    traced_assert(content_ids_.size() == content_types_.size());

    const auto& j_subsections = j[ FieldSubSections ];
    if (!j_subsections.is_array())
    {
        logerr << "subsection is not an array";
        return false;
    }

    for (const auto& jss : j_subsections)
    {
        std::shared_ptr<Section> section(new Section(this, report_));
        if (!section->fromJSON(jss))
            return false;

        sub_sections_.push_back(section);
    }

    return true;
}

/**
 */
Result Section::toJSONDocument_impl(nlohmann::json& j,
                                    const std::string* resource_dir,
                                    ReportExportMode export_style) const
{
    //section id
    j[ FieldID ] = id();
    
    j[ FieldSubSections ] = nlohmann::json::array();
    j[ FieldDocContents ] = nlohmann::json::array();
    
    return Result::succeeded();
}

/**
 */
nlohmann::json Section::jsonConfig() const
{
    nlohmann::json j;

    auto table_ids = findContents(SectionContentType::Table);

    auto contents_configs = nlohmann::json::array();

    for (const auto& c : content_)
    {
        //not yet loaded => skip
        if (!c)
            continue;

        //get config and check if it has any entries
        auto config = c->jsonConfig();

        if (config.is_object() && !config.empty())
        {
            //obtains non-zero config => remember
            nlohmann::json j_content = nlohmann::json::object();

            j_content[ FieldConfigContentID      ] = c->id();
            j_content[ FieldConfigContentType    ] = (int)c->contentType();
            j_content[ FieldConfigContentConfig  ] = config;

            contents_configs.push_back(j_content);
        }
    }

    j[ FieldConfigContentConfigs ] = contents_configs;

    return j;
}

/**
 */
bool Section::configure(const nlohmann::json& j)
{
    if (!j.is_object() || 
        !j.contains(FieldConfigContentConfigs))
        return false;

    auto content_configs = j[ FieldConfigContentConfigs ];
    if (!content_configs.is_array())
        return false;

    bool ok = true;

    for (const auto& j_content : content_configs)
    {
        if (!j_content.is_object() || 
            !j_content.contains(FieldConfigContentID) || 
            !j_content.contains(FieldConfigContentType) ||
            !j_content.contains(FieldConfigContentConfig))
            return false;

        std::string id     = j_content[ FieldConfigContentID ].get<std::string>();
        auto        type   = (SectionContentType)j_content[ FieldConfigContentType ].get<int>();
        auto        config = j_content[ FieldConfigContentConfig ];

        //find content of id and type
        auto it = std::find_if(content_.begin(), content_.end(), [&id, &type] (const std::shared_ptr<SectionContent>& c) { return c && c->id() == id && c->contentType() == type; });
        if (it == content_.end())
        {
            //not found => skip
            logwrn << "failed to find loaded content with id " << id << " in section " << name();
            ok = false;
            continue;
        }

        size_t idx = std::distance(content_.begin(), it);
        const auto& c = content_.at(idx);
        traced_assert(c);
        
        //try to configure content
        if (!c->configure(config))
        {
            logwrn << "failed to configure content " << c->name() << " of section " << name();
            ok = false;
            continue;
        }
    }

    return ok;
}

}
