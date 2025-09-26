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

#include "evaluationdetail.h"
#include "evaluationdefs.h"
#include "eval/results/evaluationdetail.h"
#include "eval/results/base/result_defs.h"

#include <QVariant>
#include <QRectF>

#include "json.hpp"

#include <memory>
#include <vector>

#include <QColor>

#include <Eigen/Core>

class EvaluationTargetData;
class EvaluationCalculator;
class SectorLayer;

namespace EvaluationRequirement 
{
    class Base;
}

namespace ResultReport 
{
    class Report;
    class Section;
    class SectionContent;
    class SectionContentTable;
}

namespace EvaluationRequirementResult
{

class FeatureDefinitions;

/**
*/
class Base
{
public:
    typedef std::vector<EvaluationDetail> EvaluationDetails;  // details vector
    typedef std::array<int, 2>            DetailIndex;        // index for a nested detail struct

    typedef std::function<bool(const EvaluationDetail&)> DetailSkipFunc;
    typedef std::function<void(const EvaluationDetail&, const EvaluationDetail*, int, int, int, int)> DetailFunc;

    enum class BaseType
    {
        Single = 0, // target results (for specific utn)
        Joined      // sum results (for specific requirement in specific sector)
    };

    /**
     * Info table entry struct.
     */
    struct Info
    {
        Info() {}
        Info(const std::string& name,
             const std::string& comment,
             const nlohmann::json& value)
        :   info_name   (name   )
        ,   info_comment(comment)
        ,   info_value  (value  )
        {}

        std::string    info_name;
        std::string    info_comment;
        nlohmann::json info_value;
    };

    Base(const std::string& type, 
         const std::string& result_id,
         std::shared_ptr<EvaluationRequirement::Base> requirement, 
         const SectorLayer& sector_layer,
         EvaluationCalculator& calculator);
    virtual ~Base();

    /// returns the base type of the result (either single or joined)
    virtual BaseType baseType() const = 0;

    std::string type() const;
    std::string resultId() const;
    std::string reqGrpId() const;
    
    virtual std::string sumSectionName() const;

    std::shared_ptr<EvaluationRequirement::Base> requirement() const;

    bool isSingle() const;
    bool isJoined() const;
    bool isResult(const Evaluation::RequirementResultID& id) const;

    bool use() const;
    void use(bool use);

    const boost::optional<double>& result() const;

    bool resultUsable() const;
    bool hasFailed() const;
    bool hasIssues() const;
    
    void setIgnoreResult(const std::string& comment);
    bool ignoreResult() const;
    const std::string& ignoreResultComment() const;

    const SectorLayer& sectorLayer() const { return sector_layer_; } 

    nlohmann::json resultValue() const;
    nlohmann::json resultValueOptional(const boost::optional<double>& value) const;
    virtual nlohmann::json resultValue(double value) const;

    /// returns the number of issues detected for this result
    virtual unsigned int numIssues() const = 0;

    /// checks if the result references a specific section of the report
    virtual bool hasReference(const ResultReport::SectionContentTable& table, 
                              const QVariant& annotation) const = 0;
    /// returns a report reference link
    virtual std::string reference(const ResultReport::SectionContentTable& table, 
                                  const QVariant& annotation) const = 0;

    /// checks if the result can generate viewable data for the given table and annotation index
    virtual bool hasViewableData (const ResultReport::SectionContentTable& table, 
                                  const QVariant& annotation) const = 0;
    /// checks if the viewable data is ready (e.g. cached or invalidated)
    virtual bool viewableDataReady() const = 0;
    /// creates suitable viewable data for the given table and annotation index
    virtual std::shared_ptr<nlohmann::json::object_t> viewableData(const ResultReport::SectionContentTable& table, 
                                                                   const QVariant& annotation) const = 0;
    /// adds the result to the report root item
    virtual void addToReport (std::shared_ptr<ResultReport::Report> report) = 0;

    /// iterate over details
    virtual void iterateDetails(const DetailFunc& func,
                                const DetailSkipFunc& skip_func = DetailSkipFunc()) const = 0;

    size_t totalNumDetails() const;
    size_t totalNumPositions() const;

    static void setContentProperties(ResultReport::SectionContent& content,
                                     const Evaluation::RequirementResultID& id);
    static boost::optional<Evaluation::RequirementResultID> contentProperties(const ResultReport::SectionContent& content);

    const static std::string RequirementOverviewTableName;
    const static std::string RequirementOverviewSectionName;

    static const QColor HistogramColorDefault;

    static const std::string ContentPropertySectorLayer;
    static const std::string ContentPropertyReqGroup;
    static const std::string ContentPropertyReqName;

protected:
    friend class EvaluationTaskResult; // for loading on-demand content

    /**
     * Used to display a certain result parameter in the report as
     * Name | Description | Value, e.g.
     * PD [%]    Probability of Detection    98.4
     */
    struct ReportParam
    {
        ReportParam() = default;
        ReportParam(const std::string& _name, 
                    const std::string& _descr, 
                    const QVariant& _value) : name(_name), descr(_descr), value(_value) {}

        std::string name;
        std::string descr;
        QVariant    value;
    };

    enum class ViewableType
    {
        Overview = 0,  // viewable represents an overview over the result data
        Highlight      // viewable represents a highlighted result detail
    };

    /**
     * Options for creating result annotations.
    */
    struct AnnotationOptions
    {
        /// configure as result overview
        AnnotationOptions& overview() 
        { 
            viewable_type = ViewableType::Overview; 
            return *this; 
        }
        /// configure as highlighted result detail
        AnnotationOptions& highlight(const DetailIndex& index)
        { 
            viewable_type = ViewableType::Highlight; 
            detail_index  = index;
            return *this; 
        }

        bool valid() const
        {
            if (viewable_type == ViewableType::Highlight)
                return detail_index.has_value();
            return true;
        }

        ViewableType                 viewable_type = ViewableType::Overview;
        boost::optional<DetailIndex> detail_index;
    };

    /**
     * Information about a viewable.
    */
    struct ViewableInfo
    {
        ViewableType             viewable_type; // viewable type
        QRectF                   bounds;        // viewable bounds (e.g. for viewpoint region of interest)
        boost::posix_time::ptime timestamp;     // viewable timestamp (e.g. for single occurrences)
    };

    double formatValue(double v, int precision = 2) const;
    std::string conditionResultString() const;

    void updateResult(const boost::optional<double>& value);

    /// creates overview viewable data
    virtual std::shared_ptr<nlohmann::json::object_t> viewableOverviewData() const = 0;

    std::unique_ptr<nlohmann::json::object_t> createViewable(const AnnotationOptions& options) const;

    void addCustomAnnotations(nlohmann::json& annotations_json) const;

    /// creates a basic viewable
    virtual std::unique_ptr<nlohmann::json::object_t> createBaseViewable() const = 0;
    /// creates additional viewable information (region of interest etc.)
    virtual ViewableInfo createViewableInfo(const AnnotationOptions& options) const = 0;
    /// creates annotations for the given options
    virtual void createAnnotations(nlohmann::json& annotations_json, 
                                   const AnnotationOptions& options) const = 0;

    /// generate definitions for the automatic generation of custom annotations (grids, histograms, etc.)
    virtual FeatureDefinitions getCustomAnnotationDefinitions() const;

    ResultReport::SectionContentTable& getReqOverviewTable (std::shared_ptr<ResultReport::Report> report);

    /// section and annotation id strings
    virtual std::string getRequirementSectionID() const;
    virtual std::string getRequirementSumSectionID() const;
    virtual std::string getRequirementAnnotationID_impl() const = 0;

    std::string getRequirementAnnotationID() const;
    
    ResultReport::Section& getRequirementSection(std::shared_ptr<ResultReport::Report> report);

    std::string type_;
    std::string result_id_;
    std::string req_grp_id_;

    bool use_ {true};

    std::shared_ptr<EvaluationRequirement::Base> requirement_;
    const SectorLayer& sector_layer_;

    EvaluationCalculator& calculator_;

private:
    boost::optional<double> result_;

    bool ignore_result_ {false};
    std::string ignore_result_comment_;
};

}
