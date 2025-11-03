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

#include "eval/results/base/base.h"

#include "view/gridview/grid2d_defs.h"

#include <fstream>

class Grid2D;

namespace ResultReport
{
    class Report;
    class SectionContentTable;
};

namespace EvaluationRequirementResult
{

class Single;

template <typename T>
struct ValueSource;

/**
*/
class Joined : public Base
{
public:
    typedef Info SectorInfo;

    typedef std::function<void(const std::shared_ptr<Single>&)> SingleResultFunc;

    Joined(const std::string& type, 
           const std::string& result_id,
           std::shared_ptr<EvaluationRequirement::Base> requirement,
           const SectorLayer& sector_layer,
           EvaluationCalculator& calculator);
    virtual ~Joined();

    BaseType baseType() const override final { return BaseType::Joined; }

    void clearResults();
    void addSingleResult(std::shared_ptr<Single> other);

    std::vector<std::shared_ptr<Single>>& singleResults();
    std::vector<std::shared_ptr<Single>> usedSingleResults() const;

    void addToReport(std::shared_ptr<ResultReport::Report> report) override final;

    bool hasViewableData (const ResultReport::SectionContentTable& table, 
                          const QVariant& annotation) const override final;
    bool viewableDataReady() const override final;
    std::shared_ptr<nlohmann::json::object_t> viewableData(const ResultReport::SectionContentTable& table, 
                                                           const QVariant& annotation) const override final;

    bool hasReference (const ResultReport::SectionContentTable& table, 
                       const QVariant& annotation) const override final;
    std::string reference(const ResultReport::SectionContentTable& table, 
                          const QVariant& annotation) const override final;

    void iterateDetails(const DetailFunc& func,
                        const DetailSkipFunc& skip_func = DetailSkipFunc()) const override final;

    void updateToChanges(bool reset_viewable);
    
    unsigned int numSingleResults() const;
    unsigned int numUsableSingleResults() const;
    unsigned int numUnusableSingleResults() const;

    bool hasStoredDetails() const;

    std::vector<double> getValues(const ValueSource<double>& source) const;
    std::vector<double> getValues(int value_id) const;

    static void setJoinedContentProperties(ResultReport::SectionContent& content,
                                           const Evaluation::RequirementResultID& id);
    static boost::optional<Evaluation::RequirementResultID> joinedContentProperties(const ResultReport::SectionContent& content);

    static const std::string SectorOverviewID;
    static const int         SectorOverviewRenderDelayMSec;

    static const std::string SectorTargetsTableName;
    static const std::string SectorOverviewTableName;

protected:
    bool resultUsed(const std::shared_ptr<Single>& result) const;
    void iterateSingleResults(const SingleResultFunc& func,
                              const SingleResultFunc& func_used,
                              const SingleResultFunc& func_unused) const;

    /// compute result value
    virtual void updateResult();
    virtual boost::optional<double> computeResult() const;
    virtual boost::optional<double> computeResult_impl() const = 0;

    /// clears result data before accumulating single results
    virtual void clearResults_impl() = 0;
    /// accumulates a single result
    virtual void accumulateSingleResult(const std::shared_ptr<Single>& single_result, 
                                        bool first,
                                        bool last) = 0;
    /// total number of updates
    virtual unsigned int numUpdates() const = 0;

    bool exportAsCSV() const;

    /// if true an export callback is added to the table
    virtual bool canExportCSV() const { return false; }
    /// implements csv export 
    virtual bool exportAsCSV(std::ofstream& strm) const { return false; }

    virtual std::string getRequirementAnnotationID_impl() const override;

    std::shared_ptr<nlohmann::json::object_t> viewableOverviewData() const override final;
    std::unique_ptr<nlohmann::json::object_t> createBaseViewable() const override final;
    ViewableInfo createViewableInfo(const AnnotationOptions& options) const override final;
    void createAnnotations(nlohmann::json& annotations, 
                           const AnnotationOptions& options) const override final;

    virtual std::vector<SectorInfo> sectorInfosCommon() const;
    std::vector<SectorInfo> sectorConditionInfos() const;

    /// returns infos for the sector overview table
    virtual std::vector<SectorInfo> sectorInfos() const = 0;

    std::vector<std::shared_ptr<Single>> results_;

private:
    void addOverview (ResultReport::Section& section,
                      const std::string& name = "Sector Overview");

    void addSectorToOverviewTable(std::shared_ptr<ResultReport::Report> root_item);
    void addSectorDetailsToReport(std::shared_ptr<ResultReport::Report> root_item);

    std::shared_ptr<nlohmann::json::object_t> getOrCreateCachedViewable() const;

    unsigned int num_targets_        = 0;
    unsigned int num_failed_targets_ = 0;

    mutable std::unique_ptr<Grid2D>                   grid_;
    mutable std::shared_ptr<nlohmann::json::object_t> viewable_;
};

}
