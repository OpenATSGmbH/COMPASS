#include "eval/requirement/base/positionbaseconfig.h"
#include "task/result/report/report.h"
#include "task/result/report/section.h"
#include "task/result/report/sectioncontenttable.h"
#include "eval/requirement/group.h"

#include "stringconv.h"

using namespace std;
using namespace Utils;

namespace EvaluationRequirement
{

/*******************************************************************************************
 * PositionBaseProbConfig
 *******************************************************************************************/

/**
 */
PositionBaseProbConfig::PositionBaseProbConfig(nlohmann::json& config,
                                               Group* parent)
    : ProbabilityBaseConfig(config, parent)
{
    registerParameter("ref_min_accuracy", &ref_min_accuracy_, ref_min_accuracy_);
}

/**
 */
PositionBaseProbConfig::~PositionBaseProbConfig() {}

/**
 */
void PositionBaseProbConfig::addToReport(std::shared_ptr<ResultReport::Report> report)
{
    //adds the table if needed
    BaseConfig::addToReport(report);

    auto& section = report->getSection("Appendix:Requirements:"+group_.name()+":"+name_);

    auto& table = section.getTable("req_table");

    table.addRow({"Reference Min. Accuracy", "", String::doubleToStringPrecision(ref_min_accuracy_, 1)});

    // prob & check type added in subclass
}

/*******************************************************************************************
 * PositionBaseValueConfig
 *******************************************************************************************/

/**
 */
PositionBaseValueConfig::PositionBaseValueConfig(nlohmann::json& config,
                                                 Group* parent)
    : BaseConfig(config, parent)
{
    registerParameter("ref_min_accuracy", &ref_min_accuracy_, ref_min_accuracy_);
}

/**
 */
PositionBaseValueConfig::~PositionBaseValueConfig() {}

/**
 */
void PositionBaseValueConfig::addToReport(std::shared_ptr<ResultReport::Report> report)
{
    //adds the table if needed
    BaseConfig::addToReport(report);

    auto& section = report->getSection("Appendix:Requirements:"+group_.name()+":"+name_);

    auto& table = section.getTable("req_table");

    table.addRow({"Reference Min. Accuracy", "", String::doubleToStringPrecision(ref_min_accuracy_, 1)});
}

}  // namespace EvaluationRequirement
