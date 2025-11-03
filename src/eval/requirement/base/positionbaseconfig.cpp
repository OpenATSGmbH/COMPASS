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

PositionBaseConfig::PositionBaseConfig(const std::string& class_id, const std::string& instance_id,
                                       Group& group, EvaluationStandard& standard,
                                       EvaluationCalculator& calculator)
    : ProbabilityBaseConfig(class_id, instance_id, group, standard, calculator)
{
    registerParameter("ref_min_accuracy", &ref_min_accuracy_, ref_min_accuracy_);
}

PositionBaseConfig::~PositionBaseConfig() {}

void PositionBaseConfig::addToReport(std::shared_ptr<ResultReport::Report> report)
{
    BaseConfig::addToReport(report);

    auto& section = report->getSection("Appendix:Requirements:"+group_.name()+":"+name_);

    auto& table = section.getTable("req_table");

    table.addRow({"Reference Min. Accuracy", "", String::doubleToStringPrecision(ref_min_accuracy_, 1)});

    // prob & check type added in subclass
}

}  // namespace EvaluationRequirement