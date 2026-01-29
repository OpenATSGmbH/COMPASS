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

#include "evaluationdefs.h"
#include "evaluationdata.h"

class EvaluationCalculator;
class EvaluationSettings;
class EvaluationStandard;

struct RequirementID;

namespace EvaluationRequirementResult
{
    class Base;
    class Single;
}

namespace ResultReport
{
    class Report;
}

/**
 */
class EvaluationResultsGenerator
{
public:
    EvaluationResultsGenerator(EvaluationCalculator& calculator);
    virtual ~EvaluationResultsGenerator();

    void evaluate(EvaluationStandard& standard,
                  const std::vector<unsigned int>& utns = std::vector<unsigned int>(),
                  const std::vector<Evaluation::RequirementResultID>& requirements = std::vector<Evaluation::RequirementResultID>(),
                  bool update_report = true);

    typedef std::map<std::string, std::map<std::string, std::shared_ptr<EvaluationRequirementResult::Base>>> ResultMap;
    typedef ResultMap::const_iterator ResultIterator;
    typedef std::vector<std::shared_ptr<EvaluationRequirementResult::Base>> ResultVector;

    ResultIterator begin() { return results_.begin(); }
    ResultIterator end() { return results_.end(); }

    const ResultMap& results() const { return results_; }
    const std::string& resultName() const { return result_name_; }

    void updateToChanges();
    void generateResultsReportGUI();

    void clear();

protected:
    void addTargetSection(const std::shared_ptr<ResultReport::Report>& report);
    void addNonResultsContent(const std::shared_ptr<ResultReport::Report>& report);

    void updateToChanges(bool reset_viewable,
                         bool update_report = true);

    EvaluationCalculator& calculator_;

    ResultMap    results_;     // rq group+name -> id -> result, e.g. "All:PD"->"UTN:22"-> or "SectorX:PD"->"All"
    ResultVector results_vec_; // ordered as generated
    std::string  result_name_;
};
