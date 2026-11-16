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

#include "datasourceinspectorbase.h"
#include "analyzedatasourcetask.h"
#include "analysisdataset.h"
#include "inspectorsettingsbase.h"
#include "dbcontent/target/targetreportchain.h"
#include "db_context_manager.h"
#include "taskmanager.h"
#include "taskresult.h"
#include "reporttable.h"
#include "traced_assert.h"

#include <cctype>

#include <QDoubleSpinBox>

DataSourceInspectorBase::DataSourceInspectorBase(AnalyzeDataSourceTask& task,
                                                 InspectorSettingsBase& settings)
    : task_(task), settings_(settings)
{
}

namespace
{
std::string joinNames(const std::set<std::string>& names, const std::string& sep = ", ")
{
    std::string out;
    for (const auto& n : names)
        out += (out.empty() ? "" : sep) + n;
    return out;
}

/// Sum of the inserted counts of `dbcontents` on `line_id` over `ds_ids`.
std::uint64_t numInsertedOnLine(const context::DBContextManager& ctx,
                                const std::set<unsigned int>& ds_ids,
                                const std::set<std::string>& dbcontents,
                                unsigned int line_id)
{
    std::uint64_t num = 0;
    for (auto ds_id : ds_ids)
    {
        for (const auto& dbc : dbcontents)
        {
            auto counts = ctx.numInsertedPerLine(ds_id, dbc);
            auto it = counts.find(line_id);
            if (it != counts.end())
                num += it->second;
        }
    }
    return num;
}
}

bool DataSourceInspectorBase::prerequisitesMet(std::string& reason_out) const
{
    reason_out.clear();

    if (requiresProfessionalLicense() && !task_.professionalLicenseEnabled())
    {
        reason_out = "Requires the Professional license.";
        return false;
    }

    if (task_.selectedDataSourceIDs().empty())
    {
        reason_out = "No test data source selected.";
        return false;
    }

    if (requiresCAT020Source() && !task_.selectionContainsCAT020())
    {
        reason_out = "Requires at least one CAT020 test data source.";
        return false;
    }

    auto& ctx = task_.manager().dbContextManager();

    const auto tst_dbcontents = testDBContentNames();
    if (!tst_dbcontents.empty()
        && numInsertedOnLine(ctx, task_.selectedDataSourceIDs(), tst_dbcontents, task_.lineIDTst()) == 0)
    {
        reason_out = "No " + joinNames(tst_dbcontents, "/") + " data in the selected test data sources on line L"
                     + std::to_string(task_.lineIDTst() + 1) + ".";
        return false;
    }

    if (requiresLoadedDataset())
    {
        // the dataset load needs a reference data source, also when the
        // inspector does not compare against the Reference Trajectory
        const auto ref_ds_ids = task_.selectedReferenceDataSourceIDs();
        if (ref_ds_ids.empty())
        {
            reason_out = "No reference data source selected.";
            return false;
        }

        if (requiresReferenceTrajectory()
            && numInsertedOnLine(ctx, ref_ds_ids, {AnalysisDataset::referenceDBContentName()},
                                 task_.lineIDRef()) == 0)
        {
            reason_out = "No Reference Trajectory data in the selected reference data sources on line L"
                         + std::to_string(task_.lineIDRef() + 1) + ".";
            return false;
        }
    }

    return specificPrerequisitesMet(reason_out);
}

bool DataSourceInspectorBase::specificPrerequisitesMet(std::string& reason_out) const
{
    reason_out.clear();
    return true;
}

bool DataSourceInspectorBase::canCompute(const AnalysisDataset* dataset, std::string& reason_out) const
{
    reason_out.clear();

    if (requiresLoadedDataset())
    {
        if (!dataset)
        {
            reason_out = "The combined dataset could not be built.";
            return false;
        }

        // test data contents with in-scope rows, associated or not
        std::set<std::string> present;
        for (const auto& dbc : testDBContentNames())
        {
            if (dataset->testDbContentsPresent().count(dbc) || !dataset->unassociatedIndices(dbc).empty())
                present.insert(dbc);
        }

        if (present.empty())
        {
            reason_out = "The loaded data holds no " + joinNames(testDBContentNames(), "/")
                         + " test data inside the scope filter.";
            return false;
        }

        if (requiresReferenceTrajectory() && dataset->numReferenceRecordsTotal() == 0)
        {
            reason_out = "The loaded data holds no Reference Trajectory data.";
            return false;
        }

        for (const auto& var : testVariables())
        {
            if (!var.required)
                continue;

            bool has_values = false;
            for (const auto& dbc : present)
            {
                auto values = dataset->testVariableHasValues(dbc, var.property, var.meta);
                if (values && *values)
                {
                    has_values = true;
                    break;
                }
            }

            if (!has_values)
            {
                reason_out = "The loaded " + joinNames(present, "/") + " test data holds no values for '"
                             + var.property.name() + "'"
                             + (var.usage.empty() ? std::string() : ", required for " + var.usage) + ".";
                return false;
            }
        }
    }

    return specificCanCompute(dataset, reason_out);
}

bool DataSourceInspectorBase::specificCanCompute(const AnalysisDataset* dataset, std::string& reason_out) const
{
    (void)dataset;
    reason_out.clear();
    return true;
}

std::vector<std::string> DataSourceInspectorBase::missingOptionalVariableNotes(const AnalysisDataset& dataset) const
{
    std::vector<std::string> notes;

    for (const auto& var : testVariables())
    {
        if (var.required)
            continue;

        std::set<std::string> without_values;
        bool has_values = false;

        for (const auto& dbc : testDBContentNames())
        {
            auto values = dataset.testVariableHasValues(dbc, var.property, var.meta);
            if (!values)
                continue; // data content not loaded or without this variable

            if (*values)
            {
                has_values = true;
                break;
            }
            without_values.insert(dbc);
        }

        if (has_values || without_values.empty())
            continue;

        notes.push_back("The loaded " + joinNames(without_values, "/") + " test data holds no values for '"
                        + var.property.name() + "'"
                        + (var.usage.empty() ? std::string() : ", " + var.usage + " is not available") + ".");
    }

    return notes;
}

ReportTableWriter& DataSourceInspectorBase::tableWriter() const
{
    auto& result = task_.manager().currentResult();
    traced_assert(result);

    return result->tableWriter();
}

std::string DataSourceInspectorBase::tableKey(const std::string& suffix) const
{
    // camel case class name to snake case, the trailing "_inspector" dropped
    const std::string cls = className();
    std::string key;

    for (std::size_t i = 0; i < cls.size(); ++i)
    {
        const char c = cls[i];

        if (std::isupper(static_cast<unsigned char>(c)) && i > 0)
        {
            const bool prev_lower = std::islower(static_cast<unsigned char>(cls[i - 1]))
                                    || std::isdigit(static_cast<unsigned char>(cls[i - 1]));
            const bool next_lower = i + 1 < cls.size()
                                    && std::islower(static_cast<unsigned char>(cls[i + 1]));
            if (prev_lower || next_lower)
                key += '_';
        }

        key += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }

    const std::string tail = "_inspector";
    if (key.size() > tail.size() && key.compare(key.size() - tail.size(), tail.size(), tail) == 0)
        key.erase(key.size() - tail.size());

    return key + suffix;
}

ReportTableDefinition DataSourceInspectorBase::recordTableDefinition(
    const std::vector<std::string>& host_dbcontents,
    const std::string& key_suffix,
    const std::string& name_suffix) const
{
    return ReportTableDefinition::record(tableKey(key_suffix), name() + name_suffix, host_dbcontents);
}

ReportTableDefinition DataSourceInspectorBase::accuracyTableDefinition(
    const std::vector<std::string>& host_dbcontents) const
{
    auto def = recordTableDefinition(host_dbcontents);

    def.addColumn("dt_prev_s", PropertyDataType::DOUBLE, "Time Since Previous",
                  "Time since the previous test report of the target", "Time", "Second");
    def.addColumn("tst_lat", PropertyDataType::DOUBLE, "Latitude",
                  "Latitude of the test report", "Angle", "Degree");
    def.addColumn("tst_lon", PropertyDataType::DOUBLE, "Longitude",
                  "Longitude of the test report", "Angle", "Degree");
    def.addColumn("ref_lat", PropertyDataType::DOUBLE, "Reference Latitude",
                  "Latitude of the reference at the time of the test report", "Angle", "Degree");
    def.addColumn("ref_lon", PropertyDataType::DOUBLE, "Reference Longitude",
                  "Longitude of the reference at the time of the test report", "Angle", "Degree");
    def.addColumn("distance_m", PropertyDataType::DOUBLE, "Distance",
                  "Horizontal distance between the test report and the reference", "Length", "Meter");
    def.addColumn("reported_stddev_m", PropertyDataType::DOUBLE, "Reported Std.Dev.",
                  "Position standard deviation reported by the sensor", "Length", "Meter");
    def.addColumn("consistency_ratio", PropertyDataType::DOUBLE, "Consistency Ratio",
                  "Distance divided by the reported standard deviation");
    def.addColumn("gated", PropertyDataType::BOOL, "Gated",
                  "Reference not accurate enough, the distance is not assessed");
    def.addColumn("ref_rec_num_1", PropertyDataType::ULONGINT, "Reference Record Number 1",
                  "Record number of the reference update before the test report");
    def.addColumn("ref_rec_num_2", PropertyDataType::ULONGINT, "Reference Record Number 2",
                  "Record number of the reference update after the test report");

    return def;
}

void DataSourceInspectorBase::writeAccuracyRow(ReportTableRows& rows, const AccuracyRow& row)
{
    rows.setRecordNumber(row.rec_num);
    rows.setUTN(row.utn);
    rows.setTimestamp(row.timestamp);

    if (row.dt_prev_s)
        rows.set<double>("dt_prev_s", *row.dt_prev_s);

    rows.set<double>("tst_lat"   , row.tst_lat);
    rows.set<double>("tst_lon"   , row.tst_lon);
    rows.set<double>("ref_lat"   , row.ref_lat);
    rows.set<double>("ref_lon"   , row.ref_lon);
    rows.set<double>("distance_m", row.distance_m);

    if (row.reported_stddev_m)
        rows.set<double>("reported_stddev_m", *row.reported_stddev_m);
    if (row.consistency_ratio)
        rows.set<double>("consistency_ratio", *row.consistency_ratio);

    rows.set<bool>("gated", row.gated);

    if (row.ref_rec_nums.first)
        rows.set<std::uint64_t>("ref_rec_num_1", *row.ref_rec_nums.first);
    if (row.ref_rec_nums.second)
        rows.set<std::uint64_t>("ref_rec_num_2", *row.ref_rec_nums.second);

    rows.nextRow();
}

double DataSourceInspectorBase::secondsBetween(const boost::posix_time::ptime& t0,
                                               const boost::posix_time::ptime& t1)
{
    return (t1 - t0).total_microseconds() / 1e6;
}

QDoubleSpinBox* DataSourceInspectorBase::makeFloatSpin(double min,
                                                       double max,
                                                       double step,
                                                       int decimals,
                                                       const QString& suffix,
                                                       double value)
{
    auto* s = new QDoubleSpinBox();
    s->setRange(min, max);
    s->setSingleStep(step);
    s->setDecimals(decimals);
    if (!suffix.isEmpty())
        s->setSuffix(suffix);
    s->setValue(value);
    return s;
}

ReportTableDefinition DataSourceInspectorBase::gapTableDefinition(const std::string& key_suffix,
                                                                  const std::string& name_suffix) const
{
    auto def = recordTableDefinition({ AnalysisDataset::referenceDBContentName() }, key_suffix, name_suffix);

    def.addColumn("gap_begin", PropertyDataType::TIMESTAMP, "Begin",
                  "Begin of the gap, the last test report before it or the period begin");
    def.addColumn("gap_end", PropertyDataType::TIMESTAMP, "End",
                  "End of the gap, the first test report after it or the period end");
    def.addColumn("duration_s", PropertyDataType::DOUBLE, "Duration",
                  "Duration of the gap", "Time", "Second");
    def.addColumn("missed_updates", PropertyDataType::UINT, "Missed Updates",
                  "Number of missed updates inside the gap");
    def.addColumn("ref_lat", PropertyDataType::DOUBLE, "Reference Latitude",
                  "Latitude of the first reference sample inside the gap", "Angle", "Degree");
    def.addColumn("ref_lon", PropertyDataType::DOUBLE, "Reference Longitude",
                  "Longitude of the first reference sample inside the gap", "Angle", "Degree");
    def.addColumn("ref_lat_end", PropertyDataType::DOUBLE, "Reference Latitude End",
                  "Latitude of the last reference sample inside the gap", "Angle", "Degree");
    def.addColumn("ref_lon_end", PropertyDataType::DOUBLE, "Reference Longitude End",
                  "Longitude of the last reference sample inside the gap", "Angle", "Degree");
    def.addColumn("ref_rec_num_last", PropertyDataType::ULONGINT, "Last Reference Record Number",
                  "Record number of the last reference sample inside the gap");
    def.addColumn("tst_rec_num_before", PropertyDataType::ULONGINT, "Test Record Number Before",
                  "Record number of the test report at the begin of the gap");
    def.addColumn("tst_rec_num_after", PropertyDataType::ULONGINT, "Test Record Number After",
                  "Record number of the test report at the end of the gap");

    return def;
}

bool DataSourceInspectorBase::writeGapRow(ReportTableRows& rows, AnalysisDataset& dataset, const GapRow& gap)
{
    auto first = dataset.firstReferenceSampleInRange(gap.utn, gap.begin, gap.end);
    if (!first)
        return false;

    auto& ref_chain = dataset.referenceChain(gap.utn);

    rows.setRecordNumber(ref_chain.recordNumber(*first));
    rows.setUTN(gap.utn);
    rows.setTimestamp(first->timestamp());

    rows.set<boost::posix_time::ptime>("gap_begin", gap.begin);
    rows.set<boost::posix_time::ptime>("gap_end", gap.end);
    rows.set<double>("duration_s", secondsBetween(gap.begin, gap.end));
    rows.set<unsigned int>("missed_updates", gap.num_missed);

    auto pos = ref_chain.posOpt(*first);
    if (pos)
    {
        rows.set<double>("ref_lat", pos->latitude_);
        rows.set<double>("ref_lon", pos->longitude_);
    }

    auto last = dataset.lastReferenceSampleInRange(gap.utn, gap.begin, gap.end);
    if (last)
    {
        rows.set<std::uint64_t>("ref_rec_num_last", ref_chain.recordNumber(*last));

        auto pos_last = ref_chain.posOpt(*last);
        if (pos_last)
        {
            rows.set<double>("ref_lat_end", pos_last->latitude_);
            rows.set<double>("ref_lon_end", pos_last->longitude_);
        }
    }

    auto before = dataset.testRecordNumberAt(gap.utn, gap.begin);
    if (before)
        rows.set<std::uint64_t>("tst_rec_num_before", *before);

    auto after = dataset.testRecordNumberAt(gap.utn, gap.end);
    if (after)
        rows.set<std::uint64_t>("tst_rec_num_after", *after);

    rows.nextRow();

    return true;
}
