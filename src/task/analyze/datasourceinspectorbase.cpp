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
#include "compass.h"
#include "taskmanager.h"
#include "taskresult.h"
#include "reporttable.h"
#include "traced_assert.h"

#include <cctype>

DataSourceInspectorBase::DataSourceInspectorBase(AnalyzeDataSourceTask& task,
                                                 InspectorSettingsBase& settings)
    : task_(task), settings_(settings)
{
}

bool DataSourceInspectorBase::prerequisitesMet(std::string& reason_out) const
{
    reason_out.clear();
    return true;
}

ReportTableWriter& DataSourceInspectorBase::tableWriter() const
{
    auto& result = task_.compass().taskManager().currentResult();
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
        rows.set<unsigned long>("ref_rec_num_1", *row.ref_rec_nums.first);
    if (row.ref_rec_nums.second)
        rows.set<unsigned long>("ref_rec_num_2", *row.ref_rec_nums.second);

    rows.nextRow();
}

double DataSourceInspectorBase::secondsBetween(const boost::posix_time::ptime& t0,
                                               const boost::posix_time::ptime& t1)
{
    return (t1 - t0).total_microseconds() / 1e6;
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
        rows.set<unsigned long>("ref_rec_num_last", ref_chain.recordNumber(*last));

        auto pos_last = ref_chain.posOpt(*last);
        if (pos_last)
        {
            rows.set<double>("ref_lat_end", pos_last->latitude_);
            rows.set<double>("ref_lon_end", pos_last->longitude_);
        }
    }

    auto before = dataset.testRecordNumberAt(gap.utn, gap.begin);
    if (before)
        rows.set<unsigned long>("tst_rec_num_before", *before);

    auto after = dataset.testRecordNumberAt(gap.utn, gap.end);
    if (after)
        rows.set<unsigned long>("tst_rec_num_after", *after);

    rows.nextRow();

    return true;
}
