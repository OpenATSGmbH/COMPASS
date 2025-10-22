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

#include "asteriximportsettings.h"
#include "job.h"

#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/thread/mutex.hpp>

#include "tbbhack.h"

#include <memory>
#include <map>

class Buffer;

class ASTERIXPostprocessJob : public Job
{
public:
    ASTERIXPostprocessJob(std::map<std::string, std::shared_ptr<Buffer>> buffers,
                          ASTERIXImportTaskSettings settings);

    ASTERIXPostprocessJob(std::map<std::string, std::shared_ptr<Buffer>> buffers);
    // ctor with no checks/overrides/filters for JSON

    virtual ~ASTERIXPostprocessJob();

    std::map<std::string, std::shared_ptr<Buffer>> buffers() { return std::move(buffers_); }

protected:
    void run_impl() override;

private:
    std::map<std::string, std::shared_ptr<Buffer>> buffers_;

    ASTERIXImportTaskSettings settings_;

    static boost::mutex m3a_map_mutex_;
    static tbb::concurrent_unordered_map<unsigned int, unsigned int> obfuscate_m3a_map_;
    static boost::mutex acad_map_mutex_;
    static tbb::concurrent_unordered_map<unsigned int, unsigned int> obfuscate_acad_map_;
    static boost::mutex acid_map_mutex_;
    static tbb::concurrent_unordered_map<std::string, std::string> obfuscate_acid_map_;

    void doRadarPlotPositionCalculations();
    void doXYPositionCalculations();
    void doADSBPositionProcessing();
    void doGroundSpeedCalculations();
    void doFilters();
    void doObfuscate();

    void obfuscateM3A (unsigned int& value);
    void obfuscateACAD (unsigned int& value);
    void obfuscateACID (std::string& value);
};

