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

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "job.h"

class ReadJSONFileJob : public Job
{
    Q_OBJECT
  signals:
    void readJSONFilePartSignal();

  public:
    ReadJSONFileJob(const std::string& file_name, unsigned int num_objects);
    virtual ~ReadJSONFileJob();

    void pause();
    void unpause();

    std::vector<std::string> objects();  // for moving out

    size_t bytesRead() const;
    size_t bytesToRead() const;

    float getStatusPercent();

  protected:
    void run_impl() override;

    std::string file_name_;
    bool archive_{false};
    unsigned int num_objects_{0};

    bool file_read_done_{false};
    bool init_performed_{false};

    std::ifstream file_stream_;
    std::stringstream tmp_stream_;

    unsigned int open_count_{0};

    struct archive* a;
    struct archive_entry* entry;
    int64_t offset;
    bool entry_done_{true};  // init to done to trigger read of next header

    size_t bytes_to_read_{0};
    size_t bytes_read_{0};
    size_t bytes_read_tmp_{0};
    std::vector<std::string> objects_;

    volatile bool pause_{false};

    void performInit();
    void readFilePart();

    void openArchive(bool raw);
    void closeArchive();

    void cleanCommas();
};
