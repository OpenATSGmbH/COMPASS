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

#include "boost/date_time/posix_time/posix_time.hpp"
#include "job.h"

class DBInterface;

class DBContentDeleteDBJob : public Job
{
    Q_OBJECT

public:
    DBContentDeleteDBJob(DBInterface& db_interface);

    virtual ~DBContentDeleteDBJob();

    void setBeforeTimestamp(boost::posix_time::ptime before_timestamp); // delete everything before ts
    void setSpecificDBContent(const std::string& specific_dbcontent); // delete everything in dbcontent
    void setSpecificSacSic(unsigned int sac, unsigned int sic);
    void setSpecificLineId(unsigned int line_id);
    void cleanupDB(bool cleanup_db);

    bool useSpecificDBContent() const;
    bool useBeforeTimestamp() const;

protected:
    virtual void run_impl();

    DBInterface& db_interface_;

    bool use_before_timestamp_ {false};
    boost::posix_time::ptime before_timestamp_;

    bool use_specific_dbcontent_ {false};
    std::string specific_dbcontent_ {false};

    bool use_specific_sac_sic_ {false};
    unsigned int specific_sac_ {0};
    unsigned int specific_sic_ {0};

    bool use_specific_line_id_ {false};
    unsigned int specific_line_id_ {0};

    bool cleanup_db_ = false;
};
