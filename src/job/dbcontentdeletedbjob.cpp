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

#include "dbcontentdeletedbjob.h"
#include "dbinterface.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
//#include "dbcontent/variable/variable.h"
#include "logger.h"
#include "compass.h"
#include "util/timeconv.h"

using namespace std;
using namespace Utils;

DBContentDeleteDBJob::DBContentDeleteDBJob(DBInterface& db_interface)
    : Job("DBContentDeleteDBJob"), db_interface_(db_interface)

{
}

DBContentDeleteDBJob::~DBContentDeleteDBJob() {}

void DBContentDeleteDBJob::setBeforeTimestamp(boost::posix_time::ptime before_timestamp)
{
    traced_assert(!use_specific_dbcontent_);
    use_before_timestamp_ = true;
    before_timestamp_ = before_timestamp;
}

void DBContentDeleteDBJob::setSpecificDBContent(const std::string& specific_dbcontent)
{
    traced_assert(!use_before_timestamp_);
    use_specific_dbcontent_ = true;
    specific_dbcontent_ = specific_dbcontent;
}

void DBContentDeleteDBJob::setSpecificSacSic(unsigned int sac, unsigned int sic)
{
    traced_assert(!use_before_timestamp_);
    traced_assert(use_specific_dbcontent_);

    use_specific_sac_sic_ = true;
    specific_sac_ = sac;
    specific_sic_ = sic;
}

void DBContentDeleteDBJob::setSpecificLineId(unsigned int line_id)
{
    traced_assert(!use_before_timestamp_);
    traced_assert(use_specific_dbcontent_);
    traced_assert(use_specific_sac_sic_);

    use_specific_line_id_ = true;
    specific_line_id_ = line_id;
}

void DBContentDeleteDBJob::cleanupDB(bool cleanup_db)
{
    cleanup_db_ = cleanup_db;
}

void DBContentDeleteDBJob::run_impl()
{
    logdbg;
    started_ = true;

    if (obsolete_)
    {
        logdbg << "obsolete before prepared";
        done_ = true;
        return;
    }

    if (!(use_before_timestamp_ || use_specific_dbcontent_))
    {
        logerr << "neither before time or dbcontent defined";
        done_ = true;
        return;
    }

    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();

    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    if (use_before_timestamp_)
    {
        for (auto& dbcont_it : dbcont_man)
        {
            if (!dbcont_it.second->existsInDB())
                continue;

            logdbg << "deleting dbcontent for " << dbcont_it.first;
            db_interface_.deleteBefore(*dbcont_it.second, before_timestamp_);
        }
    }
    else if (use_specific_dbcontent_)
    {
        if (use_specific_line_id_)
        {
            loginf << "deleting dbcontent for " << specific_dbcontent_
                   << " for specific sac/sic + line";
            traced_assert(dbcont_man.existsDBContent(specific_dbcontent_));
            traced_assert(use_specific_sac_sic_);

            db_interface_.deleteContent(dbcont_man.dbContent(specific_dbcontent_),
                                        specific_sac_, specific_sic_, specific_line_id_);
        }
        else if (use_specific_sac_sic_)
        {
            loginf << "deleting dbcontent for " << specific_dbcontent_
                   << " for specific sac/sic";
            traced_assert(dbcont_man.existsDBContent(specific_dbcontent_));

            db_interface_.deleteContent(dbcont_man.dbContent(specific_dbcontent_),
                                        specific_sac_, specific_sic_);
        }
        else // all
        {
            loginf << "deleting all dbcontent for " << specific_dbcontent_;
            traced_assert(dbcont_man.existsDBContent(specific_dbcontent_));
            db_interface_.deleteAll(dbcont_man.dbContent(specific_dbcontent_));
        }
    }
    else
        traced_assert(false);

    //cleanup db after delete?
    if (cleanup_db_)
        db_interface_.cleanupDB();

    boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::local_time() - start_time;

    logdbg << "done after " << Time::toString(diff);

    done_ = true;

    return;

}

bool DBContentDeleteDBJob::useSpecificDBContent() const
{
    return use_specific_dbcontent_;
}

bool DBContentDeleteDBJob::useBeforeTimestamp() const
{
    return use_before_timestamp_;
}
