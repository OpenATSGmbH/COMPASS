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

#include "job/job.h"

#include <list>
#include "traced_assert.h"
#include <map>
#include <memory>
#include <string>

class Buffer;
class DBInterface;
class DBContentManager;
class DBTable;

/**
 * @brief DBContent insert job
 *
 * Writes dbcontent buffer data contents to a database table.
 */
class DBContentInsertDBJob : public Job
{
    Q_OBJECT

signals:
    void insertProgressSignal(float percent);

public:
    DBContentInsertDBJob(DBInterface& db_interface, 
                         DBContentManager& db_content_man,
                         std::map<std::string, std::shared_ptr<Buffer>>& buffers,
                         bool emit_change = true);

    virtual ~DBContentInsertDBJob();

    /**
     */
    const std::map<std::string, std::shared_ptr<Buffer>>& buffers()
    {
        return buffers_;
    }

    bool emitChange() const;

protected:
    virtual void run_impl();

    DBInterface&      db_interface_;
    DBContentManager& db_content_man_;

    std::map<std::string, std::shared_ptr<Buffer>> buffers_;

    bool emit_change_{true};
};
