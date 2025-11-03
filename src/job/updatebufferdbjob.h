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

#include "job.h"

#include <list>
#include "traced_assert.h"

class Buffer;
class DBContent;
class DBInterface;

namespace dbContent
{
class Variable;
}

class UpdateBufferDBJob : public Job
{
    Q_OBJECT

  signals:
    void updateProgressSignal(float percent);

  public:
    UpdateBufferDBJob(DBInterface& db_interface, DBContent& dbcontbject, dbContent::Variable& key_var,
                      std::shared_ptr<Buffer> buffer);

    virtual ~UpdateBufferDBJob();

    std::shared_ptr<Buffer> buffer()
    {
        traced_assert(buffer_);
        return buffer_;
    }

  protected:
    virtual void run_impl();

    DBInterface& db_interface_;
    DBContent& dbcontbject_;
    dbContent::Variable& key_var_;
    std::shared_ptr<Buffer> buffer_;
};
