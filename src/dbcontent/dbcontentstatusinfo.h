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

#include "dbcontent/variable/variableset.h"
#include "buffer.h"

namespace dbContent
{

class DBContentStatusInfo
{
public:
    DBContentStatusInfo();
    virtual ~DBContentStatusInfo() = default;

    dbContent::VariableSet getReadSetFor(const std::string& dbcontent_name) const;
    void process(std::map<std::string,std::shared_ptr<Buffer>> buffers);

    bool hasInfo(unsigned int ds_id, unsigned int line_id) const;
    std::vector<boost::posix_time::ptime> getInfo(unsigned int ds_id, unsigned int line_id);

    const std::map<unsigned int, std::map<unsigned int, std::vector<boost::posix_time::ptime>>>& getInfo() const { return scan_info_; }

  protected:
    std::map<unsigned int, std::map<unsigned int, std::vector<boost::posix_time::ptime>>> scan_info_; // ds_id -> line_id-> times
};

}