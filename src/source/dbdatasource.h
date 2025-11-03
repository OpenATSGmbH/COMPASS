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

#include "source/datasourcebase.h"
#include "property.h"

#include "boost/date_time/posix_time/ptime.hpp"

#include <set>

namespace dbContent
{

class ConfigurationDataSource;
class DBDataSourceWidget;

class DBDataSource : public DataSourceBase
{
public:

    static const std::string table_name_;

    const static Property id_column_;
    const static Property ds_type_column_;
    const static Property sac_column_;
    const static Property sic_column_;
    const static Property name_column_;
    const static Property short_name_;
    const static Property info_column_;
    const static Property counts_column_;

    DBDataSource();
    virtual ~DBDataSource();

    void counts (const std::string& counts); // only for init
    std::string countsStr();
    bool hasNumInserted() const;
    bool hasNumInserted(const std::string& db_content) const;
    bool hasNumInserted(unsigned int line_id) const;
    bool hasNumInserted(const std::string& db_content, unsigned int line_id) const;
    const std::map<std::string, std::map<unsigned int, unsigned int>>& numInsertedMap() const;
    std::map<unsigned int, unsigned int> numInsertedLinesMap() const;
    std::map<std::string, unsigned int> numInsertedSummedLinesMap() const;
    std::set<std::string> insertedDBContents() const;

    void addNumInserted(const std::string& db_content, unsigned int line_id, unsigned int num);
    void clearNumInserted(const std::string& db_content);
    void clearNumInserted(const std::string& db_content, unsigned int line_id);

    void addNumLoaded(const std::string& db_content, unsigned int line_id, unsigned int num);
    unsigned int numLoaded (unsigned int line_id) const;
    unsigned int numLoaded (const std::string& db_content) const;
    unsigned int numLoaded (const std::string& db_content, unsigned int line_id) const;
    bool hasNumLoaded (unsigned int line_id) const; // for any DBContent
    bool hasAnyNumLoaded () const; // for any DBContent, line
    unsigned int getFirstLoadedLine(); // for any DBContent
    void clearNumLoaded();

    virtual unsigned int id() const override; // from saved db content
    void id(unsigned int id);

    bool loadingWanted() const;
    void loadingWanted(bool loading_wanted);

    bool lineSpecificLoadingWanted() const;
    bool anyLinesLoadingWanted() const;
    void disableAllLines();
    void enableAllLines();
    void lineLoadingWanted(unsigned int line_id, bool wanted);
    bool lineLoadingWanted(unsigned int line_id) const;
    std::set<unsigned int> getLoadingWantedLines() const;

    //DBDataSourceWidget* widget();

    bool hasMaxTimestamp(unsigned int line) const;
    void maxTimestamp(unsigned int line, boost::posix_time::ptime value);
    boost::posix_time::ptime maxTimestamp(unsigned int line) const;

    bool hasLiveData(unsigned int line, boost::posix_time::ptime current_ts) const;

    virtual nlohmann::json getAsJSON() const override;

protected:
    unsigned int id_{0};

    bool loading_wanted_ {true};

    nlohmann::json counts_;

    std::map<std::string, std::map<unsigned int, unsigned int>> num_loaded_; // db_content -> line id -> count
    std::map<std::string, std::map<unsigned int, unsigned int>> num_inserted_; // db_content -> line id -> count

    std::map<unsigned int, bool> line_loading_wanted_; // not contained means true

    std::map<unsigned int, unsigned long> max_line_tods_; // only used in live mode, line -> max tod

    //std::unique_ptr<DBDataSourceWidget> widget_;
};

}
