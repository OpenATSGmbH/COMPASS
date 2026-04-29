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

#include "dbcontent/dbcontent.h"
#include "compass.h"
#include "buffer.h"
#include "buffer_utils.h"
#include "dbinterface.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/dbcontentwidget.h"
#include "db_context_manager.h"
#include "util/number.h"
#include "dbcontentreaddbjob.h"
#include "dbcontent/variable/variable.h"
#include "filtermanager.h"
#include "jobmanager.h"
#include "propertylist.h"
#include "updatebufferdbjob.h"
#include "dbcontentdeletedbjob.h"

#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <memory>

using namespace std;
using namespace Utils;
using namespace dbContent;

DBContent::DBContent(nlohmann::json& config, DBContentManager* parent)
    : Configurable(config, parent)
    , compass_(parent->compass())
    , dbcont_manager_(*parent)
{
    registerParameter("name", &name_, std::string("Undefined"));
    registerParameter("id", &id_, 0u);
    registerParameter("info", &info_, std::string());
    registerParameter("db_table_name", &db_table_name_, std::string());

    if (name_ == "CAT001" || name_ == "CAT010"
        || name_ == "CAT020"|| name_ == "CAT021"
        || name_ == "CAT048" || name_ == "CAT062"
        || name_ == "RefTraj")
    {
        contains_target_reports_ = true;
    }

    registerParameter("contains_target_reports", &contains_target_reports_, contains_target_reports_);

    if (name_ == "CAT002" || name_ == "CAT010" || name_ == "CAT019" || name_ == "CAT023" ||
        name_ == "CAT034" || name_ == "CAT063" || name_ == "CAT065")
    {
        contains_status_content_ = true;
    }

    registerParameter("contains_status_content", &contains_status_content_, contains_status_content_);

    traced_assert(db_table_name_.size());

    createSubConfigurables();

    logdbg << "created with instance_name " << instanceName() << " name " << name_;

    checkStaticVariable(dbcontent_vars::meta_var_ds_id_);

    if (contains_target_reports_)
    {
        checkStaticVariable(dbcontent_vars::meta_var_latitude_);
        checkStaticVariable(dbcontent_vars::meta_var_longitude_);
        checkStaticVariable(dbcontent_vars::meta_var_utn_);
    }

    is_reftraj_content_ = name_ == "RefTraj";

    if (name_ == "CAT001" || name_ == "CAT048")
    {
        checkStaticVariable(dbcontent_vars::var_radar_range_);
        checkStaticVariable(dbcontent_vars::var_radar_azimuth_);
        checkStaticVariable(dbcontent_vars::var_radar_altitude_);
    }
}

/**
 */
DBContent::~DBContent()
{
    logdbg << "start" << name_;
}

/**
 */
void DBContent::generateSubConfigurable(nlohmann::json& child_json)
{
    const auto& class_name = Configuration::getClassName(child_json);
    logdbg << "generating variable " << class_name;
    if (class_name == "Variable")
    {
        Variable* var = new Variable(child_json, this, name_);

        if (hasVariable(var->name()))
            logerr << "duplicate variable " << var->instanceName()
                   << " with name '" << var->name() << "'";

        traced_assert(!hasVariable(var->name()));

        logdbg << "generating variable " << var->instanceName()
               << " with name " << var->name();

        variables_.emplace(std::piecewise_construct,
                           std::forward_as_tuple(var->name()),   // args for key
                           std::forward_as_tuple(var));  // args for mapped value
    }
    else
    {
        throw runtime_error("DBContent: generateSubConfigurable: unknown class_name " + class_name);
    }
}

/**
 */
bool DBContent::hasVariable(const string& name) const
{
    return variables_.count(name);
}

/**
 */
Variable& DBContent::variable(const string& name) const
{
    traced_assert(hasVariable(name));
    traced_assert(variables_.at(name));

    return *(variables_.at(name).get());
}

/**
 */
void DBContent::renameVariable(const string& old_name, const string& new_name)
{
    loginf << "name " << old_name << " new_name " << new_name;

    traced_assert(hasVariable(old_name));
    traced_assert(!hasVariable(new_name));

    std::unique_ptr<Variable> var = std::move(variables_.at(old_name));
    variables_.erase(old_name);
    var->name(new_name);
    traced_assert(!variables_.count(old_name));
    variables_.emplace(new_name, std::move(var));
    traced_assert(variables_.count(new_name));

    traced_assert(!hasVariable(old_name));
    traced_assert(hasVariable(new_name));
}

/**
 */
void DBContent::deleteVariable(const string& name)
{
    traced_assert(hasVariable(name));

    variables_.erase(name);
    traced_assert(!hasVariable(name));
}

/**
 */
bool DBContent::hasVariableDBColumnName(const std::string& col_name) const
{
    for (const auto& var : variables_)
    {
        if (var.second->dbColumnName() == col_name)
            return true;
    }

    return false;
}

/**
 */
unsigned int DBContent::id() const
{
    return id_;
}

/**
 */
bool DBContent::hasKeyVariable()
{
    for (const auto& var_it : variables_)
        if (var_it.second->isKey())
            return true;

    return false;
}

/**
 */
Variable& DBContent::getKeyVariable()
{
    traced_assert(hasKeyVariable());

    for (const auto& var_it : variables_)  // search in any
    {
        if (var_it.second->isKey())
        {
            loginf << name() << ": returning first found var "
                   << var_it.first;
            return *var_it.second.get();
        }
    }

    throw runtime_error("DBContent: getKeyVariable: no key variable found");
}

/**
 */
string DBContent::status()
{
    if (read_job_)
    {
        if (loadedCount())
        {
            return "Loading";
        }
        else
        {
            if (read_job_->started())
                return "Started";
            else
                return "Queued";
        }
    }
    //    else if (finalize_jobs_.size() > 0)
    //        return "Post-processing";
    else
    {
        return "Idle";
    }
}

/**
 */
DBContentWidget* DBContent::widget()
{
    if (!widget_)
    {
        widget_.reset(new DBContentWidget(this));
        traced_assert(widget_);
    }

    return widget_.get();  // needed for qt integration, not pretty
}

/**
 */
void DBContent::closeWidget() 
{ 
    widget_ = nullptr; 
}

/**
 */
void DBContent::loadInternal(dbContent::VariableSet& read_set,
                             std::string custom_filter_clause)
{
    logdbg << "name " << name_ << " loadable " << is_loadable_;

    traced_assert(is_loadable_);
    traced_assert(existsInDB());

    traced_assert(!read_job_);

    // add required vars for processing
    traced_assert(dbcont_manager_.metaCanGetVariable(name_, dbcontent_vars::meta_var_rec_num_));
    read_set.add(dbcont_manager_.metaGetVariable(name_, dbcontent_vars::meta_var_rec_num_));

    traced_assert(dbcont_manager_.metaCanGetVariable(name_, dbcontent_vars::meta_var_ds_id_));
    read_set.add(dbcont_manager_.metaGetVariable(name_, dbcontent_vars::meta_var_ds_id_));

    traced_assert(dbcont_manager_.metaCanGetVariable(name_, dbcontent_vars::meta_var_line_id_));
    read_set.add(dbcont_manager_.metaGetVariable(name_, dbcontent_vars::meta_var_line_id_));

    read_job_ = shared_ptr<DBContentReadDBJob>(
                new DBContentReadDBJob(compass_.dbInterface(), *this, read_set, custom_filter_clause));

    connect(read_job_.get(),  &DBContentReadDBJob::obsoleteSignal,
            this, &DBContent::readJobObsoleteSlot, Qt::QueuedConnection);
    connect(read_job_.get(), &DBContentReadDBJob::doneSignal,
            this, &DBContent::readJobDoneSlot, Qt::QueuedConnection);

    compass_.jobManager().addDBJob(read_job_);
}

/**
 */
void DBContent::quitLoading()
{
    if (read_job_)
    {
        read_job_->setObsolete();
    }
}

/**
 */
bool DBContent::prepareInsert(shared_ptr<Buffer>& buffer)
{
    logdbg << name_ << ": prepareInsert: buffer " << buffer->size();

    traced_assert(!insert_active_);
    insert_active_ = true;

    VariableSet list;

    for (auto prop_it : buffer->properties().properties())
    {
        traced_assert(hasVariable(prop_it.name()));

        list.add(variable(prop_it.name()));

        if (!variable(prop_it.name()).hasDBContent())
            variable(prop_it.name()).setHasDBContent();
    }

    traced_assert(hasVariable(dbcontent_vars::meta_var_rec_num_.name())); // added during final db insert
    if (!variable(dbcontent_vars::meta_var_rec_num_.name()).hasDBContent())
        variable(dbcontent_vars::meta_var_rec_num_.name()).setHasDBContent();

    // transform variable names from dbcontvars to dbcolumns
    buffer_utils::transformVariables(*buffer, list, false);

    logdbg << "end";

    return true;
}

/**
 */
bool DBContent::updateDataSourcesBeforeInsert (shared_ptr<Buffer>& buffer)
{
    logdbg << name_;

    traced_assert(hasVariable(dbcontent_vars::meta_var_ds_id_.name()));

    // ds
    Variable& datasource_var = variable(dbcontent_vars::meta_var_ds_id_.name());
    traced_assert(datasource_var.dataType() == PropertyDataType::UINT);

    string datasource_col_str = datasource_var.dbColumnName();
    traced_assert(buffer->has<unsigned int>(datasource_col_str));

    // line
    Variable& line_var = variable(dbcontent_vars::meta_var_line_id_.name());
    traced_assert(line_var.dataType() == PropertyDataType::UINT);

    string line_col_str = line_var.dbColumnName();
    traced_assert(buffer->has<unsigned int>(line_col_str));

    // timestamp
    Variable& timestamp_var = variable(dbcontent_vars::meta_var_timestamp_.name());
    traced_assert(timestamp_var.dataType() == PropertyDataType::TIMESTAMP);
    string timestamp_col_str = timestamp_var.dbColumnName();

    if (!buffer->has<boost::posix_time::ptime>(timestamp_col_str))
        logerr << "no timestamp info given in " << name_;

    traced_assert(buffer->has<boost::posix_time::ptime>(timestamp_col_str));

    auto& ctx_man = compass_.dbContextManager();

    NullableVector<unsigned int>& datasource_vec = buffer->get<unsigned int>(datasource_col_str);
    NullableVector<unsigned int>& line_vec = buffer->get<unsigned int>(line_col_str);
    NullableVector<boost::posix_time::ptime>& timestamp_vec = buffer->get<boost::posix_time::ptime>(timestamp_col_str);

    map<unsigned int, map<unsigned int, unsigned int>> line_counts; // ds_id -> line -> cnt
    map<unsigned int, map<unsigned int, boost::posix_time::ptime>> line_tods; // ds_id -> line-> last timestamp

    unsigned int buffer_size = buffer->size();

    traced_assert(datasource_vec.isNeverNull());
    traced_assert(line_vec.isNeverNull());

    for (unsigned int cnt=0; cnt < buffer_size; ++cnt)
    {
        line_counts[datasource_vec.get(cnt)][line_vec.get(cnt)]++;

        if (!timestamp_vec.isNull(cnt))
            line_tods[datasource_vec.get(cnt)][line_vec.get(cnt)] = timestamp_vec.get(cnt);
    }

    bool ds_added = false;

    for (auto& ds_id_it : line_counts) // ds_id -> line -> cnt
    {
        // add data source if not yet known
        if (!ctx_man.hasDataSource(ds_id_it.first))
        {
            unsigned int sac = Number::sacFromDsId(ds_id_it.first);
            unsigned int sic = Number::sicFromDsId(ds_id_it.first);
            ctx_man.createDataSource(sac, sic);
            ds_added = true;
        }

        traced_assert(ctx_man.hasDataSource(ds_id_it.first));

        for (auto& line_cnt_it : ds_id_it.second) // line -> cnt
        {
            logdbg << "addNumInserted ds_id " << ds_id_it.first << " dbc " << name_
                   << " line " << line_cnt_it.first << " cnt " << line_cnt_it.second;
            ctx_man.addNumInserted(ds_id_it.first, name_, line_cnt_it.first, line_cnt_it.second);
        }

        if (line_tods.count(ds_id_it.first))
        {
            for (auto& line_tod_it : line_tods.at(ds_id_it.first))
                ctx_man.maxTimestamp(ds_id_it.first, line_tod_it.first, line_tod_it.second);
        }
    }

    return ds_added;
}

/**
 */
void DBContent::finalizeInsert(std::shared_ptr<Buffer>& buffer)
{
    logdbg << name_;

    traced_assert(buffer);
    traced_assert(insert_active_);

    insert_active_ = false;

    is_loadable_ = true;
    count_ += buffer->size();

    traced_assert(existsInDB()); // check
}

/**
 */
void DBContent::updateData(Variable& key_var, shared_ptr<Buffer> buffer)
{
    traced_assert(!update_job_);
    traced_assert(!insert_active_);

    traced_assert(existsInDB());

    VariableSet list;

    for (auto prop_it : buffer->properties().properties())
    {
        traced_assert(hasVariable(prop_it.name()));
        list.add(variable(prop_it.name()));

        if (!variable(prop_it.name()).hasDBContent())
            variable(prop_it.name()).setHasDBContent();
    }

    // transform variable names from dbcontvars to dbcolumns
    buffer_utils::transformVariables(*buffer, list, false);

    update_job_ =
            make_shared<UpdateBufferDBJob>(compass_.dbInterface(), *this, key_var, buffer);

    connect(update_job_.get(), &UpdateBufferDBJob::doneSignal, this, &DBContent::updateDoneSlot,
            Qt::QueuedConnection);
    connect(update_job_.get(), &UpdateBufferDBJob::updateProgressSignal, this,
            &DBContent::updateProgressSlot, Qt::QueuedConnection);

    compass_.jobManager().addDBJob(update_job_);
}

/**
 */
void DBContent::deleteDBContentData(bool cleanup_db)
{
    loginf << "dbcontent_name '" << name_ << "'";

    if (!existsInDB())
        return;

    traced_assert(!delete_job_);

    delete_job_ = make_shared<DBContentDeleteDBJob>(compass_.dbInterface());
    delete_job_->setSpecificDBContent(name_);
    delete_job_->cleanupDB(cleanup_db);

    connect(delete_job_.get(), &DBContentDeleteDBJob::doneSignal, this, &DBContent::deleteJobDoneSlot,
            Qt::QueuedConnection);

    compass_.jobManager().addDBJob(delete_job_);
}

/**
 */
void DBContent::deleteDBContentData(unsigned int sac, unsigned int sic, bool cleanup_db)
{
    loginf << "dbcontent_name '" << name_ << "' sac/sic " << sac << "/" << sic;

    if (!existsInDB())
        return;

    traced_assert(!delete_job_);

    delete_job_ = make_shared<DBContentDeleteDBJob>(compass_.dbInterface());
    delete_job_->setSpecificDBContent(name_);
    delete_job_->setSpecificSacSic(sac, sic);
    delete_job_->cleanupDB(cleanup_db);

    connect(delete_job_.get(), &DBContentDeleteDBJob::doneSignal, this, &DBContent::deleteJobDoneSlot,
            Qt::QueuedConnection);

    compass_.jobManager().addDBJob(delete_job_);
}

/**
 */
void DBContent::deleteDBContentData(unsigned int sac, unsigned int sic, unsigned int line_id, bool cleanup_db)
{
    loginf << "dbcontent_name '" << name_ << "' sac/sic " << sac << "/" << sic
           << " line_id " << line_id;

    if (!existsInDB())
        return;

    traced_assert(!delete_job_);

    delete_job_ = make_shared<DBContentDeleteDBJob>(compass_.dbInterface());
    delete_job_->setSpecificDBContent(name_);
    delete_job_->setSpecificSacSic(sac, sic);
    delete_job_->setSpecificLineId(line_id);
    delete_job_->cleanupDB(cleanup_db);

    connect(delete_job_.get(), &DBContentDeleteDBJob::doneSignal, this, &DBContent::deleteJobDoneSlot,
            Qt::QueuedConnection);

    compass_.jobManager().addDBJob(delete_job_);
}

/**
 */
void DBContent::updateProgressSlot(float percent) 
{ 
    emit updateProgressSignal(percent); 
}

/**
 */
void DBContent::updateDoneSlot()
{
    update_job_ = nullptr;

    emit updateDoneSignal(*this);
}

/**
 */
void DBContent::deleteJobDoneSlot()
{
    loginf;

    traced_assert(delete_job_);

    delete_job_ = nullptr;

    compass_.dbContextManager().writeContextToDB();
    emit compass_.dbContextManager().dataSourcesChangedSignal();

    // remove from inserted count
    //compass_.dataSourceManager().clearInsertedCounts(name_);
    //compass_.dataSourceManager().saveDBDataSources();

    // remove from targets count
    //dbcont_manager_.removeDBContentFromTargets(name_);

    count_ = compass_.dbInterface().count(db_table_name_);
}

/**
 */
void DBContent::readJobObsoleteSlot()
{
    logdbg << name_;
    read_job_ = nullptr;

    logdbg << name_ << ": done";
    dbcont_manager_.loadingDone(*this);
}

/**
 */
void DBContent::readJobDoneSlot()
{
    logdbg << name_;

    traced_assert(read_job_);

    shared_ptr<Buffer> buffer = read_job_->takeBuffer();

    if (buffer && buffer->size())
    {
        // verify variables present and typed correctly
        const vector<Variable*>& variables = read_job_->readList().getSet();
        const PropertyList& properties = buffer->properties();

        for (auto var_it : variables)
        {
            traced_assert(properties.hasProperty(var_it->dbColumnOrExpression()));
            const Property& property = properties.get(var_it->dbColumnOrExpression());
            traced_assert(property.dataType() == var_it->dataType());
        }

        // rename DB columns to variable names
        buffer_utils::transformVariables(*buffer, read_job_->readList(), true);

        // add boolean to indicate selection
        buffer->addProperty(dbcontent_vars::selected_var_);

        dbcont_manager_.addLoadedData({{name_, buffer}});
    }

    read_job_ = nullptr;

    logdbg << name_ << ": done";
    dbcont_manager_.loadingDone(*this);
}

/**
 */
void DBContent::databaseOpenedSlot()
{
    logdbg << name_;

    //string associations_table_name = associationsTableName();

    is_loadable_ = existsInDB();

    if (is_loadable_)
        count_ = compass_.dbInterface().count(db_table_name_);

    logdbg << name_ << ": table " << db_table_name_
           << " count " << count_;
}

/**
 */
void DBContent::databaseClosedSlot()
{
    logdbg;

    is_loadable_ = false;
    count_ = 0;
}

/**
 */
string DBContent::dbTableName() const
{
    return db_table_name_;
}

/**
 */
bool DBContent::isLoading() { return read_job_ != nullptr; }

/**
 */
bool DBContent::isDeleting() { return delete_job_ != nullptr; }

/**
 */
bool DBContent::hasData() { return count_ > 0; }

/**
 */
size_t DBContent::count() { return count_; }

/**
 */
void DBContent::refreshCount()
{
    if (existsInDB())
        count_ = compass_.dbInterface().count(db_table_name_);
}

/**
 */
size_t DBContent::loadedCount()
{
    if (dbcont_manager_.data().count(name_))
        return dbcont_manager_.data().at(name_)->size();
    else
        return 0;
}

/**
 */
bool DBContent::existsInDB() const
{
    return compass_.dbInterface().existsTable(db_table_name_);
}

/**
 */
void DBContent::checkStaticVariable(const Property& property)
{
    if (!hasVariable(property.name()))
    {
        logwrn << "start " << name_ << " has no variable " << property.name();
    }
    else if (variable(property.name()).dataType() != property.dataType())
    {
        logwrn << "start " << name_ << " variable " << property.name()
               << " has wrong data type (" << variable(property.name()).dataTypeString()
               << " insteaf of " << property.dataTypeString() << ")";
    }
}

bool DBContent::containsTargetReports() const
{
    return contains_target_reports_;
}

/**
 */
bool DBContent::containsStatusContent() const
{
    return contains_status_content_;
}

/**
 */
bool DBContent::isReferenceContent() const
{
    return is_reftraj_content_;
}

