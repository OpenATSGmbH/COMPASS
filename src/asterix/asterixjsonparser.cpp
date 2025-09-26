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

#include "asterixjsonparser.h"

#include "compass.h"
#include "buffer.h"
#include "configuration.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "asteriximporttask.h"
#include "files.h"
#include "traced_assert.h"

#include <QTableView>

#include <jasterix/jasterix.h>
#include <jasterix/iteminfo.h>

#include <boost/algorithm/string.hpp>

#include <algorithm>

using namespace std;
using namespace nlohmann;
using namespace Utils;


ASTERIXJSONParser::ASTERIXJSONParser(const std::string& class_id, const std::string& instance_id,
                                     Configurable* parent, ASTERIXImportTask& task)
    : Configurable(class_id, instance_id, parent,
                   "task_import_asterix_" + boost::algorithm::to_lower_copy(instance_id) + ".json"),
      task_(task)
{
    registerParameter("name", &name_, std::string());
    registerParameter("category", &category_, 0u);
    registerParameter("db_content_name", &db_content_name_, std::string());

    traced_assert(db_content_name_.size());

    if (!name_.size())
        name_ = db_content_name_;

    traced_assert(name_.size());

    traced_assert(task_.jASTERIX()->hasCategory(category_));
    item_info_ = task_.jASTERIX()->category(category_)->itemInfo();

    createSubConfigurables();

    todo_icon_ = Files::IconProvider::getIcon("todo.png");
    unknown_icon_ = Files::IconProvider::getIcon("todo_maybe.png");
    hint_icon_ = Files::IconProvider::getIcon("hint.png");
}

void ASTERIXJSONParser::generateSubConfigurable(const std::string& class_id,
                                                const std::string& instance_id)
{
    if (class_id == "JSONDataMapping")
    {
        data_mappings_.emplace_back(new JSONDataMapping(class_id, instance_id, *this));
        (*data_mappings_.rbegin())->mandatory(false);

        mapping_checks_dirty_ = true;
    }
    else
        throw std::runtime_error("ASTERIXJSONParser: generateSubConfigurable: unknown class_id " + class_id);
}

void ASTERIXJSONParser::doMappingChecks()
{
    loginf;

    beginResetModel();

    // update non existing keys
    not_existing_json_keys_.clear();

    for (auto& map_it : data_mappings_)
    {
        map_it->check();
    }

    for (auto& map_it : data_mappings_)
    {
        if (!item_info_.count(map_it->jsonKey()))
            not_existing_json_keys_.insert(map_it->jsonKey());
    }

    // update not mapped json keys
    not_added_json_keys_.clear();

    for (auto& info_it : item_info_)
    {
        if (!hasJSONKeyMapped(info_it.first))
            not_added_json_keys_.push_back(info_it.first);
    }

    // update not mapped dbcont vars
    not_added_dbcont_variables_.clear();

    for (auto& dbcontvar_it : dbContent().variables())
    {
        if (!hasDBContentVariableMapped(dbcontvar_it.first))
            not_added_dbcont_variables_.push_back(dbcontvar_it.first);
    }

    mapping_checks_dirty_ = false;

    endResetModel();
}

unsigned int ASTERIXJSONParser::totalEntrySize () const
{
    return data_mappings_.size() + not_added_json_keys_.size() + not_added_dbcont_variables_.size();
}

bool ASTERIXJSONParser::existsJSONKeyInCATInfo(const std::string& key)
{
    return !not_existing_json_keys_.count(key);
}

bool ASTERIXJSONParser::hasJSONKeyMapped (const std::string& key)
{
    return std::find_if(data_mappings_.begin(), data_mappings_.end(),
                        [key](const unique_ptr<JSONDataMapping>& mapping) -> bool { return mapping->jsonKey() == key; })
            != data_mappings_.end();
}

bool ASTERIXJSONParser::hasDBContentVariableMapped (const std::string& var_name)
{
    return std::find_if(data_mappings_.begin(), data_mappings_.end(),
                        [var_name](const unique_ptr<JSONDataMapping>& mapping) -> bool {
        return mapping->dbcontVariableName() == var_name; }) != data_mappings_.end();
}

bool ASTERIXJSONParser::hasJSONKeyInMapping (const std::string& key)
{
    return std::find_if(data_mappings_.begin(), data_mappings_.end(),
                        [key](const unique_ptr<JSONDataMapping>& mapping) -> bool { return mapping->jsonKey() == key; })
            != data_mappings_.end();
}

unsigned int ASTERIXJSONParser::indexOfJSONKeyInMapping (const std::string& key)
{
    traced_assert(hasJSONKeyInMapping(key));

    auto iter = std::find_if(data_mappings_.begin(), data_mappings_.end(),
                             [key](const unique_ptr<JSONDataMapping>& mapping) -> bool {
        return mapping->jsonKey() == key; });

    traced_assert(iter != data_mappings_.end());

    unsigned int pos = iter - data_mappings_.begin();

    traced_assert(pos < data_mappings_.size());

    return pos;
}

void ASTERIXJSONParser::selectMapping (unsigned int index)
{
    loginf << "index " << index;

    emit modelRowChanged(index);
}

void ASTERIXJSONParser::selectUnmappedDBContentVariable (const std::string& name)
{
    auto iter = find(not_added_dbcont_variables_.begin(), not_added_dbcont_variables_.end(), name);
    traced_assert(iter != not_added_dbcont_variables_.end());

    unsigned int pos = iter - not_added_dbcont_variables_.begin();

    traced_assert(pos < not_added_dbcont_variables_.size());

    unsigned int index = data_mappings_.size() + not_added_json_keys_.size() + pos;

    emit modelRowChanged(index);
}

//void ASTERIXJSONParser::updateToChangedIndex (unsigned int row_index)
//{
//    traced_assert(row_index < totalEntrySize());
//    emit dataChanged(index(row_index, 0), index(row_index, table_columns_.size()-1));
//}

ASTERIXJSONParser::EntryType ASTERIXJSONParser::entryType (unsigned int index) const
{
    if (index < data_mappings_.size()) // is actual mapping
        return ASTERIXJSONParser::EntryType::ExistingMapping;

    index -= data_mappings_.size();

    if (index < not_added_json_keys_.size())
        return ASTERIXJSONParser::EntryType::UnmappedJSONKey;

    index -= not_added_json_keys_.size();

    traced_assert(index < not_added_dbcont_variables_.size());

    return ASTERIXJSONParser::EntryType::UnmappedDBContentVariable;
}

JSONDataMapping& ASTERIXJSONParser::mapping (unsigned int index)
{
    traced_assert(entryType(index) == ASTERIXJSONParser::EntryType::ExistingMapping);

    traced_assert(data_mappings_.at(index));
    return *data_mappings_.at(index);
}

const JSONDataMapping& ASTERIXJSONParser::mapping (unsigned int index) const
{
    traced_assert(entryType(index) == ASTERIXJSONParser::EntryType::ExistingMapping);

    traced_assert(data_mappings_.at(index));
    return *data_mappings_.at(index);
}

const std::string& ASTERIXJSONParser::unmappedJSONKey (unsigned int index) const
{
    traced_assert(entryType(index) == ASTERIXJSONParser::EntryType::UnmappedJSONKey);
    return not_added_json_keys_.at(index - data_mappings_.size());
}

const std::string& ASTERIXJSONParser::unmappedDBContentVariable (unsigned int index) const
{
    traced_assert(entryType(index) == ASTERIXJSONParser::EntryType::UnmappedDBContentVariable);
    return not_added_dbcont_variables_.at(index - data_mappings_.size() - not_added_json_keys_.size());
}

const jASTERIX::CategoryItemInfo& ASTERIXJSONParser::categoryItemInfo() const
{
    return item_info_;
}

const std::vector<std::string>& ASTERIXJSONParser::notAddedJSONKeys() const
{
    return not_added_json_keys_;
}

DBContent& ASTERIXJSONParser::dbContent() const
{

    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    if (!dbcont_man.existsDBContent(db_content_name_))
        throw runtime_error ("ASTERIXJSONParser: dbObject: dbcontbject '" + db_content_name_+ "' does not exist");
    else
        return dbcont_man.dbContent(db_content_name_);
}

void ASTERIXJSONParser::initialize()
{
    logdbg2 << "name " << name_;

    if (!initialized_)
    {
        for (auto& mapping : data_mappings_)
        {
            if (!mapping->active())
            {
                traced_assert(!mapping->mandatory());
                continue;
            }

            mapping->initializeIfRequired();

            if (!list_.hasProperty(mapping->variable().name()))
                list_.addProperty(mapping->variable().name(), mapping->variable().dataType());

            if (!var_list_.hasVariable(mapping->variable()))
                var_list_.add(mapping->variable());
        }

        initialized_ = true;
    }
}

std::shared_ptr<Buffer> ASTERIXJSONParser::getNewBuffer() const
{
    traced_assert(initialized_);
    return std::make_shared<Buffer>(list_, db_content_name_);
}

void ASTERIXJSONParser::appendVariablesToBuffer(Buffer& buffer) const
{
    traced_assert(initialized_);

    for (auto& p_it : list_.properties())
    {
        if (!buffer.properties().hasProperty(p_it.name()))
        {
            buffer.addProperty(p_it.name(), p_it.dataType());
        }
    }
}

bool ASTERIXJSONParser::parseJSON(nlohmann::json& j, Buffer& buffer) const
{
    traced_assert(initialized_);

    size_t row_cnt = buffer.size();

    bool parsed_any = false;

    logdbg2 << "single target report";
    traced_assert(j.is_object());

    parsed_any = parseTargetReport(j, buffer, row_cnt);

    return parsed_any;
}

void ASTERIXJSONParser::createMappingStubs(nlohmann::json& j)
{
    traced_assert(initialized_);


    logdbg2 << "single target report";
    traced_assert(j.is_object());

    createMappingsFromTargetReport(j);

    return;
}

bool ASTERIXJSONParser::parseTargetReport(const nlohmann::json& tr, Buffer& buffer,
                                          size_t row_cnt) const
{
    PropertyDataType data_type;
    std::string current_var_name;

    bool mandatory_missing{false};

    for (const auto& map_it : data_mappings_)
    {
        if (!map_it->active())
        {
            traced_assert(!map_it->mandatory());
            continue;
        }

        // logdbg2 << "setting data mapping key " << data_it.jsonKey();

        try
        {

            data_type = map_it->variable().dataType();
            current_var_name = map_it->variable().name();

            switch (data_type)
            {
            case PropertyDataType::BOOL:
            {
                logdbg2 << "bool " << current_var_name
                       << " format '" << map_it->jsonValueFormat() << "'";
                traced_assert(buffer.has<bool>(current_var_name));
                mandatory_missing =
                        map_it->findAndSetValue(tr, buffer.get<bool>(current_var_name), row_cnt);

                break;
            }
            case PropertyDataType::CHAR:
            {
                logdbg2 << "char " << current_var_name
                       << " format '" << map_it->jsonValueFormat() << "'";
                traced_assert(buffer.has<char>(current_var_name));
                mandatory_missing =
                        map_it->findAndSetValue(tr, buffer.get<char>(current_var_name), row_cnt);

                break;
            }
            case PropertyDataType::UCHAR:
            {
                logdbg2 << "uchar " << current_var_name
                       << " format '" << map_it->jsonValueFormat() << "'";
                traced_assert(buffer.has<unsigned char>(current_var_name));
                mandatory_missing = map_it->findAndSetValue(
                            tr, buffer.get<unsigned char>(current_var_name), row_cnt);

                break;
            }
            case PropertyDataType::INT:
            {
                logdbg2 << "int " << current_var_name
                       << " format '" << map_it->jsonValueFormat() << "'";
                traced_assert(buffer.has<int>(current_var_name));
                mandatory_missing =
                        map_it->findAndSetValue(tr, buffer.get<int>(current_var_name), row_cnt);

                break;
            }
            case PropertyDataType::UINT:
            {
                logdbg2 << "uint " << current_var_name
                       << " format '" << map_it->jsonValueFormat() << "'";
                traced_assert(buffer.has<unsigned int>(current_var_name));
                mandatory_missing =
                        map_it->findAndSetValue(tr, buffer.get<unsigned int>(current_var_name), row_cnt);

                break;
            }
            case PropertyDataType::LONGINT:
            {
                logdbg2 << "long " << current_var_name
                       << " format '" << map_it->jsonValueFormat() << "'";
                traced_assert(buffer.has<long int>(current_var_name));
                mandatory_missing =
                        map_it->findAndSetValue(tr, buffer.get<long int>(current_var_name), row_cnt);

                break;
            }
            case PropertyDataType::ULONGINT:
            {
                logdbg2 << "ulong " << current_var_name
                       << " format '" << map_it->jsonValueFormat() << "'";
                traced_assert(buffer.has<unsigned long>(current_var_name));
                mandatory_missing = map_it->findAndSetValue(
                            tr, buffer.get<unsigned long>(current_var_name), row_cnt);

                break;
            }
            case PropertyDataType::FLOAT:
            {
                logdbg2 << "float " << current_var_name
                       << " format '" << map_it->jsonValueFormat() << "'";
                traced_assert(buffer.has<float>(current_var_name));
                mandatory_missing =
                        map_it->findAndSetValue(tr, buffer.get<float>(current_var_name), row_cnt);

                break;
            }
            case PropertyDataType::DOUBLE:
            {
                logdbg2 << "double " << current_var_name
                       << " format '" << map_it->jsonValueFormat() << "'";
                traced_assert(buffer.has<double>(current_var_name));
                mandatory_missing =
                        map_it->findAndSetValue(tr, buffer.get<double>(current_var_name), row_cnt);

                break;
            }
            case PropertyDataType::STRING:
            {
                logdbg2 << "string " << current_var_name
                       << " format '" << map_it->jsonValueFormat() << "'";
                traced_assert(buffer.has<std::string>(current_var_name));

                mandatory_missing =
                        map_it->findAndSetValue(tr, buffer.get<std::string>(current_var_name), row_cnt);

                break;
            }
            case PropertyDataType::JSON: // only to be used for lists
            {
                logdbg2 << "json " << current_var_name
                       << " format '" << map_it->jsonValueFormat() << "'";
                traced_assert(buffer.has<json>(current_var_name));
//                mandatory_missing =
//                        map_it->findAndSetValue(tr, buffer.get<json>(current_var_name), row_cnt);

                 map_it->findAndSetValues(tr, buffer.get<json>(current_var_name), row_cnt);

                break;
            }
            case PropertyDataType::TIMESTAMP: // not possible for timestamp
            default:
                logerr << "impossible for property type "
                         << Property::asString(data_type);
                throw std::runtime_error(
                            "JsonMapping: parseTargetReport: impossible property type " +
                            Property::asString(data_type));
            }

        }
        catch (exception& e)
        {
            logerr << "caught exception '" << e.what() << "' in \n'"
                     << tr.dump(4) << "' mapping " << map_it->jsonKey();
            throw e;
        }

        if (mandatory_missing)
        {
            // TODO make configurable
            logdbg2 << "'" << name_ << "': mandatory variable '"
                     << current_var_name << "' missing in: \n"
                     << tr.dump(4);
            break;
        }
    }

    if (mandatory_missing)
    {
        // cleanup
        if (buffer.size() > row_cnt)
            buffer.cutToSize(row_cnt);
    }

    return !mandatory_missing;
}

void ASTERIXJSONParser::createMappingsFromTargetReport(const nlohmann::json& tr)
{
    checkIfKeysExistsInMappings("", tr);
}

void ASTERIXJSONParser::checkIfKeysExistsInMappings(const std::string& location,
                                                    const nlohmann::json& j, bool is_in_array)
{
    if (j.is_array())  // do map arrays
    {
        if (!j.size())  // do not map if empty
            return;

        bool j_array_contains_only_primitives = true;  // only

        for (auto& j_it : j.get<json::array_t>())  // iterate over array
        {
            if (j_it.is_object())  // only parse sub-objects
            {
                j_array_contains_only_primitives = false;
                checkIfKeysExistsInMappings(location, j_it, true);
            }
            else if (j_it.is_array())
                j_array_contains_only_primitives = false;
        }

        if (!j_array_contains_only_primitives)
            return;  // if objects inside, only parse objects
    }

    if (j.is_object())
    {
        for (auto& j_it : j.get<json::object_t>())
        {
            if (location.size())
                checkIfKeysExistsInMappings(location + "." + j_it.first, j_it.second, is_in_array);
            else
                checkIfKeysExistsInMappings(j_it.first, j_it.second, is_in_array);
        }
        return;
    }

    // loginf << "checking key '" << location << "'";

    bool found = false;

    for (auto& map_it : data_mappings_)
    {
        if (map_it->jsonKey() == location)
        {
            found = true;

            if (!map_it->comment().size())
            {
                std::stringstream ss;

                ss << "Type " << j.type_name() << ", value " << j.dump();
                map_it->comment(ss.str());
            }
            break;
        }
    }

    if (!found)
    {
        loginf << "creating new mapping for dbcont "
                 << db_content_name_ << "'" << location << "' type " << j.type_name() << " value "
                 << j.dump() << " in array " << is_in_array;

        auto new_cfg = Configuration::create("JSONDataMapping");
        new_cfg->addParameter<std::string>("json_key", location);
        new_cfg->addParameter<std::string>("dbcontent_name", db_content_name_);

        if (is_in_array)
            new_cfg->addParameter<bool>("in_array", true);

        std::stringstream ss;
        ss << "Type " << j.type_name() << ", value " << j.dump();
        new_cfg->addParameter<std::string>("comment", ss.str());

        Configurable::generateSubConfigurableFromConfig(std::move(new_cfg));
    }
}

bool ASTERIXJSONParser::hasMapping(unsigned int index) const
{
    return index < data_mappings_.size();
}
void ASTERIXJSONParser::removeMapping(unsigned int index)
{
    traced_assert(hasMapping(index));

    std::unique_ptr<JSONDataMapping>& mapping = data_mappings_.at(index);

    loginf << "index " << index << " key " << mapping->jsonKey()
           << " instance " << mapping->instanceId();

    logdbg2 << "size " << data_mappings_.size();

    if (mapping->active() && mapping->initialized() && mapping->hasVariable())
    {
        if (list_.hasProperty(mapping->dbcontVariableName()))
            list_.removeProperty(mapping->variable().name());

        if (var_list_.hasVariable(mapping->variable()))
            var_list_.removeVariable(mapping->variable());
    }

    logdbg2 << "removing";
    data_mappings_.erase(data_mappings_.begin() + index);

    logdbg2 << "size " << data_mappings_.size();
}

const dbContent::VariableSet& ASTERIXJSONParser::variableList() const { return var_list_; }

ASTERIXJSONParserWidget* ASTERIXJSONParser::createWidget()
{
    if (mapping_checks_dirty_)
        doMappingChecks();

    return new ASTERIXJSONParserWidget(*this);
}

std::string ASTERIXJSONParser::dbContentName() const { return db_content_name_; }

void ASTERIXJSONParser::setMappingActive(JSONDataMapping& mapping, bool active)
{
    if (!mapping.initialized())
        mapping.initializeIfRequired();

    if (active)
    {
        list_.addProperty(mapping.variable().name(), mapping.variable().dataType());
        var_list_.add(mapping.variable());
    }
    else if (mapping.hasVariable())  // remove if was added
    {
        if (list_.hasProperty(mapping.variable().name()))
            list_.removeProperty(mapping.variable().name());

        if (var_list_.hasVariable(mapping.variable()))
            var_list_.removeVariable(mapping.variable());
    }

    mapping.active(active);
}

//void ASTERIXJSONParser::updateMappings()
//{
////    if (widget_)
////        widget_->updateMappingsGrid();
//}

std::string ASTERIXJSONParser::name() const { return name_; }

void ASTERIXJSONParser::name(const std::string& name) { name_ = name; }

unsigned int ASTERIXJSONParser::category() const
{
    return category_;
}

// item stuff

int ASTERIXJSONParser::rowCount(const QModelIndex& parent) const
{
    traced_assert(!mapping_checks_dirty_);

    return totalEntrySize();
}

int ASTERIXJSONParser::columnCount(const QModelIndex& parent) const
{
    traced_assert(!mapping_checks_dirty_);

    return table_columns_.size();
}

QVariant ASTERIXJSONParser::data(const QModelIndex& index, int role) const
{
    traced_assert(!mapping_checks_dirty_);

    if (!index.isValid())
        return QVariant();

    traced_assert(index.row() >= 0);
    unsigned int row = index.row();

    traced_assert(row < totalEntrySize());

    traced_assert(index.column() < table_columns_.size());
    std::string col_name = table_columns_.at(index.column()).toStdString();

    ASTERIXJSONParser::EntryType entry_type = entryType(row);

    switch (role)
    {
    case Qt::CheckStateRole:
    {
        if (index.column() == 0 && entryType(row) == EntryType::ExistingMapping)  // selected special case
        {
            if (mapping(row).active())
                return Qt::Checked;
            else
                return Qt::Unchecked;
        }
        else
            return QVariant();
    }
    case Qt::DisplayRole:
        //case Qt::EditRole:
    {
        logdbg2 << "display role: row " << index.row() << " col " << index.column();

        if (entry_type == ASTERIXJSONParser::EntryType::ExistingMapping)
        {
            const JSONDataMapping& current_mapping = mapping(row);

            logdbg2 << "got json key " << current_mapping.jsonKey();

            if (col_name == "Active")
                return QVariant();
            else if (col_name == "JSON Key")
                return current_mapping.jsonKey().c_str();
            else if (col_name == "DBContent Variable")
                return current_mapping.dbcontVariableName().c_str();
            else
                return QVariant();
        }
        else if (entry_type == ASTERIXJSONParser::EntryType::UnmappedJSONKey)
        {
            if (col_name == "JSON Key")
                return unmappedJSONKey(row).c_str();
            else
                return QVariant();
        }
        else if (entry_type == ASTERIXJSONParser::EntryType::UnmappedDBContentVariable)
        {
            if (col_name == "DBContent Variable")
                return unmappedDBContentVariable(row).c_str();
            else
                return QVariant();
        }
        else
            return QVariant();
    }
    case Qt::DecorationRole:
    {
        if (entry_type == ASTERIXJSONParser::EntryType::ExistingMapping)
        {
            const std::unique_ptr<JSONDataMapping>& mapping = data_mappings_.at(index.row());

            logdbg2 << "got json key " << mapping->jsonKey();

            if (col_name == "JSON Key")
            {
                if (not_existing_json_keys_.count(mapping->jsonKey()))
                    return hint_icon_;
                else
                    return QVariant();
            }
            else
                return QVariant();
        }
        else if (entry_type == ASTERIXJSONParser::EntryType::UnmappedJSONKey)
        {
            if (col_name == "JSON Key")
                return todo_icon_;
            else
                return QVariant();
        }
        else if (entry_type == ASTERIXJSONParser::EntryType::UnmappedDBContentVariable)
        {

            if (col_name == "DBContent Variable")
                return todo_icon_;
            else
                return QVariant();
        }
        else
            return QVariant();
    }
    default:
    {
        return QVariant();
    }
    }
}

bool ASTERIXJSONParser::setData(const QModelIndex& index, const QVariant& value, int role)
{
    if (role == Qt::CheckStateRole && index.column() == 0)
    {
        unsigned int row = index.row();

        traced_assert(row < totalEntrySize());
        traced_assert(entryType(row) == EntryType::ExistingMapping);
        mapping(row).active(!mapping(row).active());

        emit rowContentChangedSignal(row);
    }

    return true;
}

QVariant ASTERIXJSONParser::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
    {
        traced_assert(section < table_columns_.size());
        return table_columns_.at(section);
    }

    return QVariant();
}

QModelIndex ASTERIXJSONParser::index(int row, int column, const QModelIndex& parent) const
{
    return createIndex(row, column);
}

QModelIndex ASTERIXJSONParser::parent(const QModelIndex& index) const
{
    return QModelIndex();
}

Qt::ItemFlags ASTERIXJSONParser::flags(const QModelIndex& index) const
{
    if (!expert_mode_init_)
    {
        expert_mode_ = COMPASS::instance().expertMode();
        expert_mode_init_ = true;
    }

    if (!index.isValid())
        return Qt::ItemIsEnabled;

    traced_assert(index.column() < table_columns_.size());

    unsigned int row = index.row();
    traced_assert(row < totalEntrySize());

    if (index.column() == 0 && entryType(row) == EntryType::ExistingMapping)
    {
        Qt::ItemFlags flags;

        if (mapping(row).canBeActive())
        {
            flags |= Qt::ItemIsUserCheckable;

            if (expert_mode_)
            {
                flags |= Qt::ItemIsEnabled;
                flags |= Qt::ItemIsEditable;
            }
        }

        return flags;
    }
    else
        return QAbstractItemModel::flags(index);
}

bool ASTERIXJSONParser::mappingChecksDirty() const
{
    return mapping_checks_dirty_;
}

void ASTERIXJSONParser::mappingChecksDirty(bool mapping_checks_dirty)
{
    mapping_checks_dirty_ = mapping_checks_dirty;
}
