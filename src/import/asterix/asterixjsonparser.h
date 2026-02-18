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
#include "configurable.h"
#include "dbcontent/variable/variableset.h"
#include "jsondatamapping.h"
#include "asterixjsonparserwidget.h"
#include "propertylist.h"

#include <jasterix/iteminfo.h>

#include <QAbstractItemModel>
#include <QIcon>

#include <memory>
#include <string>

class DBContent;
class Buffer;
class ASTERIXImportTask;

class ASTERIXJSONParser : public QAbstractItemModel, public Configurable
{
    Q_OBJECT

signals:
    void rowContentChangedSignal(unsigned int row_index); // emitted when something in the index was changed
    void modelRowChanged(unsigned int row_index);

private:
    using MappingIterator = std::vector<std::unique_ptr<JSONDataMapping>>::iterator;

public:
    enum EntryType {
        ExistingMapping=0, UnmappedJSONKey, UnmappedDBContentVariable
    };

    ASTERIXJSONParser(const std::string& class_id, const std::string& instance_id,
                      Configurable* parent, ASTERIXImportTask& task);

    DBContent& dbContent() const;

    MappingIterator begin() { return data_mappings_.begin(); }
    MappingIterator end() { return data_mappings_.end(); }
    bool hasMapping(unsigned int index) const;
    void removeMapping(unsigned int index);

    // returns true on successful parse
    bool parseJSON(nlohmann::json& j, Buffer& buffer) const;
    void createMappingStubs(nlohmann::json& j);

    const dbContent::VariableSet& variableList() const;

    bool initialized() const { return initialized_; }
    void initialize();

    std::shared_ptr<Buffer> getNewBuffer() const;
    void appendVariablesToBuffer(Buffer& buffer) const;

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id) override;

    ASTERIXJSONParserWidget* createWidget();

    std::string dbContentName() const;

    void setMappingActive(JSONDataMapping& mapping, bool active);

    std::string name() const;
    void name(const std::string& name);

    unsigned int category() const;

    // item stuff

    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    virtual bool setData(const QModelIndex& index, const QVariant& value, int role) override;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;
    QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;
    QModelIndex parent(const QModelIndex& index) const override;

    Qt::ItemFlags flags(const QModelIndex &index) const override;

    bool mappingChecksDirty() const;
    void mappingChecksDirty(bool mapping_checks_dirty);
    void doMappingChecks();  // to be called when total size is changed

    unsigned int totalEntrySize () const;
    bool existsJSONKeyInCATInfo(const std::string& key);
    bool hasJSONKeyMapped (const std::string& key);
    bool hasDBContentVariableMapped (const std::string& var_name);

    bool hasJSONKeyInMapping (const std::string& key);
    unsigned int indexOfJSONKeyInMapping (const std::string& key);

    //void updateToChangedIndex (unsigned int index); // to be called when existing row is changed
    void selectMapping (unsigned int index);
    void selectUnmappedDBContentVariable (const std::string& name);

    // index is not model index but directly in data_mappings_ + not_added_json_keys_ + not_added_dbcont_variables_
    EntryType entryType (unsigned int index) const;
    JSONDataMapping& mapping (unsigned int index);
    const JSONDataMapping& mapping (unsigned int index) const;
    const std::string& unmappedJSONKey (unsigned int index) const;
    const std::string& unmappedDBContentVariable (unsigned int index) const;

    const jASTERIX::CategoryItemInfo& categoryItemInfo() const;

    const std::vector<std::string>& notAddedJSONKeys() const;

private:
    ASTERIXImportTask& task_;
    std::string name_;
    unsigned int category_;

    std::string db_content_name_;

    mutable bool expert_mode_init_ {false}; // whether expert_mode_ was initialized
    mutable bool expert_mode_ {false}; // COMPASS expert mode

    jASTERIX::CategoryItemInfo item_info_;

    dbContent::VariableSet var_list_;

    bool initialized_{false};

    PropertyList list_;

    std::vector<std::unique_ptr<JSONDataMapping>> data_mappings_;
    bool mapping_checks_dirty_ {true};
    std::set<std::string> not_existing_json_keys_; // mapped keys not existing in cat info
    std::vector<std::string> not_added_json_keys_; // keys existing in cat info not in mappings
    std::vector<std::string> not_added_dbcont_variables_; // existing dbcontvars not in mappings

    QStringList table_columns_ {"Active", "JSON Key", "DBContent Variable"};

    QIcon todo_icon_;
    QIcon unknown_icon_;
    QIcon hint_icon_;

    // returns true on successful parse
    bool parseTargetReport(const nlohmann::json& tr, Buffer& buffer, size_t row_cnt) const;
    void createMappingsFromTargetReport(const nlohmann::json& tr);

    void checkIfKeysExistsInMappings(const std::string& location, const nlohmann::json& tr,
                                     bool is_in_array = false);

protected:
    virtual void checkSubConfigurables() {}

};

