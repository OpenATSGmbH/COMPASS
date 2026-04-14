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

#include <QMenu>
#include <QPushButton>
#include <QTreeWidgetItem>

class DataSourceManager;

class QTreeWidget;

namespace dbContent
{
    class DataSourceBase;
    class DBDataSource;
}

class DataSourcesWidgetBase;

/**
 * Base item to be displayed in a data source widget.
 */
class DataSourcesWidgetItemBase : public QTreeWidgetItem
{
public:
    enum class Type
    {
        DataSourceType = 0,
        DataSource,
        DataSourceCount
    };

    DataSourcesWidgetItemBase(DataSourcesWidgetBase* widget,
                              DataSourcesWidgetItemBase* parent,
                              Type type);
    virtual ~DataSourcesWidgetItemBase() = default;

    bool isInit() const { return is_init_; }
    Type type() const { return type_; }

    bool updateContent();

protected:
    virtual void init_impl() = 0;
    virtual void updateContentChanges_impl() = 0;
    virtual void updateContent_impl() = 0;

    void notifyChanges();
    void setItemWidget(int column, QWidget* w);

    DataSourcesWidgetBase*     widget_ = nullptr;
    DataSourcesWidgetItemBase* parent_ = nullptr;

    bool is_init_     = false;
    bool has_changes_ = false;
    Type type_;
};

/**
 */
class DataSourceTypeItemBase : public DataSourcesWidgetItemBase
{
public:
    DataSourceTypeItemBase(DataSourcesWidgetBase* widget,
                           DataSourcesWidgetItemBase* parent);
    virtual ~DataSourceTypeItemBase() = default;

    void setDSType(const std::string& ds_type);
    const std::string& dsType() const { return ds_type_; }

protected:
    virtual void init_impl() override;
    virtual void updateContentChanges_impl() override;
    virtual void updateContent_impl() override;

private:
    std::string ds_type_;
};

/**
 */
class DataSourceItemBase : public DataSourcesWidgetItemBase
{
public:
    DataSourceItemBase(DataSourcesWidgetBase* widget,
                       DataSourcesWidgetItemBase* parent);

    virtual ~DataSourceItemBase() = default;

    void setDSID(unsigned int ds_id);
    unsigned int dsID() const { return ds_id_; }
    const dbContent::DataSourceBase* dataSource() const { return ds_; }

protected:
    virtual void init_impl() override;
    virtual void updateContentChanges_impl() override;
    virtual void updateContent_impl() override;

private:
    unsigned int                     ds_id_;
    const dbContent::DataSourceBase* ds_         = nullptr;
};

/**
 */
class DataSourceCountItemBase : public DataSourcesWidgetItemBase
{
public:
    DataSourceCountItemBase(DataSourcesWidgetBase* widget,
                            DataSourcesWidgetItemBase* parent);

    virtual ~DataSourceCountItemBase() = default;

    void setup(unsigned int ds_id,
               const std::string& dbc_name);
    unsigned int dsID() const { return ds_id_; }
    const std::string& dbContentName() const { return dbc_name_; }

protected:
    virtual void init_impl() override;
    virtual void updateContentChanges_impl() override;
    virtual void updateContent_impl() override;

private:
    unsigned int                   ds_id_;
    std::string                    dbc_name_;
    const dbContent::DBDataSource* ds_ = nullptr;
};

class QHBoxLayout;

/**
 */
class DataSourcesWidgetBase : public QWidget
{
    Q_OBJECT

public slots:
    // Add this slot
    void onItemSelectionChanged();

signals:
    void dataSourceSelectedSignal(unsigned int ds_id);

public:
    enum class Source
    {
        Database = 0,
        Config,
        All
    };

    DataSourcesWidgetBase(DataSourceManager& ds_man,
                          Source source,
                          bool can_show_counts,
                          bool init_ui);
    virtual ~DataSourcesWidgetBase();

    virtual void updateContent(bool recreate_required = false);
    virtual void addActionsToConfigMenu(QMenu* menu) {}

    DataSourceManager& dsManager() { return ds_man_; }
    Source dsSource() const { return source_; }

    void showColumn(int col, bool show);

protected:
    void init();
    std::vector<const dbContent::DataSourceBase*> dataSources(bool filter, std::string* ds_type = nullptr) const;
    const dbContent::DataSourceBase* dataSource(unsigned int ds_id) const;
    dbContent::DataSourceBase* dataSource(unsigned int ds_id);

    virtual QStringList getCustomColumnHeaders() const { return QStringList(); }

    virtual bool showsCounts() const;

    virtual bool showDSType(const std::string& ds_type_name) const { return true; }
    virtual bool showDS(unsigned int ds_id) const { return true; }

    virtual DataSourceTypeItemBase* createDSTypeItem(DataSourcesWidgetItemBase* parent = nullptr);
    virtual DataSourceItemBase* createDSItem(DataSourcesWidgetItemBase* parent = nullptr);
    virtual DataSourceCountItemBase* createDSCountItem(DataSourcesWidgetItemBase* parent = nullptr);

    virtual void dsTypeItemChanged(DataSourceTypeItemBase* item, int column) {}
    virtual void dsItemChanged(DataSourceItemBase* item, int column) {}
    virtual void dsCountItemChanged(DataSourceCountItemBase* item, int column) {}

    DataSourceManager& ds_man_;

private:
    friend class DataSourcesWidgetItemBase;
    friend class DataSourceTypeItemBase;
    friend class DataSourceItemBase;

    QStringList getColumnHeaders() const;
    void createUI();

    void clear();
    int generateContent(bool force_rebuild);
    int generateDataSourceType(DataSourceTypeItemBase* item,
                               const std::string& ds_type_name);
    int generateDataSource(DataSourceItemBase* item,
                           DataSourcesWidgetItemBase* parent_item, 
                           const dbContent::DataSourceBase& data_source);
    int generateDataSourceCount(DataSourceCountItemBase* item,
                                DataSourcesWidgetItemBase* parent_item,
                                const dbContent::DataSourceBase& data_source,
                                const std::string& dbc_name);

    void itemChanged(QTreeWidgetItem *item, int column);

    void updateAllContent();

    Source source_;
    bool   can_show_counts_;

    bool init_ = false;

    QHBoxLayout* top_layout_ {nullptr};
    QTreeWidget* tree_widget_ {nullptr};
};

/**
 */
class DataSourcesListDBWidget : public DataSourcesWidgetBase
{
public:
    DataSourcesListDBWidget(DataSourceManager& ds_man,
                            bool can_show_counts)
    :   DataSourcesWidgetBase(ds_man, Source::Database, can_show_counts, true) {}
    virtual ~DataSourcesListDBWidget() = default;
};

/**
 */
class DataSourcesListConfigWidget : public DataSourcesWidgetBase
{
public:
    DataSourcesListConfigWidget(DataSourceManager& ds_man)
    :   DataSourcesWidgetBase(ds_man, Source::Config, false, true) {}
    virtual ~DataSourcesListConfigWidget() = default;
};
