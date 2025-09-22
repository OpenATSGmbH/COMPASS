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
    class DBDataSource;
}

class DataSourcesWidget;

/**
 */
class DataSourceLineButton : public QPushButton
{
public:
    DataSourceLineButton(DataSourcesWidget* widget,
                         unsigned int line_id,
                         unsigned int button_size_px);

    bool init(unsigned int ds_id);
    void updateContent();

    unsigned int dsID() const { return ds_id_; }
    unsigned int lineID() const { return line_id_; }

private:
    DataSourcesWidget* widget_ = nullptr;
    unsigned int       ds_id_;
    unsigned int       line_id_;
    std::string        line_str_;
    bool               is_init_ = false;

    const dbContent::DBDataSource* ds_ = nullptr;
};

/**
 */
class DataSourcesWidgetItem : public QTreeWidgetItem
{
public:
    enum class Type
    {
        DataSourceType = 0,
        DataSource,
        DataSourceCount
    };

    DataSourcesWidgetItem(DataSourcesWidget* widget,
                          DataSourcesWidgetItem* parent,
                          Type type);
    virtual ~DataSourcesWidgetItem() = default;

    bool isInit() const { return is_init_; }
    Type type() const { return type_; }

    virtual void updateContent() = 0;

protected:
    void setItemWidget(int column, QWidget* w);

    DataSourcesWidget*     widget_ = nullptr;
    DataSourcesWidgetItem* parent_ = nullptr;

    bool is_init_ = false;
    Type type_;
};

/**
 */
class DataSourceTypeItem : public DataSourcesWidgetItem
{
public:
    DataSourceTypeItem(DataSourcesWidget* widget,
                       DataSourcesWidgetItem* parent);
    virtual ~DataSourceTypeItem() = default;

    bool init(const std::string& ds_type);
    void updateContent() override final;

    const std::string& dsType() const { return ds_type_; }

private:
    std::string ds_type_;
    
};

/**
 */
class DataSourceItem : public DataSourcesWidgetItem
{
public:
    DataSourceItem(DataSourcesWidget* widget,
                   DataSourcesWidgetItem* parent);

    virtual ~DataSourceItem() = default;

    bool init(unsigned int ds_id);
    void updateContent() override final;

    unsigned int dsID() const { return ds_id_; }
    const dbContent::DBDataSource* dataSource() const { return ds_; }

private:
    QWidget* createLinesWidget();

    unsigned int                   ds_id_;
    const dbContent::DBDataSource* ds_         = nullptr;
    bool                           has_widget_ = false;

    std::vector<DataSourceLineButton*> line_buttons_;
};

/**
 */
class DataSourceCountItem : public DataSourcesWidgetItem
{
public:
    DataSourceCountItem(DataSourcesWidget* widget,
                        DataSourcesWidgetItem* parent);

    virtual ~DataSourceCountItem() = default;

    bool init(unsigned int ds_id,
              const std::string& dbc_name);
    void updateContent() override final;

    unsigned int dsID() const { return ds_id_; }
    const std::string& dbContentName() const { return dbc_name_; }

private:
    QWidget* createLinesWidget();

    unsigned int                   ds_id_;
    std::string                    dbc_name_;
    const dbContent::DBDataSource* ds_ = nullptr;
};

/**
 */
class DataSourcesWidget : public QWidget
{
    Q_OBJECT

public:
    DataSourcesWidget(DataSourceManager& ds_man);
    virtual ~DataSourcesWidget();

    void updateContent(bool recreate_required = false);

    virtual void setUseDSType(const std::string& ds_type_name, bool use);
    virtual bool getUseDSType(const std::string& ds_type_name) const;
    virtual void setUseDS(unsigned int ds_id, bool use);
    virtual bool getUseDS(unsigned int ds_id) const;
    virtual void setUseDSLine(unsigned int ds_id, unsigned int ds_line, bool use);
    virtual bool getUseDSLine(unsigned int ds_id, unsigned int ds_line) const;
    virtual void setShowCounts(bool show) const;
    virtual bool getShowCounts() const;

    DataSourceManager& dsManager() { return ds_man_; }

    void addActionsToConfigMenu(QMenu* menu);

    static const int LineButtonSize;

private:
    friend class DataSourcesWidgetItem;
    friend class DataSourceLineButton;

    DataSourceManager& ds_man_;

    QTreeWidget* tree_widget_ = nullptr;

    void createUI();

    void clear();
    int generateContent(bool force_rebuild);
    int generateDataSourceType(DataSourceTypeItem* item,
                               const std::string& ds_type_name);
    int generateDataSource(DataSourceItem* item,
                           DataSourcesWidgetItem* parent_item, 
                           const dbContent::DBDataSource& data_source);
    int generateDataSourceCount(DataSourceCountItem* item,
                                DataSourcesWidgetItem* parent_item,
                                const dbContent::DBDataSource& data_source,
                                const std::string& dbc_name);

    void itemChanged(QTreeWidgetItem *item, int column);
    void lineChanged(unsigned int ds_id, unsigned int ds_line, bool use);

    void updateAllContent();

    void selectAllDSTypes();
    void deselectAllDSTypes();
    void selectAllDataSources();
    void deselectAllDataSources();
    void selectDSTypeSpecificDataSources();
    void deselectDSTypeSpecificDataSources();
    void deselectAllLines();
    void selectSpecificLines();
    void toogleShowCounts();
};
