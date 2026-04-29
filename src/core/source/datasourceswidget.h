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


#include <QColor>
#include <QMenu>
#include <QPushButton>
#include <QTreeWidgetItem>

#include <functional>
#include <memory>
#include <string>

namespace context { class DBContextManager; class DataSource; }

class DBContentDeleteDBJob;
class QMessageBox;

class QTreeWidget;

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

    const context::DataSource* ds_ = nullptr;

    QString last_stylesheet_;
    QColor  last_palette_color_;
    bool    last_auto_fill_bg_   = false;
    bool    last_hidden_set_     = false;
    bool    last_hidden_         = false;
    bool    last_disabled_set_   = false;
    bool    last_disabled_       = false;
    bool    last_checked_set_    = false;
    bool    last_checked_        = false;
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

    /// Color this item currently represents. For leaves (DataSourceCountItem)
    /// this is derived from the active ColorProvider::Mode and the data
    /// context palettes. For groups (DataSourceItem, DataSourceTypeItem) it is
    /// the common color of the descendants, or invalid if they disagree — so
    /// colors propagate up only when every leaf below agrees. DataSourceLine
    /// mode carries no tree-icon color (the color is on the line buttons).
    virtual QColor effectiveColor() const { return QColor(); }

protected:
    void setItemWidget(int column, QWidget* w);

    /// Helpers that suppress no-op QTreeWidgetItem mutations. setText / setIcon
    /// / setCheckState / setFlags all invalidate the cell, force re-layout,
    /// and re-run text shaping (HarfBuzz). Avoid the work when the value is
    /// unchanged.
    void setTextIfChanged(int column, const QString& text);
    void setColorIconIfChanged(int column, const QColor& color);
    void setCheckStateIfChanged(int column, Qt::CheckState state);
    void setFlagsIfChanged(Qt::ItemFlags new_flags);

    DataSourcesWidget*     widget_ = nullptr;
    DataSourcesWidgetItem* parent_ = nullptr;

    bool is_init_ = false;
    Type type_;

    QColor last_color_;
    bool   last_color_set_ = false;
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
    QColor effectiveColor() const override;

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
    QColor effectiveColor() const override;

    unsigned int dsID() const { return ds_id_; }
    const context::DataSource* dataSource() const { return ds_; }

private:
    QWidget* createLinesWidget();

    unsigned int                   ds_id_;
    const context::DataSource* ds_         = nullptr;
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
    QColor effectiveColor() const override;

    unsigned int dsID() const { return ds_id_; }
    const std::string& dbContentName() const { return dbc_name_; }

private:
    QWidget* createLinesWidget();

    unsigned int                   ds_id_;
    std::string                    dbc_name_;
    const context::DataSource* ds_ = nullptr;
};

class QHBoxLayout;

/**
 */
class DataSourcesWidget : public QWidget
{
    Q_OBJECT

public slots:
    // Add this slot
    void onItemSelectionChanged();

    /// Context-menu handler — dispatches the menu based on the item under
    /// `pos` (DSType row, DataSource row, DBContent count row, or blank
    /// background).
    void showContextMenuSlot(const QPoint& pos);

signals:
    void dataSourceSelectedSignal(unsigned int ds_id);

public:
    DataSourcesWidget(bool can_show_counts, context::DBContextManager& ctx_man);
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

    context::DBContextManager& ctxManager() { return ctx_man_; }

    void addActionsToConfigMenu(QMenu* menu);

    /// Turn row selection off (no highlight, no current-item marker, no
    /// focus). Use in embeddings that don't listen to
    /// `dataSourceSelectedSignal`.
    void disableSelection();

    static const int LineButtonSize;

protected:
    bool can_show_counts_;
    context::DBContextManager& ctx_man_;

    QHBoxLayout* top_layout_ {nullptr};
    QTreeWidget* tree_widget_ {nullptr};

    void selectAllDSTypes();
    void deselectAllDSTypes();
    void selectAllDataSources();
    void deselectAllDataSources();
    void selectDSTypeSpecificDataSources();
    void deselectDSTypeSpecificDataSources();
    void deselectAllLines();
    void selectSpecificLines();
    void toogleShowCounts();

    void deleteDataSlot();
    void deleteJobDoneSlot();

    // Context-menu helpers
    void setAllCheckboxes(bool select);
    void deselectOtherDSTypes(const std::string& keep);
    void setAllChildrenOfDSType(const std::string& ds_type, bool select);
    void deselectOtherDataSources(unsigned int keep_ds_id);
    void expandSubtree(QTreeWidgetItem* item, bool expand);

    // Delete dialog with optional preselection applied before exec().
    void runDeleteDialog(std::function<void(class DeleteDataDialog&)> preselect);
    void deleteForDSType(const std::string& ds_type);
    void deleteForDataSource(unsigned int ds_id);

private:
    friend class DataSourcesWidgetItem;
    friend class DataSourceLineButton;

    void createUI();

    void clear();
    int generateContent(bool force_rebuild);
    int generateDataSourceType(DataSourceTypeItem* item,
                               const std::string& ds_type_name);
    int generateDataSource(DataSourceItem* item,
                           DataSourcesWidgetItem* parent_item,
                           const context::DataSource& data_source);
    int generateDataSourceCount(DataSourceCountItem* item,
                                DataSourcesWidgetItem* parent_item,
                                const context::DataSource& data_source,
                                const std::string& dbc_name);

    void itemChanged(QTreeWidgetItem *item, int column);
    void lineChanged(unsigned int ds_id, unsigned int ds_line, bool use);

    void updateAllContent();

    std::shared_ptr<DBContentDeleteDBJob> delete_job_;
    QMessageBox* delete_wait_dialog_{nullptr};
};
