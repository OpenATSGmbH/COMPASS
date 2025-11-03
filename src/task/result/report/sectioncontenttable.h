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

#include "task/result/report/sectioncontent.h"
#include "task/result/report/reportdefs.h"

#include "json_fwd.hpp"

#include <QVariant>
#include <QAbstractItemModel>
#include <QSortFilterProxyModel>
#include <QBrush>
#include <QTimer>
#include <QWidget>
#include <QVariant>

#include <vector>
#include <functional>

#include <boost/optional.hpp>

class TaskManager;
class Section;

class ViewableDataConfig;

class PopupMenu;

namespace Utils
{
    class StringTable;
}

class QPushButton;
class QTableView;
class QMenu;
class QToolBar;

namespace ResultReport
{

/**
 */
class TableQSortFilterProxyModel : public QSortFilterProxyModel
{
    Q_OBJECT

public:

    TableQSortFilterProxyModel(QObject* parent=nullptr)
        : QSortFilterProxyModel(parent) {}

    bool showUnused() const
    {
        return show_unused_;
    }

    void showUnused(bool value)
    {
        //emit layoutAboutToBeChanged();

        show_unused_ = value;

        //emit layoutChanged();
    }

protected:
    bool show_unused_ {true};

    bool filterAcceptsRow(int source_row, const QModelIndex& source_parent) const
    {
        //return sourceModel()->row(source_row)[0].checkState() == Qt::Checked;

        QModelIndex index = sourceModel()->index(source_row, 0, source_parent);

        bool accepted = show_unused_ ? true : (sourceModel()->flags(index) & Qt::ItemIsEnabled) != 0;

        //loginf << "row " << index.row() << " col " << index.column() << " => " << accepted;

        return accepted;
    }
};

class SectionContentTableWidget;

/**
 */
class SectionContentTable : public SectionContent
{
public:
    enum ColumnFlag
    {
        ColumnHidden      = 1 << 0,
        ColumnNotExported = 1 << 1
    };

    enum SpecialRole
    {
        CellStyleRole = Qt::UserRole + 1
    };

    typedef std::map<std::pair<int,int>, unsigned int> CellStyles;
    typedef std::map<std::string, TableColumnGroup>    ColumnGroups;
    
    SectionContentTable(unsigned int id,
                        const std::string& name, 
                        unsigned int num_columns,
                        const std::vector<std::string>& headings, 
                        Section* parent_section, 
                        bool sortable=true, 
                        unsigned int sort_column=0, 
                        Qt::SortOrder sort_order=Qt::AscendingOrder);
    SectionContentTable(Section* parent_section);
    virtual ~SectionContentTable();

    virtual std::string resourceExtension() const override;

    virtual void addContentUI(QVBoxLayout* layout, 
                              bool force_ui_reset) override;

    void enableTooltips();

    void addRow (const nlohmann::json::array_t& row,
                 const SectionContentViewable& viewable = SectionContentViewable(),
                 const std::string& section_link = "",
                 const std::string& section_figure = "", // in section_link section
                 const QVariant& viewable_index = QVariant(),
                 unsigned int row_style = 0);

    const nlohmann::json& getData(int row, int column) const;
    const nlohmann::json& getData(int row, const std::string& col_name) const;

    bool hasColumn(const std::string& col_name) const;
    int columnIndex(const std::string& col_name) const;

    void setColumnStyle(int column, unsigned int style);
    void setCellStyle(int row, int column, unsigned int style);

    unsigned int cellStyle(int row, int column) const;

    void setMaxRowCount(const boost::optional<int>& max_row_count);
    const boost::optional<int>& maxRowCount() const;

    size_t numRows() const;
    size_t numColumns() const;
    const std::vector<std::string>& headings() const;

    const ColumnGroups& columnGroups() const { return column_groups_; }
    void setColumnGroup(const std::string& name, 
                        const std::vector<int>& columns,
                        bool enabled = true);
    void enableColumnGroup(const std::string& name,
                           bool ok);
    bool hasColumnGroup(const std::string& name) const;
    bool columnGroupEnabled(const std::string& name) const;
    bool columnVisible(int column) const;
    bool columnsHidden() const;

    bool hasReference (unsigned int row) const;
    std::string reference (unsigned int row) const;

    bool showUnused() const;
    void showUnused(bool value);

    void registerCallBack(const std::string& name, const std::function<void()>& func);

    QVariant data(const QModelIndex& index, int role) const;
    QVariant data(int row, int col, int role) const;
    std::pair<Qt::ItemFlags, bool> flags(const QModelIndex &index) const;
    
    bool clicked(unsigned int row);
    void doubleClicked(unsigned int row);
    void customContextMenu(unsigned int row, const QPoint& pos);
    void addActionsToMenu(QMenu* menu);

    nlohmann::json jsonConfig() const override final;
    bool configure(const nlohmann::json& j) override final;

    nlohmann::json exportContent(unsigned int row, 
                                 unsigned int col,
                                 ReportExportMode mode,
                                 bool* ok = nullptr) const;
    nlohmann::json exportContent(unsigned int row,
                                 ReportExportMode mode) const;
    boost::optional<std::vector<nlohmann::json>> exportContent(ReportExportMode mode) const;

    nlohmann::json exportProxyContent(unsigned int row, 
                                      unsigned int col,
                                      ReportExportMode mode,
                                      bool* ok = nullptr) const;
    nlohmann::json exportProxyContent(unsigned int row,
                                      ReportExportMode mode) const;
    boost::optional<std::vector<nlohmann::json>> exportProxyContent(ReportExportMode mode) const;

    unsigned int numProxyRows () const;
    unsigned int numProxyColumns () const;
    std::vector<std::string> proxyHeadings() const;

    static boost::optional<QColor> cellTextColor(unsigned int style);
    static boost::optional<QColor> cellBGColor(unsigned int style);
    static std::string cellTextColorLatex(unsigned int style);
    static std::string cellBGColorLatex(unsigned int style);
    static boost::optional<std::pair<std::string, std::string>> cellIconFn(const nlohmann::json& data);
    static boost::optional<std::pair<QIcon, std::string>> cellIcon(const nlohmann::json& data);
    static boost::optional<bool> cellChecked(const nlohmann::json& data);
    static void cellFont(QFont& font, unsigned int style);
    static bool cellIsInactive(unsigned int style);
    static bool cellShowsText(unsigned int style);
    static bool cellShowsCheckBox(unsigned int style);
    static bool cellShowsIcon(unsigned int style);
    static bool cellShowsSpecialFont(unsigned int style);
    static bool cellFontIsBold(unsigned int style);
    static bool cellFontIsItalic(unsigned int style);
    static bool cellFontIsStrikeOut(unsigned int style);
    static std::string cellStyle2String(unsigned int style);

    static const std::string FieldHeadings;
    static const std::string FieldSortable;
    static const std::string FieldSortColumn;
    static const std::string FieldSortOrder;
    static const std::string FieldRows;
    static const std::string FieldAnnotations;
    static const std::string FieldColumnStyles;
    static const std::string FieldCellStyles;
    static const std::string FieldShowTooltips;
    static const std::string FieldMaxRowCount;
    static const std::string FieldColumnGroups;

    static const std::string FieldDocColumns;
    static const std::string FieldDocData;
    static const std::string FieldDocPath;

    static const std::string FieldAnnoFigureID;
    static const std::string FieldAnnoSectionLink;
    static const std::string FieldAnnoSectionFigure;
    static const std::string FieldAnnoOnDemand;
    static const std::string FieldAnnoIndex;
    static const std::string FieldAnnoStyle;

    static const std::string FieldColGroupName;
    static const std::string FieldColGroupColumns;
    static const std::string FieldColGroupEnabledOnInit;

    static const QColor ColorTextRed;
    static const QColor ColorTextOrange;
    static const QColor ColorTextGreen;
    static const QColor ColorTextGray;

    static const QColor ColorBGRed;
    static const QColor ColorBGOrange;
    static const QColor ColorBGGreen;
    static const QColor ColorBGGray;
    static const QColor ColorBGYellow;

    static const std::string ColorTextLatexRed;
    static const std::string ColorTextLatexOrange;
    static const std::string ColorTextLatexGreen;
    static const std::string ColorTextLatexGray;

    static const std::string ColorBGLatexRed;
    static const std::string ColorBGLatexOrange;
    static const std::string ColorBGLatexGreen;
    static const std::string ColorBGLatexGray;
    static const std::string ColorBGLatexYellow;

    static const double LatexIconWidth_cm;

protected:
    void clearContent_impl() override final;

    void toJSON_impl(nlohmann::json& j) const override final; 
    bool fromJSON_impl(const nlohmann::json& j) override final;
    Result toJSONDocument_impl(nlohmann::json& j,
                               const std::string* resource_dir,
                               ReportExportMode export_style) const override final;

    bool loadOnDemand() override final;

    unsigned int addFigure (const SectionContentViewable& viewable);

    SectionContentTableWidget* createTableWidget() const;
    const SectionContentTableWidget* tableWidget() const;
    SectionContentTableWidget* tableWidget();
    SectionContentTableWidget* getOrCreateTableWidget();
    const SectionContentTableWidget* getOrCreateTableWidget() const;

    void toggleShowUnused();
    void copyContent();
    
    void executeCallback(const std::string& name);

    void updateGroupColumns(bool update_widget = true);
    void updateGroupColumns(const TableColumnGroup& col_group,
                            bool update_widget = true);

    unsigned int               num_columns_      {0};
    unsigned int               num_columns_proxy_{0};
    std::vector<std::string>   headings_;
    std::vector<unsigned int>  column_styles_;
    std::vector<unsigned char> column_flags_;
    
    bool          sortable_     {true};
    unsigned int  sort_column_  {0};
    Qt::SortOrder sort_order_   {Qt::AscendingOrder};

    bool show_unused_   {false};
    bool show_tooltips_ {false};

    boost::optional<int> max_row_count_;

    /**
     * Describes figures and links attached to a table row.
     */
    struct RowAnnotation
    {
        boost::optional<unsigned int> figure_id;         // content id of a figure in the containing section
        std::string                   section_link;      // link to another section
        std::string                   section_figure;    // figure in the linked section
        bool                          on_demand = false; // viewable is generated on-demand
        QVariant                      index;             // detail index
        unsigned int                  style = 0;         // row style flags
    };

    mutable std::vector<nlohmann::json> rows_;
    mutable std::vector<RowAnnotation>  annotations_;
    
    CellStyles    cell_styles_;
    ColumnGroups  column_groups_;
    unsigned int  latex_ref_column_ = 0;

    mutable SectionContentTableWidget* table_widget_ {nullptr};

    std::map<std::string, std::function<void()>> callback_map_;
};

/**
 */
class SectionContentTableModel : public QAbstractItemModel
{
public:
    SectionContentTableModel(SectionContentTable* content_table, QObject* parent = nullptr);
    virtual ~SectionContentTableModel() = default;

    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;
    QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;
    QModelIndex parent(const QModelIndex& index) const override;
    Qt::ItemFlags flags(const QModelIndex &index) const override;

    void executeAndReset(const std::function<void()>& func);

protected:
    bool canFetchMore(const QModelIndex &parent) const override;
    void fetchMore(const QModelIndex &parent) override;

private:
    SectionContentTable* content_table_ = nullptr;
};

/**
 */
class SectionContentTableWidget : public QWidget
{
public:
    SectionContentTableWidget(SectionContentTable* content_table, 
                              bool show_unused,
                              int sort_column = -1,
                              Qt::SortOrder sort_order = Qt::AscendingOrder,
                              QWidget* parent = nullptr);
    virtual ~SectionContentTableWidget();

    SectionContentTableModel* itemModel();
    const SectionContentTableModel* itemModel() const;
    TableQSortFilterProxyModel* proxyModel();
    const TableQSortFilterProxyModel* proxyModel() const;
    QTableView* tableView();
    const QTableView* tableView() const;

    void showUnused(bool show);
    void resizeContent();
    void updateColumnVisibility();

    int fromProxy(int proxy_row) const;
    int scrollPosV() const;
    int scrollPosH() const;

    nlohmann::json jsonConfig() const;
    bool configure(const nlohmann::json& j);

    static const int DoubleClickCheckIntervalMSecs;

    static const std::string FieldConfigSortColumn;
    static const std::string FieldConfigSortOrder;
    static const std::string FieldConfigScrollPosV;
    static const std::string FieldConfigScrollPosH;
    static const std::string FieldColGroupStates;

protected:
    bool eventFilter(QObject *watched, QEvent *event) override final;

private:
    void clicked(const QModelIndex& index);
    void doubleClicked(const QModelIndex& index);
    void customContextMenu(const QPoint& p);
    void performClickAction();
    void updateOptionsMenu();
    void updateToolBar();
    void updateScrollBarV();
    void updateScrollBarH();

    SectionContentTable*        content_table_     = nullptr;
    SectionContentTableModel*   model_             = nullptr;
    TableQSortFilterProxyModel* proxy_model_       = nullptr;
    QTableView*                 table_view_        = nullptr;
    QPushButton*                options_button_    = nullptr;
    std::unique_ptr<PopupMenu>  options_menu_;
    QToolBar*                   col_group_toolbar_ = nullptr;

    int           sort_column_ = -1;
    Qt::SortOrder sort_order_  = Qt::AscendingOrder;

    QTimer                        click_action_timer_;
    boost::optional<unsigned int> last_clicked_row_index_;
    boost::optional<int>          scroll_pos_h_;
    boost::optional<int>          scroll_pos_v_;
};

}
