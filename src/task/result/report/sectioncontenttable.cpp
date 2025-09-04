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

#include "task/result/report/sectioncontenttable.h"
#include "task/result/report/section.h"
#include "task/result/report/sectioncontentfigure.h"
#include "task/result/report/report.h"
#include "task/result/report/reportexporter.h"
#include "task/result/taskresult.h"

#include "taskmanager.h"
#include "compass.h"
#include "dbcontentmanager.h"

#include "popupmenu.h"

#include "logger.h"
#include "stringconv.h"
#include "stringmat.h"
#include "asynctask.h"
#include "files.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTableView>
#include <QHeaderView>
#include <QSortFilterProxyModel>
#include <QMenu>
#include <QMessageBox>
#include <QClipboard>
#include <QApplication>
#include <QInputDialog>
#include <QThread>
#include <QScrollBar>
#include <QToolBar>

#include "traced_assert.h"
#include <type_traits>
#include <iostream>
#include <fstream>

namespace ResultReport
{

/***************************************************************************************************
 * SectionContentTable
 ***************************************************************************************************/

const std::string SectionContentTable::FieldHeadings      = "headings";
const std::string SectionContentTable::FieldSortable      = "sortable";
const std::string SectionContentTable::FieldSortColumn    = "sort_column";
const std::string SectionContentTable::FieldSortOrder     = "order";
const std::string SectionContentTable::FieldRows          = "rows";
const std::string SectionContentTable::FieldAnnotations   = "annotations";
const std::string SectionContentTable::FieldColumnStyles  = "column_styles";
const std::string SectionContentTable::FieldCellStyles    = "cell_styles";
const std::string SectionContentTable::FieldShowTooltips  = "show_tooltips";
const std::string SectionContentTable::FieldMaxRowCount   = "max_row_count";
const std::string SectionContentTable::FieldColumnGroups  = "column_groups";

const std::string SectionContentTable::FieldDocColumns  = "columns";
const std::string SectionContentTable::FieldDocData     = "data";
const std::string SectionContentTable::FieldDocPath     = "path";

const std::string SectionContentTable::FieldAnnoFigureID      = "figure_id";
const std::string SectionContentTable::FieldAnnoSectionLink   = "section_link";
const std::string SectionContentTable::FieldAnnoSectionFigure = "section_figure";
const std::string SectionContentTable::FieldAnnoOnDemand      = "on_demand";
const std::string SectionContentTable::FieldAnnoIndex         = "index";
const std::string SectionContentTable::FieldAnnoStyle         = "style";

const std::string SectionContentTable::FieldColGroupName          = "name";
const std::string SectionContentTable::FieldColGroupColumns       = "column_indices";
const std::string SectionContentTable::FieldColGroupEnabledOnInit = "enabled_on_init";

const QColor SectionContentTable::ColorTextRed    = Colors::TextRed;
const QColor SectionContentTable::ColorTextOrange = Colors::TextOrange;
const QColor SectionContentTable::ColorTextGreen  = Colors::TextGreen;
const QColor SectionContentTable::ColorTextGray   = Colors::TextGray;

const QColor SectionContentTable::ColorBGRed      = Colors::BGRed;
const QColor SectionContentTable::ColorBGOrange   = Colors::BGOrange;
const QColor SectionContentTable::ColorBGGreen    = Colors::BGGreen;
const QColor SectionContentTable::ColorBGGray     = Colors::BGGray;
const QColor SectionContentTable::ColorBGYellow   = Colors::BGYellow;

const std::string SectionContentTable::ColorTextLatexRed    = Colors::TextLatexRed;
const std::string SectionContentTable::ColorTextLatexOrange = Colors::TextLatexOrange;
const std::string SectionContentTable::ColorTextLatexGreen  = Colors::TextLatexGreen;
const std::string SectionContentTable::ColorTextLatexGray   = Colors::TextLatexGray;

const std::string SectionContentTable::ColorBGLatexRed      = Colors::BGLatexRed;
const std::string SectionContentTable::ColorBGLatexOrange   = Colors::BGLatexOrange;
const std::string SectionContentTable::ColorBGLatexGreen    = Colors::BGLatexGreen;
const std::string SectionContentTable::ColorBGLatexGray     = Colors::BGLatexGray;
const std::string SectionContentTable::ColorBGLatexYellow   = Colors::BGLatexYellow;

const double SectionContentTable::LatexIconWidth_cm = 0.5;

/**
 */
SectionContentTable::SectionContentTable(unsigned int id,
                                         const std::string& name,
                                         unsigned int num_columns,
                                         const std::vector<std::string>& headings,
                                         Section* parent_section,
                                         bool sortable,
                                         unsigned int sort_column,
                                         Qt::SortOrder sort_order)
:   SectionContent(ContentType::Table, id, name, parent_section)
,   num_columns_      (num_columns)
,   num_columns_proxy_(num_columns)
,   headings_         (headings)
,   column_styles_    (num_columns, 0)
,   column_flags_     (num_columns, 0)
,   sortable_         (sortable)
,   sort_column_      (sort_column)
,   sort_order_       (sort_order)
{
}

/**
 */
SectionContentTable::SectionContentTable(Section* parent_section)
:   SectionContent(ContentType::Table, parent_section)
{
}

/**
 */
SectionContentTable::~SectionContentTable() = default;

/**
 */
void SectionContentTable::enableTooltips()
{
    show_tooltips_ = true;
}

/**
 */
void SectionContentTable::addRow (const nlohmann::json::array_t& row,
                                  const SectionContentViewable& viewable,
                                  const std::string& section_link,
                                  const std::string& section_figure,
                                  const QVariant& viewable_index,
                                  unsigned int row_style)
{
    traced_assert(row.size() == num_columns_);

    rows_.push_back(row);

    //configure attached annotation
    RowAnnotation anno;
    anno.index          = viewable_index;
    anno.section_link   = section_link;
    anno.section_figure = section_figure;
    anno.on_demand      = viewable.on_demand;
    anno.style          = row_style;

    logdbg<< "'" << name() << "': viewable has callback " << viewable.hasCallback();

    if (!viewable.on_demand && viewable.hasCallback())
    {
        //add figure to containing section and remember id
        anno.figure_id = addFigure(viewable);
    }

    annotations_.push_back(anno);
}

/**
 */
const nlohmann::json& SectionContentTable::getData(int row, int column) const
{
    return rows_.at(row).at(column);
}

/**
 */
const nlohmann::json& SectionContentTable::getData(int row, const std::string& col_name) const
{
    int col = columnIndex(col_name);
    traced_assert(col >= 0);

    return getData(row, col);
}

/**
 */
bool SectionContentTable::hasColumn(const std::string& col_name) const
{
    return columnIndex(col_name) >= 0;
}

/**
 */
int SectionContentTable::columnIndex(const std::string& col_name) const
{
    auto it = std::find_if(headings_.begin(), headings_.end(), [ & ] (const std::string& cname) { return col_name == cname; });
    if (it == headings_.end())
        return -1;

    return (int)std::distance(headings_.begin(), it);
}

/**
 */
void SectionContentTable::setColumnStyle(int column, unsigned int style)
{
    column_styles_.at(column) = style;
}

/**
 */
void SectionContentTable::setCellStyle(int row, int column, unsigned int style)
{
    cell_styles_[ std::make_pair(column, row) ] = style;
}

/**
 */
unsigned int SectionContentTable::cellStyle(int row, int column) const
{
    unsigned int style = 0;

    // first add column style
    style |= column_styles_.at(column);

    // overide with row style
    style |= annotations_.at(row).style;

    // override with cell stlye if available
    if (!cell_styles_.empty())
    {
        auto it = cell_styles_.find(std::make_pair(column, row));
        if (it != cell_styles_.end())
            style |= it->second;
    }

    return style;
}

/**
 */
void SectionContentTable::setMaxRowCount(const boost::optional<int>& max_row_count)
{
    max_row_count_ = max_row_count;
}

/**
 */
const boost::optional<int>& SectionContentTable::maxRowCount() const
{
    return max_row_count_;
}

/**
 */
unsigned int SectionContentTable::addFigure(const SectionContentViewable& viewable)
{
    return parentSection()->addHiddenFigure(viewable);
}

/**
 */
SectionContentTableWidget* SectionContentTable::createTableWidget() const
{
    traced_assert(!isLocked());
    traced_assert(!table_widget_);

    SectionContentTable* tmp = const_cast<SectionContentTable*>(this); // hacky
    table_widget_ = new SectionContentTableWidget(tmp, 
                                                  show_unused_, 
                                                  sortable_ ? sort_column_ : -1, 
                                                  sort_order_);
    return table_widget_;
}

/**
 */
const SectionContentTableWidget* SectionContentTable::tableWidget() const
{
    traced_assert(table_widget_);
    return table_widget_;
}

/**
 */
SectionContentTableWidget* SectionContentTable::tableWidget()
{
    traced_assert(table_widget_);
    return table_widget_;
}

/**
 */
SectionContentTableWidget* SectionContentTable::getOrCreateTableWidget()
{
    if (table_widget_)
        return table_widget_;

    return createTableWidget();
}

/**
 */
const SectionContentTableWidget* SectionContentTable::getOrCreateTableWidget() const
{
    if (table_widget_)
        return table_widget_;

    return createTableWidget();
}

/**
 */
std::string SectionContentTable::resourceExtension() const
{
    return ReportExporter::ExportTableFormat;
}

/**
 */
void SectionContentTable::addContentUI(QVBoxLayout* layout, 
                                       bool force_ui_reset)
{
    loginf;

    traced_assert(layout);

    //force recreation of table widget?
    if (force_ui_reset)
        table_widget_ = nullptr;

    //finalize some custom stuff before showing the table in a layout
    taskResult()->postprocessTable(this);

    if (isLocked())
    {
        layout->addWidget(lockStatePlaceholderWidget());
    }
    else
    {
        //add widget to layout
        auto widget = getOrCreateTableWidget();

        widget->updateColumnVisibility();
        widget->resizeContent();

        layout->addWidget(widget);
    }
}

/**
 */
void SectionContentTable::clearContent_impl()
{
    auto func = [ & ] ()
    {
        rows_.clear();
        annotations_.clear();
        cell_styles_.clear();
    };

    if (table_widget_)
        table_widget_->itemModel()->executeAndReset(func);
    else
        func();
}

/**
 */
bool SectionContentTable::loadOnDemand()
{
    bool ok = false;

    auto func = [ & ] ()
    {
        ok = SectionContent::loadOnDemand();
    };

    if (table_widget_)
    {
        table_widget_->itemModel()->executeAndReset(func);
        table_widget_->resizeContent();
    }
    else
    {
        func();
    }

    return ok;
}

namespace
{
    /**
     * Converts a json value to a QVariant for display purpose.
     */
    QVariant qVariantFromJSON(const nlohmann::json& j)
    {
        if (j.is_boolean())
            return QVariant::fromValue(j.get<bool>());
        else if (j.is_number_integer())
            return QVariant::fromValue(j.get<qint64>());  // or int, depending on your use case
        else if (j.is_number_unsigned())
            return QVariant::fromValue(j.get<quint64>());
        else if (j.is_number_float())
            return QVariant::fromValue(j.get<double>());
        else if (j.is_string())
            return QVariant::fromValue(QString::fromStdString(j.get<std::string>()));
        else if (j.is_null())
            return QVariant();
        else
            return QVariant();
    };
}

/**
 */
boost::optional<QColor> SectionContentTable::cellTextColor(unsigned int style)
{
    if (style & CellStyleTextColorRed)
        return ColorTextRed;
    else if (style & CellStyleTextColorOrange)
        return ColorTextOrange;
    else if (style & CellStyleTextColorGreen)
        return ColorTextGreen;
    else if (style & CellStyleTextColorGray)
        return ColorTextGray;

    return boost::optional<QColor>();
}

/**
 */
boost::optional<QColor> SectionContentTable::cellBGColor(unsigned int style)
{
    if (style & CellStyleBGColorRed)
        return ColorBGRed;
    else if (style & CellStyleBGColorOrange)
        return ColorBGOrange;
    else if (style & CellStyleBGColorGreen)
        return ColorBGGreen;
    else if (style & CellStyleBGColorGray)
        return ColorBGGray;
    else if (style & CellStyleBGColorYellow)
        return ColorBGYellow;

    return boost::optional<QColor>();
}

/**
 */
std::string SectionContentTable::cellTextColorLatex(unsigned int style)
{
    if (style & CellStyleTextColorRed)
        return ColorTextLatexRed;
    else if (style & CellStyleTextColorOrange)
        return ColorTextLatexOrange;
    else if (style & CellStyleTextColorGreen)
        return ColorTextLatexGreen;
    else if (style & CellStyleTextColorGray)
        return ColorTextLatexGray;

    return "";
}

/**
 */
std::string SectionContentTable::cellBGColorLatex(unsigned int style)
{
    if (style & CellStyleBGColorRed)
        return ColorBGLatexRed;
    else if (style & CellStyleBGColorOrange)
        return ColorBGLatexOrange;
    else if (style & CellStyleBGColorGreen)
        return ColorBGLatexGreen;
    else if (style & CellStyleBGColorGray)
        return ColorBGLatexGray;
    else if (style & CellStyleBGColorYellow)
        return ColorBGLatexYellow;

    return "";
}

/**
 */
boost::optional<std::pair<std::string, std::string>> SectionContentTable::cellIconFn(const nlohmann::json& data)
{
    if (!data.is_string())
        return boost::optional<std::pair<std::string, std::string>>();

    QString icon_string;
    
    try
    {
        icon_string = QString::fromStdString(data.get<std::string>());
    }
    catch(...)
    {
        return boost::optional<std::pair<std::string, std::string>>();
    }

    if (icon_string.isEmpty())
        return boost::optional<std::pair<std::string, std::string>>();

    std::string icon_fn  = icon_string.toStdString();
    std::string icon_txt = "";

    if (icon_string.contains(';'))
    {
        auto parts = icon_string.split(';');
        if (parts.size() != 2)
            return boost::optional<std::pair<std::string, std::string>>();

        icon_fn  = parts.at(0).toStdString();
        icon_txt = parts.at(1).toStdString(); 
    }
    
    return std::make_pair(icon_fn, icon_txt);
}

/**
 */
boost::optional<std::pair<QIcon, std::string>> SectionContentTable::cellIcon(const nlohmann::json& data)
{
    auto fn = SectionContentTable::cellIconFn(data);
    if (!fn)
        return boost::optional<std::pair<QIcon, std::string>>();

    return std::make_pair(Utils::Files::IconProvider::getIcon(fn->first), fn->second);
}

/**
 */
boost::optional<bool> SectionContentTable::cellChecked(const nlohmann::json& data)
{
    if (!data.is_boolean())
        return boost::optional<bool>();

    bool check_state;

    try
    {
        check_state = data;
    }
    catch(...)
    {
        return boost::optional<bool>();
    }

    return check_state;
}

/**
 */
void SectionContentTable::cellFont(QFont& font, unsigned int style)
{
    font.setBold(style & CellStyleTextBold);
    font.setItalic(style & CellStyleTextItalic);
    font.setStrikeOut(style & CellStyleTextStrikeOut);
}

/**
 */
bool SectionContentTable::cellShowsText(unsigned int style)
{
    return !cellShowsCheckBox(style) &&
           !cellShowsIcon(style);
}

/**
 */
bool SectionContentTable::cellShowsCheckBox(unsigned int style)
{
    return (style & CellStyleCheckable) != 0;
}

/**
 */
bool SectionContentTable::cellShowsIcon(unsigned int style)
{
    return (style & CellStyleIcon) != 0;
}

/**
 */
bool SectionContentTable::cellShowsSpecialFont(unsigned int style)
{
    return cellFontIsBold(style)   ||
           cellFontIsItalic(style) ||
           cellFontIsStrikeOut(style);
}

/**
 */
bool SectionContentTable::cellFontIsBold(unsigned int style)
{
    return (style & CellStyleTextBold) != 0;
}

/**
 */
bool SectionContentTable::cellFontIsItalic(unsigned int style)
{
    return (style & CellStyleTextItalic) != 0;
}

/**
 */
bool SectionContentTable::cellFontIsStrikeOut(unsigned int style)
{
    return (style & CellStyleTextStrikeOut) != 0;
}

/**
 */
std::string SectionContentTable::cellStyle2String(unsigned int style)
{
    std::string str;

    if (style & CellStyleCheckable)
        str += (str.empty() ? "" : " | ") + std::string("Checkable");
    if (style & CellStyleIcon)
        str += (str.empty() ? "" : " | ") + std::string("Icon");

    if (style & CellStyleTextBold)
        str += (str.empty() ? "" : " | ") + std::string("Bold");
    if (style & CellStyleTextItalic)
        str += (str.empty() ? "" : " | ") + std::string("Italic");
    if (style & CellStyleTextStrikeOut)
        str += (str.empty() ? "" : " | ") + std::string("StrikeOut");

    if (style & CellStyleTextColorRed)
        str += (str.empty() ? "" : " | ") + std::string("TextRed");
    if (style & CellStyleTextColorOrange)
        str += (str.empty() ? "" : " | ") + std::string("TextOrange");
    if (style & CellStyleTextColorGreen)
        str += (str.empty() ? "" : " | ") + std::string("TextGreen");
    if (style & CellStyleTextColorGray)
        str += (str.empty() ? "" : " | ") + std::string("TextGray");

    if (style & CellStyleBGColorRed)
        str += (str.empty() ? "" : " | ") + std::string("BGRed");
    if (style & CellStyleBGColorOrange)
        str += (str.empty() ? "" : " | ") + std::string("BGOrange");
    if (style & CellStyleBGColorGreen)
        str += (str.empty() ? "" : " | ") + std::string("BGGreen");
    if (style & CellStyleBGColorGray)
        str += (str.empty() ? "" : " | ") + std::string("BGGray");
    if (style & CellStyleBGColorYellow)
        str += (str.empty() ? "" : " | ") + std::string("BGYellow");

    return str;
}

/**
 */
QVariant SectionContentTable::data(const QModelIndex& index, int role) const
{
    if (!index.isValid())
        return QVariant();

    traced_assert(index.row() >= 0);
    traced_assert(index.row() < (int)rows_.size());
    traced_assert(index.column() >= 0);
    traced_assert(index.column() < (int)num_columns_);

    return data(index.row(), index.column(), role);
}

/**
 */
QVariant SectionContentTable::data(int row, int col, int role) const
{
    switch (role)
    {
        case Qt::DisplayRole:
        {
            logdbg << "display role: row " << row << " col " << col;

            auto style = cellStyle(row, col);

            if (cellShowsText(style))
                return qVariantFromJSON(rows_.at(row).at(col));

            return QVariant(); 
        }
        case Qt::BackgroundRole:
        {
            auto style = cellStyle(row, col);

            if (cellShowsText(style))
            {
                auto c = cellBGColor(style);
                if (c.has_value())
                    return QBrush(c.value());
            }

            return QVariant();
        }
        case Qt::ForegroundRole:
        {
            auto style = cellStyle(row, col);

            if (style != 0)
            {
                auto c = cellTextColor(style);
                if (c.has_value())
                    return QBrush(c.value());
            }

            //custom heuristics for evaluation
            //@TODO: configure tables accordingly in evaluation results
            if (cellShowsText(style) &&
                taskResult()->type() == task::TaskResultType::Evaluation)
            {
                const auto& data = rows_.at(row).at(col);

                if (data.is_string())
                {
                    auto txt = data.get<std::string>();

                    if (txt == "Passed")
                        return QBrush(ColorTextGreen);
                    else if (txt == "Failed")
                        return QBrush(ColorTextRed);
                }
                if (data.is_boolean())
                {
                    bool ok = data.get<bool>();

                    if (ok == true)
                        return QBrush(ColorTextGreen);
                    else if (ok == false)
                        return QBrush(ColorTextRed);
                }
            }

            return QVariant();
        }
        case Qt::DecorationRole:
        {
            auto style = cellStyle(row, col);

            if (cellShowsIcon(style))
            {
                const auto& j = rows_.at(row).at(col);
                auto icon = cellIcon(j);
                if (icon.has_value())
                    return icon.value().first;
            }

            return QVariant();
        }
        case Qt::CheckStateRole:
        {
            auto style = cellStyle(row, col);

            if (cellShowsCheckBox(style))
            {
                const auto& j = rows_.at(row).at(col);
                auto c = cellChecked(j);
                if (c.has_value())
                    return c.value() ? Qt::Checked : Qt::Unchecked;
            }

            return QVariant();
        }
        case Qt::FontRole:
        {
            auto style = cellStyle(row, col);

            if (cellShowsSpecialFont(style))
            {
                QFont f;
                cellFont(f, style);
                return f;
            }

            return QVariant();
        }
        case Qt::ToolTipRole:
        {
            if (show_tooltips_)
            {
                auto tResult = taskResult();
                if (tResult->hasCustomTooltip(this, (unsigned int)row, (unsigned int)col))
                {
                    // custom tooltips
                    auto ttip = tResult->customTooltip(this, (unsigned int)row, (unsigned int)col);
                    if (!ttip.empty())
                        return QString::fromStdString(ttip);
                }
                else
                {
                    // default tooltips
                    auto style = cellStyle(row, col);

                    if (cellShowsIcon(style))
                    {
                        auto fn = cellIconFn(rows_.at(row).at(col));
                        if (fn.has_value() && !fn.value().second.empty())
                            return QString::fromStdString(fn->second);
                    }
                }
            }

            return QVariant();
        }
        case CellStyleRole:
        {
            return cellStyle(row, col);
        }
        default:
        {
            return QVariant();
        }
    }

    return QVariant();
}

/**
 */
Qt::ItemFlags SectionContentTable::flags(const QModelIndex &index) const
{
    if (!index.isValid())
        return Qt::NoItemFlags;

    auto style = cellStyle(index.row(), index.column());
    if (style & CellStyleCheckable)
        return Qt::ItemIsUserCheckable;

    return Qt::NoItemFlags;
}

/**
 */
size_t SectionContentTable::numRows() const
{
    return rows_.size();
}

/**
 */
size_t SectionContentTable::numColumns() const
{
    return num_columns_;
}

/**
 */
const std::vector<std::string>& SectionContentTable::headings() const
{
    return headings_;
}

/**
 */
void SectionContentTable::setColumnGroup(const std::string& name, 
                                         const std::vector<int>& columns,
                                         bool enabled)
{
    traced_assert(!table_widget_); //!no config of column groups after widget is created!

    auto& col_group = column_groups_[ name ];

    col_group         = {};
    col_group.name    = name;
    col_group.columns = columns;
    col_group.enabled = enabled;
    
    updateGroupColumns();
}

/**
 */
void SectionContentTable::enableColumnGroup(const std::string& name,
                                            bool ok)
{
    auto& column_group = column_groups_.at(name);
    column_group.enabled = ok;

    updateGroupColumns();
}

/**
 */
bool SectionContentTable::hasColumnGroup(const std::string& name) const
{
    return column_groups_.count(name) > 0;
}

/**
 */
void SectionContentTable::updateGroupColumns(bool update_widget)
{
    for (const auto& cg : column_groups_)
        updateGroupColumns(cg.second, false);

    num_columns_proxy_ = 0;
    for (unsigned int c = 0; c < num_columns_; ++c)
        if (columnVisible(c))
            ++num_columns_proxy_;

    traced_assert(num_columns_proxy_ <= num_columns_);

    if (update_widget && table_widget_)
        table_widget_->updateColumnVisibility();
}

/**
 */
void SectionContentTable::updateGroupColumns(const TableColumnGroup& col_group,
                                             bool update_widget)
{
    for (auto col : col_group.columns)
    {
        auto& f = column_flags_.at((size_t)col);
        if (col_group.enabled)
            f &= ~ColumnHidden;
        else
            f |= ColumnHidden;
    }

    if (update_widget && table_widget_)
        table_widget_->updateColumnVisibility();
}

/**
 */
bool SectionContentTable::columnGroupEnabled(const std::string& name) const
{
    return column_groups_.at(name).enabled;
}

/**
 */
bool SectionContentTable::columnVisible(int column) const
{
    return (column_flags_.at((size_t)column) & ColumnHidden) == 0;
}

/**
 */
bool SectionContentTable::columnsHidden() const
{
    return num_columns_proxy_ < num_columns_;
}

/**
 */
bool SectionContentTable::hasReference (unsigned int row) const
{
    //obtain original data row
    unsigned int row_index = getOrCreateTableWidget()->fromProxy(row);

    const auto& annotation = annotations_.at(row_index);

    return !annotation.section_link.empty();
}

/**
 */
std::string SectionContentTable::reference(unsigned int row) const
{
    //obtain original data row
    unsigned int row_index = getOrCreateTableWidget()->fromProxy(row);

    const auto& annotation = annotations_.at(row_index);

    return annotation.section_link;
}

/**
 */
bool SectionContentTable::showUnused() const
{
    return show_unused_;
}

/**
 */
void SectionContentTable::showUnused(bool value)
{
    loginf << "value " << value;

    getOrCreateTableWidget()->showUnused(value);

    show_unused_ = value;
}

/**
 */
void SectionContentTable::registerCallBack (const std::string& name, const std::function<void()>& func)
{
    traced_assert(!callback_map_.count(name));
    callback_map_.emplace(name, func);
}

/**
 */
void SectionContentTable::executeCallback(const std::string& name)
{
    traced_assert(callback_map_.count(name));
    callback_map_.at(name)();
}

/**
 */
bool SectionContentTable::clicked(unsigned int row)
{
    const auto& annotation = annotations_.at(row);

    //always trigger a blocked load (to regain focus later on)
    const bool BlockedReload = true;

    bool reload_triggered = false;

    //generate on-demand viewable?
    if (annotation.on_demand)
    {
        //only show on demand figure if result is not locked
        if (!taskResult()->isLocked())
        {
            SectionContentViewable viewable;
            bool ok = taskResult()->loadOnDemandViewable(*this, viewable, annotation.index, row);

            if (ok)
            {
                QApplication::setOverrideCursor(Qt::WaitCursor);
                {
                    auto content = viewable.viewable_func();
                    report_->setCurrentViewable(*content, BlockedReload);

                    reload_triggered = true;
                }
                QApplication::restoreOverrideCursor();
            }
            else
            {
                report_->unsetCurrentViewable();
                logerr << "on-demand viewable could not be retrieved";
            }
        }

        return reload_triggered;
    }

    //obtain figure from annotation
    bool has_valid_link = false;
    SectionContentFigure* figure = nullptr;

    if (annotation.figure_id.has_value())
    {
        loginf << "index has associated viewable via id " << annotation.figure_id.value();
        has_valid_link = true;

        //figure from content in parent section
        auto c = parentSection()->retrieveContent(annotation.figure_id.value(), true);
        figure = dynamic_cast<SectionContentFigure*>(c.get());
    }
    else if (!annotation.section_link.empty() && !annotation.section_figure.empty())
    {
        loginf << "index has associated viewable via" 
               << " section '" << annotation.section_link << "'"
               << " figure '" << annotation.section_figure << "'";
        has_valid_link = true;

        //figure from section link + figure name
        auto& section = report_->getSection(annotation.section_link);

        if (section.hasFigure(annotation.section_figure))
            figure = &section.getFigure(annotation.section_figure);
    }

    //valid figure link found?
    if (has_valid_link)
    {
        if (figure)
        {
            // if (result_ptrs_.at(row_index)->viewableDataReady())
            // {
            //     //view data ready, just get it
            //     viewable = result_ptrs_.at(row_index)->viewableData(*this, annotations_.at(row_index));
            // }
            // else
            // {
            //     //recompute async and show wait dialog, this may take a while...
            //     auto func = [ & ] (const AsyncTaskState& state, AsyncTaskProgressWrapper& progress)
            //     {
            //         viewable = result_ptrs_.at(row_index)->viewableData(*this, annotations_.at(row_index));
            //         return Result::succeeded();
            //     };

            //     AsyncFuncTask task(func, "Updating Contents", "Updating contents...", false);
            //     task.runAsyncDialog();
            // }

            //show viewable (will now recompute internally if needed)
            //              (might get cancelled if the figure is locked)
            QApplication::setOverrideCursor(Qt::WaitCursor);
            {
                reload_triggered = figure->view(BlockedReload);
            }
            QApplication::restoreOverrideCursor();
        }
        else
        {
            logerr << "figure could not be retrieved";
        }
    }

    return reload_triggered;
}

/**
 */
void SectionContentTable::doubleClicked(unsigned int row)
{
    const auto& annotation = annotations_.at(row);

    //link to other section stored in row?
    if (!annotation.section_link.empty())
    {
        loginf << "index has associated reference '"
               << annotation.section_link << "'";

        //jump to row link
        report_->setCurrentSection(annotation.section_link);
    }
    else
    {
        loginf << "index has no associated reference";
    }
}

/**
 */
void SectionContentTable::customContextMenu(unsigned int row, const QPoint& pos)
{
    QMenu menu;
    if (!taskResult()->customContextMenu(menu, this, row))
        return;

    menu.exec(pos);
}

/**
 */
void SectionContentTable::addActionsToMenu(QMenu* menu)
{
    //add general actions

    //@TODO: seems to be broken at the moment, fix functionality
    //QAction* unused_action = menu->addAction("Toggle Show Unused");
    //QObject::connect (unused_action, &QAction::triggered, [ this ] { this->toggleShowUnused(); });

    QAction* copy_action = menu->addAction("Copy Content");
    QObject::connect (copy_action, &QAction::triggered, [ this ] { this->copyContent(); });

    //add custom callbacks stored in map
    if (callback_map_.size() > 0)
    {
        menu->addSeparator();

        for (auto& cb_it : callback_map_)
        {
            QAction* action = menu->addAction(cb_it.first.c_str());
            QObject::connect(action, &QAction::triggered, [ = ] () { this->executeCallback(cb_it.first); });
        }
    }

    //add custom entries provided by task result
    taskResult()->customMenu(*menu, this);
}

/**
 */
void SectionContentTable::toggleShowUnused()
{
    showUnused(!show_unused_);
}

/**
 */
void SectionContentTable::copyContent()
{
    loginf;

    std::stringstream ss;

    unsigned int num_cols = headings_.size();

    // headings
    for (unsigned int cnt=0; cnt < num_cols; ++cnt)
    {
        if (cnt == 0)
            ss << headings_.at(cnt);
        else
            ss <<  ";" << headings_.at(cnt);
    }
    ss << "\n";

    unsigned int num_rows = numProxyRows();

    nlohmann::json row_data;

    for (unsigned int row=0; row < num_rows; ++row)
    {
        row_data = exportProxyContent(row, ReportExportMode::CSV);
        traced_assert(row_data.is_array());
        traced_assert(row_data.size() == num_cols);

        for (unsigned int cnt=0; cnt < num_cols; ++cnt)
        {
            const auto& d = row_data.at(cnt);
            traced_assert(d.is_string());

            if (cnt == 0)
                ss << d.get<std::string>();
            else
                ss <<  ";" << d.get<std::string>();
        }
        ss << "\n";
    }

    QApplication::clipboard()->setText(ss.str().c_str());
}

/**
 */
void SectionContentTable::toJSON_impl(nlohmann::json& j) const
{
    //call base
    SectionContent::toJSON_impl(j);

    j[ FieldHeadings      ] = headings_;
    j[ FieldSortable      ] = sortable_;
    j[ FieldSortColumn    ] = sort_column_;
    j[ FieldSortOrder     ] = sort_order_ == Qt::AscendingOrder ? "ascending" : "descending";
    j[ FieldRows          ] = std::vector<nlohmann::json>();
    j[ FieldAnnotations   ] = nlohmann::json::array();
    j[ FieldColumnStyles  ] = column_styles_;
    j[ FieldCellStyles    ] = cell_styles_;
    j[ FieldShowTooltips  ] = show_tooltips_;

    if (max_row_count_.has_value())
        j[ FieldMaxRowCount ] = max_row_count_.value();

    //write column groups
    auto j_col_groups = nlohmann::json::array();
    for (const auto& col_group : column_groups_)
    {
        nlohmann::json j_group;

        j_group[ FieldColGroupName          ] = col_group.second.name;
        j_group[ FieldColGroupColumns       ] = col_group.second.columns;
        j_group[ FieldColGroupEnabledOnInit ] = col_group.second.enabled_on_init;

        j_col_groups.push_back(j_group);
    }
    j[ FieldColumnGroups ] = j_col_groups;

    //write content only if not on demand
    if (!isOnDemand())
    {
        //write rows
        j[ FieldRows ] = rows_;

        //write annotations
        nlohmann::json j_annos = nlohmann::json::array();

        for (const auto& a : annotations_)
        {
            nlohmann::json j_anno;

            if (a.figure_id.has_value())
                j_anno[ FieldAnnoFigureID ] = a.figure_id.value();

            j_anno[ FieldAnnoSectionLink   ] = a.section_link;
            j_anno[ FieldAnnoSectionFigure ] = a.section_figure;
            j_anno[ FieldAnnoOnDemand      ] = a.on_demand;
            j_anno[ FieldAnnoStyle         ] = a.style;

            j_annos.push_back(j_anno);
        }

        j[ FieldAnnotations ] = j_annos;
    }
}

/**
 */
bool SectionContentTable::fromJSON_impl(const nlohmann::json& j)
{
    //call base
    if (!SectionContent::fromJSON_impl(j))
        return false;
    
    if (!j.is_object()                 ||
        !j.contains(FieldHeadings)     ||
        !j.contains(FieldSortable)     ||
        !j.contains(FieldSortColumn)   ||
        !j.contains(FieldSortOrder)    ||
        !j.contains(FieldRows)         ||
        !j.contains(FieldAnnotations)  ||
        !j.contains(FieldColumnStyles) ||
        !j.contains(FieldCellStyles)   ||
        !j.contains(FieldShowTooltips))
    {
        logerr << "section content table does not obtain needed fields";
        return false;
    }

    headings_      = j[ FieldHeadings     ].get<std::vector<std::string>>();
    column_styles_ = j[ FieldColumnStyles ].get<std::vector<unsigned int>>();
    sortable_      = j[ FieldSortable     ];
    sort_column_   = j[ FieldSortColumn   ];
    show_tooltips_ = j[ FieldShowTooltips ];

    std::string sort_order = j[ FieldSortOrder ];
    sort_order_ = sort_order == "ascending" ? Qt::AscendingOrder : Qt::DescendingOrder;

    if (j.contains(FieldMaxRowCount))
    {
        int v = j[ FieldMaxRowCount ];
        max_row_count_ = v;
    }

    if (j.contains(FieldColumnGroups))
    {
        //read in defined column groups
        const auto& j_col_groups = j[ FieldColumnGroups ];
        if (!j_col_groups.is_array())
        {
            logerr << "could not read column groups";
            return false;
        }

        for (const auto& j_col_group : j_col_groups)
        {
            if (!j_col_group.is_object() ||
                !j_col_group.contains(FieldColGroupName)    ||
                !j_col_group.contains(FieldColGroupColumns) ||
                !j_col_group.contains(FieldColGroupEnabledOnInit))
            {
                logerr << "could not read column group";
                return false;
            }

            TableColumnGroup col_group;
            col_group.name            = j_col_group[ FieldColGroupName          ];
            col_group.columns         = j_col_group[ FieldColGroupColumns       ].get<std::vector<int>>();
            col_group.enabled_on_init = j_col_group[ FieldColGroupEnabledOnInit ];
            col_group.enabled         = col_group.enabled_on_init;

            column_groups_[ col_group.name ] = col_group;
        }
    }

    num_columns_       = headings_.size();
    num_columns_proxy_ = headings_.size();

    //@TODO: maybe serialize these flags in the future
    column_flags_.assign(num_columns_, 0);

    rows_ = j[ FieldRows ].get<std::vector<nlohmann::json>>();

    auto& j_annos = j[ FieldAnnotations ];
    if (!j_annos.is_array() || j_annos.size() != rows_.size())
    {
        logerr << "annotation array invalid";
        return false;
    }

    for (const auto& j_anno : j_annos)
    {
        if (!j_anno.contains(FieldAnnoSectionLink)   ||
            !j_anno.contains(FieldAnnoSectionFigure) ||
            !j_anno.contains(FieldAnnoOnDemand)      ||
            !j_anno.contains(FieldAnnoStyle))
        {
            logerr << "could not read annotation";
            return false;
        }

        RowAnnotation anno;
        anno.section_link   = j_anno[ FieldAnnoSectionLink   ];
        anno.section_figure = j_anno[ FieldAnnoSectionFigure ];
        anno.on_demand      = j_anno[ FieldAnnoOnDemand      ];
        anno.style          = j_anno[ FieldAnnoStyle         ];

        if (j_anno.contains(FieldAnnoFigureID))
        {
            unsigned int id = j_anno[ FieldAnnoFigureID ];
            anno.figure_id = id;
        }

        annotations_.push_back(anno);
    }

    cell_styles_ = j[ FieldCellStyles ].get<CellStyles>();

    traced_assert(rows_.size() == annotations_.size());
    traced_assert(num_columns_ == column_styles_.size());

    //run some updates
    updateGroupColumns();

    return true;
}

/**
 */
Result SectionContentTable::toJSONDocument_impl(nlohmann::json& j,
                                                const std::string* resource_dir,
                                                ReportExportMode export_style) const
{
    //call base
    auto r = SectionContent::toJSONDocument_impl(j, resource_dir, export_style);
    if (!r.ok())
        return r;

    bool write_to_file = (ReportExporter::TableMaxRows    >= 0 && numRows()    > (size_t)ReportExporter::TableMaxRows   ) ||
                         (ReportExporter::TableMaxColumns >= 0 && numColumns() > (size_t)ReportExporter::TableMaxColumns);

    auto data = exportProxyContent(export_style);
    if (!data.has_value())
        return Result::failed("Could not prepare content '" + name() + "' for export");

    if (resource_dir && write_to_file)
    {
        auto res = prepareResource(*resource_dir, ResourceDir::Tables);
        if (!res.ok())
            return res;

        nlohmann::json j_ext;
        j_ext[ FieldDocColumns ] = headings_;
        j_ext[ FieldDocData    ] = data.value();

        std::ofstream of(res.result().path);
        if (!of.is_open())
            return Result::failed("Could not store resource for content '" + name() + "'");

        of << j_ext.dump(4);
        if (!of)
            return Result::failed("Could not store resource for content '" + name() + "'");

        of.close();

        j[ FieldDocPath ] = res.result().link;
    }
    else
    {
        j[ FieldDocColumns ] = headings_;
        j[ FieldDocData    ] = data.value();
    }

    return Result::succeeded();
}

/**
 */
nlohmann::json SectionContentTable::jsonConfig() const
{
    if (table_widget_)
        return table_widget_->jsonConfig();

    return nlohmann::json();
}

/**
 */
bool SectionContentTable::configure(const nlohmann::json& j)
{
    if (!table_widget_)
        return true; // no table widget, nothing to configure

    return table_widget_->configure(j);
}

/**
 * Exports a table cell to a certain format and returns it as a json object.
 * Main method for preparing cell data for export.
 * Allows fine-grained adjustments for specific export types,
 * e.g. generation of latex special statements.
 */
nlohmann::json SectionContentTable::exportContent(unsigned int row, 
                                                  unsigned int col,
                                                  ReportExportMode mode,
                                                  bool* ok) const
{
    if (ok)
        *ok = false;

    nlohmann::json j = rows_[ row ][ col ];

    auto style = cellStyle((int)row, (int)col);

    if (mode == ReportExportMode::Latex || 
        mode == ReportExportMode::LatexPDF)
    {
        std::string s;

        //cell export for latex
        if (SectionContentTable::cellShowsIcon(style))
        {
            // generate latex icon
            auto fn = SectionContentTable::cellIconFn(j);
            if (!fn.has_value())
                return {};

            // check if icon exists
            auto path = Utils::Files::getIconFilepath(fn.value().first, false);
            if (!Utils::Files::fileExists(path))
                return {};
            
            s = "\\includegraphics[width=" + std::to_string(LatexIconWidth_cm) + "cm]{" + fn.value().first + "}";
        }
        else if (cellShowsCheckBox(style))
        {
            // replace check state information with strings
            auto c = cellChecked(j);
            if (!c.has_value())
                return {};
            
            s = c.value() ? "Yes" : "No";
        }
        else
        {
            // default latex export => convert displayed data to latex string
            s = Utils::String::latexString(data((int)row, (int)col, Qt::DisplayRole).toString().toStdString());
        }

        // further format generated string depending on style flags

        //handle special fonts
        if (cellFontIsBold(style))
            s = "\\textbf{" + s + "}";
        else if (cellFontIsItalic(style))
            s = "\\textit{" + s + "}";
        //@TODO: strikeout text needs special latex package

        //handle section links
        const auto& slink = annotations_[ row ].section_link;
        if (!slink.empty() && col == latex_ref_column_)
        {
            // \hyperref[sec:marker2]{SecondSection}
            s = "\\hyperref[sec:" + slink + "]{" + s + "}";
        }

        //handle text coloring
        auto col_txt = cellTextColorLatex(style);
        if (!col_txt.empty())
            s = "\\textcolor{" + col_txt + "}{" + s + "}";

        //handle cell coloring
        auto col_bg = cellBGColorLatex(style);
        if (!col_bg.empty())
            s = "\\cellcolor{" + col_bg + "}" + s;

        j = s;
    }
    else if (mode == ReportExportMode::JSONFile || 
             mode == ReportExportMode::JSONBlob)
    {
        //cell export for json
        if (SectionContentTable::cellShowsIcon(style))
        {
            // replace internal icon information with replacement texts
            auto fn = SectionContentTable::cellIconFn(j);
            if (!fn.has_value())
                return {};

            j = fn.value().second;
        }
        else if (cellShowsCheckBox(style))
        {
            // replace check state information with strings
            auto c = cellChecked(j);
            if (!c.has_value())
                return {};
            
            j = c.value() ? "Yes" : "No";
        }

        //else = just return data directly as json
    }
    else
    {
        std::string s;

        //cell export for csv / default
        if (SectionContentTable::cellShowsIcon(style))
        {
            // replace internal icon information with replacement texts
            auto fn = SectionContentTable::cellIconFn(j);
            if (!fn.has_value())
                return {};

            s = fn.value().second;
        }
        else if (cellShowsCheckBox(style))
        {
            // replace check state information with strings
            auto c = cellChecked(j);
            if (!c.has_value())
                return {};
            
            s = c.value() ? "Yes" : "No";
        }
        else
        {
            // default text export => convert displayed data to string
            s = data((int)row, (int)col, Qt::DisplayRole).toString().toStdString();
        }

        j = s;
    }

    if (ok)
        *ok = true;

    return j;
}

/**
 */
nlohmann::json SectionContentTable::exportContent(unsigned int row,
                                                  ReportExportMode mode) const
{
    auto j_row = nlohmann::json::array();

    bool ok;
    for (unsigned int col = 0; col < num_columns_; ++col)
    {
        auto j_cell = exportContent(row, col, mode, &ok);
        if (!ok)
            return nlohmann::json();

        j_row.push_back(j_cell);
    }

    return j_row;
}

/**
 */
boost::optional<std::vector<nlohmann::json>> SectionContentTable::exportContent(ReportExportMode mode) const
{
    std::vector<nlohmann::json> data;

    size_t n = numRows();
    data.reserve(n);

    for (size_t r = 0; r < rows_.size(); ++r)
    {
        auto j_row = exportContent(r, mode);
        if (j_row.is_null())
            return boost::optional<std::vector<nlohmann::json>>();

        traced_assert(j_row.is_array());

        data.push_back(j_row);
    }

    return data;
}

/**
 */
nlohmann::json SectionContentTable::exportProxyContent(unsigned int row, 
                                                       unsigned int col,
                                                       ReportExportMode mode,
                                                       bool* ok) const
{
    auto w = getOrCreateTableWidget();
    traced_assert(w);

    auto proxy_model = w->proxyModel();
    traced_assert(proxy_model);

    auto index = proxy_model->index(row, col);
    traced_assert(index.isValid());

    auto index_src = proxy_model->mapToSource(index);
    traced_assert(index_src.isValid());

    return exportContent(index_src.row(), index_src.column(), mode, ok);
}

/**
 */
nlohmann::json SectionContentTable::exportProxyContent(unsigned int row,
                                                       ReportExportMode mode) const
{
    traced_assert(row >= 0);
    traced_assert(row < numProxyRows());
    traced_assert(row < numRows());

    auto j_row = nlohmann::json::array();

    bool cols_hidden = columnsHidden();

    bool ok;
    for (unsigned int col = 0; col < num_columns_; ++col)
    {
        if (cols_hidden && !columnVisible(col))
            continue;

        auto j_cell = exportProxyContent(row, col, mode, &ok);
        if (!ok)
            return nlohmann::json();

        j_row.push_back(j_cell);
    }

    return j_row;
}

/**
 */
boost::optional<std::vector<nlohmann::json>> SectionContentTable::exportProxyContent(ReportExportMode mode) const
{
    auto num_rows = numProxyRows();
    
    std::vector<nlohmann::json> data;
    data.reserve(num_rows);

    for (unsigned int r = 0; r < num_rows; ++r)
    {
        auto j_row = exportProxyContent(r, mode);
        if (j_row.is_null())
            return boost::optional<std::vector<nlohmann::json>>();

        traced_assert(j_row.is_array());

        data.push_back(j_row);
    }

    return data;
}

/**
 */
unsigned int SectionContentTable::numProxyRows() const
{
    return getOrCreateTableWidget()->proxyModel()->rowCount();
}

/**
 */
unsigned int SectionContentTable::numProxyColumns () const
{
    return num_columns_proxy_;
}

/***************************************************************************************************
 * SectionContentTableModel
 ***************************************************************************************************/

/**
 */
SectionContentTableModel::SectionContentTableModel(SectionContentTable* content_table, QObject* parent)
:   QAbstractItemModel(parent)
,   content_table_    (content_table)
{
    traced_assert(content_table_);
}

/**
 */
QVariant SectionContentTableModel::data(const QModelIndex& index, int role) const
{
    return content_table_->data(index, role);
}

/**
 */
QVariant SectionContentTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
    {
        traced_assert(section < (int)content_table_->numColumns());
        return content_table_->headings().at(section).c_str();
    }

    return QVariant();
}

/**
 */
QModelIndex SectionContentTableModel::index(int row, int column, const QModelIndex& parent) const
{
    return createIndex(row, column);
}

/**
 */
int SectionContentTableModel::rowCount(const QModelIndex& parent) const
{
    return (int)content_table_->numRows();
}

/**
 */
int SectionContentTableModel::columnCount(const QModelIndex& parent) const
{
    return (int)content_table_->numColumns();
}

/**
 */
QModelIndex SectionContentTableModel::parent(const QModelIndex& index) const
{
    return QModelIndex();
}

/**
 */
Qt::ItemFlags SectionContentTableModel::flags(const QModelIndex &index) const
{
    auto f = QAbstractItemModel::flags(index);
    f |= content_table_->flags(index);

    return f;
}

/**
 */
bool SectionContentTableModel::canFetchMore(const QModelIndex &parent) const
{
    return content_table_->isOnDemand() && !content_table_->isComplete();
}

/**
 */
void SectionContentTableModel::fetchMore(const QModelIndex &parent)
{
    if (!content_table_->isLoading())
        content_table_->loadOnDemandIfNeeded();
}

/**
 */
void SectionContentTableModel::executeAndReset(const std::function<void()>& func)
{
    if (!func)
        return;

    beginResetModel();

    func();

    endResetModel();
}

/***************************************************************************************************
 * SectionContentTableWidget
 ***************************************************************************************************/

const int SectionContentTableWidget::DoubleClickCheckIntervalMSecs = 300;

const std::string SectionContentTableWidget::FieldConfigSortColumn = SectionContentTable::FieldSortColumn;
const std::string SectionContentTableWidget::FieldConfigSortOrder  = SectionContentTable::FieldSortOrder;
const std::string SectionContentTableWidget::FieldConfigScrollPosV = "scroll_pos_v";
const std::string SectionContentTableWidget::FieldConfigScrollPosH = "scroll_pos_h";
const std::string SectionContentTableWidget::FieldColGroupStates   = "col_group_states";

/**
 */
SectionContentTableWidget::SectionContentTableWidget(SectionContentTable* content_table, 
                                                     bool show_unused,
                                                     int sort_column,
                                                     Qt::SortOrder sort_order,
                                                     QWidget* parent)
:   QWidget       (parent       )
,   content_table_(content_table)
,   sort_column_  (sort_column  )
,   sort_order_   (sort_order   )
{
    traced_assert(content_table_);

    QVBoxLayout* main_layout = new QVBoxLayout();
    setLayout(main_layout);

    QHBoxLayout* upper_layout  = new QHBoxLayout();

    upper_layout->addWidget(new QLabel(("Table: " + content_table_->name()).c_str()));
    upper_layout->addStretch();
    
    const auto& col_groups = content_table->columnGroups();

    if (!col_groups.empty())
    {
        col_group_toolbar_ = new QToolBar;
        col_group_toolbar_->setIconSize(UI_ICON_SIZE);

        upper_layout->addWidget(col_group_toolbar_);
    }

    upper_layout->addStretch();

    options_button_ = new QPushButton;
    options_button_->setStyleSheet("QPushButton::menu-indicator { image: none; }");
    options_button_->setIcon(Utils::Files::IconProvider::getIcon("edit.png"));
    options_button_->setFixedSize(UI_ICON_SIZE); 
    options_button_->setFlat(UI_ICON_BUTTON_FLAT);

    options_menu_.reset(new PopupMenu(options_button_));
    options_menu_->setPreShowCallback([ = ] () { this->updateOptionsMenu(); });

    upper_layout->addWidget(options_button_);

    main_layout->addLayout(upper_layout);

    table_view_ = new QTableView;

    model_ = new SectionContentTableModel(content_table, this);

    proxy_model_ = new TableQSortFilterProxyModel(this);
    proxy_model_->showUnused(show_unused);
    proxy_model_->setSourceModel(model_);

    table_view_->setModel(proxy_model_);

    if (sort_column >= 0)
    {
        table_view_->setSortingEnabled(true);
        table_view_->sortByColumn(sort_column, sort_order);
    }

    table_view_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_view_->setSelectionMode(QAbstractItemView::SingleSelection);
    table_view_->horizontalHeader()->resizeSections(QHeaderView::ResizeToContents);
    table_view_->horizontalHeader()->setSectionResizeMode(QHeaderView::Interactive);
    table_view_->setContextMenuPolicy(Qt::CustomContextMenu);
    table_view_->setWordWrap(true);
    table_view_->setFocusPolicy(Qt::StrongFocus);

    table_view_->reset();

    connect(table_view_, &QTableView::customContextMenuRequested,
            this, &SectionContentTableWidget::customContextMenu);
    connect(table_view_->selectionModel(), &QItemSelectionModel::currentRowChanged,
            this, &SectionContentTableWidget::clicked);
    connect(table_view_, &QTableView::pressed,
            this, &SectionContentTableWidget::clicked);
    connect(table_view_, &QTableView::doubleClicked,
            this, &SectionContentTableWidget::doubleClicked);

    connect(table_view_->verticalScrollBar(), &QScrollBar::rangeChanged,
            this, &SectionContentTableWidget::updateScrollBarV);
    connect(table_view_->horizontalScrollBar(), &QScrollBar::rangeChanged,
            this, &SectionContentTableWidget::updateScrollBarH);
    
    //    if (num_columns_ > 5)
    //        table_view_->horizontalHeader()->setMaximumSectionSize(150);
    
    table_view_->resizeColumnsToContents();
    table_view_->resizeRowsToContents();
    table_view_->verticalScrollBar()->installEventFilter(this);
    table_view_->horizontalScrollBar()->installEventFilter(this);

    main_layout->addWidget(table_view_);

    click_action_timer_.setSingleShot(true);
    click_action_timer_.setInterval(DoubleClickCheckIntervalMSecs);

    QObject::connect(&click_action_timer_, &QTimer::timeout, this, &SectionContentTableWidget::performClickAction);

    updateToolBar();
}

/**
 */
SectionContentTableWidget::~SectionContentTableWidget()
{
}

/**
 */
void SectionContentTableWidget::showUnused(bool show)
{
    auto func = [ this, show ] ()
    {
        this->proxy_model_->showUnused(show);
    };

    model_->executeAndReset(func);
}

/**
 */
void SectionContentTableWidget::resizeContent()
{
    table_view_->resizeColumnsToContents();
    table_view_->resizeRowsToContents();
}

/**
 */
void SectionContentTableWidget::updateColumnVisibility()
{
    for (unsigned int c = 0; c < content_table_->numColumns(); ++c)
        table_view_->setColumnHidden(c, !content_table_->columnVisible(c));
}

/**
 */
void SectionContentTableWidget::updateToolBar()
{
    if (!col_group_toolbar_)
        return;

    col_group_toolbar_->clear();

    const auto& col_groups = content_table_->columnGroups();

    for (const auto& col_group : col_groups)
    {
        auto action = col_group_toolbar_->addAction(QString::fromStdString(col_group.first));
        action->setCheckable(true);
        action->setChecked(col_group.second.enabled);

        std::string name          = col_group.first;
        auto        content_table = content_table_;

        auto cb = [ content_table, name ] (bool ok)
        {
            content_table->enableColumnGroup(name, ok);
        };

        connect(action, &QAction::toggled, cb);
    }
}

/**
 */
SectionContentTableModel* SectionContentTableWidget::itemModel()
{
    return model_;
}

/**
 */
const SectionContentTableModel* SectionContentTableWidget::itemModel() const
{
    return model_;
}

/**
 */
TableQSortFilterProxyModel* SectionContentTableWidget::proxyModel()
{
    return proxy_model_;
}

/**
 */
const TableQSortFilterProxyModel* SectionContentTableWidget::proxyModel() const
{
    return proxy_model_;
}

/**
 */
QTableView* SectionContentTableWidget::tableView()
{
    return table_view_;
}

/**
 */
const QTableView* SectionContentTableWidget::tableView() const
{
    return table_view_;
}

/**
 * Returns the original data row from the given proxy model's row.
 */
int SectionContentTableWidget::fromProxy(int proxy_row) const
{
    traced_assert(proxy_row < proxy_model_->rowCount());
    traced_assert(proxy_row < (int)content_table_->numRows());

    QModelIndex index = proxy_model_->index(proxy_row, 0);
    traced_assert(index.isValid());

    auto const source_index = proxy_model_->mapToSource(index);
    traced_assert(source_index.isValid());

    unsigned int row_index = source_index.row();

    return row_index;
}

/**
 */
void SectionContentTableWidget::clicked(const QModelIndex& index)
{
    loginf;

    if (!index.isValid())
    {
        loginf << "invalid index";
        return;
    }

    if (QApplication::mouseButtons() & Qt::RightButton)
    {
        loginf << "RMB click ignored";
        return;
    }

    auto const source_index = proxy_model_->mapToSource(index);
    traced_assert(source_index.isValid());

    traced_assert(source_index.row() >= 0);
    traced_assert(source_index.row() < (int)content_table_->numRows());

    last_clicked_row_index_ = source_index.row();

    //fire timer to perform delayed click action
    click_action_timer_.start();
}

/**
 */
void SectionContentTableWidget::performClickAction()
{
    loginf;

    //double click did not interrupt click action => perform
    if (!last_clicked_row_index_.has_value())
        return;

    unsigned int row_index = last_clicked_row_index_.value();
    last_clicked_row_index_.reset();

    //pass row to table and potentially trigger a viewable
    bool viewable_triggered = content_table_->clicked(row_index);

    //regain focus lost during potential reload
    if (viewable_triggered)
    {
        QApplication::processEvents();
        table_view_->setFocus();
    }
}

/**
 */
void SectionContentTableWidget::doubleClicked(const QModelIndex& index)
{
    loginf;

    //double click detected => interrupt any previously triggered click action
    click_action_timer_.stop();

    if (!index.isValid())
    {
        loginf << "invalid index";
        return;
    }

    auto const source_index = proxy_model_->mapToSource(index);
    traced_assert(source_index.isValid());

    traced_assert(source_index.row() >= 0);
    traced_assert(source_index.row() < (int)content_table_->numRows());

    loginf << "row " << source_index.row();

    unsigned int row_index = source_index.row();

    //pass row to table
    content_table_->doubleClicked(row_index);
}

/**
 */
void SectionContentTableWidget::customContextMenu(const QPoint& p)
{
    logdbg << "start";

    QModelIndex index = table_view_->indexAt(p);
    if (!index.isValid())
        return;

    auto const source_index = proxy_model_->mapToSource(index);
    traced_assert(source_index.isValid());

    loginf << "row " << index.row() << " src " << source_index.row();

    traced_assert(source_index.row() >= 0);
    traced_assert(source_index.row() < (int)content_table_->numRows());

    unsigned int row_index = source_index.row();

    auto pos = table_view_->viewport()->mapToGlobal(p);

    //pass row to table
    content_table_->customContextMenu(row_index, pos);
}

/**
 */
void SectionContentTableWidget::updateOptionsMenu()
{
    if (!options_menu_ || 
        !content_table_)
        return;

    options_menu_->clear();
    content_table_->addActionsToMenu(options_menu_.get());

    const auto& col_groups = content_table_->columnGroups();

    if (col_groups.size() > 0)
    {
        if (options_menu_->actions().count() > 0)
            options_menu_->addSeparator();

        auto column_menu = options_menu_->addMenu("Edit Columns");

        for (const auto& col_group : col_groups)
        {
            auto action = column_menu->addAction(QString::fromStdString(col_group.first));
            action->setCheckable(true);
            action->setChecked(col_group.second.enabled);

            std::string group_name    = col_group.first;
            auto        content_table = content_table_;

            auto cb = [ this, content_table, group_name ] (bool ok)
            {
                content_table->enableColumnGroup(group_name, ok);
                this->updateToolBar();
            };

            connect(action, &QAction::toggled, cb);
        }
    }
}

/**
 */
void SectionContentTableWidget::updateScrollBarV()
{
    //loginf;

    if (!content_table_->isComplete())
        return;

    //loginf << "applying new scroll limit: " 
    //       << "v = " << (scroll_pos_v_.has_value() ? scroll_pos_v_.value() : -1);

    //configure vertical scroll bar position
    if (scroll_pos_v_.has_value() && scroll_pos_v_.value() > 0 && table_view_->verticalScrollBar()->isVisible())
    {
        loginf << "applying v limit " << scroll_pos_v_.value();
        table_view_->verticalScrollBar()->setValue(scroll_pos_v_.value());
        scroll_pos_v_.reset();
    }
}

/**
 */
void SectionContentTableWidget::updateScrollBarH()
{
    //loginf;

    if (!content_table_->isComplete())
        return;

    //loginf << "applying new scroll limit: " 
    //      << "h = " << (scroll_pos_h_.has_value() ? scroll_pos_h_.value() : -1);

    //configure horizontal scroll bar position
    if (scroll_pos_h_.has_value() && scroll_pos_h_.value() > 0 && table_view_->horizontalScrollBar()->isVisible())
    {
        loginf << "applying h limit " << scroll_pos_h_.value();
        table_view_->horizontalScrollBar()->setValue(scroll_pos_h_.value());
        scroll_pos_h_.reset();
    }
}

/**
 */
int SectionContentTableWidget::scrollPosV() const
{
    if (table_view_ && table_view_->verticalScrollBar()->isVisible())
        return table_view_->verticalScrollBar()->value();
    
    return -1;
}

/**
 */
int SectionContentTableWidget::scrollPosH() const
{
    if (table_view_ && table_view_->horizontalScrollBar()->isVisible())
        return table_view_->horizontalScrollBar()->value();
    
    return -1;
}

/**
 */
bool SectionContentTableWidget::eventFilter(QObject *watched, QEvent *event)
{
    //react on showing the scroll bars
    if (table_view_ && watched == table_view_->verticalScrollBar() && event->type() == QEvent::Show)
        updateScrollBarV();
    if (table_view_ && watched == table_view_->horizontalScrollBar() && event->type() == QEvent::Show)
        updateScrollBarH();

    return false;
}

/**
 */
nlohmann::json SectionContentTableWidget::jsonConfig() const
{
    if (!table_view_)
        return nlohmann::json();

    nlohmann::json j;

    j[ FieldConfigSortOrder  ] = (sort_order_ == Qt::AscendingOrder ? "ascending" : "descending");
    j[ FieldConfigSortColumn ] = sort_column_;
    j[ FieldConfigScrollPosV ] = scrollPosV();
    j[ FieldConfigScrollPosH ] = scrollPosH();

    //store column group states
    if (col_group_toolbar_)
    {
        int na = col_group_toolbar_->actions().count();

        std::vector<bool> col_group_states(na);
        for (int i = 0; i < na; ++i)
            col_group_states[ i ] = col_group_toolbar_->actions().at(i)->isChecked();

        j[ FieldColGroupStates ] = col_group_states;
    }

    return j;
}

/**
 */
bool SectionContentTableWidget::configure(const nlohmann::json& j)
{
    if (!table_view_)
        return true; // no table view, nothing to configure

    if (!j.contains(FieldConfigSortOrder) ||
        !j.contains(FieldConfigSortColumn) ||
        !j.contains(FieldConfigScrollPosV) ||
        !j.contains(FieldConfigScrollPosH))
    {
        return false;
    }

    int           sort_column    = j[FieldConfigSortColumn];
    std::string   sort_order_str = j[FieldConfigSortOrder];   
    Qt::SortOrder sort_order     = (sort_order_str == "ascending" ? Qt::AscendingOrder : Qt::DescendingOrder);
    int           scroll_pos_v   = j[FieldConfigScrollPosV];
    int           scroll_pos_h   = j[FieldConfigScrollPosH];

    //configure sorting
    bool valid_sorting = sort_column >= 0 && sort_column < table_view_->model()->columnCount();
    table_view_->setSortingEnabled(valid_sorting);
    table_view_->sortByColumn(valid_sorting ? sort_column : -1, sort_order);

    //configure vertical scroll bar position for later usage
    if (scroll_pos_v >= 0)
        scroll_pos_v_ = scroll_pos_v;

    //configure horizontal scroll bar position for later usage
    if (scroll_pos_h >= 0)
        scroll_pos_h_ = scroll_pos_h;

    //restore column group states
    if (j.contains(FieldColGroupStates) && 
        j.at(FieldColGroupStates).is_array() &&
        col_group_toolbar_)
    {
        const auto& states = j.at(FieldColGroupStates);
        int n = (int)states.size();

        if (n != col_group_toolbar_->actions().count())
            return false;

        for (int i = 0; i < n; ++i)
        {
            std::string name = col_group_toolbar_->actions().at(i)->text().toStdString();
            if (!content_table_->hasColumnGroup(name))
                return false;

            content_table_->enableColumnGroup(name, states.at(i).get<bool>());
        }

        updateToolBar();
    }

    //try update the scroll bars in case they are already ready
    updateScrollBarV();
    updateScrollBarH();

    return true;
}

}
