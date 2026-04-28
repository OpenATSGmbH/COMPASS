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

#include "datasourceswidget.h"
#include "color_provider.h"
#include "db_context_manager.h"
#include "data_source.h"
#include "deletedatadialog.h"
#include "dbcontentdeletedbjob.h"

#include "compass.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "jobmanager.h"

#include "stringconv.h"
#include "number.h"
#include "files.h"
#include "timeconv.h"
#include "json.hpp"

#include <functional>
#include <set>

#include <QLabel>
#include <QCheckBox>
#include <QPainter>
#include <QPen>
#include <QPixmap>
#include "questiondialog.h"

#include <QMessageBox>
#include <QVBoxLayout>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QHeaderView>

namespace
{

/// Builds a color icon matching the Geographic View's rounded square texture:
/// a 14x14 rounded-square with a 1px darkGray border. An invalid color yields
/// a fully transparent 14x14 pixmap — the icon slot is reserved so rows stay
/// aligned, but nothing is drawn.
QIcon makeColorIcon(const QColor& color)
{
    constexpr int w = 14;
    constexpr int h = 14;
    constexpr qreal radius = 3.0;

    QPixmap pixmap(w, h);
    pixmap.fill(Qt::transparent);

    if (color.isValid())
    {
        QPainter p(&pixmap);
        p.setRenderHint(QPainter::Antialiasing, true);
        p.setPen(QPen(Qt::darkGray, 1, Qt::SolidLine));
        p.setBrush(QBrush(color));
        QRectF r(0.5, 0.5, w - 1.0, h - 1.0);
        p.drawRoundedRect(r, radius, radius);
        p.end();
    }

    return QIcon(pixmap);
}

/// Return the common effective color of `item`'s direct children. Children
/// with an invalid (no-color) effectiveColor are ignored; the result is
/// invalid only if the children with valid colors disagree, or if no child
/// has a valid color at all. This lets a subtree with some "colorless"
/// members (e.g. non-target-report DBContents) still propagate the shared
/// color of the rest up to the parent.
QColor commonChildEffectiveColor(const QTreeWidgetItem* item)
{
    QColor common;
    for (int i = 0; i < item->childCount(); ++i)
    {
        auto* child = dynamic_cast<const DataSourcesWidgetItem*>(item->child(i));
        if (!child)
            continue;
        const QColor cc = child->effectiveColor();
        if (!cc.isValid())
            continue; // colorless child — ignored
        if (!common.isValid())
            common = cc;
        else if (common != cc)
            return QColor(); // conflicting valid colors
    }
    return common; // invalid if no child contributed a color
}

} // anonymous

/**************************************************************************************************
 * DataSourcesWidgetItem
 **************************************************************************************************/

/**
 */
DataSourcesWidgetItem::DataSourcesWidgetItem(DataSourcesWidget* widget,
                                             DataSourcesWidgetItem* parent,
                                             Type type)
:   widget_(widget)
,   parent_(parent) 
,   type_  (type  )
{
    traced_assert(widget_);
}

/**
 */
void DataSourcesWidgetItem::setItemWidget(int column, QWidget* w)
{
    traced_assert(widget_);
    traced_assert(widget_->tree_widget_);

    widget_->tree_widget_->setItemWidget(this, column, w);
}

/**
 */
void DataSourcesWidgetItem::setTextIfChanged(int column, const QString& s)
{
    if (text(column) != s)
        setText(column, s);
}

/**
 */
void DataSourcesWidgetItem::setColorIconIfChanged(int column, const QColor& color)
{
    if (last_color_set_ && last_color_ == color)
        return;

    setIcon(column, makeColorIcon(color));
    last_color_     = color;
    last_color_set_ = true;
}

/**
 */
void DataSourcesWidgetItem::setCheckStateIfChanged(int column, Qt::CheckState state)
{
    if (checkState(column) != state)
        setCheckState(column, state);
}

/**
 */
void DataSourcesWidgetItem::setFlagsIfChanged(Qt::ItemFlags new_flags)
{
    if (flags() != new_flags)
        setFlags(new_flags);
}

/**************************************************************************************************
 * DataSourceTypeItem
 **************************************************************************************************/

/**
 */
DataSourceTypeItem::DataSourceTypeItem(DataSourcesWidget* widget,
                                       DataSourcesWidgetItem* parent)
:   DataSourcesWidgetItem(widget, parent, Type::DataSourceType)
{
}

/**
 */
bool DataSourceTypeItem::init(const std::string& ds_type)
{
    bool changes = false;
    if (!is_init_ || ds_type != ds_type_)
    {
        ds_type_ = ds_type;

        auto font_bold = font(0);
        font_bold.setBold(true);

        setCheckState(0, Qt::Checked);
        setText(0, QString::fromStdString(ds_type_));
        setFont(0, font_bold);

        is_init_ = true;
        changes  = true;
    }

    updateContent();

    return changes;
}

/**
 */
void DataSourceTypeItem::updateContent()
{
    traced_assert(is_init_);
    traced_assert(!ds_type_.empty());

    // The DSType is "enabled" only when at least one of its Data Sources has
    // inserted data in the current database — otherwise there is nothing to
    // act on. Disabled rows are greyed out and carry no checkbox.
    bool has_data = false;

    auto& ctx_man = widget_->ctxManager();
    if (ctx_man.hasActiveContext())
    {
        for (const auto& ds : ctx_man.activeContext().dataSources())
        {
            if (ds.dsType() == ds_type_ && ctx_man.hasNumInserted(ds.id()))
            {
                has_data = true;
                break;
            }
        }
    }

    if (has_data)
    {
        setFlagsIfChanged(flags() | Qt::ItemFlag::ItemIsEnabled | Qt::ItemFlag::ItemIsUserCheckable);
        setCheckStateIfChanged(0, widget_->getUseDSType(ds_type_) ? Qt::Checked : Qt::Unchecked);
    }
    else
    {
        // Non-interactive + greyed out; keep an Unchecked state so the
        // checkbox slot stays reserved and the row aligns with its siblings.
        setFlagsIfChanged(flags() & ~(Qt::ItemFlag::ItemIsEnabled | Qt::ItemFlag::ItemIsUserCheckable));
        setCheckStateIfChanged(0, Qt::Unchecked);
    }

    // Icon color propagates up from descendants — painted only when every
    // leaf under this DSType agrees on a single color (see effectiveColor).
    // Disabled rows (no data) show no color; makeColorIcon with an invalid
    // color yields a transparent 14x14 pixmap so the slot stays reserved and
    // rows remain aligned.
    setColorIconIfChanged(0, has_data ? effectiveColor() : QColor());
}

QColor DataSourceTypeItem::effectiveColor() const
{
    return commonChildEffectiveColor(this);
}

/**************************************************************************************************
 * DataSourceItem
 **************************************************************************************************/

/**
 */
DataSourceItem::DataSourceItem(DataSourcesWidget* widget,
                               DataSourcesWidgetItem* parent)
:   DataSourcesWidgetItem(widget, parent, Type::DataSource)
{
}

/**
 */
bool DataSourceItem::init(unsigned int ds_id)
{
    bool changes = false;
    if (!is_init_ || ds_id != ds_id_)
    {
        ds_id_ = ds_id;

        auto& ctx_man = widget_->ctxManager();
        traced_assert(ctx_man.hasDataSource(ds_id));

        ds_ = ctx_man.dataSource(ds_id);

        std::string ds_name = ds_->name();

        setCheckState(0, Qt::Checked);
        setText(0, QString::fromStdString(ds_name));

        if (!has_widget_)
        {
            auto w = createLinesWidget();
            setItemWidget(1, w);

            has_widget_ = true;

            for (auto lb : line_buttons_)
                lb->init(ds_id_);
        }

        is_init_ = true;
        changes  = true;
    }

    updateContent();

    return changes;
}

/**
 */
void DataSourceItem::updateContent()
{
    traced_assert(is_init_);
    traced_assert(has_widget_);

    // Empty Data Sources (no inserted data in the DB) are greyed out and
    // their checkbox is non-interactive, but the slot is reserved so the row
    // still aligns with its siblings.
    const bool has_data = widget_->ctxManager().hasNumInserted(ds_id_);
    if (has_data)
    {
        setFlagsIfChanged(flags() | Qt::ItemFlag::ItemIsEnabled | Qt::ItemFlag::ItemIsUserCheckable);
        setCheckStateIfChanged(0, widget_->getUseDS(ds_id_) ? Qt::Checked : Qt::Unchecked);
    }
    else
    {
        setFlagsIfChanged(flags() & ~(Qt::ItemFlag::ItemIsEnabled | Qt::ItemFlag::ItemIsUserCheckable));
        setCheckStateIfChanged(0, Qt::Unchecked);
    }

    // Disabled rows show no color icon; the slot stays reserved via the
    // transparent pixmap from makeColorIcon(QColor()).
    setColorIconIfChanged(0, has_data ? effectiveColor() : QColor());

    for (auto lb : line_buttons_)
        lb->updateContent();
}

QColor DataSourceItem::effectiveColor() const
{
    return commonChildEffectiveColor(this);
}

/**
 */
QWidget* DataSourceItem::createLinesWidget()
{
    QWidget* widget = new QWidget();
    widget->setContentsMargins(0, 0, 0, 0);

    QHBoxLayout* button_layout = new QHBoxLayout();
    button_layout->setContentsMargins(0, 1, 0, 1);

    std::string line_str;

    for (unsigned int cnt=0; cnt < 4; ++cnt)
    {
        auto w = new DataSourceLineButton(widget_, cnt, DataSourcesWidget::LineButtonSize);
        button_layout->addWidget(w);

        line_buttons_.push_back(w);
    }

    widget->setLayout(button_layout);

    return widget;
}

/**************************************************************************************************
 * DataSourceCountItem
 **************************************************************************************************/

/**
 */
DataSourceCountItem::DataSourceCountItem(DataSourcesWidget* widget,
                                         DataSourcesWidgetItem* parent)
:   DataSourcesWidgetItem(widget, parent, Type::DataSourceCount)
{
}

/**
*/
bool DataSourceCountItem::init(unsigned int ds_id,
                               const std::string& dbc_name)
{
    bool changes = false;
    if (!is_init_ || ds_id != ds_id_ || dbc_name != dbc_name_)
    {
        ds_id_    = ds_id;
        dbc_name_ = dbc_name;

        auto& ctx_man = widget_->ctxManager();
        traced_assert(ctx_man.hasDataSource(ds_id));

        ds_ = ctx_man.dataSource(ds_id);

        setText(0, QString::fromStdString(dbc_name));

        is_init_ = true;
        changes  = true;
    }

    updateContent();

    return changes;
}

/**
*/
void DataSourceCountItem::updateContent()
{
    traced_assert(is_init_);
    traced_assert(!dbc_name_.empty());

    auto& ctx_man = widget_->ctxManager();

    unsigned int num_inserted = ctx_man.numInserted(ds_id_, dbc_name_);
    unsigned int num_loaded = ctx_man.numLoaded(ds_id_, dbc_name_);

    setTextIfChanged(2, QString::number(num_loaded));
    setTextIfChanged(3, QString::number(num_inserted));

    // Leaf color depends on the active color mode; the group items above
    // propagate it up when every DBContent under a DS/DSType agrees. An
    // invalid color yields a transparent 14x14 placeholder that keeps rows
    // aligned.
    setColorIconIfChanged(0, effectiveColor());
}

QColor DataSourceCountItem::effectiveColor() const
{
    auto& ctx_man = widget_->ctxManager();
    if (!ctx_man.hasActiveContext())
        return QColor();

    // Non-target DBContents (status/service messages, etc.) never contribute
    // an icon color; they are also ignored by the parent's propagation rule.
    auto& dbcont_man = ctx_man.compass().dbContentManager();
    if (!dbcont_man.existsDBContent(dbc_name_) ||
        !dbcont_man.dbContent(dbc_name_).containsTargetReports())
        return QColor();

    const unsigned int mode = ctx_man.compass().colorMode();
    const auto& colors = ctx_man.activeContext().colors();

    switch (mode)
    {
        case 0: /* DSType */
        {
            if (!ds_) return QColor();
            const auto& palette = colors.ds_type_colors;
            auto it = palette.find(ds_->dsType());
            return (it != palette.end()) ? it->second : QColor();
        }
        case 1: /* DBContent */
        {
            const auto& palette = colors.dbcontent_colors;
            auto it = palette.find(dbc_name_);
            return (it != palette.end()) ? it->second : QColor();
        }
        case 2: /* DataSource */
        {
            return ds_ ? ds_->baseColor() : QColor();
        }
        case 3: /* DataSourceLine */
        default:
            // Line color is rendered on the line buttons, not as a tree icon.
            return QColor();
    }
}

/**************************************************************************************************
 * DataSourceLineButton
 **************************************************************************************************/

/**
 */
DataSourceLineButton::DataSourceLineButton(DataSourcesWidget* widget, 
                                           unsigned int line_id,
                                           unsigned int button_size_px)
:   widget_ (widget )
,   line_id_(line_id)
{
    traced_assert(widget_);

    line_str_ = "L" + std::to_string(line_id_ + 1);

    setText(QString::fromStdString(line_str_));

    setFixedSize(button_size_px, button_size_px);
    setCheckable(true);

    // initial style — updateContent() will re-apply once ds_ is known
    bool dark_mode = widget_->ctxManager().compass().darkMode();

    if (dark_mode)
    {
        setStyleSheet(" QPushButton:pressed { border: 3px outset white; } " \
                      " QPushButton:checked { border: 3px outset white; }");
        }
    else
    {
        setStyleSheet(" QPushButton:pressed { border: 3px outset; } " \
                      " QPushButton:checked { border: 3px outset; }");
    }

    setProperty("Line ID", line_id_);

    connect(this, &QPushButton::toggled, [ = ] (bool ok) { widget_->lineChanged(ds_id_, line_id_, ok); });

    QSizePolicy sp_retain = widget->sizePolicy();
    sp_retain.setRetainSizeWhenHidden(true);
    setSizePolicy(sp_retain);
}

/**
 */
bool DataSourceLineButton::init(unsigned int ds_id)
{
    bool changes = false;
    if (!is_init_ || ds_id != ds_id_)
    {
        ds_id_ = ds_id;

        auto& ctx_man = widget_->ctxManager();
        traced_assert(ctx_man.hasDataSource(ds_id_));

        ds_ = ctx_man.dataSource(ds_id_);

        is_init_ = true;
        changes  = true;
    }

    updateContent();

    return changes;
}

/**
 */
void DataSourceLineButton::updateContent()
{
    traced_assert(is_init_);

    auto& ctx_man = widget_->ctxManager();

    AppMode app_mode = ctx_man.compass().appMode();

    bool live_mode = app_mode == AppMode::LivePaused || app_mode == AppMode::LiveRunning;
    bool dark_mode = ctx_man.compass().darkMode();

    auto apply_stylesheet = [this](const QString& css)
    {
        if (last_stylesheet_ != css)
        {
            setStyleSheet(css);
            last_stylesheet_ = css;
        }
    };

    auto apply_hidden = [this](bool h)
    {
        if (!last_hidden_set_ || last_hidden_ != h)
        {
            setHidden(h);
            last_hidden_     = h;
            last_hidden_set_ = true;
        }
    };

    auto apply_disabled = [this](bool d)
    {
        if (!last_disabled_set_ || last_disabled_ != d)
        {
            setDisabled(d);
            last_disabled_     = d;
            last_disabled_set_ = true;
        }
    };

    auto apply_checked = [this](bool c)
    {
        if (!last_checked_set_ || last_checked_ != c)
        {
            setChecked(c);
            last_checked_     = c;
            last_checked_set_ = true;
        }
    };

    auto apply_palette_color = [this](const QColor& col)
    {
        if (last_auto_fill_bg_ && last_palette_color_ == col)
            return;
        QPalette pal = palette();
        pal.setColor(QPalette::Button, col);
        setAutoFillBackground(true);
        setPalette(pal);
        update();
        last_palette_color_ = col;
        last_auto_fill_bg_  = true;
    };

    // Color Mode: DataSource + Line -> paint a 3px border in the corresponding line color
    unsigned int color_mode = ctx_man.compass().colorMode();
    if (color_mode == 3 /*DataSourceLine*/ && ds_ && ds_->lineColor(line_id_).isValid())
    {
        QString color_name = ds_->lineColor(line_id_).name();
        apply_stylesheet(QString(" QPushButton { border: 3px solid %1; } "
                                 " QPushButton:pressed { border: 3px outset %1; } "
                                 " QPushButton:checked { border: 3px outset %1; }").arg(color_name));
    }
    else
    {
        if (dark_mode)
        {
            apply_stylesheet(" QPushButton:pressed { border: 3px outset white; } "
                             " QPushButton:checked { border: 3px outset white; }");
        }
        else
        {
            apply_stylesheet(" QPushButton:pressed { border: 3px outset; } "
                             " QPushButton:checked { border: 3px outset; }");
        }
    }

    if (live_mode)
    {
        auto net_lines = ctx_man.getNetworkLines();

        bool hidden = !net_lines.count(ds_id_) || !net_lines.at(ds_id_).count(line_str_); // hide if not defined

        apply_hidden(hidden);

        if (!hidden)
        {
            bool disabled = app_mode == AppMode::LivePaused ?
                (ctx_man.numInsertedPerLine(ds_id_, "").count(line_id_) == 0) : false;
            apply_disabled(disabled);

            if (disabled)
            {
                apply_checked(false);
                apply_palette_color(QColor(dark_mode ? Qt::darkGray : Qt::lightGray));
            }
            else
            {
                apply_checked(widget_->getUseDSLine(ds_id_, line_id_));

                boost::posix_time::ptime current_time = Utils::Time::currentUTCTime();

                auto max_ts = ctx_man.maxTimestamp(ds_id_, line_id_);
                bool has_live = !max_ts.is_not_a_date_time() &&
                    Utils::Time::partialSeconds(current_time - max_ts) < 30.0;

                if (has_live)
                    apply_palette_color(QColor(dark_mode ? Qt::darkGreen : Qt::green));
                else
                    apply_palette_color(QColor(dark_mode ? Qt::darkGray : Qt::lightGray));
            }
        }
    }
    else
    {
        std::map<unsigned int, unsigned int> inserted_lines = ctx_man.numInsertedLinesMap(ds_id_);

        apply_checked(widget_->getUseDSLine(ds_id_, line_id_));
        apply_hidden(!inserted_lines.count(line_id_)); // hide if no data
    }
}

/**************************************************************************************************
 * DataSourcesWidget
 **************************************************************************************************/

const int DataSourcesWidget::LineButtonSize = 25;

/**
 */
DataSourcesWidget::DataSourcesWidget(bool can_show_counts, context::DBContextManager& ctx_man)
:   can_show_counts_(can_show_counts), ctx_man_(ctx_man)
{
    createUI();
}

/**
 */
DataSourcesWidget::~DataSourcesWidget() = default;


/**
 */
void DataSourcesWidget::createUI()
{
    QFont font_bold;
    font_bold.setBold(true);

    QVBoxLayout* main_layout = new QVBoxLayout();
    setLayout(main_layout);

    // buttons
    QHBoxLayout* button_layout = new QHBoxLayout();

    main_layout->addLayout(button_layout);

    // tree widget

    top_layout_ = new QHBoxLayout();
    top_layout_->setContentsMargins(0, 0, 0, 0);

    tree_widget_ = new QTreeWidget;

    QStringList header_labels;
    header_labels << "Name";
    header_labels << "Lines";

    if (can_show_counts_)
    {
        header_labels << "Loaded";
        header_labels << "Count";
    }

    tree_widget_->setColumnCount(header_labels.size());
    tree_widget_->setHeaderLabels(header_labels);
    tree_widget_->header()->setSectionResizeMode(QHeaderView::ResizeMode::ResizeToContents);

    connect(tree_widget_, &QTreeWidget::itemChanged, this, &DataSourcesWidget::itemChanged);
    connect(tree_widget_, &QTreeWidget::itemSelectionChanged,
            this, &DataSourcesWidget::onItemSelectionChanged);

    tree_widget_->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(tree_widget_, &QTreeWidget::customContextMenuRequested,
            this, &DataSourcesWidget::showContextMenuSlot);

    top_layout_->addWidget(tree_widget_);

    main_layout->addLayout(top_layout_);

    // update
    updateContent(true);
}

void DataSourcesWidget::disableSelection()
{
    traced_assert(tree_widget_);
    tree_widget_->setSelectionMode(QAbstractItemView::NoSelection);
    tree_widget_->setFocusPolicy(Qt::NoFocus);
}

void DataSourcesWidget::addActionsToConfigMenu(QMenu* menu)
{
    QAction* sel_dstyp_action = menu->addAction("Select All DSTypes");
    connect(sel_dstyp_action, &QAction::triggered, this, &DataSourcesWidget::selectAllDSTypes);

    QAction* desel_dstyp_action = menu->addAction("Deselect All DSTypes");
    connect(desel_dstyp_action, &QAction::triggered, this, &DataSourcesWidget::deselectAllDSTypes);

    menu->addSeparator();

    QMenu* select_ds = menu->addMenu("Select Data Sources");

    QAction* sel_ds_action = select_ds->addAction("All");
    connect(sel_ds_action, &QAction::triggered, this, &DataSourcesWidget::selectAllDataSources);

    for (const auto& ds_type_it : context::DataSource::dsTypeStrings())
    {
        QAction* action = select_ds->addAction(("From "+ds_type_it).c_str());
        action->setProperty("ds_type", ds_type_it.c_str());
        connect(action, &QAction::triggered, this, &DataSourcesWidget::selectDSTypeSpecificDataSources);
    }

    QMenu* deselect_ds = menu->addMenu("Deselect Data Sources");

    QAction* desel_ds_action = deselect_ds->addAction("All");
    connect(desel_ds_action, &QAction::triggered, this, &DataSourcesWidget::deselectAllDataSources);

    for (const auto& ds_type_it : context::DataSource::dsTypeStrings())
    {
        QAction* action = deselect_ds->addAction(("From "+ds_type_it).c_str());
        action->setProperty("ds_type", ds_type_it.c_str());
        connect(action, &QAction::triggered, this, &DataSourcesWidget::deselectDSTypeSpecificDataSources);
    }

    menu->addSeparator();

    QMenu* set_lines = menu->addMenu("Set Line");

    QAction* desel_line_action = set_lines->addAction("Deselect All");
    connect(desel_line_action, &QAction::triggered, this, &DataSourcesWidget::deselectAllLines);

    for (unsigned int cnt=0; cnt < 4; ++cnt)
    {
        QAction* desel_line_action = set_lines->addAction(("Select " + Utils::String::lineStrFrom(cnt)).c_str());
        desel_line_action->setProperty("line_id", cnt);
        connect(desel_line_action, &QAction::triggered, this, &DataSourcesWidget::selectSpecificLines);
    }

    menu->addSeparator();

    QAction* show_cnt_action = menu->addAction("Toggle Show Counts");
    connect(show_cnt_action, &QAction::triggered, this, &DataSourcesWidget::toogleShowCounts);

    menu->addSeparator();

    QAction* delete_data_action = menu->addAction("Delete Data...");
    connect(delete_data_action, &QAction::triggered, this, &DataSourcesWidget::deleteDataSlot);
}

/**
 */
void DataSourcesWidget::clear()
{
    tree_widget_->clear();
}

/**
 */
int DataSourcesWidget::generateContent(bool force_rebuild)
{
    logdbg;

    tree_widget_->blockSignals(true);

    //clear everything to force a complete rebuild
    if (force_rebuild)
        clear();

    const auto& data_source_types = context::DataSource::dsTypeStrings();

    //create needed items
    int n    = (int)data_source_types.size();
    int ncur = tree_widget_->topLevelItemCount();

    if (n > ncur)
    {
        //add more items if needed
        for (int i = ncur; i < n; ++i)
        {
            auto dstype_item = new DataSourceTypeItem(this, nullptr);
            tree_widget_->addTopLevelItem(dstype_item);
        }
    }
    else if (n < ncur)
    {
        //remove unneeded items
        while (tree_widget_->topLevelItemCount() > n)
        {
            auto item = tree_widget_->takeTopLevelItem(0);
            delete item;
        }
    }

    //configure data source type items
    unsigned int cnt = 0;
    int changes = 0;
    for (const auto& ds_type_name : data_source_types)
    {
        logdbg << "type " << ds_type_name << " cnt " << cnt;

        auto item = dynamic_cast<DataSourceTypeItem*>(tree_widget_->topLevelItem(cnt));
        traced_assert(item);

        

        changes += generateDataSourceType(item, ds_type_name);

        ++cnt;
    }

    tree_widget_->blockSignals(false);

    // expandAll is expensive — skip when no structural change. Existing
    // expansion state is preserved across plain count updates.
    if (force_rebuild || changes > 0)
        tree_widget_->expandAll();

    //updateAdditionalInfo();

    return changes;
}

/**
 */
int DataSourcesWidget::generateDataSourceType(DataSourceTypeItem* item,
                                              const std::string& ds_type_name)
{
    //init item
    int changes = item->init(ds_type_name) ? 1 : 0;

    //create needed items
    std::vector<const context::DataSource*> matching_ds;
    if (ctx_man_.hasActiveContext())
    {
        for (const auto& ds : ctx_man_.activeContext().dataSources())
        {
            if (ds.dsType() == ds_type_name)
                matching_ds.push_back(&ds);
        }
    }

    int ncur = item->childCount();
    int n    = (int)matching_ds.size();

    if (n > ncur)
    {
        //add more items if needed
        for (int i = ncur; i < n; ++i)
        {
            auto ds_item = new DataSourceItem(this, item);
            item->addChild(ds_item);
        }
    }
    else if (n < ncur)
    {
        //remove unneeded items
        while (item->childCount() > n)
        {
            auto child = item->child(0);
            item->removeChild(child);
            delete child;
        }
    }

    //configure data source items
    for (int cnt = 0; cnt < n; ++cnt)
    {
        auto ds_item = dynamic_cast<DataSourceItem*>(item->child(cnt));
        traced_assert(ds_item);

        changes += generateDataSource(ds_item, item, *matching_ds[cnt]);
    }

    return changes;
}

/**
 */
int DataSourcesWidget::generateDataSource(DataSourceItem* item,
                                          DataSourcesWidgetItem* parent_item,
                                          const context::DataSource& data_source)
{
    unsigned int ds_id   = data_source.id();
    std::string  ds_name = data_source.name();

    logdbg << "create '" << data_source.dsType() << "' '" << ds_name << "'";

    //init item
    int changes = item->init(ds_id) ? 1 : 0;

    //handle count items
    bool show_counts = can_show_counts_ && getShowCounts();
    if (!show_counts)
    {
        //no counts shown => remove any existing children
        while (item->childCount() > 0)
        {
            auto child = item->child(0);
            item->removeChild(child);
            delete child;
        }
    }
    else
    {
        // counts shown => build dbcontent count map from context manager
        auto& dbcont_man = ctx_man_.compass().dbContentManager();
        std::map<std::string, unsigned int> count_map;
        for (auto it = dbcont_man.begin(); it != dbcont_man.end(); ++it)
        {
            unsigned int cnt_val = ctx_man_.numInserted(ds_id, it->first);
            if (cnt_val > 0)
                count_map[it->first] = cnt_val;
        }

        int n    = (int)count_map.size();
        int ncur = item->childCount();

        if (n > ncur)
        {
            //add more items if needed
            for (int i = ncur; i < n; ++i)
            {
                auto ds_cnt_item = new DataSourceCountItem(this, item);
                item->addChild(ds_cnt_item);
            }
        }
        else if (n < ncur)
        {
            //remove unneeded items
            while (item->childCount() > n)
            {
                auto child = item->child(0);
                item->removeChild(child);
                delete child;
            }
        }

        //configure count items
        int cnt = 0;
        for (auto& cnt_it : count_map)
        {
            auto ds_cnt_item = dynamic_cast<DataSourceCountItem*>(item->child(cnt));
            traced_assert(ds_cnt_item);

            changes += generateDataSourceCount(ds_cnt_item, item, data_source, cnt_it.first);

            ++cnt;
        }
    }

    return changes;
}

/**
 */
int DataSourcesWidget::generateDataSourceCount(DataSourceCountItem* item,
                                               DataSourcesWidgetItem* parent_item,
                                               const context::DataSource& data_source,
                                               const std::string& dbc_name)
{
    //init item
    int changes = item->init(data_source.id(), dbc_name) ? 1 : 0;

    return changes;
}

/**
 */
void DataSourcesWidget::updateContent(bool recreate_required)
{
    logdbg << "recreate_required " << recreate_required;

    int changes = generateContent(recreate_required);

    // show/hide count columns based on toggle
    if (can_show_counts_)
    {
        bool show = getShowCounts();
        tree_widget_->setColumnHidden(2, !show);
        tree_widget_->setColumnHidden(3, !show);
    }

    logdbg << "update generated " << changes << " change(s)";
}

/**
 * Handle tree widget selection changes
 */
void DataSourcesWidget::onItemSelectionChanged()
{
    QList<QTreeWidgetItem*> selected_items = tree_widget_->selectedItems();
    if (selected_items.isEmpty())
        return;
    
    // Get the first selected item
    QTreeWidgetItem* item = selected_items.first();
    auto ds_item = dynamic_cast<DataSourcesWidgetItem*>(item);
    
    if (!ds_item || !ds_item->isInit())
        return;
    
    if (ds_item->type() == DataSourcesWidgetItem::Type::DataSource)
    {
        auto data_source_item = dynamic_cast<DataSourceItem*>(ds_item);
        loginf << "selected data source " << data_source_item->dsID();
        // React to data source selection

        emit dataSourceSelectedSignal(data_source_item->dsID());
    }
}

/**
 */
void DataSourcesWidget::itemChanged(QTreeWidgetItem *item, int column)
{
    auto w_item = dynamic_cast<DataSourcesWidgetItem*>(item);
    if (!w_item || !w_item->isInit())
        return;

    //react on item changes
    if (w_item->type() == DataSourcesWidgetItem::Type::DataSourceType)
    {   
        auto ds_type_item = dynamic_cast<DataSourceTypeItem*>(w_item);
        traced_assert(ds_type_item);

        if (column == 0)
        {
            bool load = ds_type_item->checkState(0) == Qt::Checked;
            setUseDSType(ds_type_item->dsType(), load);

            loginf << "ds_type " << ds_type_item->dsType() << " load " << load;
        }
    }
    else if (w_item->type() == DataSourcesWidgetItem::Type::DataSource)
    {   
        auto ds_item = dynamic_cast<DataSourceItem*>(w_item);
        traced_assert(ds_item);

        if (column == 0)
        {
            bool load = ds_item->checkState(0) == Qt::Checked;
            setUseDS(ds_item->dsID(), load);

            loginf << "ds_id " << ds_item->dsID() << " load " << load;
        }
    }
    else if (w_item->type() == DataSourcesWidgetItem::Type::DataSourceCount)
    {   
        auto ds_cnt_item = dynamic_cast<DataSourceCountItem*>(w_item);
        traced_assert(ds_cnt_item);

        //nothing to do yet
    }
}

/**
 */
void DataSourcesWidget::lineChanged(unsigned int ds_id, unsigned int ds_line, bool use)
{
    logdbg << "ds_id " << ds_id << " line " << ds_line << " use " << use;

    setUseDSLine(ds_id, ds_line, use);
}

namespace
{
    void updateContentRecursive(QTreeWidgetItem* item)
    {
        auto w_item = dynamic_cast<DataSourcesWidgetItem*>(item);
        if (w_item)
            w_item->updateContent();

        for (int i = 0; i < item->childCount(); ++i)
            updateContentRecursive(item->child(i));
    }
}

/**
 * Only updates all contents.
 */
void DataSourcesWidget::updateAllContent()
{
    for (int i = 0; i < tree_widget_->topLevelItemCount(); ++i)
        updateContentRecursive(tree_widget_->topLevelItem(i));

}

/**
 */
void DataSourcesWidget::setUseDSType(const std::string& ds_type_name, bool use)
{
    ctx_man_.dsTypeLoadingWanted(ds_type_name, use);
}

/**
 */
bool DataSourcesWidget::getUseDSType(const std::string& ds_type_name) const
{
    return ctx_man_.dsTypeLoadingWanted(ds_type_name);
}

/**
 */
void DataSourcesWidget::setUseDS(unsigned int ds_id, bool use)
{
    ctx_man_.loadingWanted(ds_id, use);
}

/**
 */
bool DataSourcesWidget::getUseDS(unsigned int ds_id) const
{
    return ctx_man_.loadingWanted(ds_id);
}

/**
 */
void DataSourcesWidget::setUseDSLine(unsigned int ds_id, unsigned int ds_line, bool use)
{
    ctx_man_.lineLoadingWanted(ds_id, ds_line, use);
}

/**
 */
bool DataSourcesWidget::getUseDSLine(unsigned int ds_id, unsigned int ds_line) const
{
    return ctx_man_.lineLoadingWanted(ds_id, ds_line);
}

/**
 */
void DataSourcesWidget::setShowCounts(bool show) const
{
    ctx_man_.compass().dbContentManager().showDataCounts(show);
}

/**
 */
bool DataSourcesWidget::getShowCounts() const
{
    return ctx_man_.compass().dbContentManager().showDataCounts();
}

/**
 */
void DataSourcesWidget::selectAllDSTypes()
{
    loginf;

    for (auto& ds_type_name : context::DataSource::dsTypeStrings())
        setUseDSType(ds_type_name, true);

    updateContent();
}

/**
 */
void DataSourcesWidget::deselectAllDSTypes()
{
    loginf;

    for (auto& ds_type_name : context::DataSource::dsTypeStrings())
        setUseDSType(ds_type_name, false);

    updateContent();
}

/**
 */
void DataSourcesWidget::selectAllDataSources()
{
    loginf;

    if (ctx_man_.hasActiveContext())
        for (const auto& ds : ctx_man_.activeContext().dataSources())
            setUseDS(ds.id(), true);

    updateContent();
}

/**
 */
void DataSourcesWidget::deselectAllDataSources()
{
    loginf;

    if (ctx_man_.hasActiveContext())
        for (const auto& ds : ctx_man_.activeContext().dataSources())
            setUseDS(ds.id(), false);

    updateContent();
}

/**
 */
void DataSourcesWidget::selectDSTypeSpecificDataSources()
{
    QAction* action = dynamic_cast<QAction*>(sender());
    traced_assert(action);

    std::string ds_type = action->property("ds_type").toString().toStdString();

    loginf << "ds_type '" << ds_type << "'";

    if (ctx_man_.hasActiveContext())
        for (const auto& ds : ctx_man_.activeContext().dataSources())
            if (ds.dsType() == ds_type)
                setUseDS(ds.id(), true);

    updateContent();
}

/**
 */
void DataSourcesWidget::deselectDSTypeSpecificDataSources()
{
    QAction* action = dynamic_cast<QAction*>(sender());
    traced_assert(action);

    std::string ds_type = action->property("ds_type").toString().toStdString();

    loginf << "ds_type '" << ds_type << "'";

    if (ctx_man_.hasActiveContext())
        for (const auto& ds : ctx_man_.activeContext().dataSources())
            if (ds.dsType() == ds_type)
                setUseDS(ds.id(), false);

    updateContent();
}

/**
 */
void DataSourcesWidget::deselectAllLines()
{
    loginf;

    if (ctx_man_.hasActiveContext())
        for (const auto& ds : ctx_man_.activeContext().dataSources())
            for (int line = 0; line < 4; ++line)
                setUseDSLine(ds.id(), line, false);

    updateContent();
}

/**
 */
void DataSourcesWidget::selectSpecificLines()
{
    QAction* action = dynamic_cast<QAction*>(sender());
    traced_assert(action);

    unsigned int line_id = action->property("line_id").toUInt();

    loginf << "line_id " << line_id;

    if (ctx_man_.hasActiveContext())
        for (const auto& ds : ctx_man_.activeContext().dataSources())
            setUseDSLine(ds.id(), line_id, true);

    updateContent();
}

/**
 */
void DataSourcesWidget::toogleShowCounts()
{
    loginf;

    setShowCounts(!getShowCounts());

    updateContent();
}

void DataSourcesWidget::setAllCheckboxes(bool select)
{
    loginf << "select " << select;

    for (auto& ds_type_name : context::DataSource::dsTypeStrings())
        setUseDSType(ds_type_name, select);

    if (ctx_man_.hasActiveContext())
        for (const auto& ds : ctx_man_.activeContext().dataSources())
            setUseDS(ds.id(), select);

    updateContent();
}

void DataSourcesWidget::deselectOtherDSTypes(const std::string& keep)
{
    loginf << "keep '" << keep << "'";

    for (auto& ds_type_name : context::DataSource::dsTypeStrings())
        if (ds_type_name != keep)
            setUseDSType(ds_type_name, false);

    updateContent();
}

void DataSourcesWidget::setAllChildrenOfDSType(const std::string& ds_type, bool select)
{
    loginf << "ds_type '" << ds_type << "' select " << select;

    if (ctx_man_.hasActiveContext())
        for (const auto& ds : ctx_man_.activeContext().dataSources())
            if (ds.dsType() == ds_type)
                setUseDS(ds.id(), select);

    updateContent();
}

void DataSourcesWidget::deselectOtherDataSources(unsigned int keep_ds_id)
{
    loginf << "keep " << keep_ds_id;

    if (ctx_man_.hasActiveContext())
        for (const auto& ds : ctx_man_.activeContext().dataSources())
            if (ds.id() != keep_ds_id)
                setUseDS(ds.id(), false);

    updateContent();
}

void DataSourcesWidget::expandSubtree(QTreeWidgetItem* item, bool expand)
{
    if (!item) return;
    item->setExpanded(expand);
    for (int i = 0; i < item->childCount(); ++i)
        expandSubtree(item->child(i), expand);
}

void DataSourcesWidget::showContextMenuSlot(const QPoint& pos)
{
    // No database open → nothing to act on; suppress the context menu entirely.
    if (!ctx_man_.compass().dbOpened())
        return;

    QTreeWidgetItem* raw   = tree_widget_->itemAt(pos);
    auto*            item  = dynamic_cast<DataSourcesWidgetItem*>(raw);

    QMenu menu(this);
    menu.setToolTipsVisible(true);

    auto add = [&](const QString& text, const QString& tip, std::function<void()> fn)
    {
        QAction* a = menu.addAction(text);
        a->setToolTip(tip);
        connect(a, &QAction::triggered, this, [fn]{ fn(); });
    };

    // --- Per-item section (only when clicked on a row) ---
    if (item)
    {
        switch (item->type())
        {
            case DataSourcesWidgetItem::Type::DataSourceType:
            {
                auto*             ti      = static_cast<DataSourceTypeItem*>(item);
                const std::string ds_type = ti->dsType();
                QTreeWidgetItem*  raw_ti  = raw;

                // Skip the per-item section entirely for empty DSTypes —
                // they have no data and their row carries no interactive
                // state, so only the global "All" actions apply.
                bool dstype_has_data = false;
                if (ctx_man_.hasActiveContext())
                    for (const auto& ds : ctx_man_.activeContext().dataSources())
                        if (ds.dsType() == ds_type && ctx_man_.hasNumInserted(ds.id()))
                        { dstype_has_data = true; break; }
                if (!dstype_has_data)
                    break;

                menu.addSection(QString("DSType \"%1\"")
                                .arg(QString::fromStdString(ds_type)));
                add("Deselect Other DSTypes",
                    "Keep this DSType selected; deselect all other DSTypes",
                    [this, ds_type]{ deselectOtherDSTypes(ds_type); });
                add("Select All Children",
                    "Check every Data Source under this DSType",
                    [this, ds_type]{ setAllChildrenOfDSType(ds_type, true); });
                add("Deselect All Children",
                    "Uncheck every Data Source under this DSType",
                    [this, ds_type]{ setAllChildrenOfDSType(ds_type, false); });
                add("Expand",
                    "Expand this DSType's sub-tree",
                    [this, raw_ti]{ expandSubtree(raw_ti, true); });
                add("Collapse",
                    "Collapse this DSType's sub-tree",
                    [this, raw_ti]{ expandSubtree(raw_ti, false); });

                // Only offer Delete if at least one DS of this DSType has
                // data in the database — otherwise there is nothing to delete.
                bool any_has_data = false;
                if (ctx_man_.hasActiveContext())
                {
                    for (const auto& ds : ctx_man_.activeContext().dataSources())
                    {
                        if (ds.dsType() == ds_type && ctx_man_.hasNumInserted(ds.id()))
                        {
                            any_has_data = true;
                            break;
                        }
                    }
                }
                if (any_has_data)
                {
                    add("Delete…",
                        "Open the delete dialog with this DSType and its Data Sources preselected",
                        [this, ds_type]{ deleteForDSType(ds_type); });
                }
                break;
            }
            case DataSourcesWidgetItem::Type::DataSource:
            case DataSourcesWidgetItem::Type::DataSourceCount:
            {
                // A DBContent row uses the same menu as its parent Data Source:
                // all actions (selection, delete) operate on the containing DS.
                unsigned int ds_id;
                QString      ds_name;
                if (item->type() == DataSourcesWidgetItem::Type::DataSource)
                {
                    auto* di = static_cast<DataSourceItem*>(item);
                    ds_id    = di->dsID();
                    ds_name  = raw->text(0);
                }
                else
                {
                    auto* ci = static_cast<DataSourceCountItem*>(item);
                    ds_id    = ci->dsID();
                    ds_name  = raw->parent() ? raw->parent()->text(0) : QString();
                }

                // Empty Data Source: no per-item actions make sense, fall
                // through to the global "All" section only.
                if (!ctx_man_.hasNumInserted(ds_id))
                    break;

                const std::string ds_type =
                    ctx_man_.hasDataSource(ds_id) ? ctx_man_.dataSource(ds_id)->dsType()
                                                  : std::string();

                menu.addSection(QString("Data Source \"%1\"").arg(ds_name));
                add("Deselect Other Data Sources",
                    "Keep this Data Source selected; deselect all other Data Sources",
                    [this, ds_id]{ deselectOtherDataSources(ds_id); });
                if (!ds_type.empty())
                {
                    add(QString("Select All %1 Data Sources")
                            .arg(QString::fromStdString(ds_type)),
                        QString("Check every Data Source of type \"%1\"")
                            .arg(QString::fromStdString(ds_type)),
                        [this, ds_type]{ setAllChildrenOfDSType(ds_type, true); });
                }
                if (ctx_man_.hasNumInserted(ds_id))
                {
                    add("Delete…",
                        "Open the delete dialog with this Data Source preselected",
                        [this, ds_id]{ deleteForDataSource(ds_id); });
                }
                break;
            }
        }
    }

    // --- Global section (always) — mirrors the top-right "Data Sources"
    //     menu (addActionsToConfigMenu) with the same labels and submenus;
    //     plus a few context-menu-only conveniences (Select All, Expand All).
    menu.addSection("All");

    auto addTo = [this](QMenu* m, const QString& text, const QString& tip,
                        std::function<void()> fn)
    {
        QAction* a = m->addAction(text);
        a->setToolTip(tip);
        connect(a, &QAction::triggered, this, [fn]{ fn(); });
    };

    add("Select All",
        "Check every DSType and Data Source row (lines stay unchanged)",
        [this]{ setAllCheckboxes(true); });
    add("Deselect All",
        "Uncheck every DSType and Data Source row (lines stay unchanged)",
        [this]{ setAllCheckboxes(false); });
    menu.addSeparator();

    add("Select All DSTypes",
        "Select every DSType",
        [this]{ selectAllDSTypes(); });
    add("Deselect All DSTypes",
        "Deselect every DSType",
        [this]{ deselectAllDSTypes(); });
    menu.addSeparator();

    // Which DSTypes actually have at least one Data Source with data?
    std::set<std::string> dstypes_with_data;
    if (ctx_man_.hasActiveContext())
    {
        for (const auto& ds : ctx_man_.activeContext().dataSources())
            if (ctx_man_.hasNumInserted(ds.id()))
                dstypes_with_data.insert(ds.dsType());
    }

    // Select Data Sources submenu
    {
        QMenu* sub = menu.addMenu("Select Data Sources");
        sub->setToolTipsVisible(true);
        sub->menuAction()->setToolTip("Select every Data Source, or only those of a given DSType");
        addTo(sub, "All",
              "Select every Data Source",
              [this]{ selectAllDataSources(); });
        for (const auto& ds_type : context::DataSource::dsTypeStrings())
        {
            const std::string ds_type_s = ds_type;
            QAction* a = sub->addAction(QString::fromStdString("From " + ds_type_s));
            a->setToolTip(QString("Select every Data Source of type \"%1\"")
                              .arg(QString::fromStdString(ds_type_s)));
            if (dstypes_with_data.count(ds_type_s))
            {
                connect(a, &QAction::triggered, this, [this, ds_type_s]{
                    if (ctx_man_.hasActiveContext())
                        for (const auto& ds : ctx_man_.activeContext().dataSources())
                            if (ds.dsType() == ds_type_s)
                                setUseDS(ds.id(), true);
                    updateContent();
                });
            }
            else
            {
                a->setEnabled(false);
            }
        }
    }

    // Deselect Data Sources submenu
    {
        QMenu* sub = menu.addMenu("Deselect Data Sources");
        sub->setToolTipsVisible(true);
        sub->menuAction()->setToolTip("Deselect every Data Source, or only those of a given DSType");
        addTo(sub, "All",
              "Deselect every Data Source",
              [this]{ deselectAllDataSources(); });
        for (const auto& ds_type : context::DataSource::dsTypeStrings())
        {
            const std::string ds_type_s = ds_type;
            QAction* a = sub->addAction(QString::fromStdString("From " + ds_type_s));
            a->setToolTip(QString("Deselect every Data Source of type \"%1\"")
                              .arg(QString::fromStdString(ds_type_s)));
            if (dstypes_with_data.count(ds_type_s))
            {
                connect(a, &QAction::triggered, this, [this, ds_type_s]{
                    if (ctx_man_.hasActiveContext())
                        for (const auto& ds : ctx_man_.activeContext().dataSources())
                            if (ds.dsType() == ds_type_s)
                                setUseDS(ds.id(), false);
                    updateContent();
                });
            }
            else
            {
                a->setEnabled(false);
            }
        }
    }
    menu.addSeparator();

    // Set Line submenu
    {
        // Which of L1..L4 actually have data somewhere?
        bool line_has_data[4] = { false, false, false, false };
        if (ctx_man_.hasActiveContext())
        {
            for (const auto& ds : ctx_man_.activeContext().dataSources())
            {
                auto lines_map = ctx_man_.numInsertedLinesMap(ds.id());
                for (unsigned int l = 0; l < 4; ++l)
                    if (lines_map.count(l) && lines_map.at(l) > 0)
                        line_has_data[l] = true;
            }
        }

        QMenu* sub = menu.addMenu("Set Line");
        sub->setToolTipsVisible(true);
        sub->menuAction()->setToolTip("Set the line selection across every Data Source");
        addTo(sub, "Deselect All",
              "Disable all lines across every Data Source",
              [this]{ deselectAllLines(); });
        for (unsigned int l = 0; l < 4; ++l)
        {
            const QString line_str = QString::fromStdString(Utils::String::lineStrFrom(l));
            QAction* a = sub->addAction("Select " + line_str);
            a->setToolTip(QString("Enable line L%1 across every Data Source").arg(l + 1));
            if (line_has_data[l])
            {
                connect(a, &QAction::triggered, this, [this, l]{
                    if (ctx_man_.hasActiveContext())
                        for (const auto& ds : ctx_man_.activeContext().dataSources())
                            setUseDSLine(ds.id(), l, true);
                    updateContent();
                });
            }
            else
            {
                a->setEnabled(false);
            }
        }
    }
    menu.addSeparator();

    add("Toggle Show Counts",
        "Toggle per-DBContent count columns",
        [this]{ toogleShowCounts(); });
    add("Expand All",
        "Expand every row",
        [this]{ tree_widget_->expandAll(); });
    add("Collapse All",
        "Collapse every row",
        [this]{ tree_widget_->collapseAll(); });

    menu.exec(tree_widget_->viewport()->mapToGlobal(pos));
}

/**
 */
void DataSourcesWidget::deleteDataSlot()
{
    runDeleteDialog([](DeleteDataDialog&){}); // no preselection
}

void DataSourcesWidget::deleteForDSType(const std::string& ds_type)
{
    std::set<unsigned int> ds_ids;
    if (ctx_man_.hasActiveContext())
        for (const auto& ds : ctx_man_.activeContext().dataSources())
            if (ds.dsType() == ds_type)
                ds_ids.insert(ds.id());

    runDeleteDialog([ds_type, ds_ids](DeleteDataDialog& dlg){
        dlg.preselectDSTypes({ds_type});
        dlg.preselectDataSources(ds_ids);
    });
}

void DataSourcesWidget::deleteForDataSource(unsigned int ds_id)
{
    runDeleteDialog([ds_id](DeleteDataDialog& dlg){
        dlg.preselectDataSources({ds_id});
    });
}

void DataSourcesWidget::runDeleteDialog(std::function<void(DeleteDataDialog&)> preselect)
{
    loginf;

    if (delete_job_)
    {
        QMessageBox::warning(this, "Delete Data", "A delete operation is already in progress.");
        return;
    }

    DeleteDataDialog dlg(ctx_man_, this);

    if (preselect)
        preselect(dlg);

    if (dlg.exec() != QDialog::Accepted)
        return;

    QString description = dlg.deleteDescription();

    if (description.isEmpty())
    {
        QMessageBox::information(this, "Delete Data", "No data selected for deletion.");
        return;
    }

    nlohmann::json delete_info = dlg.selectedDeleteInfo();

    if (delete_info.empty())
    {
        QMessageBox::information(this, "Delete Data", "No data selected for deletion.");
        return;
    }

    if (!QuestionDialog::ask(this, "Confirm Delete",
            "The following data will be permanently deleted:\n\n" + description
                + "\n\nThis cannot be undone. Continue?"))
        return;

    // clear loaded dataset
    ctx_man_.compass().dbContentManager().clearData();

    // show blocking wait dialog
    delete_wait_dialog_ = new QMessageBox(this);
    delete_wait_dialog_->setWindowTitle("Deleting Data");
    delete_wait_dialog_->setText("Please wait ...");
    delete_wait_dialog_->setStandardButtons(QMessageBox::NoButton);
    delete_wait_dialog_->setWindowModality(Qt::ApplicationModal);
    delete_wait_dialog_->show();

    // create and run delete job
    delete_job_ = std::make_shared<DBContentDeleteDBJob>(ctx_man_.compass().dbInterface());
    delete_job_->setDeleteInfo(delete_info);
    delete_job_->cleanupDB(true);

    connect(delete_job_.get(), &DBContentDeleteDBJob::doneSignal,
            this, &DataSourcesWidget::deleteJobDoneSlot, Qt::QueuedConnection);

    ctx_man_.compass().jobManager().addDBJob(delete_job_);
}

/**
 */
void DataSourcesWidget::deleteJobDoneSlot()
{
    loginf;

    traced_assert(delete_job_);

    ctx_man_.applyDeleteInfo(delete_job_->deleteInfo());

    delete_job_ = nullptr;

    if (delete_wait_dialog_)
    {
        delete_wait_dialog_->close();
        delete delete_wait_dialog_;
        delete_wait_dialog_ = nullptr;
    }

    updateContent(true);
}

