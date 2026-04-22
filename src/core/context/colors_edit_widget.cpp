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

#include "colors_edit_widget.h"

#include "color_provider.h"
#include "compass.h"
#include "data_source.h"
#include "db_context.h"
#include "db_context_manager.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "logger.h"

#include <QColorDialog>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLayoutItem>
#include <QPushButton>
#include <QRadioButton>
#include <QVBoxLayout>

namespace context
{

ColorsEditWidget::ColorsEditWidget(DBContextManager& manager,
                                   std::function<void()> on_changed,
                                   QWidget* parent)
    : QWidget(parent)
    , manager_(manager)
    , on_changed_(std::move(on_changed))
{
    buildUI();
    refresh();
}

void ColorsEditWidget::buildUI()
{
    auto* main = new QVBoxLayout(this);

    // ==== Preference ====
    {
        auto* box = new QGroupBox("Preference");
        auto* row = new QHBoxLayout(box);

        light_radio_ = new QRadioButton("Light");
        dark_radio_ = new QRadioButton("Dark");

        connect(light_radio_, &QRadioButton::toggled, this,
                &ColorsEditWidget::preferenceChangedSlot);
        connect(dark_radio_, &QRadioButton::toggled, this,
                &ColorsEditWidget::preferenceChangedSlot);

        row->addWidget(light_radio_);
        row->addWidget(dark_radio_);
        row->addStretch();

        auto* hint = new QLabel(
            "Changing the preference only affects future auto-generated data source colors. "
            "Existing data source colors are kept.");
        hint->setWordWrap(true);

        auto* vbox = new QVBoxLayout();
        vbox->addLayout(row);
        vbox->addWidget(hint);
        box->setLayout(vbox);
        main->addWidget(box);
    }

    // ==== DSType Colors ====
    {
        auto* box = new QGroupBox("DSType Colors");
        auto* vbox = new QVBoxLayout(box);

        ds_type_grid_ = new QGridLayout();
        ds_type_grid_->setHorizontalSpacing(8);
        ds_type_grid_->setColumnStretch(2, 1); // push name+button to the left
        vbox->addLayout(ds_type_grid_);

        auto* reset = new QPushButton("Reset to defaults");
        reset->setIcon(QIcon());
        reset->setToolTip("Restore the default DSType color palette");
        connect(reset, &QPushButton::clicked, this,
                &ColorsEditWidget::resetDSTypeDefaultsSlot);

        auto* row = new QHBoxLayout();
        row->addStretch();
        row->addWidget(reset);
        vbox->addLayout(row);

        main->addWidget(box);
    }

    // ==== DBContent Colors ====
    {
        auto* box = new QGroupBox("DBContent Colors");
        auto* vbox = new QVBoxLayout(box);

        dbcontent_grid_ = new QGridLayout();
        dbcontent_grid_->setHorizontalSpacing(8);
        dbcontent_grid_->setColumnStretch(2, 1);
        vbox->addLayout(dbcontent_grid_);

        auto* reset = new QPushButton("Reset to defaults");
        reset->setIcon(QIcon());
        reset->setToolTip("Restore the default DBContent color palette");
        connect(reset, &QPushButton::clicked, this,
                &ColorsEditWidget::resetDBContentDefaultsSlot);

        auto* row = new QHBoxLayout();
        row->addStretch();
        row->addWidget(reset);
        vbox->addLayout(row);

        main->addWidget(box);
    }

    // ==== Data Source Colors ====
    {
        auto* box = new QGroupBox("Data Source Colors");
        auto* vbox = new QVBoxLayout(box);

        // bulk actions — applied to every data source
        auto* bulk_row = new QHBoxLayout();

        auto* all_light = new QPushButton("Auto Light All");
        all_light->setIcon(QIcon());
        all_light->setToolTip("Regenerate light-band base colors for every data source");
        connect(all_light, &QPushButton::clicked, this,
                &ColorsEditWidget::autoLightAllClickedSlot);
        bulk_row->addWidget(all_light);

        auto* all_dark = new QPushButton("Auto Dark All");
        all_dark->setIcon(QIcon());
        all_dark->setToolTip("Regenerate dark-band base colors for every data source");
        connect(all_dark, &QPushButton::clicked, this,
                &ColorsEditWidget::autoDarkAllClickedSlot);
        bulk_row->addWidget(all_dark);

        auto* all_reset = new QPushButton("Reset All");
        all_reset->setIcon(QIcon());
        all_reset->setToolTip("Regenerate base colors for every data source using the context preference");
        connect(all_reset, &QPushButton::clicked, this,
                &ColorsEditWidget::resetAllClickedSlot);
        bulk_row->addWidget(all_reset);

        bulk_row->addStretch();
        vbox->addLayout(bulk_row);

        data_source_grid_ = new QGridLayout();
        data_source_grid_->setHorizontalSpacing(8);
        vbox->addLayout(data_source_grid_);

        main->addWidget(box);
    }

    main->addStretch();
}

void ColorsEditWidget::applySwatch(QPushButton* button, const QColor& color)
{
    if (!button) return;

    if (color.isValid())
    {
        button->setStyleSheet("background-color: " + color.name() + "; color: transparent;");
        button->setText("");
    }
    else
    {
        button->setStyleSheet("");
    }
}

void ColorsEditWidget::clearLayout(QVBoxLayout* layout)
{
    while (QLayoutItem* item = layout->takeAt(0))
    {
        if (QWidget* w = item->widget())
            w->deleteLater();
        if (QLayout* l = item->layout())
            clearLayout(static_cast<QVBoxLayout*>(l));
        delete item;
    }
}

static void clearGrid(QGridLayout* grid)
{
    while (QLayoutItem* item = grid->takeAt(0))
    {
        if (QWidget* w = item->widget())
            w->deleteLater();
        delete item;
    }
}

void ColorsEditWidget::refresh()
{
    if (!manager_.hasActiveContext())
    {
        light_radio_->setEnabled(false);
        dark_radio_->setEnabled(false);
        return;
    }

    light_radio_->setEnabled(true);
    dark_radio_->setEnabled(true);

    const auto& colors = manager_.activeContext().colors();

    QSignalBlocker bl(light_radio_);
    QSignalBlocker bd(dark_radio_);
    light_radio_->setChecked(colors.preference == ContextColors::Preference::Light);
    dark_radio_->setChecked(colors.preference == ContextColors::Preference::Dark);

    // DSType rows (grid — label and button columns align across all rows)
    clearGrid(ds_type_grid_);
    ds_type_buttons_.clear();
    {
        int row_idx = 0;
        for (const auto& ds_type : DataSource::dsTypeStrings())
        {
            auto* name_lbl = new QLabel(QString::fromStdString(ds_type));
            ds_type_grid_->addWidget(name_lbl, row_idx, 0, Qt::AlignLeft | Qt::AlignVCenter);

            auto* btn = new QPushButton();
            btn->setIcon(QIcon());
            btn->setToolTip(QString::fromStdString("Color for DSType " + ds_type));
            btn->setProperty("ds_type", QString::fromStdString(ds_type));
            btn->setMinimumWidth(120);

            QColor color;
            auto it = colors.ds_type_colors.find(ds_type);
            if (it != colors.ds_type_colors.end())
                color = it->second;
            applySwatch(btn, color);
            if (!color.isValid())
                btn->setText("—");

            connect(btn, &QPushButton::clicked, this,
                    &ColorsEditWidget::dsTypeColorClickedSlot);

            ds_type_grid_->addWidget(btn, row_idx, 1);
            ds_type_buttons_[ds_type] = btn;
            ++row_idx;
        }
    }

    // DBContent rows (grid)
    clearGrid(dbcontent_grid_);
    dbcontent_buttons_.clear();

    std::vector<std::string> dbc_names;
    auto& dbcont_man = manager_.compass().dbContentManager();
    for (auto it = dbcont_man.begin(); it != dbcont_man.end(); ++it)
    {
        if (it->second && it->second->containsTargetReports())
            dbc_names.push_back(it->first);
    }

    std::sort(dbc_names.begin(), dbc_names.end());

    {
        int row_idx = 0;
        for (const auto& name : dbc_names)
        {
            auto* name_lbl = new QLabel(QString::fromStdString(name));
            dbcontent_grid_->addWidget(name_lbl, row_idx, 0, Qt::AlignLeft | Qt::AlignVCenter);

            auto* btn = new QPushButton();
            btn->setIcon(QIcon());
            btn->setToolTip(QString::fromStdString("Color for DBContent " + name));
            btn->setProperty("dbcontent", QString::fromStdString(name));
            btn->setMinimumWidth(120);

            QColor color;
            auto cit = colors.dbcontent_colors.find(name);
            if (cit != colors.dbcontent_colors.end())
                color = cit->second;
            applySwatch(btn, color);
            if (!color.isValid())
                btn->setText("—");

            connect(btn, &QPushButton::clicked, this,
                    &ColorsEditWidget::dbContentColorClickedSlot);

            dbcontent_grid_->addWidget(btn, row_idx, 1);
            dbcontent_buttons_[name] = btn;
            ++row_idx;
        }
    }

    // Data Source rows — columns align across rows:
    //   col 0: name   col 1: base   col 2: "Lines"   col 3..6: L1..L4   col 7: stretch
    clearGrid(data_source_grid_);

    int row_idx = 0;
    const auto& ds_list = manager_.activeContext().dataSources();
    for (const auto& ds : ds_list)
    {
        QString label = QString::fromStdString(ds.name()) +
                        " (" + QString::number(ds.sac()) + "/" + QString::number(ds.sic()) + ")";
        auto* name_lbl = new QLabel(label);
        data_source_grid_->addWidget(name_lbl, row_idx, 0, Qt::AlignLeft | Qt::AlignVCenter);

        auto* base_btn = new QPushButton();
        base_btn->setIcon(QIcon());
        base_btn->setToolTip("Base color for this data source");
        base_btn->setProperty("ds_id", ds.id());
        base_btn->setMinimumWidth(100);
        applySwatch(base_btn, ds.baseColor());
        if (!ds.baseColor().isValid())
            base_btn->setText("—");
        connect(base_btn, &QPushButton::clicked, this,
                &ColorsEditWidget::dsBaseColorClickedSlot);
        data_source_grid_->addWidget(base_btn, row_idx, 1);

        auto* lines_lbl = new QLabel("Lines");
        data_source_grid_->addWidget(lines_lbl, row_idx, 2, Qt::AlignRight | Qt::AlignVCenter);

        for (unsigned int i = 0; i < 4; ++i)
        {
            auto* lb = new QPushButton(QString("L%1").arg(i + 1));
            lb->setIcon(QIcon());
            lb->setToolTip(QString("Color for line L%1 — click to edit").arg(i + 1));
            lb->setProperty("ds_id", ds.id());
            lb->setProperty("line_id", i);
            lb->setMinimumWidth(56);
            applySwatch(lb, ds.lineColor(i));
            connect(lb, &QPushButton::clicked, this,
                    &ColorsEditWidget::dsLineColorClickedSlot);
            data_source_grid_->addWidget(lb, row_idx, 3 + (int)i);
        }

        ++row_idx;
    }

    // push all columns to the left by adding a stretch column on the right
    data_source_grid_->setColumnStretch(7, 1);
}

void ColorsEditWidget::preferenceChangedSlot()
{
    if (!manager_.hasActiveContext())
        return;

    // only react to the button that just became checked
    auto* radio = qobject_cast<QRadioButton*>(sender());
    if (!radio || !radio->isChecked())
        return;

    auto& ctx = manager_.activeContext();
    auto previous = ctx.colors().preference;
    auto next = light_radio_->isChecked()
                    ? ContextColors::Preference::Light
                    : ContextColors::Preference::Dark;

    if (previous == next)
        return;

    ctx.colors().preference = next;

    loginf << "context '" << ctx.name() << "' color preference "
           << (next == ContextColors::Preference::Dark ? "Dark" : "Light");

    if (on_changed_)
        on_changed_();
}

void ColorsEditWidget::dsTypeColorClickedSlot()
{
    if (!manager_.hasActiveContext())
        return;

    auto* btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;

    std::string ds_type = btn->property("ds_type").toString().toStdString();
    if (ds_type.empty()) return;

    auto& colors = manager_.activeContext().colors();
    QColor initial = colors.ds_type_colors.count(ds_type)
                         ? colors.ds_type_colors.at(ds_type)
                         : ColorProvider::defaultDSTypeColor(ds_type);

    QColor chosen = QColorDialog::getColor(initial, this,
                        QString::fromStdString("Color for DSType " + ds_type));
    if (!chosen.isValid())
        return;

    loginf << "dstype '" << ds_type << "' color " << chosen.name().toStdString();

    colors.ds_type_colors[ds_type] = chosen;
    applySwatch(btn, chosen);

    if (on_changed_)
        on_changed_();
}

void ColorsEditWidget::dbContentColorClickedSlot()
{
    if (!manager_.hasActiveContext())
        return;

    auto* btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;

    std::string dbcontent = btn->property("dbcontent").toString().toStdString();
    if (dbcontent.empty()) return;

    auto& colors = manager_.activeContext().colors();
    QColor initial = colors.dbcontent_colors.count(dbcontent)
                         ? colors.dbcontent_colors.at(dbcontent)
                         : ColorProvider::defaultDBContentColor(dbcontent);

    QColor chosen = QColorDialog::getColor(initial, this,
                        QString::fromStdString("Color for DBContent " + dbcontent));
    if (!chosen.isValid())
        return;

    loginf << "dbcontent '" << dbcontent << "' color " << chosen.name().toStdString();

    colors.dbcontent_colors[dbcontent] = chosen;
    applySwatch(btn, chosen);

    if (on_changed_)
        on_changed_();
}

void ColorsEditWidget::resetDSTypeDefaultsSlot()
{
    if (!manager_.hasActiveContext())
        return;

    auto& colors = manager_.activeContext().colors();
    colors.ds_type_colors = ColorProvider::defaultDSTypeColors();

    loginf << "reset DSType palette to defaults";

    refresh();

    if (on_changed_)
        on_changed_();
}

void ColorsEditWidget::resetDBContentDefaultsSlot()
{
    if (!manager_.hasActiveContext())
        return;

    auto& colors = manager_.activeContext().colors();
    colors.dbcontent_colors = ColorProvider::defaultDBContentColors();

    loginf << "reset DBContent palette to defaults";

    refresh();

    if (on_changed_)
        on_changed_();
}

// ============================================================
// Data-source color editing
// ============================================================

namespace
{
DataSource* dsFromSender(DBContextManager& manager, QObject* sender)
{
    auto* btn = qobject_cast<QPushButton*>(sender);
    if (!btn) return nullptr;
    bool ok = false;
    unsigned int ds_id = btn->property("ds_id").toUInt(&ok);
    if (!ok) return nullptr;
    return manager.dataSource(ds_id);
}
} // anonymous

void ColorsEditWidget::dsBaseColorClickedSlot()
{
    auto* ds = dsFromSender(manager_, sender());
    if (!ds) return;

    QColor initial = ds->baseColor().isValid() ? ds->baseColor() : Qt::white;
    QColor color = QColorDialog::getColor(initial, this, "Base Color");
    if (!color.isValid()) return;

    loginf << "ds " << ds->id() << " base color " << color.name().toStdString();

    ds->baseColor(color);

    refresh();
    if (on_changed_) on_changed_();
}

namespace
{
void regenerateAllBaseColors(DBContextManager& manager, ColorProvider::Band band)
{
    // group by ds_type so same-type sources get hue-distance spacing
    std::map<std::string, std::vector<QColor>> existing_by_type;
    for (auto& ds : manager.activeContext().dataSources())
    {
        ds.baseColor(ColorProvider::generateBaseColor(
            existing_by_type[ds.dsType()], band, ds.dsType()));
        existing_by_type[ds.dsType()].push_back(ds.baseColor());
    }
}
} // anonymous

void ColorsEditWidget::autoLightAllClickedSlot()
{
    if (!manager_.hasActiveContext()) return;

    loginf << "bulk regenerate all base colors (Light)";
    regenerateAllBaseColors(manager_, ColorProvider::Band::Light);

    refresh();
    if (on_changed_) on_changed_();
}

void ColorsEditWidget::autoDarkAllClickedSlot()
{
    if (!manager_.hasActiveContext()) return;

    loginf << "bulk regenerate all base colors (Dark)";
    regenerateAllBaseColors(manager_, ColorProvider::Band::Dark);

    refresh();
    if (on_changed_) on_changed_();
}

void ColorsEditWidget::resetAllClickedSlot()
{
    if (!manager_.hasActiveContext()) return;

    loginf << "bulk reset all base colors (using context preference)";
    for (auto& ds : manager_.activeContext().dataSources())
    {
        ds.baseColor(QColor());
        manager_.autoAssignColors(ds);
    }

    refresh();
    if (on_changed_) on_changed_();
}

void ColorsEditWidget::dsLineColorClickedSlot()
{
    auto* btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;
    bool ok = false;
    unsigned int line_id = btn->property("line_id").toUInt(&ok);
    if (!ok || line_id >= 4) return;

    auto* ds = dsFromSender(manager_, sender());
    if (!ds) return;

    QColor initial = ds->lineColor(line_id).isValid() ? ds->lineColor(line_id) : Qt::white;
    QColor color = QColorDialog::getColor(initial, this,
                                          QString("Line L%1 Color").arg(line_id + 1));
    if (!color.isValid()) return;

    loginf << "ds " << ds->id() << " line " << (line_id + 1) << " color "
           << color.name().toStdString();

    ds->setLineColor(line_id, color);

    refresh();
    if (on_changed_) on_changed_();
}

} // namespace context
